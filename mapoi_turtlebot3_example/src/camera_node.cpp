// Sample subscriber: PoiEvent 駆動の撮影 mock node (#88)。
//
// `mapoi/events` を subscribe し、`capture_trigger` custom tag が付与された POI で
// **`EVENT_PAUSED`** を受信したら撮影を実行する想定の sample。`EVENT_PAUSED` は
// pause タグ付き POI 内で navigation が停止した時に発火するため、`capture_trigger`
// POI には `pause` タグも併せて付与する必要がある (`mapoi_config.yaml` で
// `tags: [waypoint, capture_trigger, pause]` のように)。
//
// 本 sample は **mock** 実装で、ログに `[CAMERA] capture: <trigger_poi_name>` を
// 出力するだけ。実機運用では `capture()` 関数を画像保存 / カメラ trigger system call
// に差し替える。
//
// 仕様 (#220 / #88):
//   - `EVENT_PAUSED` は pause タグ POI + cmd_vel dwell + route 走行中のみ発火 (v0.5.0+ の PoiEvent 仕様)。
//   - `capture_trigger` tag 持ちの POI で EVENT_PAUSED 受信 → capture trigger。
//   - 同 visit 中は 1 回のみ発火 (PoiEvent の lifecycle で保証、EXIT で flag reset)。
//
// ---------------------------------------------------------------------------
// 実装指針: 撮影 POI の方向制御 (本 sample 未実装、本番化 path)
// ---------------------------------------------------------------------------
// capture_trigger POI (waypoint) と撮影対象 landmark (例: `capture_target_painting`、
// `landmark` + `capture_target` tag 持ち) は別位置に置くのが自然。robot を landmark
// 方向に向けて撮影する必要があるが、**動的 yaw 制御 (NavigateToPose / cmd_vel) は不要**:
// Nav2 が waypoint の `pose.yaw` を SimpleGoalChecker で整合させるので、yaml で
// `pose.yaw` を landmark 方向に **static 設定**するのが最もシンプル。
//
// 設計の根拠:
//   - mapoi の `tolerance.yaw` は **mapoi 内部メタデータのみ** (RViz 描画 / validation)、
//     Nav2 controller_server には forward されない (PoseStamped に tolerance フィールド
//     がないため)。yaw の実効精度は Nav2 controller 側の global `yaw_goal_tolerance`
//     parameter (`burger.yaml` の `controller_server.FollowPath.goal_checker`) が支配
//   - 動的 yaw 計算 (camera_node 側で TF lookup + atan2 計算 + cmd_vel 制御) は、
//     waypoint center で landmark 方向に向くだけなら **過剰**。Nav2 が pose.yaw を
//     整合させて止めるので yaml 設定で済む
//
// 推奨フロー (camera_node 側):
//   1. (yaml 編集 = 設計時の static 設定): 撮影 POI の `pose.yaw` を yaml に書く時、
//      `atan2(landmark.y - waypoint.y, landmark.x - waypoint.x)` で landmark への角度
//      を計算して入れる。例: waypoint=(0.7, -0.4)、landmark=(2.0, 0.5) → yaw ≈ 0.61 rad
//   2. (必要なら) `burger.yaml` の `yaw_goal_tolerance` を tighten。default `0.25` rad
//      (~14°) で大半の撮影 sample はカメラ画角内に収まる。tighten range は 0.05-0.30 rad
//      (それ以下は controller 振動 risk)
//   3. EVENT_PAUSED 受信 → robot は既に pose.yaw 整合状態 (Nav2 が SimpleGoalChecker
//      で目標到達確認した時点で yaw_goal_tolerance 内)、即座に **撮影実行**
//   4. **`mapoi/nav/resume` topic に publish** → bridge が `paused_waypoints_` から
//      FollowWaypoints を再送 → route 走行復帰
//
// 想定 worst case yaw 誤差 (FOV 試算):
//   yaw_goal_tolerance + atan2(xy_goal_tolerance, distance_to_landmark)
//   例: yaw_tol=0.25, xy_tol=0.25, d=2m → 0.25 + 0.12 = 0.37 rad (~21°)
//   一般的カメラ FOV (60-90°) なら両側マージン込みで余裕に収まる
//
// 動的 yaw 計算が必要なケース (本指針で対応できない、follow-up issue 候補):
//   - landmark が waypoint から非常に近い (距離 < 0.5m): xy_tol drift で角度誤差が
//     画角を超える可能性
//   - 走行中の動的 landmark (人間追跡等): static yaml 設定では不可、camera_node で
//     TF lookup + closed-loop 制御が必要
//
// 前提:
//   - 撮影直後に `mapoi/nav/resume` を出さないと、route が無限 pause のまま停止する
//
// 本 sample は「PAUSED イベントの受信 → log で trigger 確認」の mock のみ。本番化する
// 場合は撮影 sdk (camera2_v4l2 / image_transport / OpenCV 等) + `mapoi/nav/resume`
// publisher を持たせるのが実装の最小セット。
#include <algorithm>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <mapoi_interfaces/msg/poi_event.hpp>

namespace mapoi_turtlebot3_example
{

class CameraNode : public rclcpp::Node
{
public:
  CameraNode()
  : Node("camera_node")
  {
    poi_event_sub_ = this->create_subscription<mapoi_interfaces::msg::PoiEvent>(
      "mapoi/events", 10,
      std::bind(&CameraNode::on_poi_event, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
      "camera_node started; waiting for EVENT_PAUSED on POIs tagged "
      "'capture_trigger' (the POI must also have 'pause' tag for EVENT_PAUSED to fire).");
  }

private:
  void on_poi_event(const mapoi_interfaces::msg::PoiEvent::SharedPtr msg)
  {
    if (msg->event_type != mapoi_interfaces::msg::PoiEvent::EVENT_PAUSED) {
      return;
    }
    if (!has_tag(msg->poi.tags, "capture_trigger")) {
      return;
    }
    capture(msg->poi.name, msg->poi.description);
  }

  static bool has_tag(const std::vector<std::string> & tags, const std::string & target)
  {
    return std::find(tags.begin(), tags.end(), target) != tags.end();
  }

  void capture(const std::string & poi_name, const std::string & description)
  {
    // mock 撮影: 実機では camera2_v4l2 / image_view 連携 / OpenCV trigger 等に差し替える。
    // 撮影対象 POI (capture_target tag) を get_pois_info で fetch する拡張は
    // sample scope 外 (issue #88 の original spec で landmark 参照は mock OK と明記)。
    RCLCPP_INFO(this->get_logger(),
      "[CAMERA] capture: trigger_poi='%s' desc='%s'",
      poi_name.c_str(), description.c_str());
  }

  rclcpp::Subscription<mapoi_interfaces::msg::PoiEvent>::SharedPtr poi_event_sub_;
};

}  // namespace mapoi_turtlebot3_example

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mapoi_turtlebot3_example::CameraNode>());
  rclcpp::shutdown();
  return 0;
}
