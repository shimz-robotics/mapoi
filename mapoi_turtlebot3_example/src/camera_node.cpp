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
// 実装指針: 撮影対象 (landmark) への姿勢制御 (本 sample 未実装、本番化 path)
// ---------------------------------------------------------------------------
// capture_trigger POI (waypoint) と撮影対象 landmark (例: `capture_target_painting`、
// `landmark` + `capture_target` tag 持ち) は別位置に置くのが自然なので、`EVENT_PAUSED`
// で停止した後に landmark の方向を向いてから撮影する処理が必要になる。
//
// シナリオ全体像:
//   pause タグ POI ENTER → 自動 pause (bridge が FollowWaypoints を cancel + 残
//   waypoints を保存) → cmd_vel dwell → EVENT_PAUSED → camera_node が landmark
//   方向へ yaw 制御 → 撮影 → camera_node が `mapoi/nav/resume` publish → bridge が
//   `paused_waypoints_` から FollowWaypoints 再送 → route 走行復帰
//
// 推奨フロー (camera_node 側):
//   1. `mapoi_server` の `get_pois_info` service を呼び POI list を取得
//   2. `capture_target` tag 持ちの landmark POI を 1 つ選ぶ (sample yaml 命名規約として
//      `capture_target_<name>` 形式、例えば trigger POI 名 `conference_room` → 対応
//      landmark `capture_target_painting` のようにマッピングルールを yaml で表現)
//   3. robot の現位置を TF lookup (`map` → `base_link`) で取得
//   4. `dx = landmark.x - robot.x`、`dy = landmark.y - robot.y`、`yaw = atan2(dy, dx)`
//      で landmark を向く target yaw を計算
//   5. **`cmd_vel` topic へ角速度を直接 publish** して in-place rotation。簡易 P
//      制御で `angular.z = kp * (target_yaw - current_yaw)` の式、TF で current_yaw
//      を周期取得しながら closed-loop。
//      **NavigateToPose は使わない**: mapoi_nav2_bridge は pause で FollowWaypoints を
//      cancel しているだけで `paused_waypoints_` を保持しており、camera_node が
//      NavigateToPose を別途送ると bridge の resume 経路と Nav2 action server で
//      goal 上書き race になる
//   6. yaw 整合 (`|current_yaw - target_yaw| < threshold`、例: 0.05 rad) を確認、
//      角速度 = 0 を 1 tick publish して停止
//   7. 実 capture (画像保存 / camera2_v4l2 / image_transport / OpenCV trigger 等)
//   8. **`mapoi/nav/resume` topic に publish** して route 走行に復帰させる。
//      bridge が `paused_waypoints_` から FollowWaypoints を再送するので、
//      robot は次の waypoint へ自動進行 (新規 yaw 整合は Nav2 controller が解決)
//
// 前提条件:
//   - 採用 controller が pause 中に `cmd_vel` publish を停止していること (Nav2 default)。
//     pause 中も controller_server が cmd_vel publish を継続する特殊 implementation
//     では camera_node の cmd_vel と衝突する。対策は controller_server の `cmd_vel`
//     remap で出力を空 topic に逸らす、または撮影専用 sub-state を bridge 側に追加
//     する拡張 (今後 issue 化候補)
//   - 撮影直後に `mapoi/nav/resume` を出さないと、route が無限 pause のまま停止
//
// 本 sample では実装ハードルが高いため、まずは「PAUSED イベントの受信 → log で trigger
// 確認」までを示す mock に留めている。本番化する場合は上記フロー + camera_node に
// `get_pois_info` client + tf_buffer + cmd_vel publisher + `mapoi/nav/resume`
// publisher を持たせて実装する。
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
