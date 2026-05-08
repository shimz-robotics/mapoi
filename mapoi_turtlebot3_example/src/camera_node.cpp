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
// 推奨フロー:
//   1. `mapoi_server` の `get_pois_info` service を呼び POI list を取得
//   2. `capture_target` tag 持ちの landmark POI を 1 つ選ぶ (sample yaml 命名規約として
//      `capture_target_<name>` 形式、例えば trigger POI 名 `conference_room` → 対応
//      landmark `capture_target_painting` のようにマッピングルールを yaml で表現)
//   3. robot の現位置を TF lookup (`map` → `base_link`) で取得
//   4. `dx = landmark.x - robot.x`、`dy = landmark.y - robot.y`、`yaw = atan2(dy, dx)`
//      で landmark を向く yaw 角を計算
//   5. 姿勢制御:
//      a. Nav2 `NavigateToPose` action を `goal_pose = (robot.x, robot.y, yaw)` で送る
//         (in-place rotation + position 維持)、または
//      b. `cmd_vel` topic に角速度 publish して in-place rotation、TF で yaw 整合確認
//   6. yaw 整合 (`|robot_yaw - target_yaw| < threshold`、例: 0.05 rad) を確認
//   7. 実 capture (画像保存 / camera2_v4l2 / image_transport / OpenCV trigger 等)
//   8. 必要なら robot を元の方向 (route waypoint の `pose.yaw`) に戻してから resume
//
// 注: 5.a の Nav2 NavigateToPose は `mapoi_nav2_bridge` の current FollowWaypoints
// goal と競合する可能性があるため、route 内 capture では 5.b (cmd_vel 直接制御) +
// 一時的な local_costmap mute / Nav2 controller_server の cancel など慎重設計が必要。
// 撮影専用 sub-state (例: `mapoi/nav/capture_in_progress` topic) を別途定義して
// `mapoi_nav2_bridge` 側で controller を一時 mute する拡張も検討候補。
//
// 本 sample では実装ハードルが高いため、まずは「PAUSED イベントの受信 → log で trigger
// 確認」までを示す mock に留めている。本番化する場合は上記フロー + camera_node に
// `get_pois_info` client + tf_buffer + Nav2 action client (or cmd_vel publisher) を
// 持たせて実装する。
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
