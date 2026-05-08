// Sample subscriber: PoiEvent 駆動の撮影 mock node (#88)。
//
// `mapoi/events` を subscribe し、`capture_trigger` custom tag が付与された POI で
// **`EVENT_PAUSED`** を受信したら撮影を実行する想定の sample。`EVENT_PAUSED` は
// pause タグ付き POI 内で navigation が停止した時に発火するため、`capture_trigger`
// POI には `pause` タグも併せて付与する必要がある (`mapoi_config.yaml` で
// `tags: [waypoint, capture_trigger, pause]` のように)。
//
// 撮影対象は `capture_target` custom tag 付き landmark POI を `get_pois_info`
// service で参照する想定だが、本 sample は **mock** 実装で、ログに
// `[CAMERA] capture: <trigger_poi_name>` を出力するだけ。実機運用では
// `capture()` 関数を画像保存 / カメラ trigger system call に差し替える。
//
// 仕様 (#220 / #88):
//   - `EVENT_PAUSED` は pause タグ POI + cmd_vel dwell + route 走行中のみ発火 (新 PoiEvent 仕様)。
//   - `capture_trigger` tag 持ちの POI で EVENT_PAUSED 受信 → capture trigger。
//   - 同 visit 中は 1 回のみ発火 (PoiEvent の lifecycle で保証、EXIT で flag reset)。
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
