// Sample subscriber: PoiEvent 駆動の撮影 mock + auto-resume node (#238)。
//
// `mapoi/events` を subscribe し、`capture_trigger` custom tag が付与された POI で
// `EVENT_PAUSED` を受信したら撮影を mock 実行し、N 秒後に `/mapoi/nav/resume` を
// publish して route 走行を再開させる demo subscriber。
//
// 本 sample は **mock** 実装で、実カメラ API は呼ばずに log 出力 + 固定待機する。
// 実機運用では `do_capture` を image_transport publish / GStreamer / OpenCV
// VideoCapture 等に差し替える。
//
// 仕様 (#238 / #88):
//   - `EVENT_PAUSED` は `pause` 系統 POI の tolerance.xy 内で navigation が停止
//     (cmd_vel dwell) した瞬間に 1 visit 1 回だけ発火 (PoiEvent.msg 参照)。
//   - `capture_trigger` tag を持つ POI で EVENT_PAUSED 受信 → 撮影 mock を起動。
//   - 撮影 mock 完了後に `/mapoi/nav/resume` を publish して route を再開させる。
//     これによって「pause で停止 → 撮影 → 自動 resume」の demo loop が成立する。
//   - 並行する pause 系 POI に対する race は本 mock では無視 (連続 pause が
//     同 capture timer 中に起きた場合は新規 capture を弾いて log のみ)。
//
// `auto_resume_timeout_sec` (mapoi_nav2_bridge) を使えば bridge 側でも自動 resume
// できるが、camera_node は「撮影 mock が完了してから resume」という order を
// demo するのが目的なので独立して resume を publish する。両方有効でも resume が
// 二重発火するだけで実害はない (mapoi_server は idempotent な resume を受ける)。
//
// **Trust domain**: `mapoi/events` の真正性は同一 DDS domain 上のノードを信頼する
// 前提で運用する (#248)。同 domain 上の任意 publisher が偽 PoiEvent を流せば
// 撮影 mock を強制起動でき、resume publish 経由で route を進められる。
// 公開環境で運用する場合は SROS2 / domain 分離 / authentication 等で publisher
// 側を絞ること。本 mock はあくまで信頼境界内の demo 用。
#include "mapoi_turtlebot3_example/camera_node.hpp"

#include <cmath>
#include <limits>

namespace mapoi_turtlebot3_example
{

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("camera_node", options)
{
  // 撮影 mock の所要時間 (s)。撮影完了 → resume publish までのウェイト。
  // 実機ではカメラ露光時間 + 画像 IO の典型値に合わせて調整する想定。
  // 不正値 (負 / 0 / NaN / 極大) は do_capture 内で sanitize されて default に落ちる。
  this->declare_parameter<double>("capture_duration_sec", kCaptureDurationDefaultSec);

  poi_event_sub_ = this->create_subscription<mapoi_interfaces::msg::PoiEvent>(
    "mapoi/events", 10,
    std::bind(&CameraNode::on_poi_event, this, std::placeholders::_1));

  // mapoi_server の resume subscriber は transient_local ではないため、
  // publisher 側 QoS も default (volatile) で十分。late-joiner 対応は不要。
  resume_pub_ = this->create_publisher<std_msgs::msg::String>("mapoi/nav/resume", 10);

  RCLCPP_INFO(this->get_logger(),
    "camera_node started; waiting for EVENT_PAUSED on POIs tagged 'capture_trigger'.");
}

double CameraNode::sanitize_capture_duration_sec(double v)
{
  if (!std::isfinite(v) || v <= 0.0 || v > kCaptureDurationMaxSec) {
    return kCaptureDurationDefaultSec;
  }
  return v;
}

void CameraNode::on_poi_event(const mapoi_interfaces::msg::PoiEvent::SharedPtr msg)
{
  if (msg->event_type != mapoi_interfaces::msg::PoiEvent::EVENT_PAUSED) {
    return;
  }
  if (!has_tag(msg->poi.tags, "capture_trigger")) {
    return;
  }
  if (capture_in_flight_) {
    // 連続 EVENT_PAUSED で同 capture 進行中に重ねて発火するのを防ぐ。実機
    // ではカメラ device の二重起動を避ける意味でも単線化したい。
    RCLCPP_WARN(this->get_logger(),
      "[CAMERA] capture already in flight; ignoring EVENT_PAUSED for poi='%s'",
      msg->poi.name.c_str());
    return;
  }
  do_capture(msg->poi.name, msg->poi.description);
}

bool CameraNode::has_tag(const std::vector<std::string> & tags, const std::string & target)
{
  return std::find(tags.begin(), tags.end(), target) != tags.end();
}

void CameraNode::do_capture(const std::string & poi_name, const std::string & description)
{
  // mock 撮影: 実機では image_transport publish / 画像保存に差し替える。
  RCLCPP_INFO(this->get_logger(),
    "[CAMERA] capture: poi='%s' desc='%s'",
    poi_name.c_str(), description.c_str());

  capture_in_flight_ = true;
  const auto raw_duration = this->get_parameter("capture_duration_sec").as_double();
  const auto duration_sec = sanitize_capture_duration_sec(raw_duration);
  if (duration_sec != raw_duration) {
    // launch yaml / dynamic set で不正値が来た時に silent に default fallback すると
    // demo 時に「思ったより短い / 長い」原因を追えなくなるので必ず log を残す
    // (mapoi_webui_node.py の robot_radius と同じ defense-in-depth 方針、#248)。
    RCLCPP_WARN(this->get_logger(),
      "[CAMERA] capture_duration_sec=%.6f は不正値です (NaN/inf/<=0/>%.1fs)。"
      "default %.3fs を使います。",
      raw_duration, kCaptureDurationMaxSec, kCaptureDurationDefaultSec);
  }
  const auto duration_ms =
    std::chrono::milliseconds(static_cast<int64_t>(duration_sec * 1000.0));

  // one-shot timer: capture_duration_sec 後に resume を publish する。spin
  // しっぱなしの create_wall_timer に cancel() を組み合わせて one-shot 化。
  // captured POI 名を log に残すため、timer callback で poi_name をキャプチャ。
  // timer は node の member なので、`this` は timer 発火時に必ず生きている。
  resume_timer_ = this->create_wall_timer(
    duration_ms,
    [this, poi_name]() {
      publish_resume(poi_name);
    });
}

void CameraNode::publish_resume(const std::string & poi_name)
{
  if (resume_timer_) {
    resume_timer_->cancel();
    resume_timer_.reset();
  }
  std_msgs::msg::String msg;
  // mapoi_server の resume は data 内容を「resume 要求元 identifier」として log に
  // 残すだけで意味解釈はしない (cancel / pause も同じ regime)。camera_node 由来で
  // あることを debug 時に追えるよう identifier を入れる。
  msg.data = "camera_node";
  resume_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(),
    "[CAMERA] capture done; resume published for poi='%s'", poi_name.c_str());
  capture_in_flight_ = false;
}

}  // namespace mapoi_turtlebot3_example

#ifndef UNIT_TEST
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mapoi_turtlebot3_example::CameraNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
