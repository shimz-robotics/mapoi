#ifndef MAPOI_TURTLEBOT3_EXAMPLE__CAMERA_NODE_HPP_
#define MAPOI_TURTLEBOT3_EXAMPLE__CAMERA_NODE_HPP_

#ifdef UNIT_TEST
#include <gtest/gtest.h>
#endif

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <mapoi_interfaces/msg/poi_event.hpp>

namespace mapoi_turtlebot3_example
{

class CameraNode : public rclcpp::Node
{
public:
  // capture_duration_sec の安全範囲。
  //   - 下限 0 は "撮影しないで即 resume" になるため弾く。実機 capture も常に正値。
  //   - 上限 600s は demo / 試験で想定されるカメラ露光の最大値。これを超える値は
  //     大抵 typo (秒/ミリ秒の単位ミス) なので WARN して default に落とす。
  static constexpr double kCaptureDurationDefaultSec = 1.5;
  static constexpr double kCaptureDurationMaxSec = 600.0;

  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // capture_duration_sec の入力値を安全な範囲に正規化する純関数。
  //   - NaN / inf / 0 以下 / kCaptureDurationMaxSec 超 → kCaptureDurationDefaultSec
  //   - それ以外はそのまま返す
  // 純関数として切り出しているのは unit test で境界条件を pin するため。
  static double sanitize_capture_duration_sec(double v);

private:
  void on_poi_event(const mapoi_interfaces::msg::PoiEvent::SharedPtr msg);
  void do_capture(const std::string & poi_name, const std::string & description);
  void publish_resume(const std::string & poi_name);

  static bool has_tag(const std::vector<std::string> & tags, const std::string & target);

  rclcpp::Subscription<mapoi_interfaces::msg::PoiEvent>::SharedPtr poi_event_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr resume_pub_;
  rclcpp::TimerBase::SharedPtr resume_timer_;
  bool capture_in_flight_{false};

#ifdef UNIT_TEST
  friend class CameraNodeTestFixture;
  FRIEND_TEST(CameraNodeTestFixture, SanitizeCaptureDurationAcceptsPositive);
  FRIEND_TEST(CameraNodeTestFixture, SanitizeCaptureDurationRejectsZero);
  FRIEND_TEST(CameraNodeTestFixture, SanitizeCaptureDurationRejectsNegative);
  FRIEND_TEST(CameraNodeTestFixture, SanitizeCaptureDurationRejectsNaN);
  FRIEND_TEST(CameraNodeTestFixture, SanitizeCaptureDurationRejectsInfinity);
  FRIEND_TEST(CameraNodeTestFixture, SanitizeCaptureDurationRejectsTooLarge);
  FRIEND_TEST(CameraNodeTestFixture, HasTagFindsExisting);
  FRIEND_TEST(CameraNodeTestFixture, HasTagSkipsAbsent);
  FRIEND_TEST(CameraNodeTestFixture, IgnoresNonPausedEvents);
  FRIEND_TEST(CameraNodeTestFixture, IgnoresEventsWithoutCaptureTriggerTag);
  FRIEND_TEST(CameraNodeTestFixture, CapturesAndPublishesResumeAfterDuration);
  FRIEND_TEST(CameraNodeTestFixture, IgnoresSecondPausedWhileCaptureInFlight);
#endif
};

}  // namespace mapoi_turtlebot3_example

#endif  // MAPOI_TURTLEBOT3_EXAMPLE__CAMERA_NODE_HPP_
