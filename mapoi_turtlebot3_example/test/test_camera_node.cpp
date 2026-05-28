// UNIT_TEST マクロは CMakeLists.txt の target_compile_definitions で定義する
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <mapoi_interfaces/msg/poi_event.hpp>
#include <mapoi_interfaces/msg/point_of_interest.hpp>

#include "mapoi_turtlebot3_example/camera_node.hpp"

using namespace std::chrono_literals;

// 注: fixture / test class は camera_node.hpp の friend 宣言と
// 一致させるため namespace mapoi_turtlebot3_example の中に置く。
// (FRIEND_TEST マクロが生成する `friend class Fixture_X_Test` は
// CameraNode の所属 namespace に解決されるため、global namespace に
// fixture を置くと private アクセスを得られない。)
namespace mapoi_turtlebot3_example
{

namespace
{
// 撮影 mock の duration を短くして resume timer 発火を test 時間内に観測する。
// 50ms あれば spin_some の反復で十分 timer 発火を拾える。
constexpr double kTestCaptureDurationSec = 0.05;

mapoi_interfaces::msg::PoiEvent make_event(
  uint8_t event_type,
  const std::string & poi_name,
  const std::vector<std::string> & tags)
{
  mapoi_interfaces::msg::PoiEvent ev;
  ev.event_type = event_type;
  ev.poi.name = poi_name;
  ev.poi.description = "desc-" + poi_name;
  ev.poi.tags = tags;
  return ev;
}
}  // namespace

class CameraNodeTestFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("capture_duration_sec", kTestCaptureDurationSec)});
    node_ = std::make_shared<CameraNode>(opts);

    // resume publish を観測する subscriber。on_poi_event は同 process 内で呼ぶので
    // PoiEvent publisher は不要 (テストから直接 on_poi_event を叩く)。
    helper_ = rclcpp::Node::make_shared("camera_node_test_helper");
    resume_sub_ = helper_->create_subscription<std_msgs::msg::String>(
      "mapoi/nav/resume", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        resume_messages_.push_back(msg->data);
      });

    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);
    exec_->add_node(helper_);

    // discovery 安定待ち: publisher と subscriber が同 process 内で生成された
    // 直後に publish しても受信できないことがあるため、matched event を待つ。
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline &&
           resume_sub_->get_publisher_count() == 0)
    {
      exec_->spin_some(10ms);
      std::this_thread::sleep_for(10ms);
    }
  }

  void TearDown() override
  {
    exec_.reset();
    resume_sub_.reset();
    helper_.reset();
    node_.reset();
    resume_messages_.clear();
  }

  // exec_->spin_some を timeout / 期待 message 数まで反復する helper。
  // 0.05s の resume timer 発火を待つには 200ms あれば十分。
  void spin_until(std::chrono::milliseconds timeout, size_t expected_count = 0)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      exec_->spin_some(10ms);
      if (expected_count > 0 && resume_messages_.size() >= expected_count) {
        return;
      }
      std::this_thread::sleep_for(5ms);
    }
  }

  std::shared_ptr<CameraNode> node_;
  rclcpp::Node::SharedPtr helper_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr resume_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::vector<std::string> resume_messages_;
};

// --- sanitize_capture_duration_sec ---------------------------------------

TEST_F(CameraNodeTestFixture, SanitizeCaptureDurationAcceptsPositive)
{
  EXPECT_DOUBLE_EQ(CameraNode::sanitize_capture_duration_sec(1.5), 1.5);
  EXPECT_DOUBLE_EQ(CameraNode::sanitize_capture_duration_sec(0.1), 0.1);
  EXPECT_DOUBLE_EQ(CameraNode::sanitize_capture_duration_sec(60.0), 60.0);
}

TEST_F(CameraNodeTestFixture, SanitizeCaptureDurationAcceptsBoundary)
{
  // min / max は inclusive。境界値が retain されることを pin する
  // (boundary off-by-one regression 防止)。
  EXPECT_DOUBLE_EQ(
    CameraNode::sanitize_capture_duration_sec(CameraNode::kCaptureDurationMinSec),
    CameraNode::kCaptureDurationMinSec);
  EXPECT_DOUBLE_EQ(
    CameraNode::sanitize_capture_duration_sec(CameraNode::kCaptureDurationMaxSec),
    CameraNode::kCaptureDurationMaxSec);
}

TEST_F(CameraNodeTestFixture, SanitizeCaptureDurationRejectsZero)
{
  EXPECT_DOUBLE_EQ(
    CameraNode::sanitize_capture_duration_sec(0.0),
    CameraNode::kCaptureDurationDefaultSec);
}

TEST_F(CameraNodeTestFixture, SanitizeCaptureDurationRejectsNegative)
{
  EXPECT_DOUBLE_EQ(
    CameraNode::sanitize_capture_duration_sec(-1.0),
    CameraNode::kCaptureDurationDefaultSec);
  EXPECT_DOUBLE_EQ(
    CameraNode::sanitize_capture_duration_sec(-0.001),
    CameraNode::kCaptureDurationDefaultSec);
}

TEST_F(CameraNodeTestFixture, SanitizeCaptureDurationRejectsSubMillisecond)
{
  // 極小正値 (< 1ms) は static_cast<int64_t>(v * 1000.0) で 0ms タイマー化されて
  // create_wall_timer(0ms) 相当の即時発火になるため、撮影 mock として意味が無い。
  // μs と s の単位ミス typo (例: 0.0001) も同じ経路で弾く。
  EXPECT_DOUBLE_EQ(
    CameraNode::sanitize_capture_duration_sec(0.0001),
    CameraNode::kCaptureDurationDefaultSec);
  EXPECT_DOUBLE_EQ(
    CameraNode::sanitize_capture_duration_sec(std::numeric_limits<double>::denorm_min()),
    CameraNode::kCaptureDurationDefaultSec);
  // 0.001 直下 (0.0009) も拒否されることを確認 (inclusive minimum の確認)
  EXPECT_DOUBLE_EQ(
    CameraNode::sanitize_capture_duration_sec(0.0009),
    CameraNode::kCaptureDurationDefaultSec);
}

TEST_F(CameraNodeTestFixture, SanitizeCaptureDurationRejectsNaN)
{
  EXPECT_DOUBLE_EQ(
    CameraNode::sanitize_capture_duration_sec(std::nan("")),
    CameraNode::kCaptureDurationDefaultSec);
}

TEST_F(CameraNodeTestFixture, SanitizeCaptureDurationRejectsInfinity)
{
  EXPECT_DOUBLE_EQ(
    CameraNode::sanitize_capture_duration_sec(std::numeric_limits<double>::infinity()),
    CameraNode::kCaptureDurationDefaultSec);
  EXPECT_DOUBLE_EQ(
    CameraNode::sanitize_capture_duration_sec(-std::numeric_limits<double>::infinity()),
    CameraNode::kCaptureDurationDefaultSec);
}

TEST_F(CameraNodeTestFixture, SanitizeCaptureDurationRejectsTooLarge)
{
  // 600s 超は typo (ms と s の取り違え等) と看做して default fallback。
  EXPECT_DOUBLE_EQ(
    CameraNode::sanitize_capture_duration_sec(600.001),
    CameraNode::kCaptureDurationDefaultSec);
  EXPECT_DOUBLE_EQ(
    CameraNode::sanitize_capture_duration_sec(1.0e9),
    CameraNode::kCaptureDurationDefaultSec);
}

// --- has_tag (共通ユーティリティ #248 項目 8) ------------------------------

TEST_F(CameraNodeTestFixture, HasTagFindsExisting)
{
  EXPECT_TRUE(has_tag({"pause", "capture_trigger"}, "capture_trigger"));
}

TEST_F(CameraNodeTestFixture, HasTagSkipsAbsent)
{
  EXPECT_FALSE(has_tag({"pause"}, "capture_trigger"));
  EXPECT_FALSE(has_tag({}, "capture_trigger"));
}

// --- on_poi_event behavior -----------------------------------------------

TEST_F(CameraNodeTestFixture, IgnoresNonPausedEvents)
{
  auto ev = std::make_shared<mapoi_interfaces::msg::PoiEvent>(make_event(
    mapoi_interfaces::msg::PoiEvent::EVENT_ENTER, "poi_a", {"capture_trigger"}));
  node_->on_poi_event(ev);
  EXPECT_FALSE(node_->capture_in_flight_.load());
  spin_until(200ms);
  EXPECT_TRUE(resume_messages_.empty());
}

TEST_F(CameraNodeTestFixture, IgnoresEventsWithoutCaptureTriggerTag)
{
  auto ev = std::make_shared<mapoi_interfaces::msg::PoiEvent>(make_event(
    mapoi_interfaces::msg::PoiEvent::EVENT_PAUSED, "poi_a", {"pause"}));
  node_->on_poi_event(ev);
  EXPECT_FALSE(node_->capture_in_flight_.load());
  spin_until(200ms);
  EXPECT_TRUE(resume_messages_.empty());
}

TEST_F(CameraNodeTestFixture, CapturesAndPublishesResumeAfterDuration)
{
  auto ev = std::make_shared<mapoi_interfaces::msg::PoiEvent>(make_event(
    mapoi_interfaces::msg::PoiEvent::EVENT_PAUSED, "poi_a", {"pause", "capture_trigger"}));
  node_->on_poi_event(ev);
  EXPECT_TRUE(node_->capture_in_flight_.load());

  // wall timer は 50ms 後に発火する想定。500ms は十分なマージン。
  spin_until(500ms, 1);
  ASSERT_EQ(resume_messages_.size(), 1u);
  // resume identifier に POI 名を含める (#248 項目 5)
  EXPECT_EQ(resume_messages_[0], "camera_node:poi_a");
  EXPECT_FALSE(node_->capture_in_flight_.load());
}

TEST_F(CameraNodeTestFixture, IgnoresSecondPausedWhileCaptureInFlight)
{
  // 1 回目: capture を仕掛ける (timer 未発火のまま待つ)
  auto ev1 = std::make_shared<mapoi_interfaces::msg::PoiEvent>(make_event(
    mapoi_interfaces::msg::PoiEvent::EVENT_PAUSED, "poi_a", {"pause", "capture_trigger"}));
  node_->on_poi_event(ev1);
  ASSERT_TRUE(node_->capture_in_flight_.load());

  // 2 回目: capture_in_flight_ の間に来た EVENT_PAUSED は compare_exchange で
  // 弾かれる (#248 項目 3)。resume_timer_ が上書きされていないことも publish
  // 回数で間接確認する。
  auto ev2 = std::make_shared<mapoi_interfaces::msg::PoiEvent>(make_event(
    mapoi_interfaces::msg::PoiEvent::EVENT_PAUSED, "poi_b", {"pause", "capture_trigger"}));
  node_->on_poi_event(ev2);

  // timer 発火を待つ。capture が 1 回だけなら resume publish も 1 回。
  spin_until(500ms, 1);
  ASSERT_EQ(resume_messages_.size(), 1u);
  // 識別子は最初に capture を取った poi_a のみ。poi_b は弾かれている。
  EXPECT_EQ(resume_messages_[0], "camera_node:poi_a");
  // capture 完了後にさらに spin しても 2 回目の resume は来ない。
  spin_until(200ms);
  EXPECT_EQ(resume_messages_.size(), 1u);
}

TEST_F(CameraNodeTestFixture, ResumeMessageIncludesPoiName)
{
  // poi 名違いの 2 連続 capture を sequential に走らせて、それぞれ
  // `camera_node:<poi_name>` 形式で識別できることを pin する (#248 項目 5)。
  auto ev1 = std::make_shared<mapoi_interfaces::msg::PoiEvent>(make_event(
    mapoi_interfaces::msg::PoiEvent::EVENT_PAUSED, "spot_north", {"capture_trigger"}));
  node_->on_poi_event(ev1);
  spin_until(500ms, 1);
  ASSERT_EQ(resume_messages_.size(), 1u);
  EXPECT_EQ(resume_messages_[0], "camera_node:spot_north");

  auto ev2 = std::make_shared<mapoi_interfaces::msg::PoiEvent>(make_event(
    mapoi_interfaces::msg::PoiEvent::EVENT_PAUSED, "spot_south", {"capture_trigger"}));
  node_->on_poi_event(ev2);
  spin_until(500ms, 2);
  ASSERT_EQ(resume_messages_.size(), 2u);
  EXPECT_EQ(resume_messages_[1], "camera_node:spot_south");
}

TEST_F(CameraNodeTestFixture, DestructorCancelsPendingResumeTimer)
{
  // do_capture で timer を仕掛けてから timer 発火前に node を destroy する。
  // destructor で resume_timer_ が cancel + reset され、その後に
  // timer callback が走って publish_resume されないことを pin する
  // (#248 項目 7)。
  auto ev = std::make_shared<mapoi_interfaces::msg::PoiEvent>(make_event(
    mapoi_interfaces::msg::PoiEvent::EVENT_PAUSED, "poi_a", {"capture_trigger"}));
  node_->on_poi_event(ev);
  ASSERT_TRUE(node_->capture_in_flight_.load());

  // timer 未発火のうちに node を破棄。executor からも外す。
  exec_->remove_node(node_);
  node_.reset();

  // destructor 後に十分待っても resume が来ないことを確認する。
  spin_until(300ms);
  EXPECT_TRUE(resume_messages_.empty());
}

}  // namespace mapoi_turtlebot3_example
