// UNIT_TEST マクロは CMakeLists.txt の target_compile_definitions で定義する
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "mapoi_server/mapoi_nav_server.hpp"

class NavServerTestFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<MapoiNavServer>();
  }
  void TearDown() override
  {
    node_.reset();
  }

  mapoi_interfaces::msg::PointOfInterest make_poi(
    const std::string & name, double x, double y,
    double tolerance_xy, const std::vector<std::string> & tags)
  {
    mapoi_interfaces::msg::PointOfInterest poi;
    poi.name = name;
    poi.pose.position.x = x;
    poi.pose.position.y = y;
    poi.tolerance.xy = tolerance_xy;
    poi.tolerance.yaw = 0.0;
    poi.tags = tags;
    return poi;
  }

  std::shared_ptr<MapoiNavServer> node_;
};

TEST_F(NavServerTestFixture, DistanceCalculation)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 3.0;
  pose.position.y = 4.0;
  double dist = node_->distance_2d(pose, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(dist, 5.0);
}

TEST_F(NavServerTestFixture, DistanceCalculationZero)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  double dist = node_->distance_2d(pose, 1.0, 2.0);
  EXPECT_DOUBLE_EQ(dist, 0.0);
}

TEST_F(NavServerTestFixture, RebuildEventPoisIncludesAllPois)
{
  {
    std::lock_guard<std::mutex> lock(node_->data_mutex_);
    node_->pois_list_.push_back(make_poi("goal_only", 1.0, 0.0, 0.5, {"waypoint"}));
    node_->pois_list_.push_back(make_poi("with_pause", 0.0, 2.0, 0.5, {"waypoint", "pause"}));
    node_->pois_list_.push_back(make_poi("with_custom", 3.0, 0.0, 0.5, {"waypoint", "audio_info"}));
    node_->pois_list_.push_back(make_poi("landmark_ref", 0.0, 0.0, 0.5, {"landmark"}));
  }
  node_->rebuild_event_pois();
  EXPECT_EQ(node_->event_pois_.size(), 4u);
}

TEST_F(NavServerTestFixture, RebuildEventPoisEmpty)
{
  node_->rebuild_event_pois();
  EXPECT_EQ(node_->event_pois_.size(), 0u);
}

TEST_F(NavServerTestFixture, PauseTagDetection)
{
  auto poi_pause = make_poi("pause_poi", 0.0, 0.0, 0.5, {"waypoint", "pause"});
  auto poi_no_pause = make_poi("normal_poi", 0.0, 0.0, 0.5, {"waypoint", "audio_info"});

  auto has_pause = [](const mapoi_interfaces::msg::PointOfInterest & p) {
    for (const auto & tag : p.tags) {
      if (tag == "pause") return true;
    }
    return false;
  };

  EXPECT_TRUE(has_pause(poi_pause));
  EXPECT_FALSE(has_pause(poi_no_pause));
}

// SelectInitialPosePois* tests は #144 で削除。
// `initial_pose` system tag を廃止し、initial pose の選定ロジックは mapoi_server::compute_initial_poi_name
// に移動した。新ロジックの test は mapoi_server 側に置く想定 (本 PR では未追加、follow-up #148 で扱う)。

TEST_F(NavServerTestFixture, HasLandmarkTagDetection)
{
  EXPECT_FALSE(MapoiNavServer::has_landmark_tag(
    make_poi("goal_only", 0.0, 0.0, 0.5, {"waypoint"})));
  EXPECT_FALSE(MapoiNavServer::has_landmark_tag(
    make_poi("no_tags", 0.0, 0.0, 0.5, {})));
  EXPECT_TRUE(MapoiNavServer::has_landmark_tag(
    make_poi("landmark_only", 0.0, 0.0, 0.5, {"landmark"})));
  EXPECT_TRUE(MapoiNavServer::has_landmark_tag(
    make_poi("landmark_combo", 0.0, 0.0, 0.5, {"event", "landmark", "hazard"})));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
