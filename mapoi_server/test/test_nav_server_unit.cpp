#define UNIT_TEST
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
    double radius, const std::vector<std::string> & tags)
  {
    mapoi_interfaces::msg::PointOfInterest poi;
    poi.name = name;
    poi.pose.position.x = x;
    poi.pose.position.y = y;
    poi.radius = radius;
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
    node_->pois_list_.push_back(make_poi("goal_only", 1.0, 0.0, 0.5, {"goal"}));
    node_->pois_list_.push_back(make_poi("with_pause", 0.0, 2.0, 0.5, {"goal", "pause"}));
    node_->pois_list_.push_back(make_poi("with_custom", 3.0, 0.0, 0.5, {"goal", "audio_info"}));
    node_->pois_list_.push_back(make_poi("origin_only", 0.0, 0.0, 0.5, {"origin"}));
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
  auto poi_pause = make_poi("pause_poi", 0.0, 0.0, 0.5, {"goal", "pause"});
  auto poi_no_pause = make_poi("normal_poi", 0.0, 0.0, 0.5, {"goal", "audio_info"});

  auto has_pause = [](const mapoi_interfaces::msg::PointOfInterest & p) {
    for (const auto & tag : p.tags) {
      if (tag == "pause") return true;
    }
    return false;
  };

  EXPECT_TRUE(has_pause(poi_pause));
  EXPECT_FALSE(has_pause(poi_no_pause));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
