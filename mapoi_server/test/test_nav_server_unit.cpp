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

TEST_F(NavServerTestFixture, SelectInitialPosePoisEmpty)
{
  std::vector<mapoi_interfaces::msg::PointOfInterest> pois;
  pois.push_back(make_poi("goal_only", 0.0, 0.0, 0.5, {"goal"}));
  pois.push_back(make_poi("origin_only", 1.0, 0.0, 0.5, {"origin"}));
  auto matched = node_->select_initial_pose_pois(pois);
  EXPECT_EQ(matched.size(), 0u);
}

TEST_F(NavServerTestFixture, SelectInitialPosePoisSingle)
{
  std::vector<mapoi_interfaces::msg::PointOfInterest> pois;
  pois.push_back(make_poi("goal_only", 0.0, 0.0, 0.5, {"goal"}));
  pois.push_back(make_poi("entry", -2.0, -0.5, 0.5, {"goal", "initial_pose"}));
  pois.push_back(make_poi("origin_only", 1.0, 0.0, 0.5, {"origin"}));
  auto matched = node_->select_initial_pose_pois(pois);
  ASSERT_EQ(matched.size(), 1u);
  EXPECT_EQ(matched[0].name, "entry");
}

TEST_F(NavServerTestFixture, SelectInitialPosePoisMultiple)
{
  std::vector<mapoi_interfaces::msg::PointOfInterest> pois;
  pois.push_back(make_poi("first", 1.0, 1.0, 0.5, {"initial_pose"}));
  pois.push_back(make_poi("middle", 2.0, 2.0, 0.5, {"goal"}));
  pois.push_back(make_poi("second", 3.0, 3.0, 0.5, {"initial_pose", "goal"}));
  auto matched = node_->select_initial_pose_pois(pois);
  ASSERT_EQ(matched.size(), 2u);
  EXPECT_EQ(matched[0].name, "first");  // 順序保持・先頭採用
  EXPECT_EQ(matched[1].name, "second");
}

TEST_F(NavServerTestFixture, SelectInitialPosePoisExcludesLandmark)
{
  // landmark + initial_pose は排他なので auto-publish 候補から除外される (#85)。
  std::vector<mapoi_interfaces::msg::PointOfInterest> pois;
  pois.push_back(make_poi("ip_only", 1.0, 1.0, 0.5, {"initial_pose"}));
  pois.push_back(make_poi("landmark_ip", 2.0, 2.0, 0.5, {"initial_pose", "landmark"}));
  pois.push_back(make_poi("ip_with_other", 3.0, 3.0, 0.5, {"initial_pose", "goal"}));
  auto matched = node_->select_initial_pose_pois(pois);
  ASSERT_EQ(matched.size(), 2u);
  EXPECT_EQ(matched[0].name, "ip_only");
  EXPECT_EQ(matched[1].name, "ip_with_other");
}

TEST_F(NavServerTestFixture, HasLandmarkTagDetection)
{
  EXPECT_FALSE(MapoiNavServer::has_landmark_tag(
    make_poi("goal_only", 0.0, 0.0, 0.5, {"goal"})));
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
