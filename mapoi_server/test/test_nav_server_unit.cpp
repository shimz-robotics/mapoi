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

// --- build_route_poi_names (#143 / #148) ---

TEST_F(NavServerTestFixture, BuildRoutePoiNamesWaypointsOnly)
{
  auto wp1 = make_poi("wp1", 0.0, 0.0, 0.5, {"waypoint"});
  auto wp2 = make_poi("wp2", 1.0, 0.0, 0.5, {"waypoint", "pause"});
  auto result = MapoiNavServer::build_route_poi_names({wp1, wp2}, {});
  EXPECT_EQ(result.size(), 2u);
  EXPECT_EQ(result.count("wp1"), 1u);
  EXPECT_EQ(result.count("wp2"), 1u);
}

TEST_F(NavServerTestFixture, BuildRoutePoiNamesWaypointsAndLandmarks)
{
  // route 受信時、waypoints と landmarks の両方が active set に入る (#143)。
  auto wp = make_poi("wp1", 0.0, 0.0, 0.5, {"waypoint"});
  auto lm = make_poi("lm1", 1.0, 0.0, 0.5, {"landmark"});
  auto result = MapoiNavServer::build_route_poi_names({wp}, {lm});
  EXPECT_EQ(result.size(), 2u);
  EXPECT_EQ(result.count("wp1"), 1u);
  EXPECT_EQ(result.count("lm1"), 1u);
}

TEST_F(NavServerTestFixture, BuildRoutePoiNamesLandmarksEmptyBackwardCompat)
{
  // 旧 yaml (route.landmarks 未指定) は landmarks empty で読まれる。waypoints のみで
  // active set が成立し、空にならない (#143 後方互換契約)。
  auto wp = make_poi("wp1", 0.0, 0.0, 0.5, {"waypoint"});
  auto result = MapoiNavServer::build_route_poi_names({wp}, {});
  EXPECT_EQ(result.size(), 1u);
  EXPECT_EQ(result.count("wp1"), 1u);
}

TEST_F(NavServerTestFixture, BuildRoutePoiNamesEmpty)
{
  auto result = MapoiNavServer::build_route_poi_names({}, {});
  EXPECT_TRUE(result.empty());
}

TEST_F(NavServerTestFixture, BuildRoutePoiNamesDuplicateNames)
{
  // 同名が waypoint と landmark の両方に出てきても set 性質で 1 つに集約。
  auto wp = make_poi("dup", 0.0, 0.0, 0.5, {"waypoint"});
  auto lm = make_poi("dup", 1.0, 0.0, 0.5, {"landmark"});
  auto result = MapoiNavServer::build_route_poi_names({wp}, {lm});
  EXPECT_EQ(result.size(), 1u);
  EXPECT_EQ(result.count("dup"), 1u);
}

// --- is_pause_eligible (#143 / #148) ---

TEST_F(NavServerTestFixture, IsPauseEligibleRouteModeActiveWithPauseTag)
{
  auto poi = make_poi("p1", 0.0, 0.0, 0.5, {"waypoint", "pause"});
  std::unordered_set<std::string> active = {"p1"};
  EXPECT_TRUE(MapoiNavServer::is_pause_eligible(
    poi, MapoiNavServer::NavMode::ROUTE, active));
}

TEST_F(NavServerTestFixture, IsPauseEligibleRouteModeActiveWithoutPauseTag)
{
  auto poi = make_poi("p1", 0.0, 0.0, 0.5, {"waypoint"});
  std::unordered_set<std::string> active = {"p1"};
  EXPECT_FALSE(MapoiNavServer::is_pause_eligible(
    poi, MapoiNavServer::NavMode::ROUTE, active));
}

TEST_F(NavServerTestFixture, IsPauseEligibleRouteModeNonActivePoi)
{
  // pause タグはあるが active route POI ではない (= 別 route の POI に偶然 ENTER)。
  // 厳格化前は発火していたが #143 で発火しないようになった。
  auto poi = make_poi("p1", 0.0, 0.0, 0.5, {"waypoint", "pause"});
  std::unordered_set<std::string> active = {"p2"};
  EXPECT_FALSE(MapoiNavServer::is_pause_eligible(
    poi, MapoiNavServer::NavMode::ROUTE, active));
}

TEST_F(NavServerTestFixture, IsPauseEligibleGoalMode)
{
  // GOAL 走行中は pause タグ + active set 一致でも発火しない (#143)。
  auto poi = make_poi("p1", 0.0, 0.0, 0.5, {"waypoint", "pause"});
  std::unordered_set<std::string> active = {"p1"};
  EXPECT_FALSE(MapoiNavServer::is_pause_eligible(
    poi, MapoiNavServer::NavMode::GOAL, active));
}

TEST_F(NavServerTestFixture, IsPauseEligibleIdleMode)
{
  auto poi = make_poi("p1", 0.0, 0.0, 0.5, {"waypoint", "pause"});
  std::unordered_set<std::string> active = {"p1"};
  EXPECT_FALSE(MapoiNavServer::is_pause_eligible(
    poi, MapoiNavServer::NavMode::IDLE, active));
}

// --- reset_nav_state (#143 / #148) ---

TEST_F(NavServerTestFixture, ResetNavStateClearsRouteContext)
{
  // route 走行中 + pause 中に相当する状態を fixture から直接 set し、
  // reset_nav_state() が route lifecycle 終了時 (cancel / SUCCEEDED / ABORTED /
  // GOAL 切替) に行うクリーンアップ動作を再現する。`reset_nav_state()` の
  // 契約に含まれる全 member をまとめて検証することで、将来 reset 対象が漏れた
  // 場合を unit test で検出できる。
  {
    std::lock_guard<std::mutex> lock(node_->data_mutex_);
    node_->current_route_poi_names_.insert("wp1");
    node_->current_route_poi_names_.insert("lm1");
  }
  node_->nav_mode_ = MapoiNavServer::NavMode::ROUTE;
  node_->is_paused_ = true;
  node_->current_waypoint_index_ = 3;

  geometry_msgs::msg::PoseStamped wp;
  wp.pose.position.x = 1.0;
  node_->current_route_waypoints_.push_back(wp);
  node_->paused_waypoints_.push_back(wp);

  geometry_msgs::msg::PoseStamped paused_goal;
  paused_goal.pose.position.x = 9.0;
  node_->paused_goal_pose_ = paused_goal;

  node_->reset_nav_state();

  EXPECT_TRUE(node_->current_route_poi_names_.empty());
  EXPECT_EQ(node_->nav_mode_, MapoiNavServer::NavMode::IDLE);
  EXPECT_FALSE(node_->is_paused_);
  EXPECT_EQ(node_->current_waypoint_index_, 0u);
  EXPECT_TRUE(node_->current_route_waypoints_.empty());
  EXPECT_TRUE(node_->paused_waypoints_.empty());
  // paused_goal_pose_ は default-constructed PoseStamped に戻る (pose.position は 0)。
  EXPECT_DOUBLE_EQ(node_->paused_goal_pose_.pose.position.x, 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
