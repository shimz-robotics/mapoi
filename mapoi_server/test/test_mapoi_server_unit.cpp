// MapoiServer の純関数 (compute_initial_poi_name) の unit test (#149 round 4 high 対応)。
// rclcpp::Node の生成は不要 (= static 関数を直接叩ける)。
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "mapoi_server/mapoi_server.hpp"

namespace
{

YAML::Node load_pois(const std::string & yaml_text)
{
  return YAML::Load(yaml_text)["poi"];
}

}  // namespace

TEST(ComputeInitialPoiName, EmptyOrNonSequenceReturnsEmpty)
{
  YAML::Node empty;
  EXPECT_EQ(MapoiServer::compute_initial_poi_name(empty, ""), "");
  YAML::Node scalar = YAML::Load("foo");
  EXPECT_EQ(MapoiServer::compute_initial_poi_name(scalar, ""), "");
}

TEST(ComputeInitialPoiName, DefaultPicksFirstNonLandmarkValidPose)
{
  auto pois = load_pois(R"(
poi:
  - {name: a, pose: {x: 1.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
  - {name: b, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
)");
  EXPECT_EQ(MapoiServer::compute_initial_poi_name(pois, ""), "a");
}

TEST(ComputeInitialPoiName, LandmarkAtFrontIsSkipped)
{
  auto pois = load_pois(R"(
poi:
  - {name: lm, pose: {x: 1.0, y: 0.0, yaw: 0.0}, tags: [landmark]}
  - {name: a, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
)");
  EXPECT_EQ(MapoiServer::compute_initial_poi_name(pois, ""), "a");
}

TEST(ComputeInitialPoiName, MissingPoseIsSkipped)
{
  auto pois = load_pois(R"(
poi:
  - {name: no_pose, tags: [waypoint]}
  - {name: a, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
)");
  EXPECT_EQ(MapoiServer::compute_initial_poi_name(pois, ""), "a");
}

TEST(ComputeInitialPoiName, PartialPoseIsSkipped)
{
  auto pois = load_pois(R"(
poi:
  - {name: only_x, pose: {x: 1.0}, tags: [waypoint]}
  - {name: a, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
)");
  EXPECT_EQ(MapoiServer::compute_initial_poi_name(pois, ""), "a");
}

TEST(ComputeInitialPoiName, NonNumericPoseIsSkipped)
{
  auto pois = load_pois(R"(
poi:
  - {name: bad, pose: {x: not_a_number, y: 0.0, yaw: 0.0}, tags: [waypoint]}
  - {name: a, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
)");
  EXPECT_EQ(MapoiServer::compute_initial_poi_name(pois, ""), "a");
}

TEST(ComputeInitialPoiName, RequestedNameIsAdopted)
{
  auto pois = load_pois(R"(
poi:
  - {name: a, pose: {x: 1.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
  - {name: b, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
  - {name: c, pose: {x: 3.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
)");
  EXPECT_EQ(MapoiServer::compute_initial_poi_name(pois, "b"), "b");
  EXPECT_EQ(MapoiServer::compute_initial_poi_name(pois, "c"), "c");
}

TEST(ComputeInitialPoiName, RequestedLandmarkFallsBackToFirst)
{
  auto pois = load_pois(R"(
poi:
  - {name: a, pose: {x: 1.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
  - {name: lm, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [landmark]}
)");
  EXPECT_EQ(MapoiServer::compute_initial_poi_name(pois, "lm"), "a");
}

TEST(ComputeInitialPoiName, RequestedInvalidPoseFallsBackToFirst)
{
  auto pois = load_pois(R"(
poi:
  - {name: a, pose: {x: 1.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
  - {name: bad, pose: {x: 2.0}, tags: [waypoint]}
)");
  EXPECT_EQ(MapoiServer::compute_initial_poi_name(pois, "bad"), "a");
}

TEST(ComputeInitialPoiName, RequestedNotFoundFallsBackToFirst)
{
  auto pois = load_pois(R"(
poi:
  - {name: a, pose: {x: 1.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
  - {name: b, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
)");
  EXPECT_EQ(MapoiServer::compute_initial_poi_name(pois, "missing"), "a");
}

TEST(ComputeInitialPoiName, AllLandmarkOrInvalidReturnsEmpty)
{
  auto pois = load_pois(R"(
poi:
  - {name: lm1, pose: {x: 1.0, y: 0.0, yaw: 0.0}, tags: [landmark]}
  - {name: bad, tags: [waypoint]}
  - {name: lm2, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [landmark]}
)");
  EXPECT_EQ(MapoiServer::compute_initial_poi_name(pois, ""), "");
}
