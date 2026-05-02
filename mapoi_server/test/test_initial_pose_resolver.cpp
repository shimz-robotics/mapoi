// initial pose 選定純関数 (mapoi::select_initial_poi_name) の unit test
// (#149 round 4 high で導入、#150 で共通ヘッダに移行)。
// rclcpp::Node の生成は不要 (= 純関数を直接叩ける)。
//
// system tag 契約 (mapoi::kSystemTags) の固定検証も同居 (#191)。
#include <cstring>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "mapoi_server/initial_pose_resolver.hpp"
#include "mapoi_server/system_tags.hpp"

using mapoi::select_initial_poi_name;

namespace
{

YAML::Node load_pois(const std::string & yaml_text)
{
  return YAML::Load(yaml_text)["poi"];
}

}  // namespace

TEST(SelectInitialPoiName, EmptyOrNonSequenceReturnsEmpty)
{
  YAML::Node empty;
  EXPECT_EQ(select_initial_poi_name(empty, ""), "");
  YAML::Node scalar = YAML::Load("foo");
  EXPECT_EQ(select_initial_poi_name(scalar, ""), "");
}

TEST(SelectInitialPoiName, DefaultPicksFirstNonLandmarkValidPose)
{
  auto pois = load_pois(R"(
poi:
  - {name: a, pose: {x: 1.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
  - {name: b, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
)");
  EXPECT_EQ(select_initial_poi_name(pois, ""), "a");
}

TEST(SelectInitialPoiName, LandmarkAtFrontIsSkipped)
{
  auto pois = load_pois(R"(
poi:
  - {name: lm, pose: {x: 1.0, y: 0.0, yaw: 0.0}, tags: [landmark]}
  - {name: a, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
)");
  EXPECT_EQ(select_initial_poi_name(pois, ""), "a");
}

TEST(SelectInitialPoiName, MissingPoseIsSkipped)
{
  auto pois = load_pois(R"(
poi:
  - {name: no_pose, tags: [waypoint]}
  - {name: a, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
)");
  EXPECT_EQ(select_initial_poi_name(pois, ""), "a");
}

TEST(SelectInitialPoiName, PartialPoseIsSkipped)
{
  auto pois = load_pois(R"(
poi:
  - {name: only_x, pose: {x: 1.0}, tags: [waypoint]}
  - {name: a, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
)");
  EXPECT_EQ(select_initial_poi_name(pois, ""), "a");
}

TEST(SelectInitialPoiName, NonNumericPoseIsSkipped)
{
  auto pois = load_pois(R"(
poi:
  - {name: bad, pose: {x: not_a_number, y: 0.0, yaw: 0.0}, tags: [waypoint]}
  - {name: a, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
)");
  EXPECT_EQ(select_initial_poi_name(pois, ""), "a");
}

TEST(SelectInitialPoiName, RequestedNameIsAdopted)
{
  auto pois = load_pois(R"(
poi:
  - {name: a, pose: {x: 1.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
  - {name: b, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
  - {name: c, pose: {x: 3.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
)");
  EXPECT_EQ(select_initial_poi_name(pois, "b"), "b");
  EXPECT_EQ(select_initial_poi_name(pois, "c"), "c");
}

TEST(SelectInitialPoiName, RequestedLandmarkFallsBackToFirst)
{
  auto pois = load_pois(R"(
poi:
  - {name: a, pose: {x: 1.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
  - {name: lm, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [landmark]}
)");
  EXPECT_EQ(select_initial_poi_name(pois, "lm"), "a");
}

TEST(SelectInitialPoiName, RequestedInvalidPoseFallsBackToFirst)
{
  auto pois = load_pois(R"(
poi:
  - {name: a, pose: {x: 1.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
  - {name: bad, pose: {x: 2.0}, tags: [waypoint]}
)");
  EXPECT_EQ(select_initial_poi_name(pois, "bad"), "a");
}

TEST(SelectInitialPoiName, RequestedNotFoundFallsBackToFirst)
{
  auto pois = load_pois(R"(
poi:
  - {name: a, pose: {x: 1.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
  - {name: b, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [waypoint]}
)");
  EXPECT_EQ(select_initial_poi_name(pois, "missing"), "a");
}

TEST(SelectInitialPoiName, AllLandmarkOrInvalidReturnsEmpty)
{
  auto pois = load_pois(R"(
poi:
  - {name: lm1, pose: {x: 1.0, y: 0.0, yaw: 0.0}, tags: [landmark]}
  - {name: bad, tags: [waypoint]}
  - {name: lm2, pose: {x: 2.0, y: 0.0, yaw: 0.0}, tags: [landmark]}
)");
  EXPECT_EQ(select_initial_poi_name(pois, ""), "");
}

// system tag 契約は mapoi_webui / mapoi_rviz_plugins / 既存 mapoi_config.yaml が
// `waypoint` / `landmark` / `pause` の存在前提で書かれているため、kSystemTags の
// 件数 / 順序 / name / description (空でないこと) を固定検証する (#191 medium #2)。
// ここで失敗するなら get_tag_definitions service の応答契約が変わっており、
// 受信側 (webui の編集 UI、rviz plugin、custom_tags 排他検査) が破綻する可能性がある。
TEST(SystemTagsContract, ExpectedNamesAndOrder)
{
  using mapoi::kSystemTags;
  ASSERT_EQ(kSystemTags.size(), 3u);
  EXPECT_STREQ(kSystemTags[0].name, "waypoint");
  EXPECT_STREQ(kSystemTags[1].name, "landmark");
  EXPECT_STREQ(kSystemTags[2].name, "pause");
}

TEST(SystemTagsContract, DescriptionsAreNonEmpty)
{
  for (const auto & tag : mapoi::kSystemTags) {
    ASSERT_NE(tag.name, nullptr);
    ASSERT_NE(tag.description, nullptr);
    EXPECT_GT(std::strlen(tag.description), 0u)
      << "system tag '" << tag.name << "' description is empty";
  }
}
