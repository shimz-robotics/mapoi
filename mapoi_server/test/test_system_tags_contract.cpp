#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <set>
#include <string>
#include <vector>

#ifndef MAPOI_SERVER_SOURCE_DIR
#error "MAPOI_SERVER_SOURCE_DIR must be defined by CMake"
#endif

namespace
{

YAML::Node load_tag_definitions()
{
  const auto path =
    std::filesystem::path(MAPOI_SERVER_SOURCE_DIR) / "maps" / "tag_definitions.yaml";
  return YAML::LoadFile(path.string());
}

std::vector<std::string> load_system_tag_names()
{
  const auto root = load_tag_definitions();
  std::vector<std::string> names;
  for (const auto & tag : root["tags"]) {
    names.push_back(tag["name"].as<std::string>());
  }
  return names;
}

}  // namespace

TEST(SystemTagsContract, DefinesExpectedSystemTagsInStableOrder)
{
  const std::vector<std::string> expected = {"waypoint", "landmark", "pause"};
  EXPECT_EQ(load_system_tag_names(), expected);
}

TEST(SystemTagsContract, NamesAreUniqueAndNonEmpty)
{
  const auto names = load_system_tag_names();
  std::set<std::string> unique_names(names.begin(), names.end());

  EXPECT_EQ(unique_names.size(), names.size());
  for (const auto & name : names) {
    EXPECT_FALSE(name.empty());
  }
}

TEST(SystemTagsContract, DescriptionsArePresent)
{
  const auto root = load_tag_definitions();
  ASSERT_TRUE(root["tags"]);
  ASSERT_TRUE(root["tags"].IsSequence());

  for (const auto & tag : root["tags"]) {
    ASSERT_TRUE(tag["name"]);
    EXPECT_TRUE(tag["description"]);
    EXPECT_FALSE(tag["description"].as<std::string>("").empty())
      << tag["name"].as<std::string>();
  }
}
