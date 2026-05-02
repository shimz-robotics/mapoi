// Pure decision helpers for mapoi/config_path callbacks (#169).
// Qt / ROS 依存を持たせず、Panel callback はこの判定結果を UI 更新関数へ写像するだけにする。
#pragma once

#include <string>

namespace mapoi_rviz_plugins::detail
{

enum class ConfigPathUpdateAction
{
  ReinitializeMap,
  RefreshCurrentMap,
  Noop,
};

inline ConfigPathUpdateAction decide_poi_editor_config_path_action(
  const std::string & current_map,
  const std::string & new_map,
  const std::string & current_config_path,
  bool suppress_content_update)
{
  if (current_config_path.empty() || current_map != new_map) {
    return ConfigPathUpdateAction::ReinitializeMap;
  }
  if (suppress_content_update) {
    return ConfigPathUpdateAction::Noop;
  }
  return ConfigPathUpdateAction::RefreshCurrentMap;
}

inline ConfigPathUpdateAction decide_mapoi_panel_config_path_action(
  const std::string & current_map,
  const std::string & new_map)
{
  return current_map != new_map
    ? ConfigPathUpdateAction::ReinitializeMap
    : ConfigPathUpdateAction::RefreshCurrentMap;
}

}  // namespace mapoi_rviz_plugins::detail
