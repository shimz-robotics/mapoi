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
  const std::string & current_config_path)
{
  if (current_config_path.empty() || current_map != new_map) {
    return ConfigPathUpdateAction::ReinitializeMap;
  }
  return ConfigPathUpdateAction::RefreshCurrentMap;
}

inline ConfigPathUpdateAction apply_poi_editor_content_update_suppression(
  ConfigPathUpdateAction action,
  bool suppress_content_update)
{
  if (action == ConfigPathUpdateAction::RefreshCurrentMap && suppress_content_update) {
    return ConfigPathUpdateAction::Noop;
  }
  return action;
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
