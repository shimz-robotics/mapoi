#include <gtest/gtest.h>

#include "mapoi_rviz_plugins/config_path_update_policy.hpp"

using mapoi_rviz_plugins::detail::ConfigPathUpdateAction;
using mapoi_rviz_plugins::detail::apply_poi_editor_content_update_suppression;
using mapoi_rviz_plugins::detail::decide_mapoi_panel_config_path_action;
using mapoi_rviz_plugins::detail::decide_poi_editor_config_path_action;

// --- PoiEditorPanel::ConfigPathCallback policy (#169) ---

TEST(PoiEditorConfigPathAction, FirstConfigReinitializesEvenWhenMapNameMatches)
{
  EXPECT_EQ(
    decide_poi_editor_config_path_action("map_a", "map_a", ""),
    ConfigPathUpdateAction::ReinitializeMap);
}

TEST(PoiEditorConfigPathAction, MapSwitchReinitializesConfigs)
{
  EXPECT_EQ(
    decide_poi_editor_config_path_action("map_a", "map_b", "/maps/map_a/mapoi_config.yaml"),
    ConfigPathUpdateAction::ReinitializeMap);
}

TEST(PoiEditorConfigPathAction, SameMapContentChangeRefreshesTableOnly)
{
  EXPECT_EQ(
    decide_poi_editor_config_path_action("map_a", "map_a", "/maps/map_a/mapoi_config.yaml"),
    ConfigPathUpdateAction::RefreshCurrentMap);
}

TEST(PoiEditorConfigPathAction, SameMapSaveFeedbackSuppressionSkipsRefresh)
{
  EXPECT_EQ(
    apply_poi_editor_content_update_suppression(
      ConfigPathUpdateAction::RefreshCurrentMap, true),
    ConfigPathUpdateAction::Noop);
}

TEST(PoiEditorConfigPathAction, UnsuppressedRefreshStaysRefresh)
{
  EXPECT_EQ(
    apply_poi_editor_content_update_suppression(
      ConfigPathUpdateAction::RefreshCurrentMap, false),
    ConfigPathUpdateAction::RefreshCurrentMap);
}

TEST(PoiEditorConfigPathAction, SuppressionDoesNotHideMapSwitch)
{
  EXPECT_EQ(
    apply_poi_editor_content_update_suppression(
      ConfigPathUpdateAction::ReinitializeMap, true),
    ConfigPathUpdateAction::ReinitializeMap);
}

// --- MapoiPanel::ConfigPathCallback policy (#169) ---

TEST(MapoiPanelConfigPathAction, MapSwitchReinitializesComboBoxesAndHighlight)
{
  EXPECT_EQ(
    decide_mapoi_panel_config_path_action("map_a", "map_b"),
    ConfigPathUpdateAction::ReinitializeMap);
}

TEST(MapoiPanelConfigPathAction, SameMapContentChangeRefreshesGoalAndRouteOnly)
{
  EXPECT_EQ(
    decide_mapoi_panel_config_path_action("map_a", "map_a"),
    ConfigPathUpdateAction::RefreshCurrentMap);
}

TEST(MapoiPanelConfigPathAction, EmptyCurrentMapTreatsFirstCallbackAsMapSwitch)
{
  EXPECT_EQ(
    decide_mapoi_panel_config_path_action("", "map_a"),
    ConfigPathUpdateAction::ReinitializeMap);
}
