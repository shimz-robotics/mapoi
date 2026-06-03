// config_path_update_policy.hpp の純粋判定 helper の unit test (#169)。
//
// PoiEditorPanel / MapoiPanel の ConfigPathCallback は ROS subscription で受けた
// config_path から map 名を取り出し、この helper の判定結果 (ReinitializeMap /
// RefreshCurrentMap / Noop) を UI 更新関数 (InitConfigs / UpdatePoiTable /
// SetMapComboBox 等) へ写像するだけ。判定そのものは Qt / ROS 非依存の純関数なので、
// 分岐表をここで pin し、リファクタで条件を踏み外す回帰 (`!=`↔`==` 反転、抑制対象の
// 取り違え等) を安く捕える。header は <string> のみ依存。
#include <gtest/gtest.h>

#include "mapoi_rviz_plugins/config_path_update_policy.hpp"

using mapoi_rviz_plugins::detail::ConfigPathUpdateAction;
using mapoi_rviz_plugins::detail::decide_poi_editor_config_path_action;
using mapoi_rviz_plugins::detail::apply_poi_editor_content_update_suppression;
using mapoi_rviz_plugins::detail::decide_mapoi_panel_config_path_action;

// --- decide_poi_editor_config_path_action ---

TEST(DecidePoiEditorConfigPathAction, EmptyConfigPathReinitializesEvenForSameMap)
{
  // 初回受信 (config_path 未設定) は同一 map 名でも full 再初期化が要る。
  EXPECT_EQ(
    decide_poi_editor_config_path_action("mapA", "mapA", ""),
    ConfigPathUpdateAction::ReinitializeMap);
}

TEST(DecidePoiEditorConfigPathAction, DifferentMapReinitializes)
{
  EXPECT_EQ(
    decide_poi_editor_config_path_action("mapA", "mapB", "/maps/mapA/mapoi_config.yaml"),
    ConfigPathUpdateAction::ReinitializeMap);
}

TEST(DecidePoiEditorConfigPathAction, SameMapWithPathRefreshesContent)
{
  // 同一 map の内容変更 (save 後 reload による再 publish) は table 再 fetch のみで足りる。
  EXPECT_EQ(
    decide_poi_editor_config_path_action("mapA", "mapA", "/maps/mapA/mapoi_config.yaml"),
    ConfigPathUpdateAction::RefreshCurrentMap);
}

// --- apply_poi_editor_content_update_suppression ---

TEST(ApplyPoiEditorContentUpdateSuppression, RefreshSuppressedBecomesNoop)
{
  // 自分自身の save 直後 (1.5s の SAVED! feedback 保護) は content refresh を抑制する。
  EXPECT_EQ(
    apply_poi_editor_content_update_suppression(
      ConfigPathUpdateAction::RefreshCurrentMap, true),
    ConfigPathUpdateAction::Noop);
}

TEST(ApplyPoiEditorContentUpdateSuppression, RefreshNotSuppressedPassesThrough)
{
  EXPECT_EQ(
    apply_poi_editor_content_update_suppression(
      ConfigPathUpdateAction::RefreshCurrentMap, false),
    ConfigPathUpdateAction::RefreshCurrentMap);
}

TEST(ApplyPoiEditorContentUpdateSuppression, ReinitializeIsNeverSuppressed)
{
  // 抑制は RefreshCurrentMap だけに効く。map 切替の再初期化は suppress でも維持される。
  EXPECT_EQ(
    apply_poi_editor_content_update_suppression(
      ConfigPathUpdateAction::ReinitializeMap, true),
    ConfigPathUpdateAction::ReinitializeMap);
}

// --- decide_mapoi_panel_config_path_action ---

TEST(DecideMapoiPanelConfigPathAction, DifferentMapReinitializes)
{
  EXPECT_EQ(
    decide_mapoi_panel_config_path_action("mapA", "mapB"),
    ConfigPathUpdateAction::ReinitializeMap);
}

TEST(DecideMapoiPanelConfigPathAction, SameMapRefreshes)
{
  // MapoiPanel には Noop 抑制が無く、同一 map でも goal/route combo は再 fetch する。
  EXPECT_EQ(
    decide_mapoi_panel_config_path_action("mapA", "mapA"),
    ConfigPathUpdateAction::RefreshCurrentMap);
}
