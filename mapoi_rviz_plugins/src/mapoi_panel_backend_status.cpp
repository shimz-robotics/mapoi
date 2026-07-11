// MapoiPanel の backend 接続状態系メソッド定義 (#397 step 2 で mapoi_panel.cpp から別 TU へ分離)。
// BackendStatusCallback / LocalizationBackendStatusCallback / UpdateNavButtonsEnabled を収録。
// バックエンド接続状態バッジ実装 (#400) の直前分割として実施。
#include "mapoi_rviz_plugins/mapoi_panel.hpp"
#include "ui_mapoi_panel.h"

namespace mapoi_rviz_plugins
{

void MapoiPanel::BackendStatusCallback(
  mapoi_interfaces::msg::NavigationBackendStatus::SharedPtr msg)
{
  // RViz panel 側でも backend_ready を見て操作ボタンを gate する (#198, #205 minimal)。
  // backend_status 不在 (旧 mapoi_nav_server build (#208 以前 backend_status contract 未実装、#204 で nav2_bridge へ rename) / panel 単独起動) では callback が呼ばれず、
  // navigation 軸は初期値 (true = enable) を使い続ける。
  nav_backend_status_received_ = true;
  last_navigation_backend_ready_ = msg->backend_ready;
  QMetaObject::invokeMethod(this, [this]() {
    UpdateNavButtonsEnabled();
  }, Qt::QueuedConnection);
}

void MapoiPanel::LocalizationBackendStatusCallback(
  mapoi_interfaces::msg::LocalizationBackendStatus::SharedPtr msg)
{
  // localization bridge の readiness で LocalizationButton を gate する (#209)。
  // 受信前は last_localization_backend_ready_ が初期値 true のままで、互換的に enable のまま。
  localization_backend_status_received_ = true;
  last_localization_backend_ready_ = msg->backend_ready;
  QMetaObject::invokeMethod(this, [this]() {
    UpdateNavButtonsEnabled();
  }, Qt::QueuedConnection);
}

void MapoiPanel::UpdateNavButtonsEnabled()
{
  // Qt main thread で呼ぶこと (BackendStatusCallback / LocalizationBackendStatusCallback /
  // liveliness event_callback から QueuedConnection 経由で invoke される)。
  // 2 軸の minimal 仕様 (#209): navigation 操作 UI と MapComboBox は navigation backend、
  // LocalizationButton は localization backend で gate する。MapComboBox は Nav2 LoadMap +
  // AMCL initial pose の両方を必要とするが、Nav2 LoadMap が走らないと localization 側に意味のある
  // initial pose を送れないので最低限 navigation_ready を要求する。両方を AND する厳格化は
  // future work で UX 検討する。
  // Staleness gating (#208): 受信前 (`*_received_` false) は alive 判定をバイパスして
  // `last_*_backend_ready_` の初期値 (true) を使う = 旧 mapoi_nav_server build (#204 で nav2_bridge へ rename) / panel 単独起動の後方互換。
  // 受信後は MANUAL_BY_TOPIC liveliness で得た `*_alive_` flag と AND し、bridge 死亡時に
  // 自動 disable する。
  const bool nav_ready = nav_backend_status_received_
    ? (last_navigation_backend_ready_ && nav_backend_alive_)
    : last_navigation_backend_ready_;
  const bool loc_ready = localization_backend_status_received_
    ? (last_localization_backend_ready_ && localization_backend_alive_)
    : last_localization_backend_ready_;
  ui_->LocalizationButton->setEnabled(loc_ready);
  ui_->RunGoalButton->setEnabled(nav_ready);
  ui_->RunRouteButton->setEnabled(nav_ready);
  ui_->PauseButton->setEnabled(nav_ready);
  ui_->ResumeButton->setEnabled(nav_ready);
  ui_->StopButton->setEnabled(nav_ready);
  ui_->MapComboBox->setEnabled(nav_ready);
}

}  // namespace mapoi_rviz_plugins
