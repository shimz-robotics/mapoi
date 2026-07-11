// MapoiPanel::NavStatusCallback の定義 (#397 step 3 で mapoi_panel.cpp から別 TU へ分離)。
// nav status 文字列のパースと NavStatusLabel への表示更新を担う。
#include "mapoi_rviz_plugins/mapoi_panel.hpp"
#include "ui_mapoi_panel.h"
#include <algorithm>

namespace mapoi_rviz_plugins
{

void MapoiPanel::NavStatusCallback(std_msgs::msg::String::SharedPtr msg)
{
  // payload は "status" または "status:target" 形式 (#104)。
  // : split で target を抽出し、有れば後起動 panel / 外部ノード発行 nav でも
  // target 復元できるようにする。
  const auto & raw = msg->data;
  const auto colon = raw.find(':');
  const std::string status = (colon == std::string::npos) ? raw : raw.substr(0, colon);
  const std::string target = (colon == std::string::npos) ? std::string{} : raw.substr(colon + 1);
  QMetaObject::invokeMethod(this, [this, status, target]() {
    if (status == "navigating") {
      if (!target.empty() && current_nav_mode_ == "idle") {
        // 後起動 panel / 外部ノード発行 nav: payload の target を復元して表示。
        current_nav_target_ = target;
        ui_->NavStatusLabel->setText(
            QString::fromStdString("走行中: " + target));
      } else if (current_nav_mode_ == "route") {
        ui_->NavStatusLabel->setText(
            QString::fromStdString("ルート走行中: " + current_nav_target_));
      } else if (current_nav_mode_ == "idle") {
        // target も無い (旧 publisher 等) → 汎用表示にフォールバック。
        ui_->NavStatusLabel->setText(QString::fromStdString("走行中"));
      }
      // goal mode は RunGoalButton で即時表示済みのため何もしない
    } else if (status == "succeeded") {
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("到着")
                         : QString::fromStdString("到着: " + target));
      ui_->RouteProgressLabel->setText(QString{});
    } else if (status == "aborted") {
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("走行失敗")
                         : QString::fromStdString("走行失敗: " + target));
      ui_->RouteProgressLabel->setText(QString{});
    } else if (status == "canceled") {
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("走行キャンセル")
                         : QString::fromStdString("走行キャンセル: " + target));
      ui_->RouteProgressLabel->setText(QString{});
    } else if (status == "paused") {
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("一時停止中")
                         : QString::fromStdString("一時停止中: " + target));
    } else if (status == "map_switching") {
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("地図切替中")
                         : QString::fromStdString("地図切替中: " + target));
    } else if (status == "map_switch_succeeded") {
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("地図切替完了")
                         : QString::fromStdString("地図切替完了: " + target));
      ui_->RouteProgressLabel->setText(QString{});
    } else if (status == "map_switch_failed") {
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("地図切替失敗")
                         : QString::fromStdString("地図切替失敗: " + target));
      ui_->RouteProgressLabel->setText(QString{});
    } else if (status == "backend_unavailable") {
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("ナビゲーション利用不可")
                         : QString::fromStdString("ナビゲーション利用不可: " + target));
      ui_->RouteProgressLabel->setText(QString{});
    } else if (status == "rejected") {
      // #339: 受理前に拒否されたコマンド (存在しない POI 名、landmark POI を goal 指定、
      // 空 route 等)。直前の status が居座って誤操作に気づけない事態を防ぐ。
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("コマンド拒否")
                         : QString::fromStdString("コマンド拒否: " + target));
      ui_->RouteProgressLabel->setText(QString{});
    }
  }, Qt::QueuedConnection);
}

// #406: mapoi/events コールバック。ROUTE 走行中にのみ受信される (IDLE/GOAL では発火しない)。
// n/総数の総数は highlighted_route_poi_names_ (panel でルート選択済み時に確定)。
// 外部ノード起点の走行など panel が未選択の場合はリストが空になるため n/総数 を省略し
// POI 名のみ表示するフォールバックとする。未知の event_type は無視する。
void MapoiPanel::PoiEventCallback(mapoi_interfaces::msg::PoiEvent::SharedPtr msg)
{
  const uint8_t event_type = msg->event_type;
  // 未知の event_type は無視 (EVENT_ENTER/EVENT_PAUSED/EVENT_EXIT のみ処理)
  if (event_type != mapoi_interfaces::msg::PoiEvent::EVENT_ENTER &&
      event_type != mapoi_interfaces::msg::PoiEvent::EVENT_PAUSED &&
      event_type != mapoi_interfaces::msg::PoiEvent::EVENT_EXIT) {
    return;
  }
  const std::string poi_name = msg->poi.name;
  QMetaObject::invokeMethod(this, [this, event_type, poi_name]() {
    // n/総数: panel で選択したルートの POI リスト内での先頭一致 index + 1。
    // リストが空 (外部ノード起点) または見つからない場合は n/総数 を省略する。
    // highlighted_route_poi_names_ は UI スレッドが書き換えるメンバのため、参照も
    // queued lambda 内 (UI スレッド) で行う (NavStatusCallback の current_nav_mode_ と同じ規約)。
    const auto & names = highlighted_route_poi_names_;
    std::string progress_suffix;
    if (!names.empty()) {
      const auto it = std::find(names.begin(), names.end(), poi_name);
      if (it != names.end()) {
        const int n = static_cast<int>(std::distance(names.begin(), it)) + 1;
        const int total = static_cast<int>(names.size());
        progress_suffix = " (" + std::to_string(n) + "/" + std::to_string(total) + ")";
      }
    }
    std::string text;
    if (event_type == mapoi_interfaces::msg::PoiEvent::EVENT_ENTER) {
      text = "進入: " + poi_name + progress_suffix;
    } else if (event_type == mapoi_interfaces::msg::PoiEvent::EVENT_PAUSED) {
      text = "POI で一時停止中: " + poi_name + progress_suffix;
    } else {  // EVENT_EXIT
      text = "通過: " + poi_name + progress_suffix;
    }
    ui_->RouteProgressLabel->setText(QString::fromStdString(text));
  }, Qt::QueuedConnection);
}

// #398: mapoi/nav/command_rejected コールバック。payload は target 名のみの文字列
// (README §Publishers command_rejected 節参照)。NavStatusLabel (latched 状態スナップショット) には
// 一切触れず、CommandRejectedLabel に一時表示して 5 秒後にクリアする。
void MapoiPanel::CommandRejectedCallback(std_msgs::msg::String::SharedPtr msg)
{
  const std::string target = msg->data;
  QMetaObject::invokeMethod(this, [this, target]() {
    const QString text = target.empty()
      ? QString::fromStdString("コマンド拒否")
      : QString::fromStdString("コマンド拒否: " + target);
    ui_->CommandRejectedLabel->setText(text);
    reject_clear_timer_->start();  // 連続 reject 時は自動リスタートで 5 秒延長
  }, Qt::QueuedConnection);
}

}  // namespace mapoi_rviz_plugins
