// MapoiPanel::NavStatusCallback の定義 (#397 step 3 で mapoi_panel.cpp から別 TU へ分離)。
// nav status 文字列のパースと NavStatusLabel への表示更新を担う。
#include "mapoi_rviz_plugins/mapoi_panel.hpp"
#include "ui_mapoi_panel.h"

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
    } else if (status == "aborted") {
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("走行失敗")
                         : QString::fromStdString("走行失敗: " + target));
    } else if (status == "canceled") {
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("走行キャンセル")
                         : QString::fromStdString("走行キャンセル: " + target));
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
    } else if (status == "map_switch_failed") {
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("地図切替失敗")
                         : QString::fromStdString("地図切替失敗: " + target));
    } else if (status == "backend_unavailable") {
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("ナビゲーション利用不可")
                         : QString::fromStdString("ナビゲーション利用不可: " + target));
    } else if (status == "rejected") {
      // #339: 受理前に拒否されたコマンド (存在しない POI 名、landmark POI を goal 指定、
      // 空 route 等)。直前の status が居座って誤操作に気づけない事態を防ぐ。
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("コマンド拒否")
                         : QString::fromStdString("コマンド拒否: " + target));
    }
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
