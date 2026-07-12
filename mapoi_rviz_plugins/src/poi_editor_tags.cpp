// PoiEditorPanel::TagFilterChanged / PopulateTagFilter / LoadTagDefinitions /
// TagHelperSelected の定義 (#397 step 7 で poi_editor.cpp から別 TU へ分離)。
// POI 名検索フィルタ (#405) の追加直前に分割し、タグ操作 UI 系を独立 TU に集約する。
// クラス構造・宣言 (poi_editor.hpp) は変更なし。
#include "mapoi_rviz_plugins/poi_editor.hpp"
#include "mapoi_rviz_plugins/poi_editor_helpers.hpp"

// generated UI header: PoiEditorPanel::ui_ (`Ui::PoiEditorUi*`) は poi_editor.hpp では
// 前方宣言のみなので、`ui_->TagFilterComboBox` 等の完全型アクセスにはこの include が要る
// (poi_editor_save.cpp / poi_editor_validation.cpp と同じ)。
#include "ui_poi_editor.h"

// std::chrono_literals (3s) は LoadTagDefinitions の wait_for_service で使用。
using namespace std::chrono_literals;

namespace mapoi_rviz_plugins
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mapoi_rviz_plugins.poi_editor");

// PoiTable column index 定数は poi_editor_helpers.hpp に集約 (#158 で導入)。
using detail::kColName;
using detail::kColPose;
using detail::kColTolerance;
using detail::kColTags;
using detail::kColDescription;

// Tag Filter
void PoiEditorPanel::TagFilterChanged(int index)
{
  if (index <= 0) {
    UpdatePoiTable();
    return;
  }

  std::string selected_tag = ui_->TagFilterComboBox->currentText().toStdString();

  is_table_color_ = false;
  ui_->PoiTable->setRowCount(0);

  int row = 0;
  for (const auto& p : all_pois_) {
    bool has_tag = false;
    for (const auto& tag : p.tags) {
      if (tag == selected_tag) {
        has_tag = true;
        break;
      }
    }
    if (has_tag) {
      ui_->PoiTable->insertRow(row);
      // 表示精度: pose / tolerance とも小数点以下 2 桁固定。長すぎる尾桁を避ける (#159 round 5 user)。
      ui_->PoiTable->setItem(row, kColName, new QTableWidgetItem(QString::fromStdString(p.name)));
      ui_->PoiTable->setItem(row, kColPose, new QTableWidgetItem(
        tr("%1, %2, %3")
          .arg(QString::number(p.pose.position.x, 'f', 2))
          .arg(QString::number(p.pose.position.y, 'f', 2))
          .arg(QString::number(this->calcYaw(p.pose), 'f', 2))));
      ui_->PoiTable->setItem(row, kColTolerance, new QTableWidgetItem(
        tr("%1, %2")
          .arg(QString::number(p.tolerance.xy, 'f', 2))
          .arg(QString::number(p.tolerance.yaw, 'f', 2))));
      ui_->PoiTable->setItem(row, kColTags, new QTableWidgetItem(QString::fromStdString(this->join(p.tags, ", "))));
      ui_->PoiTable->setItem(row, kColDescription, new QTableWidgetItem(QString::fromStdString(p.description)));
      row++;
    }
  }
  is_table_color_ = true;
  UpdatePoiCount();
}

void PoiEditorPanel::PopulateTagFilter()
{
  std::set<std::string> unique_tags;
  for (const auto& p : all_pois_) {
    for (const auto& tag : p.tags) {
      if (!tag.empty()) {
        unique_tags.insert(tag);
      }
    }
  }

  ui_->TagFilterComboBox->clear();
  ui_->TagFilterComboBox->addItem("All");
  for (const auto& tag : unique_tags) {
    ui_->TagFilterComboBox->addItem(QString::fromStdString(tag));
  }
}

void PoiEditorPanel::LoadTagDefinitions()
{
  if (!get_tag_defs_client_->wait_for_service(3s)) {
    RCLCPP_WARN(LOGGER, "mapoi/get_tag_definitions service not available, TagHelper will be empty.");
    return;
  }

  auto request = std::make_shared<mapoi_interfaces::srv::GetTagDefinitions::Request>();
  auto result = get_tag_defs_client_->async_send_request(request);
  rclcpp::spin_until_future_complete(service_node_, result);
  auto response = result.get();

  known_tag_names_.clear();
  known_tag_is_system_.clear();
  for (const auto & def : response->definitions) {
    known_tag_names_.push_back(def.name);
    known_tag_is_system_.push_back(def.is_system);
  }

  // Build TagHelperComboBox
  ui_->TagHelperComboBox->clear();
  ui_->TagHelperComboBox->addItem("+ Add tag...");
  for (size_t i = 0; i < known_tag_names_.size(); i++) {
    QString label;
    if (known_tag_is_system_[i]) {
      label = QString("[S] %1").arg(QString::fromStdString(known_tag_names_[i]));
    } else {
      label = QString::fromStdString(known_tag_names_[i]);
    }
    ui_->TagHelperComboBox->addItem(label);
  }
}

void PoiEditorPanel::TagHelperSelected(int index)
{
  if (index <= 0) return;

  // Get tag name (strip "[S] " prefix if present)
  QString selected = ui_->TagHelperComboBox->itemText(index);
  if (selected.startsWith("[S] ")) {
    selected = selected.mid(4);
  }
  std::string tag_name = selected.toStdString();

  // Get current row's tags cell (column 5; tags は #138 で column 4→5 にシフト)
  int current_row = ui_->PoiTable->currentRow();
  if (current_row < 0) {
    ui_->TagHelperComboBox->setCurrentIndex(0);
    return;
  }

  auto* tags_item = ui_->PoiTable->item(current_row, kColTags);
  std::string current_tags = tags_item ? tags_item->text().toStdString() : "";

  // Check if tag already exists
  auto tag_list = this->SplitSentence(current_tags, ", ");
  for (const auto& t : tag_list) {
    if (t == tag_name) {
      ui_->TagHelperComboBox->setCurrentIndex(0);
      return;
    }
  }

  // Append tag (column 5 = tags、column 4 (tolerance.yaw) を上書きしないよう注意 — Codex review #139 high)
  std::string new_tags;
  if (current_tags.empty()) {
    new_tags = tag_name;
  } else {
    new_tags = current_tags + ", " + tag_name;
  }
  ui_->PoiTable->setItem(current_row, kColTags, new QTableWidgetItem(QString::fromStdString(new_tags)));

  // Reset combo to placeholder
  ui_->TagHelperComboBox->setCurrentIndex(0);
}

}  // namespace mapoi_rviz_plugins
