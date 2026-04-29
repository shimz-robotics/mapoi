#include "mapoi_rviz_plugins/poi_editor.hpp"
#include "mapoi_rviz_plugins/poi_editor_helpers.hpp"
#include <class_loader/class_loader.hpp>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>

#include <QFileDialog>
#include <QStandardPaths>
#include <QTimer>
#include <QGroupBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QButtonGroup>
#include <QVBoxLayout>

#include "ui_poi_editor.h"

using namespace std::chrono_literals;

namespace mapoi_rviz_plugins
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mapoi_rviz_plugins.poi_editor");

// pure helpers (try_parse_finite_double / split_and_trim / format_yaw_deg) は
// poi_editor_helpers.hpp に切り出した (#151 round 1 で test 容易化のため)。
using detail::try_parse_finite_double;
using detail::split_and_trim;
using detail::format_yaw_deg;

PoiEditorPanel::PoiEditorPanel(QWidget* parent) : Panel(parent),  ui_(new Ui::PoiEditorUi())
{
  ui_->setupUi(this);
}

PoiEditorPanel::~PoiEditorPanel() = default;

void PoiEditorPanel::onInitialize()
{
  node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Create shared service node and persistent clients
  service_node_ = rclcpp::Node::make_shared("poieditor_service_client");
  switch_map_client_ = service_node_->create_client<mapoi_interfaces::srv::SwitchMap>("switch_map");
  get_pois_info_client_ = service_node_->create_client<mapoi_interfaces::srv::GetPoisInfo>("get_pois_info");
  get_maps_info_client_ = service_node_->create_client<mapoi_interfaces::srv::GetMapsInfo>("get_maps_info");
  get_tag_defs_client_ = service_node_->create_client<mapoi_interfaces::srv::GetTagDefinitions>("get_tag_definitions");
  reload_map_info_client_ = service_node_->create_client<std_srvs::srv::Trigger>("reload_map_info");

  connect(ui_->MapComboBox, SIGNAL(activated(int)), this, SLOT(MapComboBox()));
  connect(ui_->ResetButton, SIGNAL(clicked()), this, SLOT(ResetButton()));
  connect(ui_->PoiTable, SIGNAL(cellChanged(int, int)), this, SLOT(TableChanged(int, int)));
  connect(ui_->PoiTable->verticalHeader(), SIGNAL(sectionMoved(int, int, int)), this, SLOT(RowMoved(int, int, int)));
  connect(ui_->NewButton, SIGNAL(clicked()), this, SLOT(NewButton()));
  connect(ui_->CopyButton, SIGNAL(clicked()), this, SLOT(CopyButton()));
  connect(ui_->DeleteButton, SIGNAL(clicked()), this, SLOT(DeleteButton()));
  connect(ui_->FileComboBox, SIGNAL(activated(int)), this, SLOT(FileComboBox()));
  connect(ui_->SaveButton, SIGNAL(clicked()), this, SLOT(SaveButton()));
  connect(ui_->TagFilterComboBox, SIGNAL(activated(int)), this, SLOT(TagFilterChanged(int)));
  connect(ui_->TagHelperComboBox, SIGNAL(activated(int)), this, SLOT(TagHelperSelected(int)));

  poi_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "mapoi_rviz_pose", 10, std::bind(&PoiEditorPanel::PoiPoseCallback, this, std::placeholders::_1));

  parentWidget()->setVisible(true);

  // MapComboBox
  if (!get_maps_info_client_->wait_for_service(3s)) {
    RCLCPP_ERROR(LOGGER, "get_maps_info service not available after 3s timeout.");
    return;
  }
  auto request = std::make_shared<mapoi_interfaces::srv::GetMapsInfo::Request>();
  auto result = get_maps_info_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(service_node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to call service get_maps_info");
    return;
  }
  auto map_info = result.get();
  current_map_ = map_info->map_name;
  map_name_list_ = map_info->maps_list;
  ui_->MapComboBox->clear();
  for (const auto & map : map_name_list_) {
    ui_->MapComboBox->addItem(QString::fromStdString(map));
  }
  InitConfigs(map_info->map_name);
  LoadTagDefinitions();

  config_path_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "mapoi_config_path", 10,
    std::bind(&PoiEditorPanel::ConfigPathCallback, this, std::placeholders::_1));

  // Display Settings group: mapoi_rviz2_publisher の表示系 parameter を Panel から制御 (#99)
  // - route_display_mode (all / selected / none): RadioButton
  // - poi_label_format (index / name / both / none): RadioButton
  // - arrow_size_ratio (double 0.0〜5.0): DoubleSpinBox
  rviz2_pub_param_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(service_node_, "/mapoi_rviz2_publisher");

  auto * display_group = new QGroupBox("Display Settings", this);
  auto * display_form = new QFormLayout(display_group);
  display_form->setContentsMargins(8, 8, 8, 8);

  // Route display mode
  auto * route_h = new QHBoxLayout();
  route_radio_all_ = new QRadioButton("all", this);
  route_radio_selected_ = new QRadioButton("selected", this);
  route_radio_none_ = new QRadioButton("none", this);
  route_radio_selected_->setChecked(true);  // default in mapoi_rviz2_publisher
  auto * route_group = new QButtonGroup(this);
  route_group->setExclusive(true);
  route_group->addButton(route_radio_all_);
  route_group->addButton(route_radio_selected_);
  route_group->addButton(route_radio_none_);
  route_h->addWidget(route_radio_all_);
  route_h->addWidget(route_radio_selected_);
  route_h->addWidget(route_radio_none_);
  route_h->addStretch();
  display_form->addRow("Route:", route_h);

  // POI label format
  auto * label_h = new QHBoxLayout();
  label_radio_index_ = new QRadioButton("index", this);
  label_radio_name_ = new QRadioButton("name", this);
  label_radio_both_ = new QRadioButton("both", this);
  label_radio_none_ = new QRadioButton("none", this);
  label_radio_index_->setChecked(true);  // default in mapoi_rviz2_publisher
  auto * label_group = new QButtonGroup(this);
  label_group->setExclusive(true);
  label_group->addButton(label_radio_index_);
  label_group->addButton(label_radio_name_);
  label_group->addButton(label_radio_both_);
  label_group->addButton(label_radio_none_);
  label_h->addWidget(label_radio_index_);
  label_h->addWidget(label_radio_name_);
  label_h->addWidget(label_radio_both_);
  label_h->addWidget(label_radio_none_);
  label_h->addStretch();
  display_form->addRow("POI label:", label_h);

  // Arrow size ratio
  arrow_size_spin_ = new QDoubleSpinBox(this);
  arrow_size_spin_->setRange(0.0, 5.0);
  arrow_size_spin_->setSingleStep(0.1);
  arrow_size_spin_->setDecimals(2);
  arrow_size_spin_->setValue(1.0);  // default in mapoi_rviz2_publisher
  // keyboard 入力中は valueChanged を発火させず、Enter / focus loss / stepper 操作のみで反応 (Codex round 1 低)
  arrow_size_spin_->setKeyboardTracking(false);
  display_form->addRow("Arrow size:", arrow_size_spin_);

  // Display Settings group を verticalLayout に append (Save 行の下)
  if (auto * panel_layout = qobject_cast<QVBoxLayout *>(this->layout())) {
    panel_layout->addWidget(display_group);
  }

  // Connect signals → set_parameters service call.
  // 失敗 (service 未起動 / spin timeout / SetParametersResult unsuccessful) いずれの場合も:
  //   1. SyncDisplaySettingsFromPublisher() で publisher 真値で UI 更新を試みる (sync 成功なら
  //      UI = publisher で cache も更新される)
  //   2. それも失敗 (= service 完全 down) なら RevertDisplaySettingsUiFromCache() で
  //      最後に publisher と一致が確認できた値 (cached_*) に UI を戻す
  // これで UI/publisher state ズレ問題は service 状態に関わらず必ず収束 (Codex round 1-3 中)。
  auto fail_recovery = [this](const std::string & name, const std::string & reason) {
    RCLCPP_WARN(LOGGER, "SetParameters failed for %s (%s), reverting UI", name.c_str(), reason.c_str());
    SyncDisplaySettingsFromPublisher();
    RevertDisplaySettingsUiFromCache();
  };
  auto send_param = [this, fail_recovery](const std::string & name, const rclcpp::ParameterValue & value) {
    // service_is_ready() は cached state を返すだけで、AsyncParametersClient が
    // 一度も spin されていないと stale で false を返す。wait_for_service で短い spin を挟む。
    if (!rviz2_pub_param_client_->wait_for_service(100ms)) {
      fail_recovery(name, "service not ready after 100ms wait");
      return;
    }
    auto fut = rviz2_pub_param_client_->set_parameters({rclcpp::Parameter(name, value)});
    auto rc = rclcpp::spin_until_future_complete(service_node_, fut, 500ms);
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      fail_recovery(name, "spin timeout");
      return;
    }
    auto results = fut.get();
    if (results.empty() || !results[0].successful) {
      fail_recovery(name, results.empty() ? "no result" : results[0].reason);
      return;
    }
    // 成功: cache 更新 (publisher が受理した値 = 確定値、次回失敗時の revert 元として記憶)
    if (name == "route_display_mode") {
      cached_route_mode_ = value.get<std::string>();
    } else if (name == "poi_label_format") {
      cached_label_fmt_ = value.get<std::string>();
    } else if (name == "arrow_size_ratio") {
      cached_arrow_size_ = value.get<double>();
    }
  };
  auto send_string_param = [send_param](const std::string & name, const std::string & value) {
    send_param(name, rclcpp::ParameterValue(value));
  };
  auto send_double_param = [send_param](const std::string & name, double value) {
    send_param(name, rclcpp::ParameterValue(value));
  };

  // route_display_mode: toggled(checked=true) のみ反応 (false 側 toggle で重複 send しない)
  connect(route_radio_all_, &QRadioButton::toggled,
    [send_string_param](bool checked) { if (checked) send_string_param("route_display_mode", "all"); });
  connect(route_radio_selected_, &QRadioButton::toggled,
    [send_string_param](bool checked) { if (checked) send_string_param("route_display_mode", "selected"); });
  connect(route_radio_none_, &QRadioButton::toggled,
    [send_string_param](bool checked) { if (checked) send_string_param("route_display_mode", "none"); });

  // poi_label_format
  connect(label_radio_index_, &QRadioButton::toggled,
    [send_string_param](bool checked) { if (checked) send_string_param("poi_label_format", "index"); });
  connect(label_radio_name_, &QRadioButton::toggled,
    [send_string_param](bool checked) { if (checked) send_string_param("poi_label_format", "name"); });
  connect(label_radio_both_, &QRadioButton::toggled,
    [send_string_param](bool checked) { if (checked) send_string_param("poi_label_format", "both"); });
  connect(label_radio_none_, &QRadioButton::toggled,
    [send_string_param](bool checked) { if (checked) send_string_param("poi_label_format", "none"); });

  // arrow_size_ratio
  connect(arrow_size_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    [send_double_param](double value) { send_double_param("arrow_size_ratio", value); });

  // 初期同期: publisher から現在 parameter 値を読んで UI に反映
  SyncDisplaySettingsFromPublisher();
}

void PoiEditorPanel::SyncDisplaySettingsFromPublisher()
{
  // CLI で先に変更されていた場合や SetParameters 失敗時に、UI を publisher の真値に寄せる。
  // 失敗時 (publisher 未起動 / timeout) は何もしない (UI は変えない、log は呼び出し側で出力)。
  // 成功時は cache (cached_*) も同時に publisher 真値で更新する。
  if (!rviz2_pub_param_client_->wait_for_service(50ms)) {
    return;  // 短い wait、ready でなければ skip
  }
  auto fut = rviz2_pub_param_client_->get_parameters(
    {"route_display_mode", "poi_label_format", "arrow_size_ratio"});
  if (rclcpp::spin_until_future_complete(service_node_, fut, 200ms) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(LOGGER, "Sync of Display Settings parameters timed out");
    return;
  }
  auto params = fut.get();
  if (params.size() < 3) {
    return;
  }
  const std::string route_mode = params[0].as_string();
  const std::string label_fmt = params[1].as_string();
  const double arrow_size = params[2].as_double();

  // cache 更新 (publisher 真値が記憶される、次回 SetParameters 失敗時の revert 元として使用)
  cached_route_mode_ = route_mode;
  cached_label_fmt_ = label_fmt;
  cached_arrow_size_ = arrow_size;

  // QSignalBlocker で setChecked / setValue が SetParameters を再発火させないようにする
  QSignalBlocker b1(route_radio_all_);
  QSignalBlocker b2(route_radio_selected_);
  QSignalBlocker b3(route_radio_none_);
  QSignalBlocker b4(label_radio_index_);
  QSignalBlocker b5(label_radio_name_);
  QSignalBlocker b6(label_radio_both_);
  QSignalBlocker b7(label_radio_none_);
  QSignalBlocker b8(arrow_size_spin_);

  if (route_mode == "all") route_radio_all_->setChecked(true);
  else if (route_mode == "none") route_radio_none_->setChecked(true);
  else route_radio_selected_->setChecked(true);  // default fallback

  if (label_fmt == "name") label_radio_name_->setChecked(true);
  else if (label_fmt == "both") label_radio_both_->setChecked(true);
  else if (label_fmt == "none") label_radio_none_->setChecked(true);
  else label_radio_index_->setChecked(true);  // default fallback

  arrow_size_spin_->setValue(arrow_size);
}

void PoiEditorPanel::RevertDisplaySettingsUiFromCache()
{
  // service 完全 down 等で SyncDisplaySettingsFromPublisher が no-op だった場合のフォールバック。
  // cached_* は最後に publisher と一致が確認できた値 (初期同期 or 直前の SetParameters 成功時)。
  // 通常は呼び出し側 (fail_recovery) で必ず Sync を試みた直後に呼ぶため、Sync 成功時は
  // cache が UI と一致していて revert は no-op になる。
  QSignalBlocker b1(route_radio_all_);
  QSignalBlocker b2(route_radio_selected_);
  QSignalBlocker b3(route_radio_none_);
  QSignalBlocker b4(label_radio_index_);
  QSignalBlocker b5(label_radio_name_);
  QSignalBlocker b6(label_radio_both_);
  QSignalBlocker b7(label_radio_none_);
  QSignalBlocker b8(arrow_size_spin_);

  if (cached_route_mode_ == "all") route_radio_all_->setChecked(true);
  else if (cached_route_mode_ == "none") route_radio_none_->setChecked(true);
  else route_radio_selected_->setChecked(true);

  if (cached_label_fmt_ == "name") label_radio_name_->setChecked(true);
  else if (cached_label_fmt_ == "both") label_radio_both_->setChecked(true);
  else if (cached_label_fmt_ == "none") label_radio_none_->setChecked(true);
  else label_radio_index_->setChecked(true);

  arrow_size_spin_->setValue(cached_arrow_size_);
}

void PoiEditorPanel::onEnable()
{
  show();
  parentWidget()->show();
}

void PoiEditorPanel::onDisable()
{
  hide();
  parentWidget()->hide();
}

void PoiEditorPanel::MapComboBox()
{
  auto request_sm = std::make_shared<mapoi_interfaces::srv::SwitchMap::Request>();
  request_sm->map_name = map_name_list_[ui_->MapComboBox->currentIndex()];

  if (!switch_map_client_->wait_for_service(3s)) {
    RCLCPP_ERROR(LOGGER, "switch_map service not available after 3s timeout.");
    return;
  }

  auto result_sm = switch_map_client_->async_send_request(request_sm);
  rclcpp::spin_until_future_complete(service_node_, result_sm);
  PoiEditorPanel::UpdatePoiTable();
}

void PoiEditorPanel::ResetButton()
{
  PoiEditorPanel::UpdatePoiTable();
}

void PoiEditorPanel::TableChanged(int row, int column)
{
  if(is_table_color_){
    ui_->PoiTable->item(row, column)->setBackground(Qt::green);
  }
  ui_->SaveButton->setText("save");
  ui_->SaveButton->setStyleSheet("QPushButton {background-color: white; color: black;}");
  UpdatePoiCount();
}

void PoiEditorPanel::NewButton()
{
  int current_row = ui_->PoiTable->currentRow();
  int new_row = current_row + 1;
  ui_->PoiTable->insertRow(new_row);
  ui_->PoiTable->setItem(new_row, 0, new QTableWidgetItem("new_poi"));
  ui_->PoiTable->setItem(new_row, 1, new QTableWidgetItem(""));
  ui_->PoiTable->setItem(new_row, 2, new QTableWidgetItem("0.0, 0.0, 0.0"));
  // tolerance を 1 column に統合 (#151): "xy m, yaw deg" 形式
  ui_->PoiTable->setItem(new_row, 3, new QTableWidgetItem("0.5, 45.0"));
  ui_->PoiTable->setItem(new_row, 4, new QTableWidgetItem(""));        // tags
  UpdatePoiCount();
}

void PoiEditorPanel::CopyButton()
{
  int current_row = ui_->PoiTable->currentRow();
  if (current_row < 0) return;
  int new_row = current_row + 1;
  ui_->PoiTable->insertRow(new_row);
  for (int col = 0; col < ui_->PoiTable->columnCount(); col++){
    auto* item = ui_->PoiTable->item(current_row, col);
    QString txt = item ? item->text() : "";
    ui_->PoiTable->setItem(new_row, col, new QTableWidgetItem(txt));
  }
  UpdatePoiCount();
}

void PoiEditorPanel::DeleteButton()
{
  int current_row = ui_->PoiTable->currentRow();
  ui_->PoiTable->removeRow(current_row);
  ui_->SaveButton->setText("save");
  ui_->SaveButton->setStyleSheet("QPushButton {background-color: white; color: black;}");
  UpdatePoiCount();
}

void PoiEditorPanel::RowMoved(int logicalIndex, int oldVisualIndex, int newVisualIndex){
  Q_UNUSED(newVisualIndex);
  Q_UNUSED(oldVisualIndex);
  int numCols = ui_->PoiTable->columnCount();
  for (int col = 0; col < numCols; col++){
    ui_->PoiTable->item(logicalIndex, col)->setBackground(Qt::green);
  }
  ui_->SaveButton->setText("save");
  ui_->SaveButton->setStyleSheet("QPushButton {background-color: white; color: black;}");
}

void PoiEditorPanel::FileComboBox()
{
  int last = ui_->FileComboBox->count() - 1;
  if(ui_->FileComboBox->currentIndex() == last){
    QString filename = QFileDialog::getOpenFileName(
        this, tr("Select a poi_file"), QString::fromStdString(config_path_), tr("YAML files(*.yaml)"));
    if(filename == ""){
      ui_->FileComboBox->setItemText(last, "the other");
      ui_->FileComboBox->setCurrentIndex(0);
    }else{
      ui_->FileComboBox->setItemText(last, filename);
    }
  } else{
    int last2 = ui_->FileComboBox->count() - 1;
    ui_->FileComboBox->setItemText(last2, "the other");
  }
  ui_->SaveButton->setText("save");
}

void PoiEditorPanel::SaveButton()
{
  // Block save when tag filter is active (not "All")
  if (ui_->TagFilterComboBox->currentIndex() > 0) {
    QMessageBox::warning(this, tr("Filter Active"),
      tr("Cannot save while tag filter is active.\nPlease select \"All\" first to show all POIs."));
    return;
  }

  // Validate before saving
  if (!ValidatePois()) {
    return;
  }

  int numRows = ui_->PoiTable->rowCount();

  YAML::Node map_info = YAML::LoadFile(ui_->FileComboBox->itemText(0).toStdString());
  std::vector<YAML::Node> pois_list;

  for (int row = 0; row < numRows; row++) {
    int logical_row = ui_->PoiTable->verticalHeader()->logicalIndex(row);
    YAML::Node poi;
    poi["name"] = ui_->PoiTable->item(logical_row, 0)->text().toStdString();
    poi["description"] = ui_->PoiTable->item(logical_row, 1)->text().toStdString();
    auto poses_str  = ui_->PoiTable->item(logical_row, 2)->text().toStdString();
    auto poses = this->SplitSentence(poses_str, ", ");
    try {
      poi["pose"]["x"] = stod(poses[0]);
      poi["pose"]["y"] = stod(poses[1]);
      poi["pose"]["yaw"] = stod(poses[2]);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(LOGGER, "Failed to parse pose at row %d: %s", row, e.what());
      QMessageBox::critical(this, tr("Parse Error"),
        tr("Failed to parse pose at row %1: %2").arg(row + 1).arg(e.what()));
      return;
    }
    // tolerance を 1 column "xy, yaw_deg" に統合 (#151): split → xy_val (m), yaw_deg を抽出して
    // tolerance.xy / tolerance.yaw (rad) に書き戻す。pose と同じ split pattern。
    // ValidatePois で同じ split + try_parse_finite_double を通しているので parse 失敗は通常
    // 起きないが、二段防御として critical error にして save を中断する。
    auto tolerance_str = ui_->PoiTable->item(logical_row, 3)->text().toStdString();
    auto tolerance_parts = split_and_trim(tolerance_str, ',');
    double xy_val = 0.0;
    double yaw_deg = 0.0;
    if (tolerance_parts.size() != 2
        || !try_parse_finite_double(tolerance_parts[0], xy_val)
        || !try_parse_finite_double(tolerance_parts[1], yaw_deg)) {
      RCLCPP_ERROR(LOGGER, "Failed to parse tolerance at row %d (post-validation race?)", row);
      QMessageBox::critical(this, tr("Save Error"),
        tr("Failed to parse tolerance at row %1.").arg(row + 1));
      return;
    }
    poi["tolerance"]["xy"] = xy_val;
    poi["tolerance"]["yaw"] = yaw_deg * M_PI / 180.0;
    auto tags_str  = ui_->PoiTable->item(logical_row, 4)->text().toStdString();
    poi["tags"] = this->SplitSentence(tags_str, ", ");
    pois_list.push_back(poi);
  }

  map_info["poi"] = pois_list;
  YAML::Emitter out;
  // double を full precision (17 桁) で書き出して round-trip での値喪失を防ぐ (#151 round 4 高)。
  // 旧 default は 6-7 桁で、`pi/4` rad が 7 桁切り捨て (`0.7853981`) で yaml に書かれていた。
  // load 時にそのまま復元すると format_yaw_deg の判定 (整数度近似) を超えて 4 桁表示になり、
  // 「45.0 で見せる」UX 要件を満たせない問題があった。
  out.SetDoublePrecision(17);
  out << map_info;

  std::string save_path = ui_->FileComboBox->currentText().toStdString();
  std::ofstream file(save_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(LOGGER, "Failed to open file for writing: %s", save_path.c_str());
    QMessageBox::critical(this, tr("Save Error"),
      tr("Failed to open file for writing:\n%1").arg(QString::fromStdString(save_path)));
    return;
  }
  file << out.c_str();
  file.close();
  if (!file.good()) {
    RCLCPP_ERROR(LOGGER, "Error occurred while writing file: %s", save_path.c_str());
    QMessageBox::critical(this, tr("Save Error"),
      tr("Error occurred while writing file:\n%1").arg(QString::fromStdString(save_path)));
    return;
  }

  ui_->SaveButton->setText("SAVED!");
  ui_->SaveButton->setStyleSheet("QPushButton {background-color: green; color: black;}");

  if (!reload_map_info_client_->wait_for_service(3s)) {
    RCLCPP_ERROR(LOGGER, "reload_map_info service not available after 3s timeout.");
    return;
  }
  auto request_reload_map_info = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result_reload_map_info = reload_map_info_client_->async_send_request(request_reload_map_info);
  rclcpp::spin_until_future_complete(service_node_, result_reload_map_info);

  // Drag による reorder は Qt の visual order だけ変えるため、Save 後も verticalHeader の
  // 番号は元の logical order のまま (例: [3, 1, 2] のように見える)。
  // saved YAML を fetch し直して table を再構築することで visual = logical を揃え、
  // 番号が 1, 2, 3, ... に並ぶようにする。drag 中の background highlight (Qt::green) も解除。
  //
  // delay を入れて SAVED + green を 1.5 秒見せてから rebuild する (issue #77)。
  // 即時 rebuild だと UpdatePoiTable() が text/style を即 reset するため、Save 成功 feedback が
  // ほぼ見えない (reload service が高速なほど顕著)。QTimer::singleShot で Qt event loop に戻して
  // SAVED + green の paint を保証してから 1.5 秒後に rebuild。
  //
  // 同じ 1.5 秒の間に再 Save 連打されると pending rebuild と新 Save flow が race するため、
  // SaveButton を disable して連打防止。timer 完了時に setEnabled(true) で復帰。
  // 注: 1.5 秒間に Reset / MapCombo 等の他 UpdatePoiTable 呼び出しが入ると順序競合あり得るが、
  // 短い窓 + Save 直後の操作頻度低さから許容。必要なら future PR で member QTimer + cancel
  // pattern に拡張可能。
  ui_->SaveButton->setEnabled(false);
  QTimer::singleShot(1500, this, [this]() {
    UpdatePoiTable();
    ui_->SaveButton->setEnabled(true);
  });
}

// Subscription Callback
void PoiEditorPanel::PoiPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  auto p = msg->pose;
  auto txt = tr("%1, %2, %3").arg(p.position.x).arg(p.position.y).arg(this->calcYaw(p));
  QMetaObject::invokeMethod(this, [this, txt]() {
    int current_row = ui_->PoiTable->currentRow();
    if (current_row >= 0) {
      ui_->PoiTable->setItem(current_row, 2, new QTableWidgetItem(txt));
    }
  }, Qt::QueuedConnection);
}

void PoiEditorPanel::ConfigPathCallback(std_msgs::msg::String::SharedPtr msg)
{
  // Resolve symlinks to get the real (src/) path instead of install/ path
  std::filesystem::path p(msg->data);
  std::string resolved_path = msg->data;
  try {
    resolved_path = std::filesystem::canonical(p).string();
  } catch (const std::filesystem::filesystem_error& e) {
    RCLCPP_WARN(LOGGER, "Could not resolve canonical path: %s", e.what());
  }

  std::filesystem::path resolved_p(resolved_path);
  std::string map_name = resolved_p.parent_path().filename().string();
  bool first_config = config_path_.empty();
  config_path_ = resolved_path;
  if(current_map_ != map_name || first_config){
    current_map_ = map_name;
    QMetaObject::invokeMethod(this, [this, map_name]() {
      InitConfigs(map_name);
    }, Qt::QueuedConnection);
  }
}

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
      ui_->PoiTable->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(p.name)));
      ui_->PoiTable->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(p.description)));
      ui_->PoiTable->setItem(row, 2, new QTableWidgetItem(tr("%1, %2, %3").arg(p.pose.position.x).arg(p.pose.position.y).arg(this->calcYaw(p.pose))));
      // tolerance を "xy, yaw_deg" 1 column に統合 (#151)。yaw は format_yaw_deg で
      // 「45° 狙いの 0.7853981 rad → 45.0」表示。
      ui_->PoiTable->setItem(row, 3, new QTableWidgetItem(
        tr("%1, %2").arg(p.tolerance.xy).arg(format_yaw_deg(p.tolerance.yaw))));
      ui_->PoiTable->setItem(row, 4, new QTableWidgetItem(QString::fromStdString(this->join(p.tags, ", "))));
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
    RCLCPP_WARN(LOGGER, "get_tag_definitions service not available, TagHelper will be empty.");
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

  // Get current row's tags cell (column 4; tolerance を 1 column 化したので #151 で 5→4)
  int current_row = ui_->PoiTable->currentRow();
  if (current_row < 0) {
    ui_->TagHelperComboBox->setCurrentIndex(0);
    return;
  }

  auto* tags_item = ui_->PoiTable->item(current_row, 4);
  std::string current_tags = tags_item ? tags_item->text().toStdString() : "";

  // Check if tag already exists
  auto tag_list = this->SplitSentence(current_tags, ", ");
  for (const auto& t : tag_list) {
    if (t == tag_name) {
      ui_->TagHelperComboBox->setCurrentIndex(0);
      return;
    }
  }

  // Append tag (column 4 = tags、tolerance を 1 column 化したので #151 で 5→4)
  std::string new_tags;
  if (current_tags.empty()) {
    new_tags = tag_name;
  } else {
    new_tags = current_tags + ", " + tag_name;
  }
  ui_->PoiTable->setItem(current_row, 4, new QTableWidgetItem(QString::fromStdString(new_tags)));

  // Reset combo to placeholder
  ui_->TagHelperComboBox->setCurrentIndex(0);
}

void PoiEditorPanel::UpdatePoiCount()
{
  int count = ui_->PoiTable->rowCount();
  ui_->PoiCountLabel->setText(tr("POIs: %1").arg(count));
}

bool PoiEditorPanel::ValidatePois()
{
  int numRows = ui_->PoiTable->rowCount();
  QStringList warnings;
  std::set<std::string> names_seen;

  for (int row = 0; row < numRows; row++) {
    int logical_row = ui_->PoiTable->verticalHeader()->logicalIndex(row);

    // Check name
    auto* name_item = ui_->PoiTable->item(logical_row, 0);
    std::string name = name_item ? name_item->text().toStdString() : "";
    if (name.empty()) {
      warnings.append(tr("Row %1: name is empty").arg(row + 1));
    } else if (names_seen.count(name) > 0) {
      warnings.append(tr("Row %1: duplicate name \"%2\"").arg(row + 1).arg(QString::fromStdString(name)));
    }
    names_seen.insert(name);

    // Check pose format (expect "x, y, yaw" — 3 elements)
    auto* pose_item = ui_->PoiTable->item(logical_row, 2);
    std::string pose_str = pose_item ? pose_item->text().toStdString() : "";
    auto poses = this->SplitSentence(pose_str, ", ");
    if (poses.size() != 3) {
      warnings.append(tr("Row %1: pose must be \"x, y, yaw\" (3 values)").arg(row + 1));
    } else {
      for (int i = 0; i < 3; i++) {
        try {
          stod(poses[i]);
        } catch (...) {
          warnings.append(tr("Row %1: pose contains non-numeric value \"%2\"").arg(row + 1).arg(QString::fromStdString(poses[i])));
          break;
        }
      }
    }

    // Check tolerance "xy m, yaw deg" (1 column に統合、#151)。"0.5, 45.0" 形式で split → 各値を
    // try_parse_finite_double で strict parse + 有限性検査 (Codex review #139 medium 対応)。
    // min: tolerance.xy >= 0.001 m / tolerance.yaw >= 0.001 rad ≒ 0.057° (#138 msg spec)。
    auto* tolerance_item = ui_->PoiTable->item(logical_row, 3);
    std::string tolerance_str = tolerance_item ? tolerance_item->text().toStdString() : "";
    auto tolerance_parts = split_and_trim(tolerance_str, ',');
    if (tolerance_parts.size() != 2) {
      warnings.append(tr("Row %1: tolerance must be \"xy, yaw_deg\" (got \"%2\")")
                        .arg(row + 1).arg(QString::fromStdString(tolerance_str)));
    } else {
      double xy_val = 0.0;
      if (!try_parse_finite_double(tolerance_parts[0], xy_val)) {
        warnings.append(tr("Row %1: invalid tolerance.xy \"%2\"")
                          .arg(row + 1).arg(QString::fromStdString(tolerance_parts[0])));
      } else if (xy_val < 0.001) {
        warnings.append(tr("Row %1: tolerance.xy must be >= 0.001 m (got %2)")
                          .arg(row + 1).arg(xy_val));
      }
      double yaw_deg = 0.0;
      if (!try_parse_finite_double(tolerance_parts[1], yaw_deg)) {
        warnings.append(tr("Row %1: invalid tolerance.yaw \"%2\"")
                          .arg(row + 1).arg(QString::fromStdString(tolerance_parts[1])));
      } else if (yaw_deg * M_PI / 180.0 < 0.001) {
        warnings.append(tr("Row %1: tolerance.yaw must be >= 0.06 deg (≒ 0.001 rad) (got %2 deg)")
                          .arg(row + 1).arg(yaw_deg));
      }
    }
  }

  if (!warnings.isEmpty()) {
    QMessageBox::warning(this, tr("Validation Errors"), warnings.join("\n"));
    return false;
  }

  // Hard validation: landmark の排他 (#85)。
  // goal+landmark は意味矛盾 (Nav2 goal にできない reference 点)、
  // initial_pose+landmark は到達不可な POI を起点に置く矛盾。
  QStringList exclusivity_warnings;
  for (int row = 0; row < numRows; row++) {
    int logical_row = ui_->PoiTable->verticalHeader()->logicalIndex(row);
    auto* tags_item = ui_->PoiTable->item(logical_row, 4);
    std::string tags_str = tags_item ? tags_item->text().toStdString() : "";
    if (tags_str.empty()) continue;

    auto tags = this->SplitSentence(tags_str, ", ");
    bool has_waypoint = false, has_landmark = false, has_pause = false;
    for (const auto& t : tags) {
      if (t == "waypoint") has_waypoint = true;
      else if (t == "landmark") has_landmark = true;
      else if (t == "pause") has_pause = true;
    }
    if (has_waypoint && has_landmark) {
      exclusivity_warnings.append(
        tr("Row %1: \"waypoint\" と \"landmark\" は併用できません (landmark は Nav2 navigation 不可)").arg(row + 1));
    }
    // landmark × pause 排他 (#143): landmark は到達不可な reference なので
    // pause (= 到達したときに止める semantics) と意味的に矛盾する。
    if (has_pause && has_landmark) {
      exclusivity_warnings.append(
        tr("Row %1: \"pause\" と \"landmark\" は併用できません (landmark は到達不可な reference のため pause 動作が成立しない)").arg(row + 1));
    }
    // (initial_pose × landmark 排他は #144 で initial_pose system tag を廃止したため不要に。)
  }
  if (!exclusivity_warnings.isEmpty()) {
    QMessageBox::warning(this, tr("Tag Exclusivity Errors"), exclusivity_warnings.join("\n"));
    return false;
  }

  // Soft validation: warn about undefined tags (non-blocking)
  QStringList tag_warnings;
  for (int row = 0; row < numRows; row++) {
    int logical_row = ui_->PoiTable->verticalHeader()->logicalIndex(row);
    auto* tags_item = ui_->PoiTable->item(logical_row, 4);
    std::string tags_str = tags_item ? tags_item->text().toStdString() : "";
    if (tags_str.empty()) continue;

    auto tags = this->SplitSentence(tags_str, ", ");
    for (const auto& tag : tags) {
      if (tag.empty()) continue;
      bool found = false;
      for (const auto& known : known_tag_names_) {
        if (known == tag) {
          found = true;
          break;
        }
      }
      if (!found) {
        tag_warnings.append(tr("Row %1: undefined tag \"%2\"").arg(row + 1).arg(QString::fromStdString(tag)));
      }
    }
  }

  if (!tag_warnings.isEmpty()) {
    QString msg = tr("The following undefined tags were found:\n\n%1\n\nSave anyway?").arg(tag_warnings.join("\n"));
    auto ret = QMessageBox::question(this, tr("Undefined Tags"), msg,
                                     QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (ret != QMessageBox::Yes) {
      return false;
    }
  }

  return true;
}

// Functions
void PoiEditorPanel::InitConfigs(std::string map_name)
{
  // MapComboBox
  int i = 0;
  for(const auto & map : map_name_list_){
    if(map == map_name){
      ui_->MapComboBox->setCurrentIndex(i);
      break;
    }
    i++;
  }

  // FileComboBox
  ui_->FileComboBox->clear();
  if (!config_path_.empty()) {
    ui_->FileComboBox->addItem(QString::fromStdString(config_path_));
  }
  ui_->FileComboBox->addItem("the other");

  // PoiTable, SaveButton, and CheckBox
  PoiEditorPanel::UpdatePoiTable();
}

void PoiEditorPanel::UpdatePoiTable()
{
  auto request_gtp = std::make_shared<mapoi_interfaces::srv::GetPoisInfo::Request>();

  if (!get_pois_info_client_->wait_for_service(3s)) {
    RCLCPP_ERROR(LOGGER, "get_pois_info service not available after 3s timeout.");
    return;
  }

  auto result_gtp = get_pois_info_client_->async_send_request(request_gtp);
  rclcpp::spin_until_future_complete(service_node_, result_gtp);
  auto poi_info = result_gtp.get();
  auto pois = poi_info->pois_list;
  auto numRows = pois.size();

  // Store all POIs for tag filtering
  all_pois_.assign(pois.begin(), pois.end());
  PopulateTagFilter();

  ui_->PoiTable->clear();
  // Drag による reorder で動いた verticalHeader の visual order は、
  // 同じ rowCount で setRowCount し直しても reset されない (Qt 既知挙動)。
  // 一度 0 行にしてから再生成することで visual = logical を強制する。
  ui_->PoiTable->setRowCount(0);
  ui_->PoiTable->setRowCount(numRows);
  ui_->PoiTable->setColumnCount(5);
  ui_->PoiTable->setHorizontalHeaderLabels(
    QStringList() << tr("name") << tr("description") << tr("x, y, yaw")
                  << tr("tolerance (xy m, yaw deg)") << tr("tags"));
  ui_->PoiTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  ui_->PoiTable->verticalHeader()->setSectionsMovable(true);
  ui_->PoiTable->horizontalHeader()->setSortIndicatorShown(true);
  ui_->PoiTable->horizontalHeader()->setSortIndicator(0, Qt::AscendingOrder);

  is_table_color_ = false;
  for (size_t row = 0; row < numRows; row++){
    const auto & p = pois[row];
    ui_->PoiTable->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(p.name)));
    ui_->PoiTable->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(p.description)));
    ui_->PoiTable->setItem(row, 2, new QTableWidgetItem(tr("%1, %2, %3").arg(p.pose.position.x).arg(p.pose.position.y).arg(this->calcYaw(p.pose))));
    // tolerance を "xy, yaw_deg" 1 column に統合 (#151)。
    ui_->PoiTable->setItem(row, 3, new QTableWidgetItem(
      tr("%1, %2").arg(p.tolerance.xy).arg(format_yaw_deg(p.tolerance.yaw))));
    ui_->PoiTable->setItem(row, 4, new QTableWidgetItem(QString::fromStdString(this->join(p.tags, ", "))));
  }
  is_table_color_ = true;
  ui_->SaveButton->setText("save");
  // 空 list (numRows == 0) では setItem 由来の cellChanged 副作用が発火せず、SAVED 時の
  // green stylesheet が残ったままになる症状 (issue #77 症状 2) を明示リセットで防ぐ。
  ui_->SaveButton->setStyleSheet("QPushButton {background-color: white; color: black;}");
  UpdatePoiCount();
}

double PoiEditorPanel::calcYaw(geometry_msgs::msg::Pose pose)
{
  tf2::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

// https://marycore.jp/prog/cpp/vector-join/
std::string PoiEditorPanel::join(const std::vector<std::string>& v, const char* delim)
{
  std::string s;
  if (!v.empty()) {
    s += v[0];
    for (decltype(v.size()) i = 1, c = v.size(); i < c; ++i) {
      if (delim) s += delim;
      s += v[i];
    }
  }
  return s;
}

// https://lilaboc.work/archives/19026007.html
std::vector<std::string> PoiEditorPanel::SplitSentence(std::string sentence, std::string delimiter)
{
	std::vector<std::string> words;
	size_t position = 0;

	while(sentence.find(delimiter.c_str(), position) != std::string::npos){
		size_t next_position = sentence.find(delimiter.c_str(), position);
		std::string word = sentence.substr(position, next_position-position);
		position = next_position + delimiter.length();
		words.push_back(word);
	}
	std::string last_word = sentence.substr(position, sentence.length()-position);
	words.push_back(last_word);

	return words;
}

}  // mapoi_rviz_plugins

CLASS_LOADER_REGISTER_CLASS(mapoi_rviz_plugins::PoiEditorPanel, rviz_common::Panel)
