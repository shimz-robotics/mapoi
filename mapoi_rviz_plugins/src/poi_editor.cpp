#include "mapoi_rviz_plugins/poi_editor.hpp"
#include <class_loader/class_loader.hpp>
#include <filesystem>
#include <fstream>

#include <QFileDialog>
#include <QStandardPaths>

#include "ui_poi_editor.h"

using namespace std::chrono_literals;

namespace mapoi_rviz_plugins
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mapoi_rviz_plugins.poi_editor");

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
  ui_->PoiTable->setItem(new_row, 3, new QTableWidgetItem("0.5"));
  ui_->PoiTable->setItem(new_row, 4, new QTableWidgetItem(""));
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
    poi["radius"] = ui_->PoiTable->item(logical_row, 3)->text().toDouble();
    auto tags_str  = ui_->PoiTable->item(logical_row, 4)->text().toStdString();
    poi["tags"] = this->SplitSentence(tags_str, ", ");
    pois_list.push_back(poi);
  }

  map_info["poi"] = pois_list;
  YAML::Emitter out;
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
      ui_->PoiTable->setItem(row, 3, new QTableWidgetItem(tr("%1").arg(p.radius)));
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

  // Get current row's tags cell (column 4)
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

  // Append tag
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

    // Check radius
    auto* radius_item = ui_->PoiTable->item(logical_row, 3);
    std::string radius_str = radius_item ? radius_item->text().toStdString() : "";
    try {
      double r = stod(radius_str);
      if (r < 0) {
        warnings.append(tr("Row %1: radius is negative").arg(row + 1));
      }
    } catch (...) {
      warnings.append(tr("Row %1: invalid radius \"%2\"").arg(row + 1).arg(QString::fromStdString(radius_str)));
    }
  }

  if (!warnings.isEmpty()) {
    QMessageBox::warning(this, tr("Validation Errors"), warnings.join("\n"));
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
  ui_->PoiTable->setRowCount(numRows);
  ui_->PoiTable->setColumnCount(5);
  ui_->PoiTable->setHorizontalHeaderLabels( QStringList() << tr("name") << tr("description") << tr("x, y, yaw") << tr("radius") << tr("tags" ) );
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
    ui_->PoiTable->setItem(row, 3, new QTableWidgetItem(tr("%1").arg(p.radius)));
    ui_->PoiTable->setItem(row, 4, new QTableWidgetItem(QString::fromStdString(this->join(p.tags, ", "))));
  }
  is_table_color_ = true;
  ui_->SaveButton->setText("save");
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
