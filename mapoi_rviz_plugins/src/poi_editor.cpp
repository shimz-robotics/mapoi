#include "mapoi_rviz_plugins/poi_editor.hpp"
#include <class_loader/class_loader.hpp>

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
  // https://qiita.com/Kotakku/items/01082cfd024a68c0d6ec
  node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  connect(ui_->MapComboBox, SIGNAL(activated(int)), this, SLOT(MapComboBox()));
  connect(ui_->ResetButton, SIGNAL(clicked()), this, SLOT(ResetButton()));
  connect(ui_->PoiTable, SIGNAL(cellChanged(int, int)), this, SLOT(TableChanged(int, int)));
  connect(ui_->PoiTable->verticalHeader(), SIGNAL(sectionMoved(int, int, int)), this, SLOT(RowMoved(int, int, int)));
  connect(ui_->NewButton, SIGNAL(clicked()), this, SLOT(NewButton()));
  connect(ui_->CopyButton, SIGNAL(clicked()), this, SLOT(CopyButton()));
  connect(ui_->DeleteButton, SIGNAL(clicked()), this, SLOT(DeleteButton()));
  connect(ui_->FileComboBox, SIGNAL(activated(int)), this, SLOT(FileComboBox()));
  connect(ui_->SaveButton, SIGNAL(clicked()), this, SLOT(SaveButton()));

  poi_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "poi_pose", 10, std::bind(&PoiEditorPanel::PoiPoseCallback, this, std::placeholders::_1));

  parentWidget()->setVisible(true);

  // MapComboBox
  auto node = rclcpp::Node::make_shared("poieditor_get_maps_info_client");
  auto get_map_info_cli = node->create_client<mapoi_interfaces::srv::GetMapsInfo>("get_maps_info");
  auto request = std::make_shared<mapoi_interfaces::srv::GetMapsInfo::Request>();
  auto result = get_map_info_cli->async_send_request(request);
  rclcpp::spin_until_future_complete(node, result);
  auto map_info = result.get();
  current_map_ = map_info->map_name;
  map_name_list_ = map_info->maps_list;
  ui_->MapComboBox->clear();
  for (auto map : map_name_list_) {
    ui_->MapComboBox->addItem(QString::fromStdString(map));
  }
  InitConfigs(map_info->map_name);

  map_name_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "mapoi_interfaces/current_map", 10,
    std::bind(&PoiEditorPanel::MapNameCallback, this, std::placeholders::_1));
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
  auto node = rclcpp::Node::make_shared("poieditorswitch_map_client");
  rclcpp::Client<mapoi_interfaces::srv::SwitchMap>::SharedPtr client_sm =
    node->create_client<mapoi_interfaces::srv::SwitchMap>("switch_map");
  auto request_sm = std::make_shared<mapoi_interfaces::srv::SwitchMap::Request>();
  request_sm->map_name = map_name_list_[ui_->MapComboBox->currentIndex()];
  auto result_sm = client_sm->async_send_request(request_sm);
  rclcpp::spin_until_future_complete(node, result_sm);
  PoiEditorPanel::UpdatePoiTable("all");
}

void PoiEditorPanel::ResetButton()
{
  PoiEditorPanel::UpdatePoiTable("all");
}

void PoiEditorPanel::TableChanged(int row, int column)
{
  if(is_table_color_){
    ui_->PoiTable->item(row, column)->setBackground(Qt::green);
  }
  ui_->SaveButton->setText("save");
  ui_->SaveButton->setStyleSheet("QPushButton {background-color: white; color: black;}");
}

void PoiEditorPanel::NewButton()
{
  int current_row = ui_->PoiTable->currentRow();
  int new_row = current_row + 1;
  ui_->PoiTable->insertRow(new_row);
  auto txt = ui_->PoiTable->item(current_row, 0)->text();
  txt = tr("%1").arg(txt.toInt()+1);
  ui_->PoiTable->setItem(new_row, 0, new QTableWidgetItem(txt));
}

void PoiEditorPanel::CopyButton()
{
  // https://www.youtube.com/watch?v=qCrU6VZToTw
  int current_row = ui_->PoiTable->currentRow();
  int new_row = current_row + 1;
  ui_->PoiTable->insertRow(new_row);
  for (int col = 0; col < 6; col++){
    auto txt = ui_->PoiTable->item(current_row, col)->text();
    ui_->PoiTable->setItem(new_row, col, new QTableWidgetItem(txt));
  }
}

void PoiEditorPanel::DeleteButton()
{
  int current_row = ui_->PoiTable->currentRow();
  ui_->PoiTable->removeRow(current_row);
  ui_->SaveButton->setText("save");
  ui_->SaveButton->setStyleSheet("QPushButton {background-color: white; color: black;}");
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
  if(ui_->FileComboBox->currentIndex() == 2){
    QString filename = QFileDialog::getOpenFileName(
        this, tr("Select a poi_file"), ui_->FileComboBox->itemText(1), tr("YAML files(*.yaml)"));
    if(filename == ""){ // choosing file was canceled
      ui_->FileComboBox->setItemText(2, "the other");
      ui_->FileComboBox->setCurrentIndex(0);
    }else{
      ui_->FileComboBox->setItemText(2, filename);
    }
  } else{
    ui_->FileComboBox->setItemText(2, "the other");
  }
  ui_->SaveButton->setText("save");
}

void PoiEditorPanel::SaveButton()
{
  int numRows = ui_->PoiTable->rowCount();

  YAML::Node map_info = YAML::LoadFile(ui_->FileComboBox->itemText(0).toStdString());
  std::vector<YAML::Node> pois_list;

  for (int row = 0; row < numRows; row++) {
    // https://stackoverflow.com/questions/41418409/how-to-determine-the-new-order-of-rows-in-qtablewidget-after-sectionmoved-event
    int logical_row = ui_->PoiTable->verticalHeader()->logicalIndex(row);
    YAML::Node poi;
    poi["id"] = ui_->PoiTable->item(logical_row, 0)->text().toInt();
    poi["name"] = ui_->PoiTable->item(logical_row, 1)->text().toStdString();
    poi["jp"] = ui_->PoiTable->item(logical_row, 2)->text().toStdString();;
    auto poses_str  = ui_->PoiTable->item(logical_row, 3)->text().toStdString();
    auto poses = this->SplitSentence(poses_str, ", ");
    poi["pose"]["x"] = stod(poses[0]);
    poi["pose"]["y"] = stod(poses[1]);
    poi["pose"]["yaw"] = stod(poses[2]);
    poi["radius"] = ui_->PoiTable->item(logical_row, 4)->text().toDouble();
    auto tags_str  = ui_->PoiTable->item(logical_row, 5)->text().toStdString();
    poi["tags"] = this->SplitSentence(tags_str, ", ");
    pois_list.push_back(poi);
  }

  map_info["poi"] = pois_list;
  YAML::Emitter out;
  out << map_info;
  std::ofstream file(ui_->FileComboBox->currentText().toStdString());
  file << out.c_str();
  file.close();

  ui_->SaveButton->setText("SAVED!");
  ui_->SaveButton->setStyleSheet("QPushButton {background-color: green; color: black;}");

  auto node = rclcpp::Node::make_shared("poieditorreload_map_info");
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_reload_map_info =
    node->create_client<std_srvs::srv::Trigger>("reload_map_info");
  auto request_reload_map_info = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result_reload_map_info = client_reload_map_info->async_send_request(request_reload_map_info);
  rclcpp::spin_until_future_complete(node, result_reload_map_info);
}

// Subscription Callback
void PoiEditorPanel::PoiPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  int current_row = ui_->PoiTable->currentRow();
  auto p = msg->pose;
  auto txt = tr("%1, %2, %3").arg(p.position.x).arg(p.position.y).arg(this->calcYaw(p));
  ui_->PoiTable->setItem(current_row, 3, new QTableWidgetItem(txt));
}

void PoiEditorPanel::MapNameCallback(std_msgs::msg::String::SharedPtr msg)
{
  if(current_map_ != msg->data){
    current_map_ = msg->data;
    InitConfigs(msg->data);
  }
}

// Functions
void PoiEditorPanel::InitConfigs(std::string map_name)
{
  // MapComboBox
  int i = 0;
  for(auto map : map_name_list_){
    if(map == map_name){
      ui_->MapComboBox->setCurrentIndex(i);
      break;
    }
    i++;
  }

  // FileComboBox
  char * home_dir = getenv("HOME");
  auto install_dir = std::string(home_dir) + "/work/mapoi5_ws/install/mapoi_interfaces/share/mapoi_interfaces/map/" + map_name + "/" + map_name + ".yaml";
  auto src_dir = std::string(home_dir) + "/work/mapoi5_ws/src/shimz_pkgs/mapoi_interfaces/map/" + map_name + "/" + map_name + ".yaml";

  ui_->FileComboBox->clear();
  ui_->FileComboBox->addItem(QString::fromStdString(install_dir));
  ui_->FileComboBox->addItem(QString::fromStdString(src_dir));
  ui_->FileComboBox->addItem("the other");

  // PoiTable, SaveButton, and CheckBox
  PoiEditorPanel::UpdatePoiTable("all");
}

void PoiEditorPanel::UpdatePoiTable(std::string tag)
{
  // get pois
  auto node = rclcpp::Node::make_shared("poieditorget_tagged_pois_client");
  rclcpp::Client<mapoi_interfaces::srv::GetTaggedPois>::SharedPtr client_gtp =
    node->create_client<mapoi_interfaces::srv::GetTaggedPois>("get_tagged_pois");

  auto request_gtp = std::make_shared<mapoi_interfaces::srv::GetTaggedPois::Request>();
  request_gtp->tag = tag;

  while(!client_gtp->wait_for_service(1s)){
    if(!rclcpp::ok()){
      RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(LOGGER, "service not available, waiting again...");
  }

  auto result_gtp = client_gtp->async_send_request(request_gtp);
  rclcpp::spin_until_future_complete(node, result_gtp);
  auto poi_info = result_gtp.get();
  auto pois = poi_info->pois_list;
  auto numRows = pois.size();

  ui_->PoiTable->clear();
  ui_->PoiTable->setRowCount(numRows);
  ui_->PoiTable->setColumnCount(5);
  ui_->PoiTable->setHorizontalHeaderLabels( QStringList() << tr("name") << tr("description") << tr("x, y, yaw") << tr("radius") << tr("tags" ) );
  ui_->PoiTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  ui_->PoiTable->verticalHeader()->setSectionsMovable(true);
  ui_->PoiTable->horizontalHeader()->setSortIndicatorShown(true);
  ui_->PoiTable->horizontalHeader()->setSortIndicator(0, Qt::AscendingOrder);

  is_table_color_ = false;
  for (int row = 0; row < numRows; row++){
    auto p = pois[row];
    ui_->PoiTable->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(p.name)));
    ui_->PoiTable->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(p.description)));
    ui_->PoiTable->setItem(row, 2, new QTableWidgetItem(tr("%1, %2, %3").arg(p.pose.position.x).arg(p.pose.position.y).arg(this->calcYaw(p.pose))));
    ui_->PoiTable->setItem(row, 3, new QTableWidgetItem(tr("%1").arg(p.radius)));
    ui_->PoiTable->setItem(row, 4, new QTableWidgetItem(QString::fromStdString(this->join(p.tags, ", "))));
  }
  is_table_color_ = true;
  ui_->SaveButton->setText("save");
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
std::string PoiEditorPanel::join(const std::vector<std::string>& v, const char* delim = 0)
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
