#include "mapoi_rviz_plugins/mapoi_panel.hpp"
#include <class_loader/class_loader.hpp>

#include "ui_mapoi_panel.h"

using namespace std::chrono_literals;

namespace mapoi_rviz_plugins
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mapoi_rviz_plugins.mapoi_panel");

MapoiPanel::MapoiPanel(QWidget* parent) : Panel(parent),  ui_(new Ui::ScUI())
{
  ui_->setupUi(this);
}

MapoiPanel::~MapoiPanel() = default;

void MapoiPanel::onInitialize()
{
  // https://qiita.com/Kotakku/items/01082cfd024a68c0d6ec
  node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  connect(ui_->MapComboBox, SIGNAL(activated(int)), this, SLOT(MapComboBox()));
  connect(ui_->DestinationComboBox, SIGNAL(activated(int)), this, SLOT(DestinationComboBox()));
  connect(ui_->FootprintComboBox, SIGNAL(activated(int)), this, SLOT(FootprintComboBox()));

  connect(ui_->LocalizationButton, SIGNAL(clicked()), this, SLOT(LocalizationButton()));
  connect(ui_->RunButton, SIGNAL(clicked()), this, SLOT(RunButton()));
  connect(ui_->StopButton, SIGNAL(clicked()), this, SLOT(StopButton()));

  parentWidget()->setVisible(true);

  // map ComboBox
  auto node = rclcpp::Node::make_shared("mapoi_panel_get_map_info_client");
  auto get_map_info_cli = node->create_client<mapoi_interfaces::srv::GetMapInfo>("get_map_info");
  while(!get_map_info_cli->wait_for_service(1s)){
    if(!rclcpp::ok()){
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }
  auto request = std::make_shared<mapoi_interfaces::srv::GetMapInfo::Request>();
  auto result = get_map_info_cli->async_send_request(request);
  if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto map_info = result.get();
    current_map_ = map_info->map_name;
    map_name_list_ = map_info->maps_list;
    for (auto map : map_name_list_) {
      ui_->MapComboBox->addItem(QString::fromStdString(map));
    }
    SetMapComboBox(map_info->map_name);
  }else{
    RCLCPP_ERROR(node->get_logger(), "Failed to call service get_tagged_pois");
  }

  // Buttons
  initialpose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1);
  goal_name_pub_ = node_->create_publisher<std_msgs::msg::String>("mapoi_nav/goal_names", 1);
  cancel_pub_ = node_->create_publisher<std_msgs::msg::String>("mapoi_nav/cancel", 1);

  cmd_vel_mode_sub_ = node_->create_subscription<CmdVelFrom>(
    "/cmd_vel/mode", 10, [this](CmdVelFrom::ConstSharedPtr msg) {
        cmd_vel_mode_ = msg->cmd_vel_mode;
      });
  cmd_vel_mode_ = CmdVelFrom::NON;
  map_name_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/mapoi_interfaces/current_map", 10,
      std::bind(&MapoiPanel::MapNameCallback, this, std::placeholders::_1));

  // footprint ComboBox
  footprint_mode_pub_ =
      node_->create_publisher<std_msgs::msg::Int8>("footprint_mode", 1);
  ui_->FootprintComboBox->addItem("NORMAL");
  ui_->FootprintComboBox->addItem("SMALLEST");
  ui_->FootprintComboBox->addItem("DYNAMIC");
  ui_->FootprintComboBox->addItem("SMALL");
}

void MapoiPanel::onEnable()
{
  show();
  parentWidget()->show();
}

void MapoiPanel::onDisable()
{
  hide();
  parentWidget()->hide();
}

void MapoiPanel::MapComboBox() //Set"Floor"
{
  // switch map
  auto node = rclcpp::Node::make_shared("mapoi_panel_switch_map_client");
  rclcpp::Client<mapoi_interfaces::srv::SwitchMap>::SharedPtr client_sm =
    node->create_client<mapoi_interfaces::srv::SwitchMap>("switch_map");

  auto request_sm = std::make_shared<mapoi_interfaces::srv::SwitchMap::Request>();
  request_sm->map_name = map_name_list_[ui_->MapComboBox->currentIndex()];

  while(!client_sm->wait_for_service(1s)){
    if(!rclcpp::ok()){
      RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(LOGGER, "service not available, waiting again...");
  }

  auto result_sm = client_sm->async_send_request(request_sm);
  if(rclcpp::spin_until_future_complete(node, result_sm) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "Success: %d", result_sm.get()->success);
  }else{
    RCLCPP_ERROR(LOGGER, "Failed to call service switch_map");
  }
}

void MapoiPanel::DestinationComboBox()
{
  dest_ind_ = ui_->DestinationComboBox->currentIndex();
}

void MapoiPanel::FootprintComboBox()
{
  std_msgs::msg::Int8 footprint_mode_msg;
  footprint_mode_msg.data = ui_->FootprintComboBox->currentIndex();
  footprint_mode_pub_->publish(footprint_mode_msg);
}

void MapoiPanel::LocalizationButton()
{  
  geometry_msgs::msg::PoseWithCovarianceStamped msg_init;
  msg_init.header.stamp = rclcpp::Clock().now();
  msg_init.header.frame_id = "map";
  msg_init.pose.pose = pois_[dest_ind_].pose;
  msg_init.pose.covariance[0] = 0.25;
  msg_init.pose.covariance[7] = 0.25;
  msg_init.pose.covariance[35] = 0.06853891945200942;

  initialpose_pub_->publish(msg_init);
  RCLCPP_INFO(LOGGER, "A initialpose was set.");
}

void MapoiPanel::RunButton()
{
  std_msgs::msg::String msg;
  msg.data = pois_[dest_ind_].name;
  goal_name_pub_->publish(msg);

  if(cmd_vel_mode_ != CmdVelFrom::AUTO){
    MapoiPanel::RequestSetCmdVelMode(CmdVelFrom::AUTO);
  }
  RCLCPP_INFO(LOGGER, "A goal pose was set");
}

void MapoiPanel::StopButton()
{
  RCLCPP_INFO(LOGGER, "cmd_vel_mode_ : %s", cmd_vel_mode_.c_str());
  // if(cmd_vel_mode_ == CmdVelFrom::AUTO){
    std_msgs::msg::String msg;
    msg.data = "mapoi_panel";
    cancel_pub_->publish(msg);
    MapoiPanel::RequestSetCmdVelMode(CmdVelFrom::NON);
    RCLCPP_INFO(LOGGER, "AI suitcase was stopped");
  // }
}

void MapoiPanel::RequestSetCmdVelMode(std::string cm)
{
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("set_cmd_vel_mode_client");
  auto cmd_vel_mode_cli = node->create_client<cmd_vel_manager::srv::SetCmdVelMode>("set_cmd_vel_mode");

  while(!cmd_vel_mode_cli->wait_for_service(1s)){
    if(!rclcpp::ok()){
      RCLCPP_ERROR(rclcpp::get_logger("set_cmd_vel_mode_client"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("set_cmd_vel_mode_client"), "service not available, waiting again...");
  }

  auto request = std::make_shared<cmd_vel_manager::srv::SetCmdVelMode::Request>();
  request->cmd_vel_mode = cm;
  while(!cmd_vel_mode_cli->wait_for_service(1s)){
    if(!rclcpp::ok()){
      RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(LOGGER, "service not available, waiting again...");
  }
  auto result = cmd_vel_mode_cli->async_send_request(request);
  // Wait for the result.
  if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    if(result.get()->success){
      RCLCPP_INFO(LOGGER, "cmd_vel_mode was switched into %s", request->cmd_vel_mode.c_str());
    }else{
      RCLCPP_ERROR(LOGGER, "Something wrong");
    }
  }else{
    RCLCPP_ERROR(LOGGER, "Failed to call service set_cmd_vel_mode");
  }
}

void MapoiPanel::MapNameCallback(std_msgs::msg::String::SharedPtr msg)
{
  if(current_map_ != msg->data){
    current_map_ = msg->data;
    SetMapComboBox(msg->data);
  }
}

void MapoiPanel::SetMapComboBox(std::string map_name)
{
  int i = 0;
  for(auto map : map_name_list_){
    if(map == map_name){
      ui_->MapComboBox->setCurrentIndex(i);
      SetDestComboBox();
      return;
    }
    i++;
  }
  i = 0;
  for(auto map : map_name_list_){
    RCLCPP_ERROR_STREAM(LOGGER, "map" << i << " : " << map << " correctly");
    i++;
  }
  RCLCPP_ERROR_STREAM(LOGGER, "Couldn't get " << map_name << " correctly");
}

void MapoiPanel::SetDestComboBox()
{
  auto node = rclcpp::Node::make_shared("mapoi_panel_get_tagged_pois_client");
  rclcpp::Client<mapoi_interfaces::srv::GetTaggedPois>::SharedPtr client_gtp =
    node->create_client<mapoi_interfaces::srv::GetTaggedPois>("get_tagged_pois");

  auto request_gtp = std::make_shared<mapoi_interfaces::srv::GetTaggedPois::Request>();
  request_gtp->tag = "destination";

  while(!client_gtp->wait_for_service(1s)){
    if(!rclcpp::ok()){
      RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(LOGGER, "service not available, waiting again...");
  }

  auto result_gtp = client_gtp->async_send_request(request_gtp);
  // Wait for the result.
  if(rclcpp::spin_until_future_complete(node, result_gtp) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto poi_info = result_gtp.get();
    auto tag_poi = poi_info->pois_list;
    RCLCPP_INFO(LOGGER, "Size of result: %ld", tag_poi.size());
    pois_ = poi_info->pois_list;
    ui_->DestinationComboBox->clear();
    for(mapoi_interfaces::msg::PointOfInterest p : pois_)
      ui_->DestinationComboBox->addItem(QString::fromStdString(p.description));
    dest_ind_ = 0;
  }else{
    RCLCPP_ERROR(LOGGER, "Failed to call service get_tagged_pois");
  }
}

}  // mapoi_rviz_plugins
CLASS_LOADER_REGISTER_CLASS(mapoi_rviz_plugins::MapoiPanel, rviz_common::Panel)
