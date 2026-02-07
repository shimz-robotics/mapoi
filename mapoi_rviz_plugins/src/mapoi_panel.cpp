#include "mapoi_rviz_plugins/mapoi_panel.hpp"
#include <class_loader/class_loader.hpp>
#include <filesystem>
#include <set>

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
  connect(ui_->Nav2GoalComboBox, SIGNAL(activated(int)), this, SLOT(Nav2GoalComboBox()));
  connect(ui_->MapoiRouteComboBox, SIGNAL(activated(int)), this, SLOT(MapoiRouteComboBox()));

  connect(ui_->LocalizationButton, SIGNAL(clicked()), this, SLOT(LocalizationButton()));
  connect(ui_->RunGoalButton, SIGNAL(clicked()), this, SLOT(RunGoalButton()));
  connect(ui_->RunRouteButton, SIGNAL(clicked()), this, SLOT(RunRouteButton()));
  connect(ui_->StopButton, SIGNAL(clicked()), this, SLOT(StopButton()));

  parentWidget()->setVisible(true);

  // map ComboBox
  auto node = rclcpp::Node::make_shared("mapoi_panel_get_maps_info_client");
  auto get_maps_info_cli = node->create_client<mapoi_interfaces::srv::GetMapsInfo>("get_maps_info");
  while(!get_maps_info_cli->wait_for_service(1s)){
    if(!rclcpp::ok()){
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }
  auto request = std::make_shared<mapoi_interfaces::srv::GetMapsInfo::Request>();
  auto result = get_maps_info_cli->async_send_request(request);
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
    RCLCPP_ERROR(node->get_logger(), "Failed to call service get_pois_info");
  }

  // Buttons
  nav2_initialpose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1);
  nav2_goal_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);

  mapoi_cancel_pub_ = node_->create_publisher<std_msgs::msg::String>("mapoi_cancel", 1);
  mapoi_route_pub_ = node_->create_publisher<std_msgs::msg::String>("mapoi_route", 1);
  mapoi_highlight_goal_pub_ = node_->create_publisher<std_msgs::msg::String>("mapoi_highlight_goal", 1);
  mapoi_highlight_route_pub_ = node_->create_publisher<std_msgs::msg::String>("mapoi_highlight_route", 1);

  config_path_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "mapoi_config_path", 10,
      std::bind(&MapoiPanel::ConfigPathCallback, this, std::placeholders::_1));

  nav_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "mapoi_nav_status", 10,
      std::bind(&MapoiPanel::NavStatusCallback, this, std::placeholders::_1));

  current_nav_mode_ = "idle";

  // Mapoi Route ComboBox
  SetMapoiRouteComboBox();
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

void MapoiPanel::Nav2GoalComboBox()
{
  goal_combobox_ind_ = ui_->Nav2GoalComboBox->currentIndex();
  if (goal_combobox_ind_ >= 0 && goal_combobox_ind_ < static_cast<int>(pois_.size())) {
    highlighted_goal_name_ = pois_[goal_combobox_ind_].name;
  } else {
    highlighted_goal_name_.clear();
  }
  PublishHighlightPois();
}

void MapoiPanel::MapoiRouteComboBox()
{
  route_combobox_ind_ = ui_->MapoiRouteComboBox->currentIndex();
  highlighted_route_poi_names_.clear();
  if (route_combobox_ind_ < 1) {
    // "(なし)" selected — clear highlight
    PublishHighlightPois();
    return;
  }
  int route_index = route_combobox_ind_ - 1;
  if (route_index >= 0 && route_index < static_cast<int>(route_name_list_.size())) {
    auto node = rclcpp::Node::make_shared("mapoi_panel_get_route_pois_client");
    auto client = node->create_client<mapoi_interfaces::srv::GetRoutePois>("get_route_pois");
    if (client->wait_for_service(1s)) {
      auto request = std::make_shared<mapoi_interfaces::srv::GetRoutePois::Request>();
      request->route_name = route_name_list_[route_index];
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result, 5s) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result.get();
        if (response) {
          for (const auto & poi : response->pois_list) {
            highlighted_route_poi_names_.push_back(poi.name);
          }
        }
      }
    }
  }
  PublishHighlightPois();
}

void MapoiPanel::LocalizationButton()
{  
  geometry_msgs::msg::PoseWithCovarianceStamped msg_init;
  msg_init.header.stamp = rclcpp::Clock().now();
  msg_init.header.frame_id = "map";
  msg_init.pose.pose = pois_[goal_combobox_ind_].pose;
  msg_init.pose.covariance[0] = 0.25;
  msg_init.pose.covariance[7] = 0.25;
  msg_init.pose.covariance[35] = 0.06853891945200942;

  nav2_initialpose_pub_->publish(msg_init);
  RCLCPP_INFO(LOGGER, "A initialpose was set.");
}

void MapoiPanel::RunGoalButton()
{
  if (goal_combobox_ind_ < 0 || goal_combobox_ind_ >= static_cast<int>(pois_.size())) {
    return;
  }
  geometry_msgs::msg::PoseStamped msg_goal;
  msg_goal.header.stamp = rclcpp::Clock().now();
  msg_goal.header.frame_id = "map";
  msg_goal.pose = pois_[goal_combobox_ind_].pose;
  nav2_goal_pose_pub_->publish(msg_goal);

  current_nav_mode_ = "goal";
  current_nav_target_ = pois_[goal_combobox_ind_].name;
  ui_->NavStatusLabel->setText(
      QString::fromStdString("目的地走行中: " + current_nav_target_));
  RCLCPP_INFO(LOGGER, "A goal pose was set: %s", current_nav_target_.c_str());
}

void MapoiPanel::RunRouteButton()
{
  if (route_combobox_ind_ < 1) {
    return;
  }
  int route_index = route_combobox_ind_ - 1;
  if (route_index >= static_cast<int>(route_name_list_.size())) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = route_name_list_[route_index];
  mapoi_route_pub_->publish(msg);

  current_nav_mode_ = "route";
  current_nav_target_ = route_name_list_[route_index];
  ui_->NavStatusLabel->setText(
      QString::fromStdString("ルート走行中: " + current_nav_target_));
  RCLCPP_INFO(LOGGER, "A route was set: %s", msg.data.c_str());
}

void MapoiPanel::StopButton()
{
  std_msgs::msg::String msg;
  msg.data = "mapoi_panel";
  mapoi_cancel_pub_->publish(msg);

  current_nav_mode_ = "idle";
  ui_->NavStatusLabel->setText(QString::fromStdString("走行キャンセル"));
  RCLCPP_INFO(LOGGER, "The robot was stopped");
}

void MapoiPanel::ConfigPathCallback(std_msgs::msg::String::SharedPtr msg)
{
  std::filesystem::path config_path(msg->data);
  std::string map_name = config_path.parent_path().filename().string();
  if(current_map_ != map_name){
    current_map_ = map_name;
    SetMapComboBox(map_name);
  }
}

void MapoiPanel::SetMapComboBox(std::string map_name)
{
  highlighted_goal_name_.clear();
  highlighted_route_poi_names_.clear();
  PublishHighlightPois();

  int i = 0;
  for(auto map : map_name_list_){
    if(map == map_name){
      ui_->MapComboBox->setCurrentIndex(i);
      SetNav2GoalComboBox();
      SetMapoiRouteComboBox();
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

void MapoiPanel::SetNav2GoalComboBox()
{
  auto node = rclcpp::Node::make_shared("mapoi_panel_get_pois_info_client");
  rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedPtr client_gtp =
    node->create_client<mapoi_interfaces::srv::GetPoisInfo>("get_pois_info");

  auto request_gtp = std::make_shared<mapoi_interfaces::srv::GetPoisInfo::Request>();

  while(!client_gtp->wait_for_service(1s)){
    if(!rclcpp::ok()){
      RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(LOGGER, "service not available, waiting again...");
  }

  auto result_get_pois_info = client_gtp->async_send_request(request_gtp);
  // Wait for the result.
  if(rclcpp::spin_until_future_complete(node, result_get_pois_info) == rclcpp::FutureReturnCode::SUCCESS)
  {

    auto pois_all = result_get_pois_info.get()->pois_list;
    ui_->Nav2GoalComboBox->clear();
    pois_.clear();

    for(mapoi_interfaces::msg::PointOfInterest p : pois_all){
      // もしpoiがgoalかwaypointタグを持っていたらpois_に追加
      auto tags = p.tags;
      bool is_goal_or_waypoint = false;
      for(auto tag : tags){
        if(tag == "goal" || tag == "waypoint"){
          is_goal_or_waypoint = true;
          break;
        }
      }
      if(is_goal_or_waypoint){
        pois_.push_back(p);
        ui_->Nav2GoalComboBox->addItem(QString::fromStdString(p.name));
      }
    }
    goal_combobox_ind_ = 0;
  }else{
    RCLCPP_ERROR(LOGGER, "Failed to call service get_pois_info");
  }
}

void MapoiPanel::SetMapoiRouteComboBox()
{
  auto node = rclcpp::Node::make_shared("mapoi_panel_get_routes_info_client");
  auto client = node->create_client<mapoi_interfaces::srv::GetRoutesInfo>("get_routes_info");

  while(!client->wait_for_service(1s)){
    if(!rclcpp::ok()){
      RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(LOGGER, "get_routes_info service not available, waiting again...");
  }

  auto request = std::make_shared<mapoi_interfaces::srv::GetRoutesInfo::Request>();
  auto result = client->async_send_request(request);
  if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    ui_->MapoiRouteComboBox->clear();
    ui_->MapoiRouteComboBox->addItem(QString::fromStdString("(なし)"));
    route_name_list_ = result.get()->routes_list;
    for(const auto & route : route_name_list_){
      ui_->MapoiRouteComboBox->addItem(QString::fromStdString(route));
    }
    route_combobox_ind_ = 0;
  }else{
    RCLCPP_ERROR(LOGGER, "Failed to call service get_routes_info");
  }
}

void MapoiPanel::NavStatusCallback(std_msgs::msg::String::SharedPtr msg)
{
  const auto & status = msg->data;
  if (status == "navigating") {
    if (current_nav_mode_ == "route") {
      ui_->NavStatusLabel->setText(
          QString::fromStdString("ルート走行中: " + current_nav_target_));
    }
  } else if (status == "succeeded") {
    current_nav_mode_ = "idle";
    ui_->NavStatusLabel->setText(QString::fromStdString("到着"));
  } else if (status == "aborted") {
    current_nav_mode_ = "idle";
    ui_->NavStatusLabel->setText(QString::fromStdString("走行失敗"));
  } else if (status == "canceled") {
    current_nav_mode_ = "idle";
    ui_->NavStatusLabel->setText(QString::fromStdString("走行キャンセル"));
  }
}

void MapoiPanel::PublishHighlightPois()
{
  if (mapoi_highlight_goal_pub_) {
    std_msgs::msg::String goal_msg;
    goal_msg.data = highlighted_goal_name_;
    mapoi_highlight_goal_pub_->publish(goal_msg);
  }

  if (mapoi_highlight_route_pub_) {
    std_msgs::msg::String route_msg;
    for (const auto & name : highlighted_route_poi_names_) {
      if (!route_msg.data.empty()) {
        route_msg.data += ",";
      }
      route_msg.data += name;
    }
    mapoi_highlight_route_pub_->publish(route_msg);
  }
}

}  // mapoi_rviz_plugins
CLASS_LOADER_REGISTER_CLASS(mapoi_rviz_plugins::MapoiPanel, rviz_common::Panel)
