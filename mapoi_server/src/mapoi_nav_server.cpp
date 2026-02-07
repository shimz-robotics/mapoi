#include "mapoi_server/mapoi_nav_server.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

MapoiNavServer::MapoiNavServer(const rclcpp::NodeOptions & options)
: Node("mapoi_nav_server", options)
{
  this->get_logger().set_level(rclcpp::Logger::Level::Info);

  // initialpose subscriber and publisher
  mapoi_initialpose_poi_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_initialpose_poi", 1, std::bind(&MapoiNavServer::mapoi_initialpose_poi_cb, this, std::placeholders::_1));
  nav2_initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1);

  // goal_pose subscriber and publisher
  mapoi_goal_pose_poi_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_goal_pose_poi", 1, std::bind(&MapoiNavServer::mapoi_goal_pose_poi_cb, this, std::placeholders::_1));
  nav2_goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);

  // route subscriber
  mapoi_route_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_route", 1, std::bind(&MapoiNavServer::mapoi_route_cb, this, std::placeholders::_1));

  // cancel subscriber
  mapoi_cancel_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_cancel", 1, std::bind(&MapoiNavServer::mapoi_cancel_cb, this, std::placeholders::_1));

  // アクションクライアントの作成
  // テンプレート引数にエイリアス FollowWaypoints を使用
  this->action_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");

  // サービスクライアントの作成
  this->pois_info_client_ = this->create_client<mapoi_interfaces::srv::GetPoisInfo>("get_pois_info");
  this->route_client_ = this->create_client<mapoi_interfaces::srv::GetRoutePois>("get_route_pois");

  RCLCPP_INFO(this->get_logger(), "MapoiNavServer initialized.");
}

void MapoiNavServer::get_pois_list(){
  while(!this->pois_info_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for pois_info service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "pois_info service not available, waiting again...");
  }
  auto pois_info_request = std::make_shared<mapoi_interfaces::srv::GetPoisInfo::Request>();
  pois_info_client_->async_send_request(
    pois_info_request, std::bind(&MapoiNavServer::on_pois_info_received, this, _1));
}

void MapoiNavServer::mapoi_initialpose_poi_cb(const std_msgs::msg::String::SharedPtr msg)
{
  get_pois_list();
  RCLCPP_INFO(this->get_logger(), "Received POI name for initialpose: %s", msg->data.c_str());

  // Find the POI in the pois_list_
  for (const auto &poi : pois_list_) {
    if (poi.name == msg->data) {
      geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
      init_pose.header.frame_id = "map";
      init_pose.pose.pose = poi.pose;
      init_pose.pose.covariance[0] = 0.25;
      init_pose.pose.covariance[7] = 0.25;
      init_pose.pose.covariance[35] = 0.06853891945200942;

      nav2_initialpose_pub_->publish(init_pose);
      RCLCPP_INFO(this->get_logger(), "Published initial pose from POI: %s", msg->data.c_str());
      return;
    }
  }
  RCLCPP_WARN(this->get_logger(), "POI named '%s' not found!", msg->data.c_str());
}

void MapoiNavServer::mapoi_goal_pose_poi_cb(const std_msgs::msg::String::SharedPtr msg)
{
  get_pois_list();
  RCLCPP_INFO(this->get_logger(), "Received POI name for goal pose: %s", msg->data.c_str());

  // Find the POI in the pois_list_
  for (const auto &poi : pois_list_) {
    if (poi.name == msg->data) {
      geometry_msgs::msg::PoseStamped goal_pose;
      goal_pose.header.frame_id = "map";
      goal_pose.pose = poi.pose;
      nav2_goal_pose_pub_->publish(goal_pose);
      RCLCPP_INFO(this->get_logger(), "Published goal pose from POI: %s", msg->data.c_str());
      return;
    }
  }
  RCLCPP_WARN(this->get_logger(), "POI named '%s' not found!", msg->data.c_str());
}

void MapoiNavServer::mapoi_route_cb(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received route name: %s", msg->data.c_str());

  auto route_request = std::make_shared<mapoi_interfaces::srv::GetRoutePois::Request>();
  route_request->route_name = msg->data;

  this->route_client_->async_send_request(
    route_request, std::bind(&MapoiNavServer::on_route_received, this, _1));
}

void MapoiNavServer::on_pois_info_received(rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedFuture future)
{
  auto result = future.get();
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get Tagged POIs.");
    return;
  }

  pois_list_ = result->pois_list;
  RCLCPP_INFO(this->get_logger(), "Received %ld Tagged POIs.", pois_list_.size());
}

void MapoiNavServer::on_route_received(rclcpp::Client<mapoi_interfaces::srv::GetRoutePois>::SharedFuture future)
{
  auto result = future.get();
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get Route info.");
    return;
  }

  const auto & route_poi = result->pois_list;
  RCLCPP_INFO(this->get_logger(), "Received Route with %ld waypoints.", route_poi.size());

  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  for (size_t i = 0; i < route_poi.size(); ++i) {
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "map";
    wp.header.stamp = this->now();
    wp.pose = route_poi[i].pose;
    waypoints.push_back(wp);
  }

  if (waypoints.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Route is empty. Cannot navigate.");
    return;
  }

  // Send waypoints to Nav2 with action client
  while(!this->action_client_->wait_for_action_server(1s)) {
        if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for action server. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Action server not available, waiting again...");
  }
  RCLCPP_INFO(this->get_logger(), "Sending goal with %ld waypoints.", waypoints.size());

  auto goal_msg = FollowWaypoints::Goal();
  goal_msg.poses = waypoints;

  auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
  
  send_goal_options.goal_response_callback =
    std::bind(&MapoiNavServer::goal_response_callback, this, _1);
  
  send_goal_options.feedback_callback =
    std::bind(&MapoiNavServer::feedback_callback, this, _1, _2);
  
  send_goal_options.result_callback =
    std::bind(&MapoiNavServer::result_callback, this, _1);

  this->action_client_->async_send_goal(goal_msg, send_goal_options);
}

void MapoiNavServer::goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    current_goal_handle_ = goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void MapoiNavServer::feedback_callback(
  GoalHandleFollowWaypoints::SharedPtr,
  const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
{
  // RCLCPP_INFO(this->get_logger(), "Current Waypoint Index: %u", feedback->current_waypoint);
}

void MapoiNavServer::result_callback(const GoalHandleFollowWaypoints::WrappedResult & result)
{
  current_goal_handle_.reset();
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "✅ Navigation SUCCEEDED!");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "❌ Navigation ABORTED");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "⚠️ Navigation CANCELED");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "❓ Unknown result code");
      break;
  }
}

void MapoiNavServer::mapoi_cancel_cb(const std_msgs::msg::String::SharedPtr msg)
{
  (void)msg;
  if (current_goal_handle_) {
    RCLCPP_INFO(this->get_logger(), "Canceling current navigation goal...");
    action_client_->async_cancel_goal(current_goal_handle_);
  } else {
    RCLCPP_WARN(this->get_logger(), "No active navigation goal to cancel.");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapoiNavServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
