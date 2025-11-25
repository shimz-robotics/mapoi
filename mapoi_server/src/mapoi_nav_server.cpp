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

  this->declare_parameter("route_name", "route_1");
  this->declare_parameter("poi_tag", "all");

  // アクションクライアントの作成
  // テンプレート引数にエイリアス FollowWaypoints を使用
  this->action_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");

  // サービスクライアントの作成
  this->route_client_ = this->create_client<mapoi_interfaces::srv::GetRoutePois>("get_route_pois");
  while(!this->route_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for route_pois service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "route_pois service not available, waiting again...");
  }
  auto route_request = std::make_shared<mapoi_interfaces::srv::GetRoutePois::Request>();
  route_request->route_name = this->get_parameter("route_name").as_string();

  RCLCPP_INFO(this->get_logger(), "Requesting Route Info for: %s", route_request->route_name.c_str());

  route_client_->async_send_request(
    route_request, std::bind(&MapoiNavServer::on_route_received, this, _1));

  RCLCPP_INFO(this->get_logger(), "MapoiNavServer initialized.");
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

  this->send_waypoints(waypoints);
}

void MapoiNavServer::send_waypoints(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
{
  if (!this->action_client_->wait_for_action_server(1s)) {
    RCLCPP_WARN(this->get_logger(), "Action server not available. Retrying in 1s...");
    if(!this->retry_timer_) {
        this->retry_timer_ = this->create_wall_timer(
            1s, [this, waypoints]() { this->send_waypoints(waypoints); });
    }
    return;
  }

  if(this->retry_timer_) {
      this->retry_timer_->cancel();
      this->retry_timer_.reset();
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
  
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapoiNavServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
