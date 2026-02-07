#ifndef MAPOI_SERVER__MAPOI_NAV_SERVER_HPP_
#define MAPOI_SERVER__MAPOI_NAV_SERVER_HPP_

#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "mapoi_interfaces/srv/get_pois_info.hpp"
#include "mapoi_interfaces/srv/get_route_pois.hpp"
#include "mapoi_interfaces/msg/point_of_interest.hpp"

class MapoiNavServer : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  explicit MapoiNavServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --- publisher & subscriber ---
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_initialpose_poi_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr nav2_initialpose_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_goal_pose_poi_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav2_goal_pose_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_route_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_cancel_sub_;

  void mapoi_initialpose_poi_cb(const std_msgs::msg::String::SharedPtr msg);
  void mapoi_goal_pose_poi_cb(const std_msgs::msg::String::SharedPtr msg);
  void mapoi_route_cb(const std_msgs::msg::String::SharedPtr msg);
  void mapoi_cancel_cb(const std_msgs::msg::String::SharedPtr msg);

  // Service Callbacks
  void on_pois_info_received(rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedFuture future);
  void on_route_received(rclcpp::Client<mapoi_interfaces::srv::GetRoutePois>::SharedFuture future);

  void get_pois_list();

  // Action Callbacks
  void goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr & goal_handle);
  void feedback_callback(GoalHandleFollowWaypoints::SharedPtr, const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
  void result_callback(const GoalHandleFollowWaypoints::WrappedResult & result);

  // Clients
  rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
  GoalHandleFollowWaypoints::SharedPtr current_goal_handle_;
  rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedPtr pois_info_client_;
  rclcpp::Client<mapoi_interfaces::srv::GetRoutePois>::SharedPtr route_client_;

  std::vector<mapoi_interfaces::msg::PointOfInterest> pois_list_;

  // Nav status publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_status_pub_;
  void publish_nav_status(const std::string & status);
};

#endif  // MAPOI_SERVER__MAPOI_NAV_SERVER_HPP_
