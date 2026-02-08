#ifndef MAPOI_SERVER__MAPOI_NAV_SERVER_HPP_
#define MAPOI_SERVER__MAPOI_NAV_SERVER_HPP_

#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mapoi_interfaces/srv/get_pois_info.hpp"
#include "mapoi_interfaces/srv/get_route_pois.hpp"
#include "mapoi_interfaces/srv/get_tag_definitions.hpp"
#include "mapoi_interfaces/msg/point_of_interest.hpp"
#include "mapoi_interfaces/msg/poi_event.hpp"

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

  // --- POI radius event detection ---
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr radius_check_timer_;
  rclcpp::Publisher<mapoi_interfaces::msg::PoiEvent>::SharedPtr poi_event_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_path_sub_;
  rclcpp::Client<mapoi_interfaces::srv::GetTagDefinitions>::SharedPtr tag_defs_client_;

  std::string last_config_path_;
  std::unordered_set<std::string> system_tags_;
  bool system_tags_loaded_ = false;
  std::unordered_map<std::string, bool> poi_inside_state_;  // key: poi.name
  std::vector<mapoi_interfaces::msg::PointOfInterest> event_pois_;

  void fetch_system_tags();
  void on_system_tags_received(rclcpp::Client<mapoi_interfaces::srv::GetTagDefinitions>::SharedFuture future);
  void on_config_path_changed(const std_msgs::msg::String::SharedPtr msg);
  void rebuild_event_pois();
  void radius_check_callback();
  double distance_2d(const geometry_msgs::msg::Pose & poi_pose, double rx, double ry);
};

#endif  // MAPOI_SERVER__MAPOI_NAV_SERVER_HPP_
