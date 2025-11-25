#pragma once

#include <string>
#include <vector>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "yaml-cpp/yaml.h"

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "mapoi_interfaces/srv/get_tagged_pois.hpp"
#include "mapoi_interfaces/srv/get_route_pois.hpp"
#include "mapoi_interfaces/msg/point_of_interest.hpp"
#include "mapoi_interfaces/action/follow_poi_list.hpp"

class MapoiPoiServer : public rclcpp::Node
{
public:
  MapoiPoiServer();

private:
  // --- parameters & internal state ---
  std::string mapoi_server_pkg_;
  std::string current_map_info_file_;
  std::string current_map_;

  YAML::Node pois_list_;
  YAML::Node routes_list_;

  // --- service ---
  rclcpp::Service<mapoi_interfaces::srv::GetTaggedPois>::SharedPtr get_tagged_pois_service_;
  rclcpp::Service<mapoi_interfaces::srv::GetRoutePois>::SharedPtr get_route_pois_service_;

  // --- publisher & subscriber ---
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_initialpose_poi_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr nav2_initialpose_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_goal_pose_poi_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav2_goal_pose_pub_;

  // --- callbacks ---
  void get_tagged_pois_service(
      const std::shared_ptr<mapoi_interfaces::srv::GetTaggedPois::Request> request,
      std::shared_ptr<mapoi_interfaces::srv::GetTaggedPois::Response> response);
  void get_route_pois_service(
      const std::shared_ptr<mapoi_interfaces::srv::GetRoutePois::Request> request,
      std::shared_ptr<mapoi_interfaces::srv::GetRoutePois::Response> response);
  void mapoi_initialpose_poi_cb(const std_msgs::msg::String::SharedPtr msg);
  void mapoi_goal_pose_poi_cb(const std_msgs::msg::String::SharedPtr msg);
};
