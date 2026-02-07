#pragma once

#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <filesystem>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "mapoi_interfaces/srv/get_pois_info.hpp"
#include "mapoi_interfaces/srv/get_route_pois.hpp"
#include "mapoi_interfaces/msg/point_of_interest.hpp"
#include "mapoi_interfaces/srv/get_maps_info.hpp"
#include "mapoi_interfaces/srv/get_routes_info.hpp"
#include "mapoi_interfaces/srv/switch_map.hpp"
#include "mapoi_interfaces/srv/get_tag_definitions.hpp"


class MapoiServer : public rclcpp::Node
{
public:
  MapoiServer();

private:
  // parameters & internal state
  std::string mapoi_server_pkg_;
  std::string maps_path_;
  std::string map_name_;
  std::string config_file_;
  std::string config_path_;

  // publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr config_path_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // methods
  void load_mapoi_config_file();
  void load_tag_definitions();
  YAML::Node pois_list_;
  YAML::Node routes_list_;
  YAML::Node nav2_map_list_;

  // tag definitions
  std::vector<std::string> tag_names_;
  std::vector<std::string> tag_descriptions_;
  std::vector<bool> tag_is_system_;

  // services
  rclcpp::Service<mapoi_interfaces::srv::GetPoisInfo>::SharedPtr get_pois_info_service_;
  rclcpp::Service<mapoi_interfaces::srv::GetRoutePois>::SharedPtr get_route_pois_service_;
  rclcpp::Service<mapoi_interfaces::srv::GetMapsInfo>::SharedPtr get_maps_info_service_;
  rclcpp::Service<mapoi_interfaces::srv::GetRoutesInfo>::SharedPtr get_routes_info_service_;
  rclcpp::Service<mapoi_interfaces::srv::SwitchMap>::SharedPtr switch_map_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_map_info_service_;
  rclcpp::Service<mapoi_interfaces::srv::GetTagDefinitions>::SharedPtr get_tag_definitions_srv_;

  // callbacks
  void get_pois_info_service(
    const std::shared_ptr<mapoi_interfaces::srv::GetPoisInfo::Request> request,
    std::shared_ptr<mapoi_interfaces::srv::GetPoisInfo::Response> response);
  void get_route_pois_service(
    const std::shared_ptr<mapoi_interfaces::srv::GetRoutePois::Request> request,
    std::shared_ptr<mapoi_interfaces::srv::GetRoutePois::Response> response);
  void get_maps_info_service(
    const std::shared_ptr<mapoi_interfaces::srv::GetMapsInfo::Request> request,
    std::shared_ptr<mapoi_interfaces::srv::GetMapsInfo::Response> response);
  void get_routes_info_service(
    const std::shared_ptr<mapoi_interfaces::srv::GetRoutesInfo::Request> request,
    std::shared_ptr<mapoi_interfaces::srv::GetRoutesInfo::Response> response);
  void switch_map_service(
    const std::shared_ptr<mapoi_interfaces::srv::SwitchMap::Request> request,
    std::shared_ptr<mapoi_interfaces::srv::SwitchMap::Response> response);
  void reload_map_info_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void get_tag_definitions_service(
    const std::shared_ptr<mapoi_interfaces::srv::GetTagDefinitions::Request> request,
    std::shared_ptr<mapoi_interfaces::srv::GetTagDefinitions::Response> response);

  // helper functions
  bool send_load_map_request(
    rclcpp::Node::SharedPtr node,
    const std::string& server_name,
    const std::string& map_file);

  // publisher for initialpose
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr nav2_initialpose_pub_;
};
