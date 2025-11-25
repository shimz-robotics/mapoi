
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <dirent.h>
#include <filesystem>
namespace fs = std::filesystem;

#include <nav2_msgs/srv/load_map.hpp>

#include "mapoi_interfaces/srv/switch_map.hpp"
#include "mapoi_interfaces/srv/get_tagged_pois.hpp"
#include "mapoi_interfaces/srv/get_map_info.hpp"

class MapoiServer : public rclcpp::Node {
public:
  MapoiServer();
 
private:
  std::string mapoi_server_pkg_;

  std::string localization_map_server_;
  std::string pathplanning_map_server_;

  std::string current_maps_path_;
  std::vector<std::string> map_list_;
  std::string current_map_;
  YAML::Node pois_list_;
  std::string model_name_;
  int id_buf_;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr warp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_dest_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_event_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<rclcpp::Node> node_client_;
  rclcpp::Client<mapoi_interfaces::srv::GetTaggedPois>::SharedPtr sc_map_client_;

  rclcpp::Service<mapoi_interfaces::srv::SwitchMap>::SharedPtr switch_map_service_;
  rclcpp::Service<mapoi_interfaces::srv::GetTaggedPois>::SharedPtr get_tagged_pois_service_;
  rclcpp::Service<mapoi_interfaces::srv::GetMapInfo>::SharedPtr get_map_info_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_map_info_service_;

  void switch_map_service(const std::shared_ptr<mapoi_interfaces::srv::SwitchMap::Request> request,
          std::shared_ptr<mapoi_interfaces::srv::SwitchMap::Response> response);
  void get_tagged_pois_service(const std::shared_ptr<mapoi_interfaces::srv::GetTaggedPois::Request> request,
          std::shared_ptr<mapoi_interfaces::srv::GetTaggedPois::Response> response);
  void get_map_info_service(const std::shared_ptr<mapoi_interfaces::srv::GetMapInfo::Request> request,
          std::shared_ptr<mapoi_interfaces::srv::GetMapInfo::Response> response);
  void reload_map_info_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void timer_callback();

  void send_reload_map_request(rclcpp::Node::SharedPtr node, const std::string& server_name, const std::string& map_file);
  void send_load_map_info_request();
};
