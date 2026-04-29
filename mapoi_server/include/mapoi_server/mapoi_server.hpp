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
#include <std_srvs/srv/trigger.hpp>

#include "mapoi_interfaces/srv/get_pois_info.hpp"
#include "mapoi_interfaces/srv/get_route_pois.hpp"
#include "mapoi_interfaces/msg/point_of_interest.hpp"
#include "mapoi_interfaces/srv/get_maps_info.hpp"
#include "mapoi_interfaces/srv/get_routes_info.hpp"
#include "mapoi_interfaces/srv/switch_map.hpp"
#include "mapoi_interfaces/srv/get_tag_definitions.hpp"
#include "mapoi_interfaces/msg/tag_definition.hpp"


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
  // initial pose POI 名 publisher (#144): mapoi_server が新 map の初期 POI 名を決定して
  // mapoi_initialpose_poi topic に publish。mapoi_nav_server がそれを受けて /initialpose を流す。
  // QoS: transient_local (depth=1) で後起動 subscriber でも受信できる。
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr initialpose_poi_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // SwitchMap で受け取った initial_poi_name を load_mapoi_config_file 後の publish_initial_poi_name
  // で消費するためのバッファ。空文字列なら POI list 先頭 (landmark / pose 欠落 を除外) を default
  // 採用 (#144)。**single-thread executor 前提** (rclcpp::spin) で、SwitchMap callee と
  // コンストラクタからの呼び出しが直列化される想定。MultiThreaded executor に移す場合は
  // mutex 保護が必要 (Cursor review #149 medium 対応)。
  std::string pending_initial_poi_name_;
  void publish_initial_poi_name();
  std::string compute_initial_poi_name(const std::string & requested_name) const;

  // methods
  void load_mapoi_config_file();
  void load_tag_definitions();
  YAML::Node pois_list_;
  YAML::Node routes_list_;
  YAML::Node nav2_map_list_;

  // tag definitions (system tags loaded once, user tags merged on config load)
  std::vector<mapoi_interfaces::msg::TagDefinition> system_tags_;
  std::vector<mapoi_interfaces::msg::TagDefinition> tag_definitions_;

  // POI YAML conversion helper
  mapoi_interfaces::msg::PointOfInterest yaml_to_poi_msg(const YAML::Node & poi);

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

};
