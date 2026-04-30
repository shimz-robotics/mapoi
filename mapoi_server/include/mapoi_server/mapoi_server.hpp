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
#include "mapoi_interfaces/msg/initial_pose_request.hpp"
#include "mapoi_interfaces/srv/get_maps_info.hpp"
#include "mapoi_interfaces/srv/get_routes_info.hpp"
#include "mapoi_interfaces/srv/switch_map.hpp"
#include "mapoi_interfaces/srv/get_tag_definitions.hpp"
#include "mapoi_interfaces/msg/tag_definition.hpp"


class MapoiServer : public rclcpp::Node
{
public:
  MapoiServer();

  // 純関数版: pois_list (YAML::Node) と requested_name から initial POI 名を決定する (#144)。
  // ロジック:
  //   1) requested_name が指定されていれば、その POI を探す
  //      - landmark タグ持ち → fall back (round 1 high)
  //      - pose ノード or x/y/yaw 欠落 / numeric 不可 → fall back (round 2 low)
  //   2) fall back: POI list 先頭で「landmark なし & pose 完備」の POI を採用
  //   3) 候補なしなら空文字列を返す
  // static にしてあるのは unit test で直接呼べるようにするため (#149 round 4 high 対応)。
  static std::string compute_initial_poi_name(
    const YAML::Node & pois_list, const std::string & requested_name);

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
  rclcpp::Publisher<mapoi_interfaces::msg::InitialPoseRequest>::SharedPtr initialpose_poi_publisher_;

  // initial pose 用の POI 名 publish (#144)。compute_initial_poi_name は class-level public static。
  void publish_initial_poi_name(const std::string & requested_name);
  // transient_local の latched 値を「採用候補なし」を示す skip message (poi_name 空) で
  // 上書きする (#154)。reload で呼び、reload 直前の古い POI 名が late subscriber に
  // 配信される問題を防ぐ。
  void publish_initialpose_clear();

  // mapoi_config_path topic の publish (transient_local QoS で latched)。起動時 / SwitchMap /
  // reload_map_info で呼ぶ。定期 publish は subscriber を transient_local に揃えて廃止 (#135)。
  void publish_config_path();

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
