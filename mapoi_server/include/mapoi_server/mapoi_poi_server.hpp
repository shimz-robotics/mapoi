#pragma once

#include <string>
#include <vector>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "yaml-cpp/yaml.h"

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

  // --- callbacks ---
  void get_tagged_pois_service(
      const std::shared_ptr<mapoi_interfaces::srv::GetTaggedPois::Request> request,
      std::shared_ptr<mapoi_interfaces::srv::GetTaggedPois::Response> response);
  void get_route_pois_service(
      const std::shared_ptr<mapoi_interfaces::srv::GetRoutePois::Request> request,
      std::shared_ptr<mapoi_interfaces::srv::GetRoutePois::Response> response);
};
