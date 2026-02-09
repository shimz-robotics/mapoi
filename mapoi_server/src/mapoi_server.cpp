#include "mapoi_server/mapoi_server.hpp"

using namespace std::chrono_literals;

MapoiServer::MapoiServer() : Node("mapoi_server") {
  // parameters
  mapoi_server_pkg_ = ament_index_cpp::get_package_share_directory("mapoi_server");
  this->declare_parameter<std::string>("maps_path", mapoi_server_pkg_ + "/maps");
  this->declare_parameter<std::string>("map_name", "turtlebot3_world");
  this->declare_parameter<std::string>("config_file", "mapoi_config.yaml");
  this->declare_parameter<int>("pub_interval_ms", 5000);

  // initial maps path
  maps_path_ = this->get_parameter("maps_path").as_string();
  map_name_ = this->get_parameter("map_name").as_string();
  config_file_ = this->get_parameter("config_file").as_string();

  load_tag_definitions();
  load_mapoi_config_file();

  // Publish config_path_
  config_path_publisher_ = this->create_publisher<std_msgs::msg::String>("mapoi_config_path", 10);
  int pub_interval_ms = this->get_parameter("pub_interval_ms").as_int();
  timer_ = this->create_wall_timer(std::chrono::milliseconds(pub_interval_ms), [this]() {
    auto msg = std_msgs::msg::String();
    msg.data = config_path_;
    config_path_publisher_->publish(msg);
  });

  get_pois_info_service_ = this->create_service<mapoi_interfaces::srv::GetPoisInfo>("get_pois_info",
    std::bind(&MapoiServer::get_pois_info_service, this, std::placeholders::_1, std::placeholders::_2));

  get_route_pois_service_ = this->create_service<mapoi_interfaces::srv::GetRoutePois>("get_route_pois",
    std::bind(&MapoiServer::get_route_pois_service, this, std::placeholders::_1, std::placeholders::_2));

  get_maps_info_service_ = this->create_service<mapoi_interfaces::srv::GetMapsInfo>("get_maps_info",
    std::bind(&MapoiServer::get_maps_info_service, this, std::placeholders::_1, std::placeholders::_2));

  get_routes_info_service_ = this->create_service<mapoi_interfaces::srv::GetRoutesInfo>("get_routes_info",
    std::bind(&MapoiServer::get_routes_info_service, this, std::placeholders::_1, std::placeholders::_2));

  switch_map_service_ = this->create_service<mapoi_interfaces::srv::SwitchMap>("switch_map",
    std::bind(&MapoiServer::switch_map_service, this, std::placeholders::_1, std::placeholders::_2));

  reload_map_info_service_ = this->create_service<std_srvs::srv::Trigger>("reload_map_info",
    std::bind(&MapoiServer::reload_map_info_service, this, std::placeholders::_1, std::placeholders::_2));

  get_tag_definitions_srv_ = this->create_service<mapoi_interfaces::srv::GetTagDefinitions>("get_tag_definitions",
    std::bind(&MapoiServer::get_tag_definitions_service, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Ready to serve. The current map_name is %s", config_path_.c_str());
}

mapoi_interfaces::msg::PointOfInterest MapoiServer::yaml_to_poi_msg(const YAML::Node & poi)
{
  mapoi_interfaces::msg::PointOfInterest msg;
  msg.name = poi["name"].as<std::string>("");
  msg.pose.position.x = poi["pose"]["x"].as<double>(0.0);
  msg.pose.position.y = poi["pose"]["y"].as<double>(0.0);
  auto yaw = poi["pose"]["yaw"].as<double>(0.0);
  msg.pose.orientation.z = std::sin(yaw / 2.0);
  msg.pose.orientation.w = std::cos(yaw / 2.0);
  msg.radius = poi["radius"].as<double>(0.5);
  msg.tags = poi["tags"].as<std::vector<std::string>>(std::vector<std::string>{});
  msg.description = poi["description"].as<std::string>("");
  return msg;
}

void MapoiServer::load_mapoi_config_file()
{
  config_path_ = maps_path_ + "/" + map_name_ + "/" + config_file_;
  RCLCPP_INFO(this->get_logger(), "Loading mapoi config file: %s", config_path_.c_str());
  try {
    YAML::Node mapoi_config = YAML::LoadFile(config_path_);
    pois_list_ = mapoi_config["poi"] ? mapoi_config["poi"] : YAML::Node(YAML::NodeType::Sequence);
    routes_list_ = mapoi_config["route"] ? mapoi_config["route"] : YAML::Node(YAML::NodeType::Sequence);
    nav2_map_list_ = mapoi_config["map"] ? mapoi_config["map"] : YAML::Node(YAML::NodeType::Sequence);

    // Rebuild tag list: system tags + user tags from config
    tag_definitions_.clear();
    for (const auto & tag : system_tags_) {
      tag_definitions_.push_back(tag);
    }
    if (mapoi_config["custom_tags"]) {
      for (const auto & tag : mapoi_config["custom_tags"]) {
        mapoi_interfaces::msg::TagDefinition td;
        td.name = tag["name"].as<std::string>();
        td.description = tag["description"].as<std::string>("");
        td.is_system = false;
        tag_definitions_.push_back(td);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Tag definitions: %zu system + %zu user",
                system_tags_.size(), tag_definitions_.size() - system_tags_.size());
  } catch (const YAML::BadFile & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load mapoi config file: %s", e.what());
    rclcpp::shutdown();
  }
}

void MapoiServer::get_pois_info_service(const std::shared_ptr<mapoi_interfaces::srv::GetPoisInfo::Request> request,
          std::shared_ptr<mapoi_interfaces::srv::GetPoisInfo::Response> response)
{
  (void)request;
  if (!pois_list_ || !pois_list_.IsSequence()) {
    return;
  }
  for (const auto & poi : pois_list_) {
    response->pois_list.push_back(yaml_to_poi_msg(poi));
  }
  RCLCPP_DEBUG(this->get_logger(), "sending back response '%zu'", response->pois_list.size());
}

void MapoiServer::get_route_pois_service(
    const std::shared_ptr<mapoi_interfaces::srv::GetRoutePois::Request> request,
    std::shared_ptr<mapoi_interfaces::srv::GetRoutePois::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "Incoming get_route_pois request: %s", request->route_name.c_str());

  if (!routes_list_ || !routes_list_.IsSequence()) {
    return;
  }

  for (const auto &route : routes_list_) {
    if (route["name"].as<std::string>("") == request->route_name) {
      const auto &wps = route["waypoints"];
      if (!wps || !wps.IsSequence()) {
        break;
      }

      for (const auto &wp : wps) {
        std::string wp_name = wp.as<std::string>();

        if (!pois_list_ || !pois_list_.IsSequence()) {
          break;
        }
        for (const auto &poi : pois_list_) {
          if (poi["name"].as<std::string>("") == wp_name) {
            response->pois_list.push_back(yaml_to_poi_msg(poi));
          }
        }
      }

      break;
    }
  }

  // debug log
  std::string poi_names;
  bool first = true;
  for (const auto &poi : response->pois_list) {
    if (!first) poi_names += ", ";
    first = false;
    poi_names += poi.name;
  }

  RCLCPP_DEBUG(this->get_logger(), "Collected POIs in route '%s': [%s]",
               request->route_name.c_str(), poi_names.c_str());
}

void MapoiServer::get_maps_info_service(
    const std::shared_ptr<mapoi_interfaces::srv::GetMapsInfo::Request> request,
    std::shared_ptr<mapoi_interfaces::srv::GetMapsInfo::Response> response)
{
  (void)request;
  RCLCPP_DEBUG(this->get_logger(), "Incoming get_maps_info request");

  std::vector<std::string> maps_list;
  for (const auto & entry : std::filesystem::directory_iterator(maps_path_)) {
    if (entry.is_directory()) {
      maps_list.push_back(entry.path().filename().string());
    }
  }

  response->maps_list = maps_list;
  response->map_name = map_name_;

  RCLCPP_DEBUG(this->get_logger(), "sending back response '%zu' maps", response->maps_list.size());
}

void MapoiServer::get_routes_info_service(
    const std::shared_ptr<mapoi_interfaces::srv::GetRoutesInfo::Request> request,
    std::shared_ptr<mapoi_interfaces::srv::GetRoutesInfo::Response> response)
{
  (void)request;
  RCLCPP_DEBUG(this->get_logger(), "Incoming get_routes_info request");

  std::vector<std::string> routes_list;
  if (routes_list_ && routes_list_.IsSequence()) {
    for (const auto &route : routes_list_) {
      routes_list.push_back(route["name"].as<std::string>(""));
    }
  }

  response->routes_list = routes_list;
  RCLCPP_DEBUG(this->get_logger(), "sending back response '%zu' routes", response->routes_list.size());
}

// Map
bool MapoiServer::send_load_map_request(rclcpp::Node::SharedPtr node, const std::string& server_name, const std::string& map_file)
{
  auto load_map_client = node->create_client<nav2_msgs::srv::LoadMap>(server_name + "/load_map");
  auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  req->map_url = map_file;

  if (!load_map_client->wait_for_service(10s)) {
    RCLCPP_INFO(node->get_logger(), "%s/load_map service could not available.", server_name.c_str());
    return false;
  }
  auto future = load_map_client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node, future, 1s) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "[%s] Request has been accepted.", __func__);
    return true;
  } else {
    RCLCPP_ERROR(node->get_logger(), "[%s] Didn't return request.", __func__);
    return false;
  }
}

void MapoiServer::switch_map_service(const std::shared_ptr<mapoi_interfaces::srv::SwitchMap::Request> request,
          std::shared_ptr<mapoi_interfaces::srv::SwitchMap::Response> response)
{
  auto map_name_new = request->map_name;
  RCLCPP_INFO(this->get_logger(), "Incoming request: %s", map_name_new.c_str());
  if (map_name_ == map_name_new) {
    response->success = false;
    response->error_message = "The map is already " + map_name_new;
    RCLCPP_INFO(this->get_logger(), "The map is already %s", map_name_new.c_str());
    return;
  }

  map_name_ = map_name_new;
  load_mapoi_config_file();

  auto node = rclcpp::Node::make_shared("sc_map_client");
  if (!nav2_map_list_ || !nav2_map_list_.IsSequence()) {
    response->success = false;
    response->error_message = "No map entries defined in config for " + map_name_new;
    return;
  }
  for (const auto& map : nav2_map_list_) {
    auto map_url = maps_path_ + "/" + map_name_new + "/" + map["map_file"].as<std::string>("");
    bool result = MapoiServer::send_load_map_request(node, map["node_name"].as<std::string>(""), map_url);
    if (!result) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load map for %s", map_url.c_str());
      response->success = false;
      response->error_message = "Failed to load map: " + map_url;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded map for %s", map_url.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "The map was switched into %s", map_name_.c_str());
  response->success = true;
}


void MapoiServer::reload_map_info_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  load_mapoi_config_file();
  response->success = true;
  response->message = config_path_;
  RCLCPP_INFO(this->get_logger(), "Reloaded mapoi config: %s", config_path_.c_str());
}

void MapoiServer::load_tag_definitions()
{
  std::string tag_defs_path = mapoi_server_pkg_ + "/maps/tag_definitions.yaml";
  system_tags_.clear();

  try {
    YAML::Node tag_config = YAML::LoadFile(tag_defs_path);
    if (tag_config["tags"]) {
      for (const auto& tag : tag_config["tags"]) {
        mapoi_interfaces::msg::TagDefinition td;
        td.name = tag["name"].as<std::string>();
        td.description = tag["description"].as<std::string>("");
        td.is_system = true;
        system_tags_.push_back(td);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu system tag definitions from %s",
                system_tags_.size(), tag_defs_path.c_str());
  } catch (const YAML::BadFile& e) {
    RCLCPP_WARN(this->get_logger(), "System tag definitions file not found: %s. Continuing with empty list.",
                tag_defs_path.c_str());
  }
}

void MapoiServer::get_tag_definitions_service(
    const std::shared_ptr<mapoi_interfaces::srv::GetTagDefinitions::Request> request,
    std::shared_ptr<mapoi_interfaces::srv::GetTagDefinitions::Response> response)
{
  (void)request;
  response->definitions = tag_definitions_;
  RCLCPP_DEBUG(this->get_logger(), "Sending %zu tag definitions", tag_definitions_.size());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapoiServer>());
  rclcpp::shutdown();
  return 0;
}
