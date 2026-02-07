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
  
  switch_map_service_ = this->create_service<mapoi_interfaces::srv::SwitchMap>("switch_map",
    std::bind(&MapoiServer::switch_map_service, this, std::placeholders::_1, std::placeholders::_2));

  reload_map_info_service_ = this->create_service<std_srvs::srv::Trigger>("reload_map_info",
    std::bind(&MapoiServer::reload_map_info_service, this, std::placeholders::_1, std::placeholders::_2));

  nav2_initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1);

  RCLCPP_INFO(this->get_logger(), "Ready to serve. The current map_name is %s", config_path_.c_str());
}

void MapoiServer::load_mapoi_config_file()
{
  config_path_ = maps_path_ + "/" + map_name_ + "/" + config_file_;
  RCLCPP_INFO(this->get_logger(), "Loading mapoi config file: %s", config_path_.c_str());
  try {
    YAML::Node mapoi_config = YAML::LoadFile(config_path_);
    pois_list_ = mapoi_config["poi"];
    routes_list_ = mapoi_config["route"];
    nav2_map_list_ = mapoi_config["map"];
  } catch (const YAML::BadFile & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load mapoi config file: %s", e.what());
    rclcpp::shutdown();
  }
}

void MapoiServer::get_pois_info_service(const std::shared_ptr<mapoi_interfaces::srv::GetPoisInfo::Request> request,
          std::shared_ptr<mapoi_interfaces::srv::GetPoisInfo::Response> response)
{
  for(auto poi : pois_list_){
    mapoi_interfaces::msg::PointOfInterest single_poi;
    single_poi.name = poi["name"].as<std::string>();
    single_poi.pose.position.x = poi["pose"]["x"].as<std::double_t>();
    single_poi.pose.position.y = poi["pose"]["y"].as<std::double_t>();
    auto yaw = poi["pose"]["yaw"].as<std::double_t>();
    single_poi.pose.orientation.z = sin(yaw/2.0);
    single_poi.pose.orientation.w = cos(yaw/2.0);
    single_poi.radius = poi["radius"].as<std::double_t>();
    single_poi.tags = poi["tags"].as<std::vector<std::string>>();
    single_poi.description = poi["description"].as<std::string>();
    response->pois_list.push_back(single_poi);
  }
  RCLCPP_DEBUG(this->get_logger(), "sending back response '%ld'", response->pois_list.size());
}

void MapoiServer::get_route_pois_service(
    const std::shared_ptr<mapoi_interfaces::srv::GetRoutePois::Request> request,
    std::shared_ptr<mapoi_interfaces::srv::GetRoutePois::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "Incoming get_route_pois request: %s", request->route_name.c_str());

  // routes_list_ の中から route_name に一致するルートを探す
  for (const auto &route : routes_list_) {
    if (route["name"].as<std::string>() == request->route_name) {

      // ルート内の waypoints を取得
      const auto &wps = route["waypoints"];

      // waypoints は文字列のリスト（POI の name）であると仮定
      for (const auto &wp : wps) {
        std::string wp_name = wp.as<std::string>();

        // pois_list_ から wp_name に一致する POI を探して response に追加
        for (const auto &poi : pois_list_) {
          if (poi["name"].as<std::string>() == wp_name) {
            mapoi_interfaces::msg::PointOfInterest single_poi;
            single_poi.name = poi["name"].as<std::string>();
            single_poi.pose.position.x = poi["pose"]["x"].as<double>();
            single_poi.pose.position.y = poi["pose"]["y"].as<double>();
            auto yaw = poi["pose"]["yaw"].as<double>();
            single_poi.pose.orientation.z = std::sin(yaw / 2.0);
            single_poi.pose.orientation.w = std::cos(yaw / 2.0);
            single_poi.radius = poi["radius"].as<double>();
            single_poi.tags = poi["tags"].as<std::vector<std::string>>();
            single_poi.description = poi["description"].as<std::string>();
            response->pois_list.push_back(single_poi);
          }
        }
      }

      break;
    }
  }

  // デバッグログ用に POI 名一覧を作成
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
  RCLCPP_DEBUG(this->get_logger(), "Incoming get_maps_info request");

  // maps_path_ ディレクトリ内のサブディレクトリを調べる
  std::vector<std::string> maps_list;
  for (const auto & entry : std::filesystem::directory_iterator(maps_path_)) {
    if (entry.is_directory()) {
      maps_list.push_back(entry.path().filename().string());
    }
  }

  response->maps_list = maps_list;
  response->map_name = map_name_;

  RCLCPP_DEBUG(this->get_logger(), "sending back response '%ld' maps", response->maps_list.size());
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
    RCLCPP_INFO(this->get_logger(), "The map is already %s", map_name_new.c_str());
    return;
  }

  map_name_ = map_name_new;
  load_mapoi_config_file();

  auto node = rclcpp::Node::make_shared("sc_map_client");
  for (const auto& map : nav2_map_list_) {
    auto map_url = maps_path_ + "/" + map_name_new + "/" + map["map_file"].as<std::string>();
    bool result = MapoiServer::send_load_map_request(node, map["node_name"].as<std::string>(), map_url);
    if (!result) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load map for %s", map_url.c_str());
      response->success = false;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded map for %s", map_url.c_str());
  }

  // send initial pose
  for (const auto& poi: pois_list_) {
    // tags に initial_pose が含まれている POI を探す
    auto tags = poi["tags"].as<std::vector<std::string>>();
    if (std::find(tags.begin(), tags.end(), "initial_pose") != tags.end()) {
      geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
      init_pose.header.frame_id = "map";
      init_pose.pose.pose.position.x = poi["pose"]["x"].as<std::double_t>();
      init_pose.pose.pose.position.y = poi["pose"]["y"].as<std::double_t>();
      auto yaw = poi["pose"]["yaw"].as<std::double_t>();
      init_pose.pose.pose.orientation.z = sin(yaw/2.0);
      init_pose.pose.pose.orientation.w = cos(yaw/2.0);
      init_pose.pose.covariance[0] = 0.25;
      init_pose.pose.covariance[7] = 0.25;
      init_pose.pose.covariance[35] = 0.06853891945200942;
      nav2_initialpose_pub_->publish(init_pose);
      RCLCPP_INFO(this->get_logger(), "Publish initial pose");
    }
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapoiServer>());
  rclcpp::shutdown();
  return 0;
}