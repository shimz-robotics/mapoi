// 以下のソースコードからPublisherやSubscriber、serviceなどの情報を抽出してリストにまとめてください。

#include "mapoi_server/mapoi_server.hpp"

using namespace std::chrono_literals;

MapoiServer::MapoiServer() : Node("mapoi_server") {
  mapoi_server_pkg_ = ament_index_cpp::get_package_share_directory("mapoi_server");

  // parameters
  this->declare_parameter<std::string>("pathplanning_map_server", "map_server");
  pathplanning_map_server_ = this->get_parameter("pathplanning_map_server").as_string();
  this->declare_parameter<std::string>("localization_map_server", "map_server_localization");
  localization_map_server_ = this->get_parameter("localization_map_server").as_string();

  // initial maps path
  this->declare_parameter<std::string>("maps_path", mapoi_server_pkg_ + "/maps");
  current_maps_path_ = this->get_parameter("maps_path").as_string();
  RCLCPP_INFO(this->get_logger(), "Maps directory: %s", current_maps_path_.c_str());

  // get map list from maps_path
  for (const auto & entry : fs::directory_iterator(current_maps_path_)) {
    if (entry.is_directory()) {
      map_list_.push_back(entry.path().filename().string());
    }
  }
  if (map_list_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No maps found in directory: %s", current_maps_path_.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Found %zu maps:", map_list_.size());
  for (const auto & map_name : map_list_) {
    RCLCPP_INFO(this->get_logger(), "  - %s", map_name.c_str());
  }

  // initial map
  this->declare_parameter<std::string>("map_name", map_list_[0]);
  current_map_ = this->get_parameter("map_name").as_string();
  this->send_load_map_info_request();

  RCLCPP_INFO(this->get_logger(), "Ready to serve. The current map_name is %s", current_map_.c_str());
}

void MapoiServer::send_reload_map_request(rclcpp::Node::SharedPtr node, const std::string& server_name, const std::string& map_file)
{
  auto client = node->create_client<nav2_msgs::srv::LoadMap>(server_name + "/load_map");
  auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  req->map_url = map_file;

  while (!client->wait_for_service(1s)) {
    RCLCPP_INFO(node->get_logger(), "%s service not available, waiting again...", server_name.c_str());
  }
  auto future = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node, future, 1s) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "[%s] Request has been accepted.", __func__);
  } else {
    RCLCPP_ERROR(node->get_logger(), "[%s] Didn't return request.", __func__);
  }
}

void MapoiServer::switch_map_service(const std::shared_ptr<mapoi_interfaces::srv::SwitchMap::Request> request,
          std::shared_ptr<mapoi_interfaces::srv::SwitchMap::Response> response)
{
  auto new_map = request->map_name;
  RCLCPP_INFO(this->get_logger(), "Incoming request: %s", new_map.c_str());
  if (current_map_ == new_map) {
    response->success = false;
    RCLCPP_INFO(this->get_logger(), "The map is already %s", new_map.c_str());
    return;
  }

  auto node = rclcpp::Node::make_shared("sc_map_client");

  std::string map_path = current_maps_path_ + "/" + new_map + "/";

  std::string map_file = map_path + "map_info.yaml";
  RCLCPP_INFO(this->get_logger(), "Map info URL: %s", map_file.c_str());
  auto map_info = YAML::Node();
  try{
    map_info = YAML::LoadFile(map_file);
  }catch(...){
      RCLCPP_ERROR(this->get_logger(), "Failed to open map_file : \n%s", map_file.c_str());
      response->success = false;
  }
  pois_list_ = map_info["poi"];

  std::string pathplanning_url = map_path + map_info["map_pathplanning"].as<std::string>() + ".yaml";
  std::string localization_url = map_path + map_info["map_localization"].as<std::string>() + ".yaml";

  RCLCPP_INFO(this->get_logger(), "Path planning URL:\n%s", pathplanning_url.c_str());
  RCLCPP_INFO(this->get_logger(), "Localization URL:\n%s", localization_url.c_str());

  MapoiServer::send_reload_map_request(node, localization_map_server_, pathplanning_url);
  if (localization_url.find(".pcd") == std::string::npos) {
    MapoiServer::send_reload_map_request(node, pathplanning_map_server_, localization_url);
  }

  // send initial pose
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
  init_pose.header.frame_id = "map";
  init_pose.pose.pose.position.x = map_info["initial_pose"]["x"].as<std::double_t>();
  init_pose.pose.pose.position.y = map_info["initial_pose"]["y"].as<std::double_t>();
  auto yaw = map_info["initial_pose"]["yaw"].as<std::double_t>();
  init_pose.pose.pose.orientation.z = sin(yaw/2.0);
  init_pose.pose.pose.orientation.w = cos(yaw/2.0);
  init_pose.pose.covariance[0] = 0.25;
  init_pose.pose.covariance[7] = 0.25;
  init_pose.pose.covariance[35] = 0.06853891945200942;
  init_pose_pub_->publish(init_pose);
  RCLCPP_INFO(this->get_logger(), "Publish initial pose");

  current_map_ = new_map;
  RCLCPP_INFO(this->get_logger(), "The map was switched into %s", current_map_.c_str());
  response->success = true;
}

void MapoiServer::get_map_info_service(const std::shared_ptr<mapoi_interfaces::srv::GetMapInfo::Request> request,
          std::shared_ptr<mapoi_interfaces::srv::GetMapInfo::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Incoming get_map_list_info request");
  response->current_maps_path = current_maps_path_;
  response->current_map = current_map_;
  for (const auto & map_name : map_list_) {
    response->map_names.push_back(map_name.c_str());
  }
}

void MapoiServer::reload_map_info_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  this->send_load_map_info_request();
  response->success = true;
  response->message = current_map_;
}

void MapoiServer::send_load_map_info_request()
{
  auto map_file = mapoi_server_pkg_ + "/map/" + current_map_ + "/" + current_map_ + ".yaml";
  RCLCPP_INFO(this->get_logger(), "Map info URL:\n%s", map_file.c_str());
  YAML::Node map_info = YAML::LoadFile(map_file);
  pois_list_ = map_info["poi"];
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapoiServer>());
  rclcpp::shutdown();
  return 0;
}