#include "mapoi_server/mapoi_poi_server.hpp"

using namespace std::chrono_literals;

MapoiPoiServer::MapoiPoiServer() : Node("mapoi_poi_server") {
  mapoi_server_pkg_ = ament_index_cpp::get_package_share_directory("mapoi_server");

  // initial maps path
  this->declare_parameter<std::string>("map_info_file", mapoi_server_pkg_ + "/maps/turtlebot3_world/map_info.yaml");
  current_map_info_file_ = this->get_parameter("map_info_file").as_string();
  RCLCPP_INFO(this->get_logger(), "Maps directory: %s", current_map_info_file_.c_str());
  YAML::Node map_info = YAML::LoadFile(current_map_info_file_);
  pois_list_ = map_info["poi"];
  routes_list_ = map_info["route"];

  get_tagged_pois_service_ = this->create_service<mapoi_interfaces::srv::GetTaggedPois>("get_tagged_pois",
    std::bind(&MapoiPoiServer::get_tagged_pois_service, this, std::placeholders::_1, std::placeholders::_2));

  get_route_pois_service_ = this->create_service<mapoi_interfaces::srv::GetRoutePois>("get_route_pois",
    std::bind(&MapoiPoiServer::get_route_pois_service, this, std::placeholders::_1, std::placeholders::_2));

  // initialpose subscriber and publisher
  mapoi_initialpose_poi_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_initialpose_poi", 1, std::bind(&MapoiPoiServer::mapoi_initialpose_poi_cb, this, std::placeholders::_1));
  nav2_initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1);

  // goal_pose subscriber and publisher
  mapoi_goal_pose_poi_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_goal_pose_poi", 1, std::bind(&MapoiPoiServer::mapoi_goal_pose_poi_cb, this, std::placeholders::_1));
  nav2_goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);

  RCLCPP_INFO(this->get_logger(), "Ready to serve. The current map_name is %s", current_map_info_file_.c_str());
}

void MapoiPoiServer::get_tagged_pois_service(const std::shared_ptr<mapoi_interfaces::srv::GetTaggedPois::Request> request,
          std::shared_ptr<mapoi_interfaces::srv::GetTaggedPois::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "Incoming get_tagged_pois request: %s", request->tag.c_str());
  for(auto poi : pois_list_){
    if(request->tag == "all"){
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
    }else{
      for(auto tag : poi["tags"]){
        if(tag.as<std::string>() == request->tag){
          mapoi_interfaces::msg::PointOfInterest tag_poi;
          tag_poi.name = poi["name"].as<std::string>();
          tag_poi.pose.position.x = poi["pose"]["x"].as<std::double_t>();
          tag_poi.pose.position.y = poi["pose"]["y"].as<std::double_t>();
          auto yaw = poi["pose"]["yaw"].as<std::double_t>();
          tag_poi.pose.orientation.z = sin(yaw/2.0);
          tag_poi.pose.orientation.w = cos(yaw/2.0);
          tag_poi.radius = poi["radius"].as<std::double_t>();
          tag_poi.tags = poi["tags"].as<std::vector<std::string>>();
          tag_poi.description = poi["description"].as<std::string>();
          response->pois_list.push_back(tag_poi);
        }
      }
    }
  }
  RCLCPP_DEBUG(this->get_logger(), "sending back response '%ld' %ss", response->pois_list.size(), request->tag.c_str());
}

void MapoiPoiServer::get_route_pois_service(
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
    poi_names += poi.name;  // PointOfInterest 型なのでこう書く
  }

  RCLCPP_DEBUG(this->get_logger(), "Collected POIs in route '%s': [%s]",
               request->route_name.c_str(), poi_names.c_str());
}

void MapoiPoiServer::mapoi_initialpose_poi_cb(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received POI name for initialpose: %s", msg->data.c_str());

  // Find the POI in the pois_list_
  for (const auto &poi : pois_list_) {
    if (poi["name"].as<std::string>() == msg->data) {
      geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
      init_pose.header.frame_id = "map";
      init_pose.pose.pose.position.x = poi["pose"]["x"].as<double>();
      init_pose.pose.pose.position.y = poi["pose"]["y"].as<double>();
      auto yaw = poi["pose"]["yaw"].as<double>();
      init_pose.pose.pose.orientation.z = std::sin(yaw / 2.0);
      init_pose.pose.pose.orientation.w = std::cos(yaw / 2.0);
      init_pose.pose.covariance[0] = 0.25;
      init_pose.pose.covariance[7] = 0.25;
      init_pose.pose.covariance[35] = 0.06853891945200942;

      nav2_initialpose_pub_->publish(init_pose);
      RCLCPP_INFO(this->get_logger(), "Published initial pose from POI: %s", msg->data.c_str());
      return;
    }
  }
  RCLCPP_WARN(this->get_logger(), "POI named '%s' not found!", msg->data.c_str());
}

void MapoiPoiServer::mapoi_goal_pose_poi_cb(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received POI name for goal pose: %s", msg->data.c_str());

  // Find the POI in the pois_list_
  for (const auto &poi : pois_list_) {
    if (poi["name"].as<std::string>() == msg->data) {
      geometry_msgs::msg::PoseStamped goal_pose;
      goal_pose.header.frame_id = "map";
      goal_pose.pose.position.x = poi["pose"]["x"].as<double>();
      goal_pose.pose.position.y = poi["pose"]["y"].as<double>();
      auto yaw = poi["pose"]["yaw"].as<double>();
      goal_pose.pose.orientation.z = std::sin(yaw / 2.0);
      goal_pose.pose.orientation.w = std::cos(yaw / 2.0);
      nav2_goal_pose_pub_->publish(goal_pose);
      RCLCPP_INFO(this->get_logger(), "Published goal pose from POI: %s", msg->data.c_str());
      return;
    }
  }
  RCLCPP_WARN(this->get_logger(), "POI named '%s' not found!", msg->data.c_str());
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapoiPoiServer>());
  rclcpp::shutdown();
  return 0;
}