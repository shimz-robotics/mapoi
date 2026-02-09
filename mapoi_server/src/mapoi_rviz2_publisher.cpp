#include "mapoi_server/mapoi_rviz2_publisher.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;


MapoiRviz2Publisher::MapoiRviz2Publisher() : Node("mapoi_rviz2_publisher") {
  id_buf_ = 0;
  marker_waypoints_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mapoi_goal_marks", 10);
  marker_events_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mapoi_event_marks", 10);

  this->poi_client_ = this->create_client<mapoi_interfaces::srv::GetPoisInfo>("get_pois_info");

  highlight_goal_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_highlight_goal", 10,
    std::bind(&MapoiRviz2Publisher::on_highlight_goal_received, this, _1));
  highlight_route_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_highlight_route", 10,
    std::bind(&MapoiRviz2Publisher::on_highlight_route_received, this, _1));

  // 初期化シーケンスをデッドロック回避のため少し遅延させて開始
  this->init_timer_ = this->create_wall_timer(100ms, std::bind(&MapoiRviz2Publisher::start_sequence, this));

  timer_ = this->create_wall_timer(1s, std::bind(&MapoiRviz2Publisher::timer_callback, this));
}


void MapoiRviz2Publisher::start_sequence()
{
  this->init_timer_->cancel();

  if (!poi_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for services.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Services not available, retrying...");
    this->init_timer_->reset();
    return;
  }

  auto request = std::make_shared<mapoi_interfaces::srv::GetPoisInfo::Request>();
  RCLCPP_INFO(this->get_logger(), "Requesting POI Info...");
  
  poi_client_->async_send_request(
    request, std::bind(&MapoiRviz2Publisher::on_poi_received, this, _1));
}

void MapoiRviz2Publisher::on_poi_received(rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedFuture future)
{
  auto result = future.get();
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get Pois Info.");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    pois_list_ = result->pois_list;
  }
  RCLCPP_INFO(this->get_logger(), "Received %zu POIs.", pois_list_.size());
}


void MapoiRviz2Publisher::on_highlight_goal_received(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  highlighted_goal_names_.clear();
  if (!msg->data.empty()) {
    highlighted_goal_names_.insert(msg->data);
  }
}

void MapoiRviz2Publisher::on_highlight_route_received(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  highlighted_route_names_.clear();
  highlighted_route_ordered_.clear();
  if (!msg->data.empty()) {
    std::istringstream ss(msg->data);
    std::string token;
    int order = 1;
    while (std::getline(ss, token, ',')) {
      if (!token.empty()) {
        highlighted_route_names_[token] = order++;
        highlighted_route_ordered_.push_back(token);
      }
    }
  }
}

void MapoiRviz2Publisher::timer_callback(){
  std::lock_guard<std::mutex> lock(data_mutex_);
  // publish markers on rviz
  visualization_msgs::msg::Marker default_arrow_marker;
  default_arrow_marker.header.frame_id = "map";
  default_arrow_marker.header.stamp = rclcpp::Clock().now();
  default_arrow_marker.action = visualization_msgs::msg::Marker::ADD;
  default_arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
  default_arrow_marker.scale.x = 0.3; default_arrow_marker.scale.y = 0.2; default_arrow_marker.scale.z = 0.1;
  default_arrow_marker.color.r = 0.0; default_arrow_marker.color.g = 1.0; default_arrow_marker.color.b = 0.0; default_arrow_marker.color.a = 0.7;
  default_arrow_marker.lifetime.sec = 2.0;

  visualization_msgs::msg::Marker default_text_marker = default_arrow_marker;
  default_text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  default_text_marker.scale.x = 0.2; default_text_marker.scale.y = 0.2; default_text_marker.scale.z = 0.2;
  default_text_marker.color.r = 0.0; default_text_marker.color.g = 0.0; default_text_marker.color.b = 0.0; default_text_marker.color.a = 1.0;

  visualization_msgs::msg::Marker default_cylinder_marker = default_arrow_marker;
  default_cylinder_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  default_cylinder_marker.scale.x = 1.0*2; default_cylinder_marker.scale.y = 1.0*2; default_cylinder_marker.scale.z = 0.1;
  default_cylinder_marker.color.r = 0.64; default_cylinder_marker.color.g = 0.89; default_cylinder_marker.color.b = 0.85; default_cylinder_marker.color.a = 0.3;

  visualization_msgs::msg::MarkerArray ma_waypoints;
  visualization_msgs::msg::MarkerArray ma_events;
  int id = 0;
  for (const auto & poi : pois_list_) {
    geometry_msgs::msg::Pose pose = poi.pose;

    for(const auto & tag : poi.tags){
      if(tag == "goal"){
        visualization_msgs::msg::Marker m_waypoint = default_arrow_marker;
        m_waypoint.pose = pose;
        m_waypoint.pose.position.z = 0.1;
        {
          bool is_goal = highlighted_goal_names_.count(poi.name) > 0;
          if (is_goal) {
            m_waypoint.color.r = 1.0; m_waypoint.color.g = 0.6; m_waypoint.color.b = 0.0; m_waypoint.color.a = 0.7;
          }
        }
        m_waypoint.id = id;
        ma_waypoints.markers.push_back(m_waypoint);
        id += 1;

        visualization_msgs::msg::Marker m_text = default_text_marker;
        {
          auto it = highlighted_route_names_.find(poi.name);
          if (it != highlighted_route_names_.end()) {
            m_text.text = "[" + std::to_string(it->second) + "] " + poi.name;
          } else {
            m_text.text = poi.name;
          }
        }
        m_text.pose = pose;
        m_text.pose.position.z = 0.1;
        m_text.id = id;
        ma_waypoints.markers.push_back(m_text);
        id += 1;
      }
      else if(tag == "event"){
        visualization_msgs::msg::Marker m_event = default_arrow_marker;
        m_event.pose = pose;
        m_event.pose.position.z = 0.1;
        m_event.color.r = 0.0; m_event.color.g = 0.0; m_event.color.b = 1.0; m_event.color.a = 0.7;
        m_event.id = id;
        ma_events.markers.push_back(m_event);
        id += 1;

        visualization_msgs::msg::Marker m_area = default_cylinder_marker;
        m_area.pose = pose;
        m_area.pose.position.z = 0.1;
        m_area.scale.x = poi.radius * 2;
        m_area.scale.y = poi.radius * 2;
        m_area.id = id;
        ma_events.markers.push_back(m_area);
        id += 1;

        visualization_msgs::msg::Marker m_text = default_text_marker;
        m_text.text = poi.name;
        m_text.pose = pose;
        m_text.pose.position.z = 0.1;
        m_text.id = id;
        ma_events.markers.push_back(m_text);
        id += 1;
      }
      else if(tag == "origin"){
        visualization_msgs::msg::Marker m_event = default_arrow_marker;
        m_event.pose = pose;
        m_event.pose.position.z = 0.1;
        m_event.color.r = 1.0; m_event.color.g = 0.0; m_event.color.b = 0.0; m_event.color.a = 0.7;
        m_event.id = id;
        ma_events.markers.push_back(m_event);
        id += 1;
      }
    }
  }

  // ルート経由点間の矢印マーカー生成
  if (highlighted_route_ordered_.size() >= 2) {
    // POI名→poseのルックアップマップを構築
    std::map<std::string, geometry_msgs::msg::Pose> poi_pose_map;
    for (const auto & poi : pois_list_) {
      poi_pose_map[poi.name] = poi.pose;
    }

    for (size_t i = 0; i + 1 < highlighted_route_ordered_.size(); ++i) {
      auto it_start = poi_pose_map.find(highlighted_route_ordered_[i]);
      auto it_end = poi_pose_map.find(highlighted_route_ordered_[i + 1]);
      if (it_start == poi_pose_map.end() || it_end == poi_pose_map.end()) {
        continue;
      }

      visualization_msgs::msg::Marker m_arrow;
      m_arrow.header.frame_id = "map";
      m_arrow.header.stamp = rclcpp::Clock().now();
      m_arrow.action = visualization_msgs::msg::Marker::ADD;
      m_arrow.type = visualization_msgs::msg::Marker::ARROW;
      m_arrow.scale.x = 0.05;  // shaft diameter
      m_arrow.scale.y = 0.1;   // head diameter
      m_arrow.scale.z = 0.1;   // head length
      m_arrow.color.r = 0.6; m_arrow.color.g = 0.2; m_arrow.color.b = 1.0; m_arrow.color.a = 0.7;
      m_arrow.lifetime.sec = 2.0;

      geometry_msgs::msg::Point p_start;
      p_start.x = it_start->second.position.x;
      p_start.y = it_start->second.position.y;
      p_start.z = 0.15;
      geometry_msgs::msg::Point p_end;
      p_end.x = it_end->second.position.x;
      p_end.y = it_end->second.position.y;
      p_end.z = 0.15;

      m_arrow.points.push_back(p_start);
      m_arrow.points.push_back(p_end);
      m_arrow.id = id;
      ma_waypoints.markers.push_back(m_arrow);
      id += 1;
    }
  }

  // delete remaining incorrect marker
  if(id_buf_ > id) {
    visualization_msgs::msg::Marker m_del;
    m_del.action = visualization_msgs::msg::Marker::DELETEALL;
    visualization_msgs::msg::MarkerArray ma_del;
    ma_del.markers.push_back(m_del);
    marker_waypoints_pub_->publish(ma_del);
    marker_events_pub_->publish(ma_del);
  }
  id_buf_ = id;

  marker_waypoints_pub_->publish(ma_waypoints);
  marker_events_pub_->publish(ma_events);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapoiRviz2Publisher>());
  rclcpp::shutdown();
  return 0;
}