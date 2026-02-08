#ifndef MAPOI_SERVER__MAPOI_RVIZ2_PUBLISHER_HPP_
#define MAPOI_SERVER__MAPOI_RVIZ2_PUBLISHER_HPP_

#include <vector>
#include <set>
#include <map>
#include <string>
#include <sstream>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>

#include <yaml-cpp/yaml.h>
#include "mapoi_interfaces/srv/get_pois_info.hpp"
#include "mapoi_interfaces/msg/point_of_interest.hpp"

class MapoiRviz2Publisher : public rclcpp::Node
{
public:
  MapoiRviz2Publisher();

private:
  /**
   * @brief 初期化待ち合わせ用タイマーコールバック
   */
  void start_sequence();

  /**
   * @brief マーカー配信周期実行用コールバック
   */
  void timer_callback();

  /**
   * @brief POI情報取得サービスのレスポンス処理
   * @param future サービスの結果
   */
  void on_poi_received(rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedFuture future);

  // --- Member Variables ---

  // マーカーID管理用
  int id_buf_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_waypoints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_events_pub_;

  // Clients
  rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedPtr poi_client_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr highlight_goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr highlight_route_sub_;
  void on_highlight_goal_received(const std_msgs::msg::String::SharedPtr msg);
  void on_highlight_route_received(const std_msgs::msg::String::SharedPtr msg);

  // Timers
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data storage
  std::vector<mapoi_interfaces::msg::PointOfInterest> pois_list_;
  std::set<std::string> highlighted_goal_names_;
  std::map<std::string, int> highlighted_route_names_;
  std::vector<std::string> highlighted_route_ordered_;
};

#endif  // MAPOI_SERVER__MAPOI_RVIZ2_PUBLISHER_HPP_