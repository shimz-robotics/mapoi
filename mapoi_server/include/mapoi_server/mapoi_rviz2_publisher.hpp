#ifndef MAPOI_SERVER__MAPOI_RVIZ2_PUBLISHER_HPP_
#define MAPOI_SERVER__MAPOI_RVIZ2_PUBLISHER_HPP_

#include <vector>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <yaml-cpp/yaml.h>
#include "mapoi_interfaces/srv/get_tagged_pois.hpp"
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
  void on_poi_received(rclcpp::Client<mapoi_interfaces::srv::GetTaggedPois>::SharedFuture future);

  // --- Member Variables ---

  // マーカーID管理用
  int id_buf_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_dest_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_event_pub_;

  // Clients
  rclcpp::Client<mapoi_interfaces::srv::GetTaggedPois>::SharedPtr poi_client_;

  // Timers
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data storage
  std::vector<mapoi_interfaces::msg::PointOfInterest> pois_list_;
};

#endif  // MAPOI_SERVER__MAPOI_RVIZ2_PUBLISHER_HPP_