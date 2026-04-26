#ifndef MAPOI_SERVER__MAPOI_RVIZ2_PUBLISHER_HPP_
#define MAPOI_SERVER__MAPOI_RVIZ2_PUBLISHER_HPP_

#include <vector>
#include <set>
#include <map>
#include <array>
#include <string>
#include <sstream>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>

#include <yaml-cpp/yaml.h>
#include "mapoi_interfaces/srv/get_pois_info.hpp"
#include "mapoi_interfaces/srv/get_routes_info.hpp"
#include "mapoi_interfaces/srv/get_route_pois.hpp"
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

  /**
   * @brief get_pois_info service を呼び出して pois_list_ を更新する共通処理
   */
  void request_pois_list();

  /**
   * @brief mapoi_config_path topic の変化検出時に POI list を再取得する
   */
  void on_config_path_changed(const std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief get_routes_info service を呼び出し、各 route の waypoint を順次 fetch する
   */
  void request_routes_info();
  void on_routes_info_received(rclcpp::Client<mapoi_interfaces::srv::GetRoutesInfo>::SharedFuture future);
  void on_route_pois_received(
    const std::string & route_name,
    rclcpp::Client<mapoi_interfaces::srv::GetRoutePois>::SharedFuture future);

  // --- Member Variables ---

  // マーカーID管理用
  int id_buf_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_waypoints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_events_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_routes_pub_;

  // Clients
  rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedPtr poi_client_;
  rclcpp::Client<mapoi_interfaces::srv::GetRoutesInfo>::SharedPtr routes_info_client_;
  rclcpp::Client<mapoi_interfaces::srv::GetRoutePois>::SharedPtr route_pois_client_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr highlight_goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr highlight_route_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_path_sub_;
  void on_highlight_goal_received(const std_msgs::msg::String::SharedPtr msg);
  void on_highlight_route_received(const std_msgs::msg::String::SharedPtr msg);

  // Config path + mtime guard。
  // path だけ覚えると周期 publish に対して常に skip する一方、WebUI/Panel Save (内容のみ変更で path 不変)
  // も skip してしまう。mtime も併せて見ることで SwitchMap (path 変更) と Save (mtime 変更) の両方を検出。
  // single-thread executor 前提で書き込みは callback context のみ。MultiThreadedExecutor 移行時は mutex が必要。
  std::string last_config_path_;
  std::filesystem::file_time_type last_config_mtime_{};

  // Timers
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data storage
  std::mutex data_mutex_;
  std::vector<mapoi_interfaces::msg::PointOfInterest> pois_list_;
  std::set<std::string> highlighted_goal_names_;
  std::map<std::string, int> highlighted_route_names_;
  std::vector<std::string> highlighted_route_ordered_;

  // route 名 → waypoint POI 列。get_routes_info で名前一覧を取得後、
  // 各 route について get_route_pois を非同期に呼び結果を蓄積する。
  std::map<std::string, std::vector<mapoi_interfaces::msg::PointOfInterest>> all_routes_;
  // 前回 publish 時の route marker max id (DELETEALL 判定用、id_buf_ と独立)
  int route_id_buf_ = 0;
};

#endif  // MAPOI_SERVER__MAPOI_RVIZ2_PUBLISHER_HPP_