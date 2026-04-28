#ifndef MAPOI_SERVER__MAPOI_NAV_SERVER_HPP_
#define MAPOI_SERVER__MAPOI_NAV_SERVER_HPP_

#ifdef UNIT_TEST
#include <gtest/gtest.h>
#endif

#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <mutex>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mapoi_interfaces/srv/get_pois_info.hpp"
#include "mapoi_interfaces/srv/get_route_pois.hpp"
#include "mapoi_interfaces/srv/get_tag_definitions.hpp"
#include "mapoi_interfaces/msg/point_of_interest.hpp"
#include "mapoi_interfaces/msg/poi_event.hpp"

class MapoiNavServer : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  enum class NavMode { IDLE, GOAL, ROUTE };

  explicit MapoiNavServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --- publisher & subscriber ---
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_initialpose_poi_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr nav2_initialpose_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_goal_pose_poi_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav2_goal_pose_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_route_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_cancel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_pause_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_resume_sub_;

  void mapoi_initialpose_poi_cb(const std_msgs::msg::String::SharedPtr msg);
  void mapoi_goal_pose_poi_cb(const std_msgs::msg::String::SharedPtr msg);
  void mapoi_route_cb(const std_msgs::msg::String::SharedPtr msg);
  void mapoi_cancel_cb(const std_msgs::msg::String::SharedPtr msg);
  void mapoi_pause_cb(const std_msgs::msg::String::SharedPtr msg);
  void mapoi_resume_cb(const std_msgs::msg::String::SharedPtr msg);

  // Service Callbacks
  void on_pois_info_received(rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedFuture future);
  void on_route_received(rclcpp::Client<mapoi_interfaces::srv::GetRoutePois>::SharedFuture future);

  void get_pois_list();

  // Action Callbacks (FollowWaypoints — routes)
  void goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr & goal_handle);
  void feedback_callback(GoalHandleFollowWaypoints::SharedPtr, const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
  void result_callback(const GoalHandleFollowWaypoints::WrappedResult & result);

  // Action Callbacks (NavigateToPose — single POI)
  void ntp_goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
  void ntp_result_callback(const GoalHandleNavigateToPose::WrappedResult & result);

  // Clients
  rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
  GoalHandleFollowWaypoints::SharedPtr current_goal_handle_;
  GoalHandleNavigateToPose::SharedPtr current_ntp_goal_handle_;
  rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedPtr pois_info_client_;
  rclcpp::Client<mapoi_interfaces::srv::GetRoutePois>::SharedPtr route_client_;

  // --- Pause / Resume state ---
  NavMode nav_mode_ = NavMode::IDLE;
  bool is_paused_ = false;
  geometry_msgs::msg::PoseStamped paused_goal_pose_;
  std::vector<geometry_msgs::msg::PoseStamped> current_route_waypoints_;
  uint32_t current_waypoint_index_ = 0;
  std::vector<geometry_msgs::msg::PoseStamped> paused_waypoints_;

  void reset_nav_state();

  std::mutex data_mutex_;
  std::vector<mapoi_interfaces::msg::PointOfInterest> pois_list_;

  // Nav status publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_status_pub_;
  // payload は target 空なら "status"、target 有りなら "status:target" を送る (#104)。
  // subscriber 側 (mapoi_panel / mapoi_webui_node) は : split で target を復元する。
  void publish_nav_status(const std::string & status, const std::string & target = "");

  // 現在 nav の target POI 名 / route 名 (mapoi_goal_pose_poi_cb / mapoi_route_cb で
  // 更新)。reset_nav_state では clear せず、次の nav 開始で必ず上書きされる前提で
  // 終端 status (succeeded / aborted / canceled) でも target を含めて publish できる
  // ようにする (#104)。
  std::string current_target_name_;

  // --- POI radius event detection ---
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr radius_check_timer_;
  rclcpp::Publisher<mapoi_interfaces::msg::PoiEvent>::SharedPtr poi_event_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_path_sub_;
  rclcpp::Client<mapoi_interfaces::srv::GetTagDefinitions>::SharedPtr tag_defs_client_;

  // path だけだと WebUI/Panel Save (path 不変、内容のみ変更) を取りこぼすため、mtime も併用。
  // guard は fetch 成功 callback (on_pois_info_received) で確定し、失敗時は pending のまま
  // 次回 publish で retry する。pending_guard_active_ が false の時 (start_sequence 経由の初回 fetch)
  // は guard 確定処理を skip。single-thread executor 前提で書き込みは callback context のみ。
  // 詳細は PR #78 (#75) と issue #80。
  std::string last_config_path_;
  std::filesystem::file_time_type last_config_mtime_{};
  std::string pending_config_path_;
  std::filesystem::file_time_type pending_config_mtime_{};
  bool pending_guard_active_ = false;
  std::unordered_set<std::string> system_tags_;
  bool system_tags_loaded_ = false;
  std::unordered_map<std::string, bool> poi_inside_state_;  // key: poi.name
  std::vector<mapoi_interfaces::msg::PointOfInterest> event_pois_;

  void fetch_system_tags();
  void on_system_tags_received(rclcpp::Client<mapoi_interfaces::srv::GetTagDefinitions>::SharedFuture future);
  void on_config_path_changed(const std_msgs::msg::String::SharedPtr msg);
  void rebuild_event_pois();
  void radius_check_callback();
  double distance_2d(const geometry_msgs::msg::Pose & poi_pose, double rx, double ry);

  // initial_pose タグ付き POI を抽出して /initialpose に自動 publish。
  // 0 個: skip / 1 個: そのまま / 複数: 警告して先頭を採用。
  // 同一 config_path に対する重複 publish はスキップする。
  void auto_publish_initial_pose();

  // POI リストから initial_pose タグ付き候補を返す純関数（test 容易化のため分離）。
  // 複数あれば WARN を出し先頭、0 個なら空ベクタ。
  std::vector<mapoi_interfaces::msg::PointOfInterest> select_initial_pose_pois(
    const std::vector<mapoi_interfaces::msg::PointOfInterest> & pois);

  // /initialpose 配信の単一エントリポイント（手動経路 / 自動経路で共通化）。
  void publish_initial_pose(
    const geometry_msgs::msg::Pose & pose, const std::string & source);

  // initialpose subscriber (主に AMCL) が ready になるまで待つ。
  // subscriber が既に ready なら即 return。timeout 内に ready にならなければ WARN。
  // 200ms 間隔の polling で blocking wait する (single-thread executor 前提)。
  void wait_for_initialpose_subscriber(double timeout_sec);

  // 同一 config_path への重複 publish 防止
  std::string last_initial_pose_config_path_;

#ifdef UNIT_TEST
  friend class NavServerTestFixture;
  FRIEND_TEST(NavServerTestFixture, DistanceCalculation);
  FRIEND_TEST(NavServerTestFixture, DistanceCalculationZero);
  FRIEND_TEST(NavServerTestFixture, RebuildEventPoisIncludesAllPois);
  FRIEND_TEST(NavServerTestFixture, RebuildEventPoisEmpty);
  FRIEND_TEST(NavServerTestFixture, PauseTagDetection);
  FRIEND_TEST(NavServerTestFixture, SelectInitialPosePoisEmpty);
  FRIEND_TEST(NavServerTestFixture, SelectInitialPosePoisSingle);
  FRIEND_TEST(NavServerTestFixture, SelectInitialPosePoisMultiple);
#endif
};

#endif  // MAPOI_SERVER__MAPOI_NAV_SERVER_HPP_
