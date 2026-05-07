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
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mapoi_interfaces/srv/get_pois_info.hpp"
#include "mapoi_interfaces/srv/get_route_pois.hpp"
#include "mapoi_interfaces/srv/select_map.hpp"
#include "mapoi_interfaces/srv/get_tag_definitions.hpp"
#include "mapoi_interfaces/msg/point_of_interest.hpp"
#include "mapoi_interfaces/msg/poi_event.hpp"
#include "mapoi_interfaces/msg/initial_pose_request.hpp"
#include "mapoi_interfaces/msg/navigation_backend_status.hpp"

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
  // mapoi/initialpose_poi の publisher は LoadMap 完了 trigger 用に残す (#209)。
  // sub と /initialpose 配信は mapoi_amcl_localization_bridge に移設済。
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_goal_pose_poi_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav2_goal_pose_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_route_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_switch_map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_cancel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_pause_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapoi_resume_sub_;

  void mapoi_goal_pose_poi_cb(const std_msgs::msg::String::SharedPtr msg);
  void mapoi_route_cb(const std_msgs::msg::String::SharedPtr msg);
  void mapoi_switch_map_cb(const std_msgs::msg::String::SharedPtr msg);
  void mapoi_cancel_cb(const std_msgs::msg::String::SharedPtr msg);
  void mapoi_pause_cb(const std_msgs::msg::String::SharedPtr msg);
  void mapoi_resume_cb(const std_msgs::msg::String::SharedPtr msg);

  // Service Callbacks
  void on_pois_info_received(rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedFuture future);
  void on_select_map_received(
    std::string map_name,
    rclcpp::Client<mapoi_interfaces::srv::SelectMap>::SharedFuture future);
  // route_name は #104 で current_target_name_ を action send 直前に更新するために
  // mapoi_route_cb から bind 経由で渡す。リクエスト時点で代入すると concurrent
  // request で active nav の target が汚染されるため、実際の send_goal 直前まで
  // 遅延させる。
  void on_route_received(std::string route_name,
                         rclcpp::Client<mapoi_interfaces::srv::GetRoutePois>::SharedFuture future);

  void get_pois_list();

  // Action Callbacks (FollowWaypoints — routes)
  // target は #104 race fix のため goal 固有の target を bind 経由で受け取る。
  // callback 内で publish_nav_status はこの引数を使い、共有 current_target_name_
  // を読まない (concurrent request で別 nav の target が混入するのを防ぐ)。
  // current_target_name_ は acceptance 時に更新し、pause 等で active nav の
  // target として参照される用途のみ。
  // nav_attempt_generation: navigation 開始 (route or GOAL) ごとに増分。各 callback で stale 判定に
  // 使い、旧 navigation の遅延 callback が新 navigation の global state (current_goal_handle_ /
  // current_route_poi_names_ / current_target_name_ 等) を上書き / clear しないようにする
  // (Codex review #147 round 2 high 対応)。
  void goal_response_callback(std::string target, size_t nav_generation,
                                const GoalHandleFollowWaypoints::SharedPtr & goal_handle);
  void feedback_callback(size_t nav_generation, GoalHandleFollowWaypoints::SharedPtr,
                          const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
  void result_callback(std::string target, size_t nav_generation,
                       const GoalHandleFollowWaypoints::WrappedResult & result);

  // Action Callbacks (NavigateToPose — single POI)
  void ntp_goal_response_callback(std::string target, size_t nav_generation,
                                    const GoalHandleNavigateToPose::SharedPtr & goal_handle);
  void ntp_result_callback(std::string target, size_t nav_generation,
                            const GoalHandleNavigateToPose::WrappedResult & result);

  // Clients
  rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
  GoalHandleFollowWaypoints::SharedPtr current_goal_handle_;
  GoalHandleNavigateToPose::SharedPtr current_ntp_goal_handle_;
  rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedPtr pois_info_client_;
  rclcpp::Client<mapoi_interfaces::srv::GetRoutePois>::SharedPtr route_client_;
  rclcpp::Client<mapoi_interfaces::srv::SelectMap>::SharedPtr select_map_client_;

  // --- Pause / Resume state ---
  NavMode nav_mode_ = NavMode::IDLE;
  bool is_paused_ = false;
  geometry_msgs::msg::PoseStamped paused_goal_pose_;
  std::vector<geometry_msgs::msg::PoseStamped> current_route_waypoints_;
  uint32_t current_waypoint_index_ = 0;
  std::vector<geometry_msgs::msg::PoseStamped> paused_waypoints_;

  // active route の POI 名 set (waypoints + landmarks 両方を含む) (#143)。
  // route 受信 (on_route_received) で set、route 終端 / cancel / GOAL 切替で clear。
  // tolerance_check_callback の pause 発火条件 (active route POI に含まれる時のみ) で参照。
  std::unordered_set<std::string> current_route_poi_names_;
  void clear_current_route_poi_names_();

  // nav_attempt_generation: navigation 開始 (route or GOAL) ごとに 1 ずつ増分。各 action
  // callback で stale 判定に使う (Codex review #147 round 1+2 high 対応)。route ↔ route /
  // route ↔ GOAL / GOAL ↔ GOAL いずれの切替でも、旧 navigation の遅延 callback が新 nav state
  // を消さないようにする。
  // 安全根拠 (#213 で MultiThreadedExecutor 化後): nav state を読み書きする callback は全て
  // default MutuallyExclusive callback group 内で直列化される。Reentrant callback group で
  // 動く backend_status timer は nav state member に触らない (read-only な *_is_ready() のみ)
  // ため、mutex なしで読み書き OK。
  size_t nav_attempt_generation_ = 0;

  void reset_nav_state();

  std::mutex data_mutex_;
  std::vector<mapoi_interfaces::msg::PointOfInterest> pois_list_;

  // Navigation backend readiness publisher (#198).
  // 1Hz timer で action / service の存在を polling し、`mapoi/nav/backend_status` に publish する。
  // transient_local QoS で後起動 subscriber も最後の状態を受信できる。WebUI / panel はこの値で
  // 「Navigation connected」UI を gate する (= command topic subscriber 数だけでは Nav2 不在を
  // 検知できなかった #198 の課題に対応)。
  rclcpp::Publisher<mapoi_interfaces::msg::NavigationBackendStatus>::SharedPtr backend_status_pub_;
  rclcpp::TimerBase::SharedPtr backend_status_timer_;
  // 独立 MutuallyExclusive callback_group + MultiThreadedExecutor で backend_status timer を
  // 他 callback と独立 thread で動かすための group (#213)。Reentrant ではなく独立
  // MutuallyExclusive を選ぶ理由は cpp 側コメント参照 (#214 cursor review medium)。
  rclcpp::CallbackGroup::SharedPtr backend_status_callback_group_;
  void publish_backend_status();

  // Nav status publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_status_pub_;
  // payload は target 空なら "status"、target 有りなら "status:target" を送る (#104)。
  // subscriber 側 (mapoi_panel / mapoi_webui_node) は : split で target を復元する。
  void publish_nav_status(const std::string & status, const std::string & target = "");
  bool send_load_map_request(const std::string & server_name, const std::string & map_file);
  void publish_initial_poi_request(const std::string & map_name, const std::string & poi_name);

  // 現在 nav の target POI 名 / route 名。Acceptance 時 (goal_response_callback /
  // ntp_goal_response_callback) に更新され、pause / resume が active nav の target
  // として参照する用途のみ。
  // 終端 status (succeeded / aborted / canceled) は callback に lambda capture で
  // bind された goal 固有の target を使うので、ここを読まない (#104 race fix)。
  // reset_nav_state では clear せず、次の acceptance で上書きされる前提。
  std::string current_target_name_;

  // --- POI radius event detection ---
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr tolerance_check_timer_;
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
  // STOPPED/RESUMED 判定用の per-POI 状態 (#140)。
  //   was_stopped == true: 既に EVENT_STOPPED 発火済 (RESUMED まで持続)。
  //   was_stopped == false: 未停止 / OUTSIDE / RESUMED 後。
  // ENTER/EXIT 判定 (poi_inside_state_) と独立して持つ: STOPPED は inside 内での
  // sub-state なので、EXIT 時は inside_state→false と同時に stopped_state→false に reset する。
  std::unordered_map<std::string, bool> poi_stopped_state_;  // key: poi.name
  std::vector<mapoi_interfaces::msg::PointOfInterest> event_pois_;

  // initial pose POI 名の publisher (Nav2 LoadMap 完了後 trigger 用、#209)。
  // 配信先は mapoi_amcl_localization_bridge / mapoi_gz_bridge / WebUI 等が subscribe する
  // `mapoi/initialpose_poi` (transient_local)。実際の `/initialpose` 配信は localization
  // bridge が担当する (#209 で nav_server から分離)。
  rclcpp::Publisher<mapoi_interfaces::msg::InitialPoseRequest>::SharedPtr mapoi_initialpose_poi_pub_;

  // --- STOPPED/RESUMED 判定 (#140) ---
  // cmd_vel subscriber: 速度判定の source の一つ。Nav2 action SUCCEEDED もう一つの source は
  // result_callback / ntp_result_callback で hook する (両方 OR で STOPPED 判定)。
  // dwell 判定は robot 全体で 1 つ (POI ごとには持たない)。線速ノルムと角速度絶対値の両方が
  // 同じ閾値 ``stopped_speed_threshold`` 未満のとき停止扱い。線速 (m/s) と角速 (rad/s) に
  // 単位の異なる同一閾値を使う割り切りは、撮影シナリオで「その場旋回も停止扱いしたくない」
  // 用途と整合する (param 説明 / CHANGELOG にも明記)。
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  // 速度が閾値以下になり始めた時刻 (steady_clock)。閾値超えに戻ったら reset。
  std::chrono::steady_clock::time_point last_zero_velocity_start_ {};
  bool zero_velocity_active_ {false};
  // STOPPED 判定 param のキャッシュ (cmd_vel callback で毎 tick 取得すると lock コストが高いため
  // constructor で読み込んで保持。ROS 動的 reconfigure で変更したい場合は再起動が必要)。
  double stopped_speed_threshold_ {0.01};
  double stopped_dwell_time_sec_ {1.0};

  // 状態遷移判定の純関数 (unit test 用、#140)。inputs から transition を返すだけで副作用なし。
  enum class StoppedTransition { NONE, TO_STOPPED, TO_RESUMED };
  struct StoppedDetectionInputs {
    bool inside;                 // robot が POI の tolerance.xy 内に居るか
    bool was_stopped;            // 前 tick での poi_stopped_state_
    bool zero_velocity_dwelled;  // 速度が閾値以下で dwell_time 以上経過
  };
  static StoppedTransition compute_stopped_transition(const StoppedDetectionInputs & in);

  // STOPPED publish (Nav2 SUCCEEDED 受信時)。``target_poi_name`` が指定された場合、その POI は
  // inside check を skip して強制的に STOPPED publish する (SUCCEEDED と tolerance_check tick の
  // 非同期で「まだ inside と判定されていない」瞬間に SUCCEEDED が来ても取りこぼさない)。それ以外
  // の inside POI は通常通り poi_inside_state_ を見て publish。``reason`` は log 用。
  void stop_all_inside_pois(const std::string & reason, const std::string & target_poi_name = "");
  // 全 stopped POI に対して RESUMED publish (新規 goal 受信時)。
  void resume_all_stopped_pois();

  void fetch_system_tags();
  void on_system_tags_received(rclcpp::Client<mapoi_interfaces::srv::GetTagDefinitions>::SharedFuture future);
  void on_config_path_changed(const std_msgs::msg::String::SharedPtr msg);
  void rebuild_event_pois();
  void tolerance_check_callback();
  double distance_2d(const geometry_msgs::msg::Pose & poi_pose, double rx, double ry);

  // landmark system tag を持つかを判定する純関数 (#85)。
  // landmark POI は Nav2 navigation goal / initial_pose に使えない reference 専用。
  static bool has_landmark_tag(const mapoi_interfaces::msg::PointOfInterest & poi);

  // active route の POI 名集合を組み立てる純関数 (#143 / #148)。
  // waypoints と landmarks の両方を 1 つの set にマージし、tolerance_check の
  // pause 発火条件 (active route POI に含まれるか) を判定する用に使う。
  // 同名の重複は set 性質で自動的に 1 つに集約される。
  static std::unordered_set<std::string> build_route_poi_names(
    const std::vector<mapoi_interfaces::msg::PointOfInterest> & waypoints,
    const std::vector<mapoi_interfaces::msg::PointOfInterest> & landmarks);

  // pause 自動発火の eligibility 判定純関数 (#143 / #148)。
  // ROUTE 走行中 + active route POI + "pause" タグありの 3 条件を全て満たす場合のみ true。
  // GOAL 走行中 / IDLE では false (操作者の意図しない停止を避けるため、route POI 限定)。
  // **lock 契約**: `current_route_poi_names_` をそのまま渡す呼び出しでは、参照が
  // 安定するように `data_mutex_` を保持中に呼ぶこと。snapshot を渡す場合は不要。
  static bool is_pause_eligible(
    const mapoi_interfaces::msg::PointOfInterest & poi,
    NavMode nav_mode,
    const std::unordered_set<std::string> & active_route_poi_names);

#ifdef UNIT_TEST
  friend class NavServerTestFixture;
  FRIEND_TEST(NavServerTestFixture, DistanceCalculation);
  FRIEND_TEST(NavServerTestFixture, DistanceCalculationZero);
  FRIEND_TEST(NavServerTestFixture, RebuildEventPoisIncludesAllPois);
  FRIEND_TEST(NavServerTestFixture, RebuildEventPoisEmpty);
  FRIEND_TEST(NavServerTestFixture, PauseTagDetection);
  FRIEND_TEST(NavServerTestFixture, HasLandmarkTagDetection);
  FRIEND_TEST(NavServerTestFixture, BuildRoutePoiNamesWaypointsOnly);
  FRIEND_TEST(NavServerTestFixture, BuildRoutePoiNamesWaypointsAndLandmarks);
  FRIEND_TEST(NavServerTestFixture, BuildRoutePoiNamesLandmarksEmptyBackwardCompat);
  FRIEND_TEST(NavServerTestFixture, BuildRoutePoiNamesEmpty);
  FRIEND_TEST(NavServerTestFixture, BuildRoutePoiNamesDuplicateNames);
  FRIEND_TEST(NavServerTestFixture, IsPauseEligibleRouteModeActiveWithPauseTag);
  FRIEND_TEST(NavServerTestFixture, IsPauseEligibleRouteModeActiveWithoutPauseTag);
  FRIEND_TEST(NavServerTestFixture, IsPauseEligibleRouteModeNonActivePoi);
  FRIEND_TEST(NavServerTestFixture, IsPauseEligibleGoalMode);
  FRIEND_TEST(NavServerTestFixture, IsPauseEligibleIdleMode);
  FRIEND_TEST(NavServerTestFixture, ResetNavStateClearsRouteContext);
  FRIEND_TEST(NavServerTestFixture, ComputeStoppedTransitionNoOutside);
  FRIEND_TEST(NavServerTestFixture, ComputeStoppedTransitionEnterStopped);
  FRIEND_TEST(NavServerTestFixture, ComputeStoppedTransitionEnterMoving);
  FRIEND_TEST(NavServerTestFixture, ComputeStoppedTransitionResumeOnVelocity);
  FRIEND_TEST(NavServerTestFixture, ComputeStoppedTransitionStaysStopped);
#endif
};

#endif  // MAPOI_SERVER__MAPOI_NAV_SERVER_HPP_
