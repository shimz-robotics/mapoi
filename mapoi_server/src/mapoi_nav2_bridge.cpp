#include "mapoi_server/mapoi_nav2_bridge.hpp"

#include <chrono>
#include <functional>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

MapoiNav2Bridge::MapoiNav2Bridge(const rclcpp::NodeOptions & options)
: Node("mapoi_nav2_bridge", options)
{
  this->get_logger().set_level(rclcpp::Logger::Level::Info);

  // initialpose POI 名の publisher (Nav2 LoadMap 完了後 trigger 用、#209)。
  // AMCL adapter は mapoi_amcl_localization_bridge に分離済み。本 publisher は
  // `mapoi_switch_map_cb` で LoadMap 成功後に新 map の `initial_poi_name` を流すための 1 用途のみ。
  mapoi_initialpose_poi_pub_ = this->create_publisher<mapoi_interfaces::msg::InitialPoseRequest>(
    "mapoi/initialpose_poi", rclcpp::QoS(1).transient_local());

  // goal_pose subscriber and publisher
  mapoi_goal_pose_poi_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi/nav/goal_pose_poi", 1, std::bind(&MapoiNav2Bridge::mapoi_goal_pose_poi_cb, this, std::placeholders::_1));
  nav2_goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);

  // route subscriber
  mapoi_route_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi/nav/route", 1, std::bind(&MapoiNav2Bridge::mapoi_route_cb, this, std::placeholders::_1));

  // map switch subscriber
  mapoi_switch_map_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi/nav/switch_map", 1, std::bind(&MapoiNav2Bridge::mapoi_switch_map_cb, this, std::placeholders::_1));

  // cancel subscriber
  mapoi_cancel_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi/nav/cancel", 1, std::bind(&MapoiNav2Bridge::mapoi_cancel_cb, this, std::placeholders::_1));

  // pause / resume subscribers
  mapoi_pause_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi/nav/pause", 1, std::bind(&MapoiNav2Bridge::mapoi_pause_cb, this, _1));
  mapoi_resume_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi/nav/resume", 1, std::bind(&MapoiNav2Bridge::mapoi_resume_cb, this, _1));

  this->action_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
  this->nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  // Nav status publisher
  // transient_local: 後起動 subscriber でも最後の状態を受信できる (mapoi/config_path と同 pattern)
  nav_status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "mapoi/nav/status", rclcpp::QoS(1).transient_local());

  // Navigation backend readiness publisher + 1Hz polling timer (#198)。
  // Nav2 action / service の存在は実行時に変化し得る (Nav2 を後起動 / 落とすケース) ため、
  // event-driven ではなく polling で publish する割り切り。1Hz は WebUI 表示の応答性として十分。
  // QoS は NavigationBackendStatus.msg の contract に従う (#208):
  // transient_local + MANUAL_BY_TOPIC liveliness + 5s lease。各 publish() が assert を兼ねる
  // ので 1Hz timer が止まれば 5s 後に subscriber 側 Liveliness Changed event が発火する。
  backend_status_pub_ = this->create_publisher<mapoi_interfaces::msg::NavigationBackendStatus>(
    "mapoi/nav/backend_status",
    rclcpp::QoS(1)
      .transient_local()
      .liveliness(rclcpp::LivelinessPolicy::ManualByTopic)
      .liveliness_lease_duration(5s));
  // 独立 MutuallyExclusive callback_group + MultiThreadedExecutor (main) で backend_status
  // timer を他 callback と独立 thread で動かす (#213)。`select_map_callback` 内の
  // `wait_for_service(10s)` / `spin_until_future_complete` で default group の thread が最大
  // ~11s blocking する間も 1Hz publish が継続するため、5s lease を超える false-positive
  // lost が発火しなくなる。Reentrant ではなく独立 MutuallyExclusive にする理由 (#214 cursor
  // review medium): backend_status timer は callback が単一 (`publish_backend_status` のみ)
  // で、必要なのは「default group の thread と並列に動く」こと。Reentrant は同一 callback の
  // self-overlap を許す性質で、将来 callback が長時間化した時の race risk を生む。MutuallyExclusive
  // 独立 group なら他 callback と並列実行されつつ self-overlap は起きない。`publish_backend_status`
  // は read-only な `*_is_ready()` checks + thread-safe な `Publisher::publish()` のみで member
  // 書き込みなしのため、他 callback (default group で serialize) との data race も無い。
  backend_status_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  backend_status_timer_ = this->create_wall_timer(
    1s, std::bind(&MapoiNav2Bridge::publish_backend_status, this),
    backend_status_callback_group_);

  this->pois_info_client_ = this->create_client<mapoi_interfaces::srv::GetPoisInfo>("get_pois_info");
  this->route_client_ = this->create_client<mapoi_interfaces::srv::GetRoutePois>("get_route_pois");
  this->select_map_client_ = this->create_client<mapoi_interfaces::srv::SelectMap>("select_map");

  // --- POI radius event detection ---
  this->declare_parameter<double>("tolerance_check_hz", 5.0);
  this->declare_parameter<double>("hysteresis_exit_multiplier", 1.15);
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("base_frame", "base_link");
  // EVENT_PAUSED_AT 判定 (#220 で旧 STOPPED/RESUMED から spec 変更): 線速ノルムと角速度絶対値
  // の両方が ``stopped_speed_threshold`` 未満の状態が ``stopped_dwell_time_sec`` 続いたら、
  // pause タグ付き route POI 内に居る場合のみ EVENT_PAUSED_AT publish。線速 (m/s) と角速 (rad/s)
  // に同一閾値を使う割り切りで「その場旋回も停止扱いしない」用途を表現する。
  this->declare_parameter<double>("stopped_speed_threshold", 0.01);
  this->declare_parameter<double>("stopped_dwell_time_sec", 1.0);
  this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
  // パラメータを cmd_vel_callback / tolerance_check_callback の hot path で毎回取得すると
  // lock コストが高いため、起動時にキャッシュして以降は member を読む (#176 review low #4)。
  stopped_speed_threshold_ = this->get_parameter("stopped_speed_threshold").as_double();
  stopped_dwell_time_sec_ = this->get_parameter("stopped_dwell_time_sec").as_double();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  poi_event_pub_ = this->create_publisher<mapoi_interfaces::msg::PoiEvent>("mapoi/events", 10);

  config_path_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi/config_path", rclcpp::QoS(1).transient_local(),
    std::bind(&MapoiNav2Bridge::on_config_path_changed, this, _1));

  // cmd_vel subscribe (#140): STOPPED 判定の source の一つ (もう一つは Nav2 SUCCEEDED)。
  // QoS は Nav2 の cmd_vel publisher と同じ default (reliable, depth=10)。
  const std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, 10,
    std::bind(&MapoiNav2Bridge::cmd_vel_callback, this, _1));

  tag_defs_client_ = this->create_client<mapoi_interfaces::srv::GetTagDefinitions>("get_tag_definitions");
  fetch_system_tags();

  double hz = this->get_parameter("tolerance_check_hz").as_double();
  auto period = std::chrono::duration<double>(1.0 / hz);
  tolerance_check_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&MapoiNav2Bridge::tolerance_check_callback, this));

  RCLCPP_INFO(this->get_logger(), "MapoiNav2Bridge initialized.");
}

void MapoiNav2Bridge::get_pois_list(){
  while(!this->pois_info_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for pois_info service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "pois_info service not available, waiting again...");
  }
  auto pois_info_request = std::make_shared<mapoi_interfaces::srv::GetPoisInfo::Request>();
  pois_info_client_->async_send_request(
    pois_info_request, std::bind(&MapoiNav2Bridge::on_pois_info_received, this, _1));
}

void MapoiNav2Bridge::mapoi_goal_pose_poi_cb(const std_msgs::msg::String::SharedPtr msg)
{
  std::string poi_name = msg->data;
  RCLCPP_INFO(this->get_logger(), "Received POI name for goal pose: %s", poi_name.c_str());
  // #220 で RESUMED event は撤去 (resume は client request + status topic で観測可能)。
  // poi_paused_at_published_ flag は VISITED → EXIT の lifecycle で reset される (EXIT 時 clear)。
  // target は send_goal_options に lambda capture で bind し、callback 側で
  // goal 固有の target を使って publish_nav_status する (#104 race fix)。
  // current_target_name_ は acceptance 時 (goal_response_callback) に更新され、
  // pause / resume 用の active target として参照される。

  // Fetch POI list asynchronously, then navigate in the callback
  if (!this->pois_info_client_->wait_for_service(2s)) {
    RCLCPP_ERROR(this->get_logger(), "get_pois_info service not available");
    return;
  }
  auto request = std::make_shared<mapoi_interfaces::srv::GetPoisInfo::Request>();
  pois_info_client_->async_send_request(
    request, [this, poi_name](rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedFuture future) {
      auto result = future.get();
      if (!result) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get POI info for goal navigation.");
        return;
      }
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pois_list_ = result->pois_list;
      }
      rebuild_event_pois();

      for (const auto &poi : result->pois_list) {
        if (poi.name == poi_name) {
          // landmark POI は Nav2 navigation goal にできない reference 専用 (#85)。
          // goal+landmark 併用も同様に弾く。
          if (has_landmark_tag(poi)) {
            RCLCPP_ERROR(this->get_logger(),
              "POI '%s' has 'landmark' tag; cannot be set as Nav2 goal.",
              poi_name.c_str());
            return;
          }
          geometry_msgs::msg::PoseStamped goal_pose;
          goal_pose.header.frame_id = "map";
          goal_pose.header.stamp = this->now();
          goal_pose.pose = poi.pose;

          // GOAL 切替時の state 更新と generation 増分は action 利用可否に関わらず先に行う
          // (Codex review #147 round 2 / round 3 high): fallback (topic publish) でも nav_mode_
          // / generation を進めないと、ROUTE/GOAL A の遅延 callback が fallback GOAL B の state を
          // 上書きする経路が残る。
          paused_goal_pose_ = goal_pose;
          nav_mode_ = NavMode::GOAL;
          is_paused_ = false;
          // 直前まで ROUTE モードだった場合に備えて active route POI set を clear (#143)。
          // GOAL モードでは route POI 限定の pause 発火 logic を走らせない。
          {
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_route_poi_names_.clear();
          }
          const size_t my_generation = ++nav_attempt_generation_;

          // Use NavigateToPose action client for result feedback.
          // Nav2 action 不在時は `goal_pose` topic への fallback を試みる (古い simple navigation 構成との互換性)。
          // ただし topic にも subscriber が居なければ backend_unavailable として WebUI / panel に通知する (#198)。
          // 旧実装の blocking `wait_for_action_server(timeout)` は single-thread executor で
          // cmd_vel / tolerance_check 等の他 callback を止めるため、即時判定に変更 (#198 review high)。
          // backend_status timer が 1Hz で readiness を更新しているので、別途 wait は不要。
          if (!this->nav_to_pose_client_->action_server_is_ready()) {
            if (nav2_goal_pose_pub_->get_subscription_count() == 0) {
              RCLCPP_ERROR(this->get_logger(),
                "Neither NavigateToPose action nor /goal_pose subscriber available; aborting GOAL '%s'.",
                poi_name.c_str());
              publish_nav_status("backend_unavailable", poi_name);
              reset_nav_state();
              return;
            }
            RCLCPP_WARN(this->get_logger(), "NavigateToPose action not available, falling back to topic");
            nav2_goal_pose_pub_->publish(goal_pose);
            return;
          }

          auto goal_msg = NavigateToPose::Goal();
          goal_msg.pose = goal_pose;

          auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
          // target + generation を lambda capture で bind し、callback が goal 固有 + stale 判定で
          // publish_nav_status / state 更新を呼べるようにする (#104 / #147)。
          send_goal_options.goal_response_callback =
            [this, target = poi_name, my_generation](const GoalHandleNavigateToPose::SharedPtr & h) {
              this->ntp_goal_response_callback(target, my_generation, h);
            };
          send_goal_options.result_callback =
            [this, target = poi_name, my_generation](const GoalHandleNavigateToPose::WrappedResult & r) {
              this->ntp_result_callback(target, my_generation, r);
            };

          this->nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
          RCLCPP_INFO(this->get_logger(), "Sent NavigateToPose goal from POI: %s", poi_name.c_str());
          return;
        }
      }
      RCLCPP_WARN(this->get_logger(), "POI named '%s' not found!", poi_name.c_str());
    });
}

void MapoiNav2Bridge::mapoi_route_cb(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received route name: %s", msg->data.c_str());
  // #220 で RESUMED event は撤去 (resume は client request + status topic で観測可能)。
  // target は on_route_received → send_goal_options に lambda capture で bind し、
  // callback 側で goal 固有の target を使って publish_nav_status する (#104 race fix)。
  // current_target_name_ は acceptance 時 (goal_response_callback) に更新される。

  auto route_request = std::make_shared<mapoi_interfaces::srv::GetRoutePois::Request>();
  route_request->route_name = msg->data;

  // route_name を lambda capture で渡す。std::bind は rclcpp Jazzy の
  // function_traits::same_arguments テンプレート制約をパスせず compile error
  // になるので lambda を使う (PR #119 regression fix)。
  this->route_client_->async_send_request(
    route_request,
    [this, route_name = msg->data](
      rclcpp::Client<mapoi_interfaces::srv::GetRoutePois>::SharedFuture future) {
        this->on_route_received(route_name, future);
    });
}

void MapoiNav2Bridge::mapoi_switch_map_cb(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string map_name = msg->data;
  if (map_name.empty()) {
    RCLCPP_ERROR(this->get_logger(), "mapoi/nav/switch_map received empty map name.");
    publish_nav_status("map_switch_failed", "empty_map_name");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Received map switch request: %s", map_name.c_str());

  if (current_goal_handle_) {
    action_client_->async_cancel_goal(current_goal_handle_);
    current_goal_handle_.reset();
  }
  if (current_ntp_goal_handle_) {
    nav_to_pose_client_->async_cancel_goal(current_ntp_goal_handle_);
    current_ntp_goal_handle_.reset();
  }
  reset_nav_state();
  ++nav_attempt_generation_;

  if (!this->select_map_client_->wait_for_service(2s)) {
    RCLCPP_ERROR(this->get_logger(), "select_map service not available");
    publish_nav_status("map_switch_failed", map_name);
    return;
  }

  publish_nav_status("map_switching", map_name);
  auto request = std::make_shared<mapoi_interfaces::srv::SelectMap::Request>();
  request->map_name = map_name;
  select_map_client_->async_send_request(
    request,
    [this, map_name](rclcpp::Client<mapoi_interfaces::srv::SelectMap>::SharedFuture future) {
      this->on_select_map_received(map_name, future);
    });
}

void MapoiNav2Bridge::on_select_map_received(
  std::string map_name,
  rclcpp::Client<mapoi_interfaces::srv::SelectMap>::SharedFuture future)
{
  auto result = future.get();
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "select_map returned no response for '%s'.", map_name.c_str());
    publish_nav_status("map_switch_failed", map_name);
    return;
  }
  if (!result->success) {
    RCLCPP_ERROR(this->get_logger(),
      "select_map failed for '%s': %s", map_name.c_str(), result->error_message.c_str());
    publish_nav_status("map_switch_failed", map_name);
    return;
  }
  if (result->nav2_node_names.size() != result->nav2_map_urls.size()) {
    RCLCPP_ERROR(this->get_logger(),
      "select_map returned mismatched Nav2 map lists for '%s' (%zu node names, %zu URLs).",
      map_name.c_str(), result->nav2_node_names.size(), result->nav2_map_urls.size());
    publish_nav_status("map_switch_failed", map_name);
    return;
  }
  if (result->nav2_node_names.empty()) {
    RCLCPP_ERROR(this->get_logger(),
      "select_map returned no Nav2 map entries for '%s'. Operator map switch requires map entries.",
      map_name.c_str());
    publish_nav_status("map_switch_failed", map_name);
    return;
  }

  for (size_t i = 0; i < result->nav2_node_names.size(); ++i) {
    const auto & node_name = result->nav2_node_names[i];
    const auto & map_url = result->nav2_map_urls[i];
    if (!send_load_map_request(node_name, map_url)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load Nav2 map '%s' via '%s'.",
        map_url.c_str(), node_name.c_str());
      publish_nav_status("map_switch_failed", map_name);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded Nav2 map '%s' via '%s'.",
      map_url.c_str(), node_name.c_str());
  }

  publish_initial_poi_request(map_name, result->initial_poi_name);
  publish_nav_status("map_switch_succeeded", map_name);
  RCLCPP_INFO(this->get_logger(), "Map switch completed: %s", map_name.c_str());
}

bool MapoiNav2Bridge::send_load_map_request(const std::string & server_name, const std::string & map_file)
{
  auto node = rclcpp::Node::make_shared("mapoi_load_map_client");
  auto load_map_client = node->create_client<nav2_msgs::srv::LoadMap>(server_name + "/load_map");
  auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  req->map_url = map_file;

  if (!load_map_client->wait_for_service(10s)) {
    RCLCPP_ERROR(this->get_logger(), "%s/load_map service is not available.", server_name.c_str());
    return false;
  }
  auto future = load_map_client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node, future, 1s) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "%s/load_map did not return.", server_name.c_str());
    return false;
  }
  auto response = future.get();
  if (!response || response->result != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
    const uint8_t result_code = response ? response->result : 0;
    RCLCPP_ERROR(this->get_logger(),
      "%s/load_map rejected '%s' (result=%u).", server_name.c_str(), map_file.c_str(), result_code);
    return false;
  }
  return true;
}

void MapoiNav2Bridge::publish_initial_poi_request(const std::string & map_name, const std::string & poi_name)
{
  if (poi_name.empty()) {
    RCLCPP_WARN(this->get_logger(),
      "No initial pose POI resolved for map '%s'; skipping mapoi/initialpose_poi publish.",
      map_name.c_str());
    return;
  }
  auto msg = mapoi_interfaces::msg::InitialPoseRequest();
  msg.map_name = map_name;
  msg.poi_name = poi_name;
  mapoi_initialpose_poi_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(),
    "Published initial pose POI '%s' for switched map '%s'.",
    poi_name.c_str(), map_name.c_str());
}

void MapoiNav2Bridge::on_pois_info_received(rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedFuture future)
{
  auto result = future.get();
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get Tagged POIs.");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    pois_list_ = result->pois_list;
  }
  RCLCPP_INFO(this->get_logger(), "Received %zu Tagged POIs.", pois_list_.size());
  rebuild_event_pois();

  // config_path 由来の fetch (on_config_path_changed → get_pois_list) が成功した時のみ guard を確定。
  // start_sequence 経由の初回 fetch では pending_guard_active_ が false で、ここは no-op。
  // fetch 失敗 (result == nullptr) で早期 return した場合も pending を残し、次回 publish で retry する。
  // (#144 で auto_publish_initial_pose は廃止、initial pose は mapoi_server が
  // mapoi/initialpose_poi topic に publish し、mapoi_amcl_localization_bridge 経由で
  // /initialpose に配信される (#209 で mapoi_nav2_bridge から AMCL adapter を分離)。)
  if (pending_guard_active_) {
    last_config_path_ = pending_config_path_;
    last_config_mtime_ = pending_config_mtime_;
    pending_guard_active_ = false;
  }
}

bool MapoiNav2Bridge::has_landmark_tag(const mapoi_interfaces::msg::PointOfInterest & poi)
{
  for (const auto & tag : poi.tags) {
    if (tag == "landmark") {
      return true;
    }
  }
  return false;
}

std::unordered_set<std::string> MapoiNav2Bridge::build_route_poi_names(
  const std::vector<mapoi_interfaces::msg::PointOfInterest> & waypoints,
  const std::vector<mapoi_interfaces::msg::PointOfInterest> & landmarks)
{
  std::unordered_set<std::string> result;
  for (const auto & p : waypoints) {
    result.insert(p.name);
  }
  for (const auto & p : landmarks) {
    result.insert(p.name);
  }
  return result;
}

bool MapoiNav2Bridge::is_pause_eligible(
  const mapoi_interfaces::msg::PointOfInterest & poi,
  NavMode nav_mode,
  const std::unordered_set<std::string> & active_route_poi_names)
{
  if (nav_mode != NavMode::ROUTE) {
    return false;
  }
  if (active_route_poi_names.count(poi.name) == 0) {
    return false;
  }
  for (const auto & tag : poi.tags) {
    if (tag == "pause") {
      return true;
    }
  }
  return false;
}

void MapoiNav2Bridge::on_route_received(
  std::string route_name,
  rclcpp::Client<mapoi_interfaces::srv::GetRoutePois>::SharedFuture future)
{
  auto result = future.get();
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get Route info.");
    return;
  }

  const auto & route_poi = result->pois_list;
  const auto & route_landmarks = result->landmark_pois;
  RCLCPP_INFO(this->get_logger(),
    "Received Route '%s' with %zu waypoints + %zu landmarks.",
    route_name.c_str(), route_poi.size(), route_landmarks.size());

  // Nav2 FollowWaypoints へ送るのは waypoints のみ。landmark は radius 監視専用 (#143)。
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  for (size_t i = 0; i < route_poi.size(); ++i) {
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "map";
    wp.header.stamp = this->now();
    wp.pose = route_poi[i].pose;
    waypoints.push_back(wp);
  }

  if (waypoints.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Route is empty. Cannot navigate.");
    return;
  }

  // active route の POI 名 set を構築 (waypoints + landmarks 両方を tolerance_check で
  // pause 発火対象として扱う)。lock 下で書き込み (#143)。
  // 構築ロジックは pure helper に切り出し、unit test で検証する (#148)。
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_route_poi_names_ = build_route_poi_names(route_poi, route_landmarks);
  }

  current_route_waypoints_ = waypoints;
  current_waypoint_index_ = 0;
  nav_mode_ = NavMode::ROUTE;
  is_paused_ = false;
  // navigation attempt generation を増分し、callback の stale 判定で使う
  // (Codex review #147 round 1 + 2 high)。
  const size_t my_generation = ++nav_attempt_generation_;

  // Send waypoints to Nav2 with action client.
  // Nav2 が起動していない場合は無限待ちせず、backend_unavailable を publish して route を放棄する (#198)。
  // 旧実装の `while(!wait_for_action_server(1s))` ループおよび review fix の blocking timeout は
  // Nav2 不在時に single-thread executor で callback を blocking する原因だった。
  // 即時判定 + 1Hz backend_status polling で readiness は別途維持される (#198 review high)。
  if (!this->action_client_->action_server_is_ready()) {
    RCLCPP_ERROR(this->get_logger(),
      "FollowWaypoints action server not available; aborting route '%s'.", route_name.c_str());
    publish_nav_status("backend_unavailable", route_name);
    reset_nav_state();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Sending goal with %zu waypoints.", waypoints.size());

  auto goal_msg = FollowWaypoints::Goal();
  goal_msg.poses = waypoints;

  auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();

  // target を lambda capture で bind し、callback が goal 固有の target で
  // publish_nav_status を呼べるようにする (#104 Codex round 2 medium 対応)。
  send_goal_options.goal_response_callback =
    [this, target = route_name, my_generation](const GoalHandleFollowWaypoints::SharedPtr & h) {
      this->goal_response_callback(target, my_generation, h);
    };

  send_goal_options.feedback_callback =
    [this, my_generation](GoalHandleFollowWaypoints::SharedPtr h,
                          const std::shared_ptr<const FollowWaypoints::Feedback> f) {
      this->feedback_callback(my_generation, h, f);
    };

  send_goal_options.result_callback =
    [this, target = route_name, my_generation](const GoalHandleFollowWaypoints::WrappedResult & r) {
      this->result_callback(target, my_generation, r);
    };

  this->action_client_->async_send_goal(goal_msg, send_goal_options);
}

void MapoiNav2Bridge::goal_response_callback(std::string target, size_t nav_generation,
                                              const GoalHandleFollowWaypoints::SharedPtr & goal_handle)
{
  // stale check (Codex review #147 round 2 high): 旧 route の goal_response が新 navigation
  // (route or GOAL) の受理後に届くと、current_goal_handle_ / current_target_name_ が旧
  // navigation に巻き戻り、pause / status publish が混乱する。bound generation と現在
  // generation を照合し、旧 navigation の callback は何もしない。
  if (nav_generation != nav_attempt_generation_) {
    RCLCPP_INFO(this->get_logger(),
      "Stale FollowWaypoints goal response for '%s' (gen=%zu, current=%zu); ignoring.",
      target.c_str(), nav_generation, nav_attempt_generation_);
    return;
  }
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    current_goal_handle_ = goal_handle;
    // Acceptance 確定時に current_target_name_ を更新 (pause / resume が active
    // nav の target として参照する用途)。reject 時は更新しない。
    current_target_name_ = target;
    publish_nav_status("navigating", target);
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void MapoiNav2Bridge::feedback_callback(
  size_t nav_generation,
  GoalHandleFollowWaypoints::SharedPtr,
  const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
{
  // stale check (Codex review #147 round 3 medium): 旧 route の feedback が新 navigation
  // 受理後に届いた場合、current_waypoint_index_ を誤更新して次回 pause 時の paused_waypoints_
  // slice が壊れる risk があるため、bound generation と照合して stale なら無視する。
  if (nav_generation != nav_attempt_generation_) {
    return;
  }
  current_waypoint_index_ = feedback->current_waypoint;
}

void MapoiNav2Bridge::result_callback(std::string target, size_t nav_generation,
                                       const GoalHandleFollowWaypoints::WrappedResult & result)
{
  // stale check (Codex review #147 round 1 + 2 high): route A 実行中に別 navigation を
  // 開始し、A の result が新 nav 受理後に届くケースで、新 nav の current_route_poi_names_
  // / current_goal_handle_ を消さないよう、bound generation と現在 generation を照合する。
  if (nav_generation != nav_attempt_generation_) {
    RCLCPP_INFO(this->get_logger(),
      "Stale FollowWaypoints result for route '%s' (gen=%zu, current=%zu); ignoring.",
      target.c_str(), nav_generation, nav_attempt_generation_);
    return;
  }
  current_goal_handle_.reset();
  // bound target を使う (#104 race fix)。共有 current_target_name_ は別 nav の
  // 開始で上書きされる可能性があるため読まない。
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      reset_nav_state();
      publish_nav_status("succeeded", target);
      RCLCPP_INFO(this->get_logger(), "Navigation SUCCEEDED!");
      // #220 で Nav2 SUCCEEDED hook 経由の event publish は撤去
      // (PAUSED_AT は cmd_vel dwell only で trigger、STOPPED event 自体撤去)。
      break;
    case rclcpp_action::ResultCode::ABORTED:
      reset_nav_state();
      publish_nav_status("aborted", target);
      RCLCPP_ERROR(this->get_logger(), "Navigation ABORTED");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      if (is_paused_) {
        RCLCPP_INFO(this->get_logger(), "Cancel confirmed (pause triggered). Waiting for resume.");
      } else {
        reset_nav_state();
        publish_nav_status("canceled", target);
        RCLCPP_WARN(this->get_logger(), "Navigation CANCELED");
      }
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }
}

void MapoiNav2Bridge::ntp_goal_response_callback(std::string target, size_t nav_generation,
                                                  const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
  // stale check (Codex review #147 round 2 high): GOAL A の goal_response が新 navigation
  // 受理後に届く race を防ぐ。
  if (nav_generation != nav_attempt_generation_) {
    RCLCPP_INFO(this->get_logger(),
      "Stale NavigateToPose goal response for '%s' (gen=%zu, current=%zu); ignoring.",
      target.c_str(), nav_generation, nav_attempt_generation_);
    return;
  }
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal was rejected by server");
  } else {
    current_ntp_goal_handle_ = goal_handle;
    // Acceptance 確定時に current_target_name_ を更新 (pause / resume 用)。
    current_target_name_ = target;
    publish_nav_status("navigating", target);
    RCLCPP_INFO(this->get_logger(), "NavigateToPose goal accepted, waiting for result");
  }
}

void MapoiNav2Bridge::ntp_result_callback(std::string target, size_t nav_generation,
                                            const GoalHandleNavigateToPose::WrappedResult & result)
{
  // stale check (Codex review #147 round 2 high): GOAL A の result が新 navigation 受理後に
  // 届くと、新 nav の current_route_poi_names_ / current_ntp_goal_handle_ が消える race を防ぐ。
  if (nav_generation != nav_attempt_generation_) {
    RCLCPP_INFO(this->get_logger(),
      "Stale NavigateToPose result for '%s' (gen=%zu, current=%zu); ignoring.",
      target.c_str(), nav_generation, nav_attempt_generation_);
    return;
  }
  current_ntp_goal_handle_.reset();
  // bound target を使う (#104 race fix)。
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      reset_nav_state();
      publish_nav_status("succeeded", target);
      RCLCPP_INFO(this->get_logger(), "NavigateToPose SUCCEEDED!");
      // #220 で Nav2 SUCCEEDED hook 経由の event publish は撤去。
      break;
    case rclcpp_action::ResultCode::ABORTED:
      reset_nav_state();
      publish_nav_status("aborted", target);
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose ABORTED");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      if (is_paused_) {
        RCLCPP_INFO(this->get_logger(), "Cancel confirmed (pause triggered). Waiting for resume.");
      } else {
        reset_nav_state();
        publish_nav_status("canceled", target);
        RCLCPP_WARN(this->get_logger(), "NavigateToPose CANCELED");
      }
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose unknown result code");
      break;
  }
}

void MapoiNav2Bridge::publish_nav_status(const std::string & status, const std::string & target)
{
  std_msgs::msg::String msg;
  // target 空なら "status" のみ、有りなら "status:target" 形式で送る (#104)。
  // subscriber 側 (mapoi_panel / mapoi_webui_node) は : split で target を復元。
  // 後方互換: 旧 "status" 単独 payload しか読まない subscriber は target 部を
  // 読まないだけで従来通り動く。
  msg.data = target.empty() ? status : status + ':' + target;
  nav_status_pub_->publish(msg);
}

void MapoiNav2Bridge::publish_backend_status()
{
  // Nav2 bridge としての readiness を 1Hz polling で集約して publish する (#198)。
  // contract は minimal 3 フィールドだけ (#205 review): bridge 実装者は backend_ready を
  // 真にするだけで mapoi UI と統合できる。Per-capability の内訳が必要なら reason 文字列に
  // 詰める。Localization readiness は別軸 (#209) で、本 msg では扱わない。
  // 二重管理に見える点 (1Hz timer の集約 + 各 cb 内 `action_server_is_ready()` 即時判定) は
  // 意図的: cb 内で 1Hz timer の cache を読むと最大 1 秒の lag が発生し、operator が ready
  // 表示直後に Run を押した場合に偽の backend_unavailable を出しかねない。即時判定で current
  // を見る (#205 review low #2)。
  // backend_ready の AND に `select_map` service を含めるのは README contract が「map switch
  // を含む」と書いていることと整合させるため (#205 round 3 review high)。bridge 単独起動で
  // mapoi_server が居ない構成では `backend_ready=false` になるが、これは妥当な挙動。
  // この AND は Nav2 bridge が `goal` / `route` / `switch_map` の 3 capability 全部を expose
  // する前提に閉じた算出 (#207)。custom bridge は自前で expose する capability だけを AND
  // すること (例: goal-only bridge なら `backend_ready = goal_ready`)。詳細は
  // `mapoi_interfaces/msg/NavigationBackendStatus.msg` 冒頭コメント参照。
  const bool goal_ready =
    nav_to_pose_client_ && nav_to_pose_client_->action_server_is_ready();
  const bool route_ready =
    action_client_ && action_client_->action_server_is_ready();
  const bool switch_map_ready =
    select_map_client_ && select_map_client_->service_is_ready();

  mapoi_interfaces::msg::NavigationBackendStatus msg;
  msg.backend_type = "nav2";
  msg.backend_ready = goal_ready && route_ready && switch_map_ready;
  if (!msg.backend_ready) {
    std::vector<std::string> missing;
    if (!goal_ready) {
      missing.emplace_back("navigate_to_pose action");
    }
    if (!route_ready) {
      missing.emplace_back("follow_waypoints action");
    }
    if (!switch_map_ready) {
      missing.emplace_back("select_map service");
    }
    std::string joined;
    for (size_t i = 0; i < missing.size(); ++i) {
      if (i > 0) {
        joined += ", ";
      }
      joined += missing[i];
    }
    msg.reason = "not ready: " + joined;
  }
  backend_status_pub_->publish(msg);
}

void MapoiNav2Bridge::mapoi_cancel_cb(const std_msgs::msg::String::SharedPtr msg)
{
  (void)msg;
  bool canceled = false;
  if (current_goal_handle_) {
    RCLCPP_INFO(this->get_logger(), "Canceling FollowWaypoints goal...");
    action_client_->async_cancel_goal(current_goal_handle_);
    canceled = true;
  }
  if (current_ntp_goal_handle_) {
    RCLCPP_INFO(this->get_logger(), "Canceling NavigateToPose goal...");
    nav_to_pose_client_->async_cancel_goal(current_ntp_goal_handle_);
    canceled = true;
  }
  if (!canceled) {
    RCLCPP_WARN(this->get_logger(), "No active navigation goal to cancel.");
  }
  reset_nav_state();
}

void MapoiNav2Bridge::reset_nav_state()
{
  nav_mode_ = NavMode::IDLE;
  is_paused_ = false;
  paused_goal_pose_ = geometry_msgs::msg::PoseStamped{};
  current_route_waypoints_.clear();
  paused_waypoints_.clear();
  current_waypoint_index_ = 0;
  // active route POI set もここで clear (#143)。route 終端 / cancel / failure 等で route が
  // 終わったあとに pause 発火条件が誤って残らないようにする。
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_route_poi_names_.clear();
  }
}

void MapoiNav2Bridge::mapoi_pause_cb(const std_msgs::msg::String::SharedPtr msg)
{
  (void)msg;

  if (is_paused_) {
    RCLCPP_WARN(this->get_logger(), "Already paused — ignoring mapoi/nav/pause.");
    return;
  }

  if (nav_mode_ == NavMode::GOAL && current_ntp_goal_handle_) {
    RCLCPP_INFO(this->get_logger(), "Pausing NavigateToPose goal.");
    nav_to_pose_client_->async_cancel_goal(current_ntp_goal_handle_);
    is_paused_ = true;
    publish_nav_status("paused", current_target_name_);

  } else if (nav_mode_ == NavMode::ROUTE && current_goal_handle_) {
    uint32_t from = std::min(
      current_waypoint_index_,
      static_cast<uint32_t>(current_route_waypoints_.size()));
    paused_waypoints_ = std::vector<geometry_msgs::msg::PoseStamped>(
      current_route_waypoints_.begin() + from,
      current_route_waypoints_.end());
    RCLCPP_INFO(this->get_logger(),
      "Pausing FollowWaypoints: saving %zu remaining waypoints (from index %u).",
      paused_waypoints_.size(), from);
    action_client_->async_cancel_goal(current_goal_handle_);
    is_paused_ = true;
    publish_nav_status("paused", current_target_name_);

  } else {
    RCLCPP_WARN(this->get_logger(), "mapoi/nav/pause received but no active navigation.");
  }
}

void MapoiNav2Bridge::mapoi_resume_cb(const std_msgs::msg::String::SharedPtr msg)
{
  (void)msg;

  if (!is_paused_) {
    RCLCPP_WARN(this->get_logger(), "Not paused — ignoring mapoi/nav/resume.");
    return;
  }
  is_paused_ = false;

  // Resume の readiness check も即時判定に統一 (#198 review high): blocking wait は
  // single-thread executor で他 callback を止めるため避ける。1Hz backend_status timer で
  // readiness は維持されており、operator は ready 表示を見て resume を押す前提。
  if (nav_mode_ == NavMode::GOAL) {
    if (!nav_to_pose_client_->action_server_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available for resume.");
      is_paused_ = true;
      publish_nav_status("backend_unavailable", current_target_name_);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Resuming NavigateToPose goal.");

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = paused_goal_pose_;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    // resume: pause 直前と同じ active nav の継続だが、resume 自体は別 action attempt なので
    // generation を進めて pre-pause cancel result を stale 化する (Codex review #147 round 3 high)。
    // 新 navigation が割り込んできた場合の stale 判定もこの increment で同時に成立する。
    const size_t resumed_generation = ++nav_attempt_generation_;
    send_goal_options.goal_response_callback =
      [this, target = current_target_name_, resumed_generation](const GoalHandleNavigateToPose::SharedPtr & h) {
        this->ntp_goal_response_callback(target, resumed_generation, h);
      };
    send_goal_options.result_callback =
      [this, target = current_target_name_, resumed_generation](const GoalHandleNavigateToPose::WrappedResult & r) {
        this->ntp_result_callback(target, resumed_generation, r);
      };

    nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);

  } else if (nav_mode_ == NavMode::ROUTE) {
    if (paused_waypoints_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot resume: no saved waypoints.");
      return;
    }
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "FollowWaypoints action server not available for resume.");
      is_paused_ = true;
      publish_nav_status("backend_unavailable", current_target_name_);
      return;
    }
    current_route_waypoints_ = paused_waypoints_;
    paused_waypoints_.clear();
    current_waypoint_index_ = 0;

    RCLCPP_INFO(this->get_logger(),
      "Resuming FollowWaypoints with %zu remaining waypoints.", current_route_waypoints_.size());

    auto goal_msg = FollowWaypoints::Goal();
    goal_msg.poses = current_route_waypoints_;

    auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
    // resume: pause 直前と同じ active nav の継続だが、resume 自体は別 action attempt なので
    // generation を進めて pre-pause cancel result を stale 化する (Codex review #147 round 3 high)。
    const size_t resumed_generation = ++nav_attempt_generation_;
    send_goal_options.goal_response_callback =
      [this, target = current_target_name_, resumed_generation](const GoalHandleFollowWaypoints::SharedPtr & h) {
        this->goal_response_callback(target, resumed_generation, h);
      };
    send_goal_options.feedback_callback =
      [this, resumed_generation](GoalHandleFollowWaypoints::SharedPtr h,
                                  const std::shared_ptr<const FollowWaypoints::Feedback> f) {
        this->feedback_callback(resumed_generation, h, f);
      };
    send_goal_options.result_callback =
      [this, target = current_target_name_, resumed_generation](
        const GoalHandleFollowWaypoints::WrappedResult & r) {
        this->result_callback(target, resumed_generation, r);
      };

    action_client_->async_send_goal(goal_msg, send_goal_options);

  } else {
    RCLCPP_WARN(this->get_logger(), "mapoi/nav/resume: inconsistent state (paused but IDLE).");
  }
}

// --- POI radius event detection methods ---

void MapoiNav2Bridge::fetch_system_tags()
{
  if (!tag_defs_client_->service_is_ready()) {
    RCLCPP_INFO(this->get_logger(), "get_tag_definitions service not available yet, waiting...");
    return;  // tolerance_check_callback will retry via guard check
  }
  auto request = std::make_shared<mapoi_interfaces::srv::GetTagDefinitions::Request>();
  tag_defs_client_->async_send_request(
    request, std::bind(&MapoiNav2Bridge::on_system_tags_received, this, _1));
}

void MapoiNav2Bridge::on_system_tags_received(
  rclcpp::Client<mapoi_interfaces::srv::GetTagDefinitions>::SharedFuture future)
{
  auto result = future.get();
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get tag definitions.");
    return;
  }
  system_tags_.clear();
  for (const auto & def : result->definitions) {
    if (def.is_system) {
      system_tags_.insert(def.name);
    }
  }
  system_tags_loaded_ = true;
  RCLCPP_INFO(this->get_logger(), "Loaded %zu system tags for POI event filtering.", system_tags_.size());
  rebuild_event_pois();
  // 起動時に POI リストを取得（mapoi/config_path QoS 不一致のフォールバック）
  get_pois_list();
}

void MapoiNav2Bridge::on_config_path_changed(const std_msgs::msg::String::SharedPtr msg)
{
  // mapoi_server は config path 文字列を周期 publish (default 5s) する。path だけで dedup すると
  // map switch (path 変更) は拾えるが、WebUI/Panel Save (path 不変、内容のみ変更) を取りこぼし、
  // event_pois_ が古いまま radius event 監視が外れる。YAML mtime も併せて比較する (PR #78 と同 pattern)。
  const std::string & current_path = msg->data;
  std::filesystem::file_time_type current_mtime{};
  std::error_code ec;
  auto stat_mtime = std::filesystem::last_write_time(current_path, ec);
  if (!ec) {
    current_mtime = stat_mtime;
    if (current_path == last_config_path_ && current_mtime == last_config_mtime_) {
      return;  // 周期 publish (path も内容も不変) → skip
    }
  }
  // stat 失敗時は dedup 不能とみなし refresh を試みる (起動 race などで一時的に発生し得る)

  // guard は fetch 成功時 (on_pois_info_received) に確定。失敗時は pending のまま、
  // 次回 publish (周期 5s) で path/mtime が同じでも guard が前回値のままなので retry される。
  pending_config_path_ = current_path;
  pending_config_mtime_ = current_mtime;
  pending_guard_active_ = true;

  RCLCPP_INFO(this->get_logger(), "Map config changed: %s — refreshing POI list.", current_path.c_str());
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    poi_inside_state_.clear();
    event_pois_.clear();
  }
  get_pois_list();
}

void MapoiNav2Bridge::rebuild_event_pois()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  event_pois_ = pois_list_;
  RCLCPP_INFO(this->get_logger(), "Monitoring %zu POIs for radius events.", event_pois_.size());
}

void MapoiNav2Bridge::tolerance_check_callback()
{
  if (!system_tags_loaded_) {
    fetch_system_tags();
    return;
  }

  std::string map_frame = this->get_parameter("map_frame").as_string();
  std::string base_frame = this->get_parameter("base_frame").as_string();

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(map_frame, base_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    // TF not yet available — silently skip
    return;
  }

  double rx = transform.transform.translation.x;
  double ry = transform.transform.translation.y;
  double hysteresis = this->get_parameter("hysteresis_exit_multiplier").as_double();
  // PAUSED_AT 判定 (#220): zero_velocity が dwell threshold 以上続いたら停止扱い。
  // 全 POI 共通の判定なので timer tick 内で 1 回計算して使い回す。
  bool zero_velocity_dwelled = false;
  if (zero_velocity_active_) {
    const double dwell = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_zero_velocity_start_).count();
    zero_velocity_dwelled = (dwell >= stopped_dwell_time_sec_);
  }

  // pause タグ POI の VISITED を収集（mutex 外で auto-pause trigger するため）
  std::string pause_triggered_poi;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (event_pois_.empty()) {
      return;
    }

    // EVENT_VISITED は route 走行中 + route 登録 POI のみが publish 対象 (#220)。
    // route 走行外 (IDLE / GOAL mode) や non-route POI への侵入は通知しない。
    const bool route_active = (nav_mode_ == NavMode::ROUTE);

    for (const auto & poi : event_pois_) {
      double dist = distance_2d(poi.pose, rx, ry);
      bool was_inside = poi_inside_state_[poi.name];

      // route 登録 POI かつ ROUTE mode 走行中なら event 発火対象 (#220)。
      const bool is_route_poi = route_active &&
        current_route_poi_names_.find(poi.name) != current_route_poi_names_.end();

      // pause 自動発火は ROUTE 走行中 + active route POI + "pause" タグの 3 条件 (#143)。
      // (is_route_poi と前提が一致するため、is_pause_eligible で同じ check を行う。)
      const bool is_pause_poi = is_pause_eligible(poi, nav_mode_, current_route_poi_names_);

      if (!was_inside && dist <= poi.tolerance.xy) {
        // VISITED event: route POI のみ publish
        poi_inside_state_[poi.name] = true;
        if (is_route_poi) {
          mapoi_interfaces::msg::PoiEvent event;
          event.event_type = mapoi_interfaces::msg::PoiEvent::EVENT_VISITED;
          event.poi = poi;
          event.stamp = this->now();
          poi_event_pub_->publish(event);
          RCLCPP_INFO(this->get_logger(), "POI VISITED: %s (dist=%.2f, tolerance.xy=%.2f)",
                      poi.name.c_str(), dist, poi.tolerance.xy);
        }
        // pauseタグがあれば自動pause対象 (active route POI かつ ROUTE 走行中のみ)
        if (is_pause_poi) {
          pause_triggered_poi = poi.name;
        }
      } else if (was_inside && dist > poi.tolerance.xy * hysteresis) {
        // EXIT event: route POI のみ publish
        poi_inside_state_[poi.name] = false;
        // PAUSED_AT 発火済 flag も EXIT で reset (#220 lifecycle: VISITED → [PAUSED_AT] → EXIT)
        poi_paused_at_published_[poi.name] = false;
        if (is_route_poi) {
          mapoi_interfaces::msg::PoiEvent event;
          event.event_type = mapoi_interfaces::msg::PoiEvent::EVENT_EXIT;
          event.poi = poi;
          event.stamp = this->now();
          poi_event_pub_->publish(event);
          RCLCPP_INFO(this->get_logger(), "POI EXIT: %s (dist=%.2f, tolerance.xy=%.2f)",
                      poi.name.c_str(), dist, poi.tolerance.xy);
        }
        continue;  // PAUSED_AT 判定不要 (今 tick で OUTSIDE 化)
      }

      // PAUSED_AT 判定 (#220): inside かつ pause タグ持ち かつ cmd_vel dwell かつ未発火
      if (!poi_inside_state_[poi.name]) continue;
      if (!is_pause_poi) continue;
      if (!zero_velocity_dwelled) continue;
      if (poi_paused_at_published_[poi.name]) continue;
      poi_paused_at_published_[poi.name] = true;
      mapoi_interfaces::msg::PoiEvent event;
      event.event_type = mapoi_interfaces::msg::PoiEvent::EVENT_PAUSED_AT;
      event.poi = poi;
      event.stamp = this->now();
      poi_event_pub_->publish(event);
      RCLCPP_INFO(this->get_logger(), "POI PAUSED_AT: %s", poi.name.c_str());
    }
  }  // data_mutex_ をここで解放

  // pause タグ POI の VISITED → 自動 pause (is_pause_poi 判定で route + ROUTE mode を確認済)
  if (!pause_triggered_poi.empty() && !is_paused_) {
    RCLCPP_INFO(this->get_logger(),
      "Auto-pausing route navigation: entered pause POI '%s'", pause_triggered_poi.c_str());
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = "poi_event:" + pause_triggered_poi;
    mapoi_pause_cb(msg);
  }
}

void MapoiNav2Bridge::clear_current_route_poi_names_()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_route_poi_names_.clear();
}

double MapoiNav2Bridge::distance_2d(const geometry_msgs::msg::Pose & poi_pose, double rx, double ry)
{
  double dx = poi_pose.position.x - rx;
  double dy = poi_pose.position.y - ry;
  return std::sqrt(dx * dx + dy * dy);
}

// --- STOPPED / RESUMED 判定 (#140) ---

void MapoiNav2Bridge::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 線速ノルム (3D) と角速度絶対値 (yaw 軸) の両方が閾値以下のとき「停止」とみなす。
  // 撮影シナリオでは「その場旋回も停止扱いしない」前提のため、angular.z も判定に含める。
  // 単位が異なるが同一閾値を使う割り切り (param 説明参照、#140)。
  const double linear_norm = std::sqrt(
    msg->linear.x * msg->linear.x +
    msg->linear.y * msg->linear.y +
    msg->linear.z * msg->linear.z);
  const double angular_norm = std::abs(msg->angular.z);
  const bool zero_now =
    (linear_norm < stopped_speed_threshold_) && (angular_norm < stopped_speed_threshold_);
  if (zero_now) {
    if (!zero_velocity_active_) {
      zero_velocity_active_ = true;
      last_zero_velocity_start_ = std::chrono::steady_clock::now();
    }
  } else {
    zero_velocity_active_ = false;
  }
}

#ifndef UNIT_TEST
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapoiNav2Bridge>();
  // MultiThreadedExecutor で spin (#213): backend_status timer の Reentrant callback_group が
  // 別 thread で動くようにする。thread 数は明示的に 2 を指定する: default の
  // `std::thread::hardware_concurrency()` は Docker CPU 制限環境などで 1/0 を返し得るため、
  // 1 thread に縮退すると本 PR の修正 (blocking 中も timer を回す) が成立しない (#214 codex
  // review medium)。backend_status timer (Reentrant) + 他 callback (default MutuallyExclusive)
  // で独立 thread が 2 本あれば足りるので、上限を絞ることでリソースも節約できる。
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
#endif
