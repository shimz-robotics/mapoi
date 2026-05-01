#include "mapoi_server/mapoi_nav_server.hpp"

#include <chrono>
#include <functional>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

MapoiNavServer::MapoiNavServer(const rclcpp::NodeOptions & options)
: Node("mapoi_nav_server", options)
{
  this->get_logger().set_level(rclcpp::Logger::Level::Info);

  // localization-agnostic 化のための parameter:
  // - initial_pose_topic: 配信先 topic 名 (default `/initialpose`、AMCL/slam_toolbox 等の de-facto standard)
  // - initialpose_retry_interval_sec / initialpose_retry_max_attempts: subscriber 後起動時の
  //   async retry 設定 (#152)。default 0.1s × 50 = 最大 5 秒待つ。
  this->declare_parameter<std::string>("initial_pose_topic", "/initialpose");
  this->declare_parameter<double>("initialpose_retry_interval_sec", 0.1);
  this->declare_parameter<int>("initialpose_retry_max_attempts", 50);
  // subscriber 検知後 republish 回数 (#149 round 5 medium)。
  // AMCL が「subscriber visible だが処理 ready 直前」のケースで初回 publish を取りこぼすことが
  // あるため、検知後に N 回 republish する。1 = 検知時の 1 回のみ (= 旧挙動)。
  this->declare_parameter<int>("initialpose_post_subscribe_republish_count", 3);

  // initialpose subscriber and publisher
  // mapoi_server が initial pose POI 名を transient_local で publish する (#144) ので、
  // 後起動でも latched 値を受信できるよう sub も transient_local に揃える。
  mapoi_initialpose_poi_sub_ = this->create_subscription<mapoi_interfaces::msg::InitialPoseRequest>(
    "mapoi/initialpose_poi", rclcpp::QoS(1).transient_local(),
    std::bind(&MapoiNavServer::mapoi_initialpose_poi_cb, this, std::placeholders::_1));
  mapoi_initialpose_poi_pub_ = this->create_publisher<mapoi_interfaces::msg::InitialPoseRequest>(
    "mapoi/initialpose_poi", rclcpp::QoS(1).transient_local());
  nav2_initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    this->get_parameter("initial_pose_topic").as_string(), 1);

  // goal_pose subscriber and publisher
  mapoi_goal_pose_poi_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi/nav/goal_pose_poi", 1, std::bind(&MapoiNavServer::mapoi_goal_pose_poi_cb, this, std::placeholders::_1));
  nav2_goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);

  // route subscriber
  mapoi_route_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi/nav/route", 1, std::bind(&MapoiNavServer::mapoi_route_cb, this, std::placeholders::_1));

  // map switch subscriber
  mapoi_switch_map_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi/nav/switch_map", 1, std::bind(&MapoiNavServer::mapoi_switch_map_cb, this, std::placeholders::_1));

  // cancel subscriber
  mapoi_cancel_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi/nav/cancel", 1, std::bind(&MapoiNavServer::mapoi_cancel_cb, this, std::placeholders::_1));

  // pause / resume subscribers
  mapoi_pause_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi/nav/pause", 1, std::bind(&MapoiNavServer::mapoi_pause_cb, this, _1));
  mapoi_resume_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi/nav/resume", 1, std::bind(&MapoiNavServer::mapoi_resume_cb, this, _1));

  this->action_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
  this->nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  // Nav status publisher
  // transient_local: 後起動 subscriber でも最後の状態を受信できる (mapoi/config_path と同 pattern)
  nav_status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "mapoi/nav/status", rclcpp::QoS(1).transient_local());

  this->pois_info_client_ = this->create_client<mapoi_interfaces::srv::GetPoisInfo>("get_pois_info");
  this->route_client_ = this->create_client<mapoi_interfaces::srv::GetRoutePois>("get_route_pois");
  this->select_map_client_ = this->create_client<mapoi_interfaces::srv::SelectMap>("select_map");

  // --- POI radius event detection ---
  this->declare_parameter<double>("tolerance_check_hz", 5.0);
  this->declare_parameter<double>("hysteresis_exit_multiplier", 1.15);
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("base_frame", "base_link");
  // STOPPED/RESUMED 判定 (#140): 線速ノルムと角速度絶対値の両方が ``stopped_speed_threshold``
  // 未満の状態が ``stopped_dwell_time_sec`` 続いたら EVENT_STOPPED publish。線速 (m/s) と角速
  // (rad/s) に同一閾値を使う割り切りで「その場旋回も停止扱いしない」用途を表現する。
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
    std::bind(&MapoiNavServer::on_config_path_changed, this, _1));

  // cmd_vel subscribe (#140): STOPPED 判定の source の一つ (もう一つは Nav2 SUCCEEDED)。
  // QoS は Nav2 の cmd_vel publisher と同じ default (reliable, depth=10)。
  const std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, 10,
    std::bind(&MapoiNavServer::cmd_vel_callback, this, _1));

  tag_defs_client_ = this->create_client<mapoi_interfaces::srv::GetTagDefinitions>("get_tag_definitions");
  fetch_system_tags();

  double hz = this->get_parameter("tolerance_check_hz").as_double();
  auto period = std::chrono::duration<double>(1.0 / hz);
  tolerance_check_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&MapoiNavServer::tolerance_check_callback, this));

  RCLCPP_INFO(this->get_logger(), "MapoiNavServer initialized.");
}

void MapoiNavServer::get_pois_list(){
  while(!this->pois_info_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for pois_info service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "pois_info service not available, waiting again...");
  }
  auto pois_info_request = std::make_shared<mapoi_interfaces::srv::GetPoisInfo::Request>();
  pois_info_client_->async_send_request(
    pois_info_request, std::bind(&MapoiNavServer::on_pois_info_received, this, _1));
}

void MapoiNavServer::mapoi_initialpose_poi_cb(
  const mapoi_interfaces::msg::InitialPoseRequest::SharedPtr msg)
{
  std::string poi_name = msg->poi_name;
  if (poi_name.empty()) {
    RCLCPP_DEBUG(this->get_logger(),
      "Received empty initialpose POI name for map '%s'; skipping.", msg->map_name.c_str());
    return;
  }
  // 注: map_name 世代検証は #149 round 10 で取り下げ (#155 で改めて検討)。
  //   round 9 で「current map と不一致なら無視」とした実装は、map switch 中に
  //   InitialPoseRequest が config_path より先に到着する正当ケースを「stale」と
  //   誤判定して捨てる regression を生むため。pending 戦略 (mismatch を保持して
  //   後で再評価) が必要だが scope が大きいので別 PR に分離する。
  //   現状は「publisher 上書き (transient_local depth=1) で stale を排除」を期待。
  RCLCPP_INFO(this->get_logger(),
    "Received initialpose POI '%s' for map '%s'.", poi_name.c_str(), msg->map_name.c_str());

  // Fetch POI list asynchronously, then publish initialpose in the callback
  if (!this->pois_info_client_->wait_for_service(2s)) {
    RCLCPP_ERROR(this->get_logger(), "get_pois_info service not available");
    return;
  }
  auto request = std::make_shared<mapoi_interfaces::srv::GetPoisInfo::Request>();
  pois_info_client_->async_send_request(
    request, [this, poi_name](rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedFuture future) {
      auto result = future.get();
      if (!result) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get POI info for initialpose.");
        return;
      }
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pois_list_ = result->pois_list;
      }
      rebuild_event_pois();

      for (const auto &poi : result->pois_list) {
        if (poi.name == poi_name) {
          // landmark + initial_pose は排他 (#85)。explicit POI 指定でも reject する。
          if (has_landmark_tag(poi)) {
            RCLCPP_ERROR(this->get_logger(),
              "POI '%s' has 'landmark' tag; cannot be used as initial pose.",
              poi_name.c_str());
            return;
          }
          // subscriber readiness race (#152): publish_initial_pose 内で subscriber 0 を検知したら
          // async retry timer が起動するので、blocking wait は不要。
          publish_initial_pose(poi.pose, "explicit POI '" + poi_name + "'");
          return;
        }
      }
      RCLCPP_WARN(this->get_logger(), "POI named '%s' not found!", poi_name.c_str());
    });
}

void MapoiNavServer::mapoi_goal_pose_poi_cb(const std_msgs::msg::String::SharedPtr msg)
{
  std::string poi_name = msg->data;
  RCLCPP_INFO(this->get_logger(), "Received POI name for goal pose: %s", poi_name.c_str());
  // 新規 goal 受信時に現在 STOPPED 状態の POI を全て RESUMED publish する (#140)。
  // 以降 Nav2 への goal 送信で robot が動き始めるため、STOPPED 中の subscriber が
  // 「停止解除」を即時に検知できる (cmd_vel callback で観測する前に publish)。
  resume_all_stopped_pois();
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

          // Use NavigateToPose action client for result feedback
          if (!this->nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(2))) {
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

void MapoiNavServer::mapoi_route_cb(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received route name: %s", msg->data.c_str());
  // 新規 route 受信時に現在 STOPPED 状態の POI を全て RESUMED publish (#140)。
  // 以降の send_goal で robot が動き始めるため、即時に「停止解除」通知する。
  resume_all_stopped_pois();
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

void MapoiNavServer::mapoi_switch_map_cb(const std_msgs::msg::String::SharedPtr msg)
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

void MapoiNavServer::on_select_map_received(
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

bool MapoiNavServer::send_load_map_request(const std::string & server_name, const std::string & map_file)
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

void MapoiNavServer::publish_initial_poi_request(const std::string & map_name, const std::string & poi_name)
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

void MapoiNavServer::on_pois_info_received(rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedFuture future)
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
  // mapoi/initialpose_poi topic に publish して mapoi_initialpose_poi_cb 経由で配信する。)
  if (pending_guard_active_) {
    last_config_path_ = pending_config_path_;
    last_config_mtime_ = pending_config_mtime_;
    pending_guard_active_ = false;
  }
}

bool MapoiNavServer::has_landmark_tag(const mapoi_interfaces::msg::PointOfInterest & poi)
{
  for (const auto & tag : poi.tags) {
    if (tag == "landmark") {
      return true;
    }
  }
  return false;
}

std::unordered_set<std::string> MapoiNavServer::build_route_poi_names(
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

bool MapoiNavServer::is_pause_eligible(
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

void MapoiNavServer::publish_initial_pose(
  const geometry_msgs::msg::Pose & pose, const std::string & source)
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = this->get_parameter("map_frame").as_string();
  msg.header.stamp = this->now();
  msg.pose.pose = pose;
  msg.pose.covariance[0] = 0.25;
  msg.pose.covariance[7] = 0.25;
  msg.pose.covariance[35] = 0.06853891945200942;
  // 過去の retry が pending な状態で新 publish が来たら、必ず先に cancel する (#149 round 4 high)。
  // でないと「新 publish 後に古い pose を retry が再送 → localization が古い位置へ戻る」回帰になる。
  if (initialpose_retry_timer_) {
    initialpose_retry_timer_->cancel();
    initialpose_retry_timer_.reset();
  }

  nav2_initialpose_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Published initial pose (%s).", source.c_str());

  // subscriber 未 ready の場合のみ async retry を起動 (#149 round 7 high 対応で round 6 を revert)。
  // 「常時 retry」は subscriber visible but not ready 取りこぼし対策には有効だが、運用中
  // (AMCL fully ready) で `mapoi/initialpose_poi` が来た際に意図しない自己位置 jump を
  // 引き起こす副作用が大きい。グレーゾーン (起動直後の visible but not ready) は捨て、
  // 「count == 0 のみ retry」=「後起動 path のみ retry」のロバスト性を優先する。
  if (nav2_initialpose_pub_->get_subscription_count() == 0) {
    schedule_initialpose_retry(pose, source);
  }
}

void MapoiNavServer::schedule_initialpose_retry(
  const geometry_msgs::msg::Pose & pose, const std::string & source)
{
  initialpose_retry_pose_ = pose;
  initialpose_retry_source_ = source;
  initialpose_retry_attempt_ = 0;
  initialpose_post_subscribe_republish_done_ = 0;
  if (initialpose_retry_timer_) {
    initialpose_retry_timer_->cancel();
    initialpose_retry_timer_.reset();
  }
  // パラメータの最低限の clamp (#149 round 4 low): 0 以下は default にフォールバック。
  // 過密タイマや無限 retry の防止。
  double interval_sec =
    this->get_parameter("initialpose_retry_interval_sec").as_double();
  if (interval_sec < 0.01) {
    RCLCPP_WARN(this->get_logger(),
      "initialpose_retry_interval_sec=%.3f is too small; clamping to 0.01.", interval_sec);
    interval_sec = 0.01;
  }
  int max_attempts =
    this->get_parameter("initialpose_retry_max_attempts").as_int();
  if (max_attempts < 1) {
    RCLCPP_WARN(this->get_logger(),
      "initialpose_retry_max_attempts=%d is invalid; clamping to 1.", max_attempts);
    max_attempts = 1;
  }
  initialpose_retry_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(interval_sec * 1000)),
    std::bind(&MapoiNavServer::initialpose_retry_callback, this));
  const int post_n =
    this->get_parameter("initialpose_post_subscribe_republish_count").as_int();
  RCLCPP_INFO(this->get_logger(),
    "Scheduled initialpose retry/republish every %.2fs "
    "(max %d wait attempts; %d post-subscribe republish).",
    interval_sec, max_attempts, post_n);
}

void MapoiNavServer::initialpose_retry_callback()
{
  if (nav2_initialpose_pub_->get_subscription_count() > 0) {
    // subscriber 検知後は N 回 republish する (#149 round 5 medium)。
    // AMCL が「subscriber visible でも処理 ready 直前」のケースで取りこぼすため。
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = this->get_parameter("map_frame").as_string();
    msg.header.stamp = this->now();
    msg.pose.pose = initialpose_retry_pose_;
    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.06853891945200942;
    nav2_initialpose_pub_->publish(msg);
    ++initialpose_post_subscribe_republish_done_;
    int post_n =
      this->get_parameter("initialpose_post_subscribe_republish_count").as_int();
    if (post_n < 1) post_n = 1;
    if (initialpose_post_subscribe_republish_done_ >= post_n) {
      RCLCPP_INFO(this->get_logger(),
        "initialpose subscriber detected; re-published %d times after %d retries (%s).",
        initialpose_post_subscribe_republish_done_, initialpose_retry_attempt_,
        initialpose_retry_source_.c_str());
      initialpose_retry_timer_->cancel();
      initialpose_retry_timer_.reset();
    }
    return;
  }
  ++initialpose_retry_attempt_;
  int max_attempts =
    this->get_parameter("initialpose_retry_max_attempts").as_int();
  if (max_attempts < 1) max_attempts = 1;
  if (initialpose_retry_attempt_ >= max_attempts) {
    RCLCPP_WARN(this->get_logger(),
      "initialpose subscriber not detected after %d retries; giving up "
      "(user can re-publish via WebUI/RViz/mapoi/initialpose_poi).",
      max_attempts);
    initialpose_retry_timer_->cancel();
    initialpose_retry_timer_.reset();
  }
}

void MapoiNavServer::on_route_received(
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

  // Send waypoints to Nav2 with action client
  while(!this->action_client_->wait_for_action_server(1s)) {
        if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for action server. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Action server not available, waiting again...");
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

void MapoiNavServer::goal_response_callback(std::string target, size_t nav_generation,
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

void MapoiNavServer::feedback_callback(
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

void MapoiNavServer::result_callback(std::string target, size_t nav_generation,
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
      // Nav2 SUCCEEDED 時点で robot は goal で停止している。target POI に対して強制 publish
      // (inside check skip で取りこぼし防止) + 他の inside POI にも STOPPED publish (#140)。
      // cmd_vel ベースの dwell 待ちを経由せず即時発火。
      stop_all_inside_pois("Nav2 FollowWaypoints succeeded", target);
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

void MapoiNavServer::ntp_goal_response_callback(std::string target, size_t nav_generation,
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

void MapoiNavServer::ntp_result_callback(std::string target, size_t nav_generation,
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
      stop_all_inside_pois("Nav2 NavigateToPose succeeded", target);
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

void MapoiNavServer::publish_nav_status(const std::string & status, const std::string & target)
{
  std_msgs::msg::String msg;
  // target 空なら "status" のみ、有りなら "status:target" 形式で送る (#104)。
  // subscriber 側 (mapoi_panel / mapoi_webui_node) は : split で target を復元。
  // 後方互換: 旧 "status" 単独 payload しか読まない subscriber は target 部を
  // 読まないだけで従来通り動く。
  msg.data = target.empty() ? status : status + ':' + target;
  nav_status_pub_->publish(msg);
}

void MapoiNavServer::mapoi_cancel_cb(const std_msgs::msg::String::SharedPtr msg)
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

void MapoiNavServer::reset_nav_state()
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

void MapoiNavServer::mapoi_pause_cb(const std_msgs::msg::String::SharedPtr msg)
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

void MapoiNavServer::mapoi_resume_cb(const std_msgs::msg::String::SharedPtr msg)
{
  (void)msg;

  if (!is_paused_) {
    RCLCPP_WARN(this->get_logger(), "Not paused — ignoring mapoi/nav/resume.");
    return;
  }
  is_paused_ = false;

  if (nav_mode_ == NavMode::GOAL) {
    if (!nav_to_pose_client_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available for resume.");
      is_paused_ = true;
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
    if (!action_client_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(this->get_logger(), "FollowWaypoints action server not available for resume.");
      is_paused_ = true;
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

void MapoiNavServer::fetch_system_tags()
{
  if (!tag_defs_client_->service_is_ready()) {
    RCLCPP_INFO(this->get_logger(), "get_tag_definitions service not available yet, waiting...");
    return;  // tolerance_check_callback will retry via guard check
  }
  auto request = std::make_shared<mapoi_interfaces::srv::GetTagDefinitions::Request>();
  tag_defs_client_->async_send_request(
    request, std::bind(&MapoiNavServer::on_system_tags_received, this, _1));
}

void MapoiNavServer::on_system_tags_received(
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

void MapoiNavServer::on_config_path_changed(const std_msgs::msg::String::SharedPtr msg)
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

void MapoiNavServer::rebuild_event_pois()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  event_pois_ = pois_list_;
  RCLCPP_INFO(this->get_logger(), "Monitoring %zu POIs for radius events.", event_pois_.size());
}

void MapoiNavServer::tolerance_check_callback()
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
  // STOPPED 判定: zero_velocity が dwell threshold 以上続いたら停止扱い (#140)。
  // 全 POI 共通の判定なので timer tick 内で 1 回計算して使い回す。
  bool zero_velocity_dwelled = false;
  if (zero_velocity_active_) {
    const double dwell = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_zero_velocity_start_).count();
    zero_velocity_dwelled = (dwell >= stopped_dwell_time_sec_);
  }

  // pause タグ POI の ENTER を収集（mutex 外で処理するため）
  std::string pause_triggered_poi;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (event_pois_.empty()) {
      return;
    }

    for (const auto & poi : event_pois_) {
      double dist = distance_2d(poi.pose, rx, ry);
      bool was_inside = poi_inside_state_[poi.name];

      // pause 自動発火は ROUTE 走行中 + active route POI + "pause" タグの 3 条件 (#143)。
      // 判定 logic は is_pause_eligible に切り出して unit test (#148)。
      const bool is_pause_poi = is_pause_eligible(poi, nav_mode_, current_route_poi_names_);

      if (!was_inside && dist <= poi.tolerance.xy) {
        // ENTER event: 全POIでPoiEvent発行
        poi_inside_state_[poi.name] = true;
        mapoi_interfaces::msg::PoiEvent event;
        event.event_type = mapoi_interfaces::msg::PoiEvent::EVENT_ENTER;
        event.poi = poi;
        event.stamp = this->now();
        poi_event_pub_->publish(event);
        RCLCPP_INFO(this->get_logger(), "POI ENTER: %s (dist=%.2f, tolerance.xy=%.2f)",
                    poi.name.c_str(), dist, poi.tolerance.xy);
        // pauseタグがあれば自動pause対象 (active route POI かつ ROUTE 走行中のみ)
        if (is_pause_poi) {
          pause_triggered_poi = poi.name;
        }
      } else if (was_inside && dist > poi.tolerance.xy * hysteresis) {
        // EXIT event: 全POIでPoiEvent発行
        poi_inside_state_[poi.name] = false;
        // STOPPED state も EXIT で clear (#140)。RESUMED publish は EXIT で兼ねる扱いとする
        // (ライフサイクル: ENTER → STOPPED → (RESUMED |) EXIT。EXIT 時に明示 RESUMED を出すと
        // 同 tick で 2 イベントが連続発火して subscriber 側の handler 設計が複雑化するため見送り)。
        poi_stopped_state_[poi.name] = false;
        mapoi_interfaces::msg::PoiEvent event;
        event.event_type = mapoi_interfaces::msg::PoiEvent::EVENT_EXIT;
        event.poi = poi;
        event.stamp = this->now();
        poi_event_pub_->publish(event);
        RCLCPP_INFO(this->get_logger(), "POI EXIT: %s (dist=%.2f, tolerance.xy=%.2f)",
                    poi.name.c_str(), dist, poi.tolerance.xy);
        continue;  // STOPPED/RESUMED 判定不要 (今 tick で OUTSIDE 化)
      }

      // STOPPED / RESUMED 判定 (#140): inside (新規 ENTER 含む) かつ velocity / RESUMED 条件。
      if (!poi_inside_state_[poi.name]) continue;
      const bool was_stopped = poi_stopped_state_[poi.name];
      const StoppedDetectionInputs in{true, was_stopped, zero_velocity_dwelled};
      const auto trans = compute_stopped_transition(in);
      if (trans == StoppedTransition::TO_STOPPED) {
        poi_stopped_state_[poi.name] = true;
        mapoi_interfaces::msg::PoiEvent event;
        event.event_type = mapoi_interfaces::msg::PoiEvent::EVENT_STOPPED;
        event.poi = poi;
        event.stamp = this->now();
        poi_event_pub_->publish(event);
        RCLCPP_INFO(this->get_logger(), "POI STOPPED: %s", poi.name.c_str());
      } else if (trans == StoppedTransition::TO_RESUMED) {
        poi_stopped_state_[poi.name] = false;
        mapoi_interfaces::msg::PoiEvent event;
        event.event_type = mapoi_interfaces::msg::PoiEvent::EVENT_RESUMED;
        event.poi = poi;
        event.stamp = this->now();
        poi_event_pub_->publish(event);
        RCLCPP_INFO(this->get_logger(), "POI RESUMED: %s", poi.name.c_str());
      }
    }
  }  // data_mutex_ をここで解放

  // pause タグ POI の ENTER → 自動 pause (is_pause_poi 判定で route + ROUTE mode を確認済)
  if (!pause_triggered_poi.empty() && !is_paused_) {
    RCLCPP_INFO(this->get_logger(),
      "Auto-pausing route navigation: entered pause POI '%s'", pause_triggered_poi.c_str());
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = "poi_event:" + pause_triggered_poi;
    mapoi_pause_cb(msg);
  }
}

void MapoiNavServer::clear_current_route_poi_names_()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_route_poi_names_.clear();
}

double MapoiNavServer::distance_2d(const geometry_msgs::msg::Pose & poi_pose, double rx, double ry)
{
  double dx = poi_pose.position.x - rx;
  double dy = poi_pose.position.y - ry;
  return std::sqrt(dx * dx + dy * dy);
}

// --- STOPPED / RESUMED 判定 (#140) ---

void MapoiNavServer::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
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

// 純関数 (副作用なし、unit test 用)。inside / was_stopped / zero_velocity / zero_velocity_dwelled
// の組み合わせから次の遷移を返す。
//
// 仕様:
//   - !inside: NONE (EXIT 側で stopped state は別途 reset する)
//   - !was_stopped + zero_velocity_dwelled (停止が dwell_time 続いた): TO_STOPPED
//   - was_stopped + !zero_velocity (動き始めた): TO_RESUMED (dwell 不要、即時)
//   - それ以外: NONE
MapoiNavServer::StoppedTransition MapoiNavServer::compute_stopped_transition(
  const StoppedDetectionInputs & in)
{
  if (!in.inside) return StoppedTransition::NONE;
  if (!in.was_stopped) {
    return in.zero_velocity_dwelled ? StoppedTransition::TO_STOPPED : StoppedTransition::NONE;
  }
  // was_stopped == true
  return in.zero_velocity_dwelled ? StoppedTransition::NONE : StoppedTransition::TO_RESUMED;
}

void MapoiNavServer::stop_all_inside_pois(
  const std::string & reason, const std::string & target_poi_name)
{
  // Nav2 SUCCEEDED 受信時に呼ぶ: 現在 inside の全 POI に対して STOPPED publish
  // (既に STOPPED state の POI は idempotent に skip)。
  // ``target_poi_name`` が指定された場合は、その POI は inside check を skip して
  // 強制的に STOPPED publish する (SUCCEEDED と tolerance_check tick の非同期による
  // 取りこぼし防止、#176 review medium #1)。
  // target POI でまだ ENTER 未発火の場合、lifecycle invariant ``ENTER → STOPPED → EXIT`` を
  // 守るため ENTER を先に publish してから STOPPED を発火する (#176 review r2 high #1)。
  // event_pois_ / poi_inside_state_ / poi_stopped_state_ を lock で守って snapshot を
  // 取り、publish は lock 外で実行 (既存 tolerance_check の pattern と整合)。
  std::vector<mapoi_interfaces::msg::PointOfInterest> to_enter;
  std::vector<mapoi_interfaces::msg::PointOfInterest> to_stop;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (const auto & poi : event_pois_) {
      const bool is_target = !target_poi_name.empty() && poi.name == target_poi_name;
      const bool was_inside = poi_inside_state_[poi.name];
      const bool eligible = is_target || was_inside;
      if (!eligible) continue;
      // target POI で inside_state がまだ false なら ENTER 先行 publish (lifecycle 整合)。
      if (is_target && !was_inside) {
        poi_inside_state_[poi.name] = true;
        to_enter.push_back(poi);
      }
      if (!poi_stopped_state_[poi.name]) {
        poi_stopped_state_[poi.name] = true;
        to_stop.push_back(poi);
      }
    }
  }
  // ENTER → STOPPED の順で publish (subscriber 側の handler 設計が ENTER を前提にできる)。
  for (const auto & poi : to_enter) {
    mapoi_interfaces::msg::PoiEvent event;
    event.event_type = mapoi_interfaces::msg::PoiEvent::EVENT_ENTER;
    event.poi = poi;
    event.stamp = this->now();
    poi_event_pub_->publish(event);
    RCLCPP_INFO(this->get_logger(),
      "POI ENTER (forced by %s): %s", reason.c_str(), poi.name.c_str());
  }
  for (const auto & poi : to_stop) {
    mapoi_interfaces::msg::PoiEvent event;
    event.event_type = mapoi_interfaces::msg::PoiEvent::EVENT_STOPPED;
    event.poi = poi;
    event.stamp = this->now();
    poi_event_pub_->publish(event);
    RCLCPP_INFO(this->get_logger(), "POI STOPPED (%s): %s", reason.c_str(), poi.name.c_str());
  }
}

void MapoiNavServer::resume_all_stopped_pois()
{
  // 新規 goal 受信時に呼ぶ: 現在 STOPPED 状態の全 POI に RESUMED publish。
  // event_pois_ と poi_stopped_state_ を lock で守って snapshot を取り、publish は lock 外。
  std::vector<mapoi_interfaces::msg::PointOfInterest> to_resume;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (const auto & poi : event_pois_) {
      if (poi_stopped_state_[poi.name]) {
        poi_stopped_state_[poi.name] = false;
        to_resume.push_back(poi);
      }
    }
  }
  for (const auto & poi : to_resume) {
    mapoi_interfaces::msg::PoiEvent event;
    event.event_type = mapoi_interfaces::msg::PoiEvent::EVENT_RESUMED;
    event.poi = poi;
    event.stamp = this->now();
    poi_event_pub_->publish(event);
    RCLCPP_INFO(this->get_logger(), "POI RESUMED (new goal): %s", poi.name.c_str());
  }
}

#ifndef UNIT_TEST
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapoiNavServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
