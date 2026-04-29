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
  // - initial_pose_subscriber_wait_timeout_sec: subscriber readiness 待ちの上限秒数 (起動の遅い localization 対応)
  this->declare_parameter<std::string>("initial_pose_topic", "/initialpose");
  this->declare_parameter<double>("initial_pose_subscriber_wait_timeout_sec", 10.0);

  // initialpose subscriber and publisher
  mapoi_initialpose_poi_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_initialpose_poi", 1, std::bind(&MapoiNavServer::mapoi_initialpose_poi_cb, this, std::placeholders::_1));
  nav2_initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    this->get_parameter("initial_pose_topic").as_string(), 1);

  // goal_pose subscriber and publisher
  mapoi_goal_pose_poi_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_goal_pose_poi", 1, std::bind(&MapoiNavServer::mapoi_goal_pose_poi_cb, this, std::placeholders::_1));
  nav2_goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);

  // route subscriber
  mapoi_route_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_route", 1, std::bind(&MapoiNavServer::mapoi_route_cb, this, std::placeholders::_1));

  // cancel subscriber
  mapoi_cancel_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_cancel", 1, std::bind(&MapoiNavServer::mapoi_cancel_cb, this, std::placeholders::_1));

  // pause / resume subscribers
  mapoi_pause_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_pause", 1, std::bind(&MapoiNavServer::mapoi_pause_cb, this, _1));
  mapoi_resume_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_resume", 1, std::bind(&MapoiNavServer::mapoi_resume_cb, this, _1));

  this->action_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
  this->nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  // Nav status publisher
  // transient_local: 後起動 subscriber でも最後の状態を受信できる (mapoi_config_path と同 pattern)
  nav_status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "mapoi_nav_status", rclcpp::QoS(1).transient_local());

  this->pois_info_client_ = this->create_client<mapoi_interfaces::srv::GetPoisInfo>("get_pois_info");
  this->route_client_ = this->create_client<mapoi_interfaces::srv::GetRoutePois>("get_route_pois");

  // --- POI radius event detection ---
  this->declare_parameter<double>("radius_check_hz", 5.0);
  this->declare_parameter<double>("hysteresis_exit_multiplier", 1.15);
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("base_frame", "base_link");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  poi_event_pub_ = this->create_publisher<mapoi_interfaces::msg::PoiEvent>("mapoi_poi_events", 10);

  config_path_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_config_path", rclcpp::QoS(1).transient_local(),
    std::bind(&MapoiNavServer::on_config_path_changed, this, _1));

  tag_defs_client_ = this->create_client<mapoi_interfaces::srv::GetTagDefinitions>("get_tag_definitions");
  fetch_system_tags();

  double hz = this->get_parameter("radius_check_hz").as_double();
  auto period = std::chrono::duration<double>(1.0 / hz);
  radius_check_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&MapoiNavServer::radius_check_callback, this));

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

void MapoiNavServer::mapoi_initialpose_poi_cb(const std_msgs::msg::String::SharedPtr msg)
{
  std::string poi_name = msg->data;
  RCLCPP_INFO(this->get_logger(), "Received POI name for initialpose: %s", poi_name.c_str());

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

          // GOAL 切替時の state 更新は action 利用可否に関わらず先に行う (Codex review #147 medium):
          // fallback (topic publish) でも nav_mode_ = GOAL にしないと、直前 route の
          // current_route_poi_names_ が残り、route-scoped pause が GOAL 走行中も走ってしまう。
          paused_goal_pose_ = goal_pose;
          nav_mode_ = NavMode::GOAL;
          is_paused_ = false;
          // 直前まで ROUTE モードだった場合に備えて active route POI set を clear (#143)。
          // GOAL モードでは route POI 限定の pause 発火 logic を走らせない。
          {
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_route_poi_names_.clear();
          }

          // Use NavigateToPose action client for result feedback
          if (!this->nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "NavigateToPose action not available, falling back to topic");
            nav2_goal_pose_pub_->publish(goal_pose);
            return;
          }

          auto goal_msg = NavigateToPose::Goal();
          goal_msg.pose = goal_pose;

          // navigation attempt generation を進めて、stale callback (旧 GOAL の result が新 nav 受理後に
          // 来る race) を識別する (Codex review #147 round 2 high)。
          const size_t my_generation = ++nav_attempt_generation_;

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
  // auto_publish_initial_pose() は last_config_path_ を見て二重発火判定するため、guard 確定を先に行う。
  if (pending_guard_active_) {
    last_config_path_ = pending_config_path_;
    last_config_mtime_ = pending_config_mtime_;
    pending_guard_active_ = false;
  }

  auto_publish_initial_pose();
}

std::vector<mapoi_interfaces::msg::PointOfInterest> MapoiNavServer::select_initial_pose_pois(
  const std::vector<mapoi_interfaces::msg::PointOfInterest> & pois)
{
  std::vector<mapoi_interfaces::msg::PointOfInterest> matched;
  for (const auto & poi : pois) {
    // landmark + initial_pose は排他 (#85)。auto-publish 候補からも除外する。
    if (has_landmark_tag(poi)) {
      continue;
    }
    for (const auto & tag : poi.tags) {
      if (tag == "initial_pose") {
        matched.push_back(poi);
        break;
      }
    }
  }
  if (matched.size() > 1) {
    std::string names;
    for (size_t i = 0; i < matched.size(); ++i) {
      if (i > 0) names += ", ";
      names += matched[i].name;
    }
    RCLCPP_WARN(this->get_logger(),
      "Multiple POIs with 'initial_pose' tag (%zu): [%s]. Using first: '%s'.",
      matched.size(), names.c_str(), matched[0].name.c_str());
  }
  return matched;
}

void MapoiNavServer::auto_publish_initial_pose()
{
  // config_path 未確定 / 同一 config で publish 済み → スキップ（起動時の二重発火防止）
  if (last_config_path_.empty() || last_config_path_ == last_initial_pose_config_path_) {
    return;
  }

  std::vector<mapoi_interfaces::msg::PointOfInterest> matched;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    matched = select_initial_pose_pois(pois_list_);
  }

  if (matched.empty()) {
    RCLCPP_INFO(this->get_logger(),
      "No POI with 'initial_pose' tag found; skipping auto initial pose.");
    last_initial_pose_config_path_ = last_config_path_;  // 同一 config の再評価を抑止
    return;
  }

  // localization (主に AMCL) より早く起動した場合の subscription readiness race を
  // 回避するため、initialpose subscriber を一定時間 wait してから publish する。
  // SwitchMap 等で localization が既に起動している経路では即時 return するので overhead なし。
  const double timeout_sec =
    this->get_parameter("initial_pose_subscriber_wait_timeout_sec").as_double();
  wait_for_initialpose_subscriber(timeout_sec);

  publish_initial_pose(matched[0].pose, "auto from POI '" + matched[0].name + "'");
  last_initial_pose_config_path_ = last_config_path_;
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
  nav2_initialpose_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Published initial pose (%s).", source.c_str());
}

void MapoiNavServer::wait_for_initialpose_subscriber(double timeout_sec)
{
  // 既に subscriber がいれば即 return (SwitchMap 等の localization 起動済み経路)
  if (nav2_initialpose_pub_->get_subscription_count() > 0) {
    return;
  }
  RCLCPP_INFO(this->get_logger(),
    "Waiting for initialpose subscriber (e.g. AMCL) up to %.1fs...", timeout_sec);

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int>(timeout_sec * 1000));
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (nav2_initialpose_pub_->get_subscription_count() > 0) {
      RCLCPP_INFO(this->get_logger(), "initialpose subscriber detected; proceeding to publish.");
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  RCLCPP_WARN(this->get_logger(),
    "initialpose subscriber not detected within %.1fs; publishing anyway "
    "(localization may miss the message; user can re-publish via WebUI/RViz/mapoi_initialpose_poi).",
    timeout_sec);
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

  // active route の POI 名 set を構築 (waypoints + landmarks 両方を radius_check で
  // pause 発火対象として扱う)。lock 下で書き込み (#143)。
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_route_poi_names_.clear();
    for (const auto & p : route_poi) {
      current_route_poi_names_.insert(p.name);
    }
    for (const auto & p : route_landmarks) {
      current_route_poi_names_.insert(p.name);
    }
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
    std::bind(&MapoiNavServer::feedback_callback, this, _1, _2);

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
  GoalHandleFollowWaypoints::SharedPtr,
  const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
{
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
    RCLCPP_WARN(this->get_logger(), "Already paused — ignoring mapoi_pause.");
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
    RCLCPP_WARN(this->get_logger(), "mapoi_pause received but no active navigation.");
  }
}

void MapoiNavServer::mapoi_resume_cb(const std_msgs::msg::String::SharedPtr msg)
{
  (void)msg;

  if (!is_paused_) {
    RCLCPP_WARN(this->get_logger(), "Not paused — ignoring mapoi_resume.");
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
    // resume: pause 直前と同じ active nav なので current_target_name_ + 現在 generation を再利用。
    // 新 navigation が割り込んできたら nav_attempt_generation_ が増分されて stale 判定 (#147)。
    const size_t resumed_generation = nav_attempt_generation_;
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
    // resume: pause 直前と同じ active nav なので current_target_name_ + 現在 generation を再利用 (#143)。
    // 新 navigation が割り込んできたら nav_attempt_generation_ が増分されて stale 判定される (#147)。
    const size_t resumed_generation = nav_attempt_generation_;
    send_goal_options.goal_response_callback =
      [this, target = current_target_name_, resumed_generation](const GoalHandleFollowWaypoints::SharedPtr & h) {
        this->goal_response_callback(target, resumed_generation, h);
      };
    send_goal_options.feedback_callback =
      std::bind(&MapoiNavServer::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      [this, target = current_target_name_, resumed_generation](
        const GoalHandleFollowWaypoints::WrappedResult & r) {
        this->result_callback(target, resumed_generation, r);
      };

    action_client_->async_send_goal(goal_msg, send_goal_options);

  } else {
    RCLCPP_WARN(this->get_logger(), "mapoi_resume: inconsistent state (paused but IDLE).");
  }
}

// --- POI radius event detection methods ---

void MapoiNavServer::fetch_system_tags()
{
  if (!tag_defs_client_->service_is_ready()) {
    RCLCPP_INFO(this->get_logger(), "get_tag_definitions service not available yet, waiting...");
    return;  // radius_check_callback will retry via guard check
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
  // 起動時に POI リストを取得（mapoi_config_path QoS 不一致のフォールバック）
  get_pois_list();
}

void MapoiNavServer::on_config_path_changed(const std_msgs::msg::String::SharedPtr msg)
{
  // mapoi_server は config path 文字列を周期 publish (default 5s) する。path だけで dedup すると
  // SwitchMap (path 変更) は拾えるが、WebUI/Panel Save (path 不変、内容のみ変更) を取りこぼし、
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

void MapoiNavServer::radius_check_callback()
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

      // pause タグを持つかつ active route の POI かを確認 (#143):
      //   - pause 自動発火は **ROUTE 走行中の active route POI のみ** に厳格化
      //   - 別 route で pause タグが付いた POI に偶然 ENTER しても発火しない
      //   - GOAL 走行 / IDLE では発火しない (操作者の意図しない停止を避ける)
      bool is_pause_poi = false;
      if (nav_mode_ == NavMode::ROUTE && current_route_poi_names_.count(poi.name) > 0) {
        for (const auto & tag : poi.tags) {
          if (tag == "pause") { is_pause_poi = true; break; }
        }
      }

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
        mapoi_interfaces::msg::PoiEvent event;
        event.event_type = mapoi_interfaces::msg::PoiEvent::EVENT_EXIT;
        event.poi = poi;
        event.stamp = this->now();
        poi_event_pub_->publish(event);
        RCLCPP_INFO(this->get_logger(), "POI EXIT: %s (dist=%.2f, tolerance.xy=%.2f)",
                    poi.name.c_str(), dist, poi.tolerance.xy);
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
