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

  // initialpose subscriber and publisher
  mapoi_initialpose_poi_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_initialpose_poi", 1, std::bind(&MapoiNavServer::mapoi_initialpose_poi_cb, this, std::placeholders::_1));
  nav2_initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1);

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
  nav_status_pub_ = this->create_publisher<std_msgs::msg::String>("mapoi_nav_status", 10);

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
          geometry_msgs::msg::PoseStamped goal_pose;
          goal_pose.header.frame_id = "map";
          goal_pose.header.stamp = this->now();
          goal_pose.pose = poi.pose;

          // Use NavigateToPose action client for result feedback
          if (!this->nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "NavigateToPose action not available, falling back to topic");
            nav2_goal_pose_pub_->publish(goal_pose);
            return;
          }

          paused_goal_pose_ = goal_pose;
          nav_mode_ = NavMode::GOAL;
          is_paused_ = false;

          auto goal_msg = NavigateToPose::Goal();
          goal_msg.pose = goal_pose;

          auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
          send_goal_options.goal_response_callback =
            std::bind(&MapoiNavServer::ntp_goal_response_callback, this, _1);
          send_goal_options.result_callback =
            std::bind(&MapoiNavServer::ntp_result_callback, this, _1);

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

  auto route_request = std::make_shared<mapoi_interfaces::srv::GetRoutePois::Request>();
  route_request->route_name = msg->data;

  this->route_client_->async_send_request(
    route_request, std::bind(&MapoiNavServer::on_route_received, this, _1));
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
  auto_publish_initial_pose();
}

std::vector<mapoi_interfaces::msg::PointOfInterest> MapoiNavServer::select_initial_pose_pois(
  const std::vector<mapoi_interfaces::msg::PointOfInterest> & pois)
{
  std::vector<mapoi_interfaces::msg::PointOfInterest> matched;
  for (const auto & poi : pois) {
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

  // AMCL より早く起動した場合の subscription readiness race を回避するため、
  // /initialpose subscriber を一定時間 wait してから publish する (#33)。
  // SwitchMap 等で AMCL が既に起動している経路では即時 return するので overhead なし。
  wait_for_initialpose_subscriber(std::chrono::seconds(10));

  publish_initial_pose(matched[0].pose, "auto from POI '" + matched[0].name + "'");
  last_initial_pose_config_path_ = last_config_path_;
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

void MapoiNavServer::wait_for_initialpose_subscriber(std::chrono::seconds timeout)
{
  // 既に subscriber がいれば即 return (SwitchMap 等の AMCL 起動済み経路)
  if (nav2_initialpose_pub_->get_subscription_count() > 0) {
    return;
  }
  RCLCPP_INFO(this->get_logger(),
    "Waiting for /initialpose subscriber (e.g. AMCL) up to %lds...",
    static_cast<long>(timeout.count()));

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (nav2_initialpose_pub_->get_subscription_count() > 0) {
      RCLCPP_INFO(this->get_logger(), "/initialpose subscriber detected; proceeding to publish.");
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  RCLCPP_WARN(this->get_logger(),
    "/initialpose subscriber not detected within %lds; publishing anyway "
    "(AMCL may miss the message; user can re-publish via WebUI/RViz/mapoi_initialpose_poi).",
    static_cast<long>(timeout.count()));
}

void MapoiNavServer::on_route_received(rclcpp::Client<mapoi_interfaces::srv::GetRoutePois>::SharedFuture future)
{
  auto result = future.get();
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get Route info.");
    return;
  }

  const auto & route_poi = result->pois_list;
  RCLCPP_INFO(this->get_logger(), "Received Route with %zu waypoints.", route_poi.size());

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

  current_route_waypoints_ = waypoints;
  current_waypoint_index_ = 0;
  nav_mode_ = NavMode::ROUTE;
  is_paused_ = false;

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

  send_goal_options.goal_response_callback =
    std::bind(&MapoiNavServer::goal_response_callback, this, _1);

  send_goal_options.feedback_callback =
    std::bind(&MapoiNavServer::feedback_callback, this, _1, _2);

  send_goal_options.result_callback =
    std::bind(&MapoiNavServer::result_callback, this, _1);

  this->action_client_->async_send_goal(goal_msg, send_goal_options);
}

void MapoiNavServer::goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    current_goal_handle_ = goal_handle;
    publish_nav_status("navigating");
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void MapoiNavServer::feedback_callback(
  GoalHandleFollowWaypoints::SharedPtr,
  const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
{
  current_waypoint_index_ = feedback->current_waypoint;
}

void MapoiNavServer::result_callback(const GoalHandleFollowWaypoints::WrappedResult & result)
{
  current_goal_handle_.reset();
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      reset_nav_state();
      publish_nav_status("succeeded");
      RCLCPP_INFO(this->get_logger(), "Navigation SUCCEEDED!");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      reset_nav_state();
      publish_nav_status("aborted");
      RCLCPP_ERROR(this->get_logger(), "Navigation ABORTED");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      if (is_paused_) {
        RCLCPP_INFO(this->get_logger(), "Cancel confirmed (pause triggered). Waiting for resume.");
      } else {
        reset_nav_state();
        publish_nav_status("canceled");
        RCLCPP_WARN(this->get_logger(), "Navigation CANCELED");
      }
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }
}

void MapoiNavServer::ntp_goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal was rejected by server");
  } else {
    current_ntp_goal_handle_ = goal_handle;
    publish_nav_status("navigating");
    RCLCPP_INFO(this->get_logger(), "NavigateToPose goal accepted, waiting for result");
  }
}

void MapoiNavServer::ntp_result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
{
  current_ntp_goal_handle_.reset();
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      reset_nav_state();
      publish_nav_status("succeeded");
      RCLCPP_INFO(this->get_logger(), "NavigateToPose SUCCEEDED!");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      reset_nav_state();
      publish_nav_status("aborted");
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose ABORTED");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      if (is_paused_) {
        RCLCPP_INFO(this->get_logger(), "Cancel confirmed (pause triggered). Waiting for resume.");
      } else {
        reset_nav_state();
        publish_nav_status("canceled");
        RCLCPP_WARN(this->get_logger(), "NavigateToPose CANCELED");
      }
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose unknown result code");
      break;
  }
}

void MapoiNavServer::publish_nav_status(const std::string & status)
{
  std_msgs::msg::String msg;
  msg.data = status;
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
    publish_nav_status("paused");

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
    publish_nav_status("paused");

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
    send_goal_options.goal_response_callback =
      std::bind(&MapoiNavServer::ntp_goal_response_callback, this, _1);
    send_goal_options.result_callback =
      std::bind(&MapoiNavServer::ntp_result_callback, this, _1);

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
    send_goal_options.goal_response_callback =
      std::bind(&MapoiNavServer::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&MapoiNavServer::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&MapoiNavServer::result_callback, this, _1);

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
  if (msg->data == last_config_path_) {
    return;
  }
  last_config_path_ = msg->data;
  RCLCPP_INFO(this->get_logger(), "Map config changed: %s — refreshing POI list.", msg->data.c_str());
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

      // pause タグを持つか確認
      bool is_pause_poi = false;
      for (const auto & tag : poi.tags) {
        if (tag == "pause") { is_pause_poi = true; break; }
      }

      if (!was_inside && dist <= poi.radius) {
        // ENTER event: 全POIでPoiEvent発行
        poi_inside_state_[poi.name] = true;
        mapoi_interfaces::msg::PoiEvent event;
        event.event_type = mapoi_interfaces::msg::PoiEvent::EVENT_ENTER;
        event.poi = poi;
        event.stamp = this->now();
        poi_event_pub_->publish(event);
        RCLCPP_INFO(this->get_logger(), "POI ENTER: %s (dist=%.2f, radius=%.2f)", poi.name.c_str(), dist, poi.radius);
        // pauseタグがあれば自動pause対象
        if (is_pause_poi) {
          pause_triggered_poi = poi.name;
        }
      } else if (was_inside && dist > poi.radius * hysteresis) {
        // EXIT event: 全POIでPoiEvent発行
        poi_inside_state_[poi.name] = false;
        mapoi_interfaces::msg::PoiEvent event;
        event.event_type = mapoi_interfaces::msg::PoiEvent::EVENT_EXIT;
        event.poi = poi;
        event.stamp = this->now();
        poi_event_pub_->publish(event);
        RCLCPP_INFO(this->get_logger(), "POI EXIT: %s (dist=%.2f, radius=%.2f)", poi.name.c_str(), dist, poi.radius);
      }
    }
  }  // data_mutex_ をここで解放

  // pause タグ POI の ENTER → 走行中かつ未一時停止なら自動 pause
  if (!pause_triggered_poi.empty() && nav_mode_ != NavMode::IDLE && !is_paused_) {
    RCLCPP_INFO(this->get_logger(),
      "Auto-pausing navigation: entered pause POI '%s'", pause_triggered_poi.c_str());
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = "poi_event:" + pause_triggered_poi;
    mapoi_pause_cb(msg);
  }
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
