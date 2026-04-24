// mapoi_gazebo_bridge: SwitchMap 時に Gazebo Classic (gazebo_msgs) の entity を入れ替える。
//
// mapoi_config_path topic を購読し、map_name 変化を検知すると:
// - 旧マップの障害物 entity を /delete_entity で削除
// - 新マップの障害物 entity を /spawn_entity で生成 (model://... URI で)
// - ロボットも /delete_entity + /spawn_entity で initial_pose POI 座標に再生成
// Gazebo 本体は無停止で /clock, /tf, /odom, /scan の継続性を保つ。
//
// map 依存情報 (障害物 model の URI / name) は各 map の mapoi_config.yaml の
// gazebo: セクションに置き、mapoi_config_path 受信時に本 node が直接 parse する。
// ロボット依存情報 (entity_name / SDF path) は launch から parameter で渡す。
//
// NOTE: Humble (Gazebo Classic) 専用。Jazzy (gz-sim) 用は別 node
// mapoi_gz_bridge として #42 と統合した別 PR で対応予定。API が異なるため
// 同一コードでの distro 分岐はせず、別 node に分離する方針。

#include "mapoi_server/mapoi_gazebo_bridge.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
using std::placeholders::_1;


MapoiGazeboBridge::MapoiGazeboBridge()
: Node("mapoi_gazebo_bridge")
{
  this->declare_parameter<std::string>("robot_entity_name", "burger");
  this->declare_parameter<std::string>("robot_sdf_path", "");

  robot_name_ = this->get_parameter("robot_entity_name").as_string();
  robot_sdf_path_ = this->get_parameter("robot_sdf_path").as_string();

  // Reentrant callback group + MultiThreadedExecutor で worker thread から
  // async_send_request().future.wait_for() が executor の別 thread で resolve される。
  cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  spawn_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>(
    "spawn_entity", rmw_qos_profile_services_default, cb_group_);
  delete_client_ = this->create_client<gazebo_msgs::srv::DeleteEntity>(
    "delete_entity", rmw_qos_profile_services_default, cb_group_);

  auto sub_opts = rclcpp::SubscriptionOptions();
  sub_opts.callback_group = cb_group_;
  config_path_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_config_path", 10,
    std::bind(&MapoiGazeboBridge::on_config_path, this, _1),
    sub_opts);

  worker_ = std::thread(&MapoiGazeboBridge::worker_loop, this);

  RCLCPP_INFO(this->get_logger(),
    "mapoi_gazebo_bridge initialized: robot=%s, sdf=%s",
    robot_name_.c_str(),
    robot_sdf_path_.empty() ? "(not set)" : robot_sdf_path_.c_str());
}

MapoiGazeboBridge::~MapoiGazeboBridge()
{
  stop_worker_ = true;
  queue_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void MapoiGazeboBridge::on_config_path(const std_msgs::msg::String::SharedPtr msg)
{
  // 軽量。queue に push して即 return。state 操作と service call は worker。
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.push(msg->data);
  }
  queue_cv_.notify_one();
}

void MapoiGazeboBridge::worker_loop()
{
  while (!stop_worker_) {
    std::string path;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] { return !queue_.empty() || stop_worker_; });
      if (stop_worker_) {
        break;
      }
      path = queue_.front();
      queue_.pop();
    }
    try {
      process_config_path(path);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "worker exception: %s", e.what());
    }
  }
}

void MapoiGazeboBridge::process_config_path(const std::string & path)
{
  if (path == current_config_path_) {
    return;
  }

  // path の構造: <maps_path>/<map_name>/<config_file>
  std::filesystem::path p(path);
  std::string map_name = p.parent_path().filename().string();
  RCLCPP_INFO(this->get_logger(),
    "config_path changed → map_name=%s", map_name.c_str());

  if (map_name == current_map_name_) {
    current_config_path_ = path;
    return;
  }

  std::string prev_map_name = current_map_name_;

  MapGazeboInfo info;
  const bool has_info = load_gazebo_info(path, info);
  if (!has_info) {
    RCLCPP_WARN(this->get_logger(),
      "No gazebo section in %s; skipping entity swap", path.c_str());
    // state は進める (次の same map_name で早期 return するため)
    current_map_name_ = map_name;
    current_config_path_ = path;
    return;
  }

  if (prev_map_name.empty()) {
    // 起動時の最初の通知: launch が既に対応する world で起動済みと仮定。
    // 内部 state のみ更新、entity 入れ替えはしない。
    current_obstacle_name_ = info.obstacle.name;
    current_map_name_ = map_name;
    current_config_path_ = path;
    RCLCPP_INFO(this->get_logger(),
      "Initial map=%s; assuming Gazebo already loaded matching world",
      map_name.c_str());
    return;
  }

  if (switch_world(prev_map_name, map_name, info)) {
    current_map_name_ = map_name;
    current_config_path_ = path;
  } else {
    RCLCPP_WARN(this->get_logger(),
      "switch_world %s → %s failed; keeping state for retry",
      prev_map_name.c_str(), map_name.c_str());
  }
}

bool MapoiGazeboBridge::load_gazebo_info(
  const std::string & config_path, MapGazeboInfo & out)
{
  try {
    YAML::Node cfg = YAML::LoadFile(config_path);
    if (cfg["gazebo"] && cfg["gazebo"]["obstacle"]) {
      const auto & obs = cfg["gazebo"]["obstacle"];
      out.obstacle.uri = obs["uri"] ? obs["uri"].as<std::string>() : "";
      out.obstacle.name = obs["name"] ? obs["name"].as<std::string>() : "";
      out.has_gazebo = !out.obstacle.uri.empty();
    }
    if (cfg["poi"] && cfg["poi"].IsSequence()) {
      for (const auto & poi : cfg["poi"]) {
        if (!poi["tags"] || !poi["tags"].IsSequence()) {
          continue;
        }
        bool is_initial = false;
        for (const auto & tag : poi["tags"]) {
          if (tag.as<std::string>() == "initial_pose") {
            is_initial = true;
            break;
          }
        }
        if (!is_initial) {
          continue;
        }
        if (poi["pose"]) {
          const auto & pose = poi["pose"];
          out.initial_x = pose["x"] ? pose["x"].as<double>() : 0.0;
          out.initial_y = pose["y"] ? pose["y"].as<double>() : 0.0;
          out.initial_yaw = pose["yaw"] ? pose["yaw"].as<double>() : 0.0;
          out.has_initial_pose = true;
        }
        break;
      }
    }
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(this->get_logger(),
      "Failed to parse %s: %s", config_path.c_str(), e.what());
    return false;
  }
  return out.has_gazebo;
}

bool MapoiGazeboBridge::switch_world(
  const std::string & prev_map, const std::string & new_map,
  const MapGazeboInfo & info)
{
  (void)prev_map;
  (void)new_map;

  // 旧障害物 delete
  if (!current_obstacle_name_.empty()) {
    if (!delete_entity(current_obstacle_name_)) {
      RCLCPP_WARN(this->get_logger(),
        "delete obstacle failed; aborting world switch (旧 obstacle stays)");
      return false;
    }
    current_obstacle_name_.clear();
  }

  bool all_ok = true;

  // 新障害物 spawn
  if (!info.obstacle.uri.empty()) {
    if (spawn_entity_from_uri(info.obstacle.name, info.obstacle.uri)) {
      current_obstacle_name_ = info.obstacle.name;
    } else {
      all_ok = false;
    }
  }

  // robot respawn (initial_pose POI がある場合のみ)
  if (info.has_initial_pose) {
    if (!respawn_robot(info.initial_x, info.initial_y, info.initial_yaw)) {
      all_ok = false;
    }
  } else {
    RCLCPP_WARN(this->get_logger(),
      "No initial_pose POI for %s; skipping robot respawn", new_map.c_str());
  }

  return all_ok;
}

bool MapoiGazeboBridge::delete_entity(const std::string & name)
{
  if (!delete_client_->wait_for_service(10s)) {
    RCLCPP_WARN(this->get_logger(), "delete_entity service not available");
    return false;
  }
  auto req = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
  req->name = name;
  auto future = delete_client_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(),
      "delete_entity(%s) timed out", name.c_str());
    return false;
  }
  auto res = future.get();
  if (!res->success) {
    RCLCPP_WARN(this->get_logger(),
      "delete_entity(%s) failed: %s",
      name.c_str(), res->status_message.c_str());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "delete_entity(%s): success", name.c_str());
  return true;
}

bool MapoiGazeboBridge::spawn_entity_from_uri(
  const std::string & name, const std::string & uri)
{
  if (!spawn_client_->wait_for_service(10s)) {
    RCLCPP_WARN(this->get_logger(), "spawn_entity service not available");
    return false;
  }
  std::ostringstream sdf;
  sdf << "<?xml version=\"1.0\"?>"
      << "<sdf version=\"1.6\"><world name=\"default\">"
      << "<include><uri>" << uri << "</uri></include>"
      << "</world></sdf>";
  auto req = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
  req->name = name;
  req->xml = sdf.str();
  auto future = spawn_client_->async_send_request(req);
  if (future.wait_for(10s) != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(),
      "spawn_entity(%s) timed out", name.c_str());
    return false;
  }
  auto res = future.get();
  if (!res->success) {
    RCLCPP_WARN(this->get_logger(),
      "spawn_entity(%s) failed: %s",
      name.c_str(), res->status_message.c_str());
    return false;
  }
  RCLCPP_INFO(this->get_logger(),
    "spawn_entity(%s, %s): success", name.c_str(), uri.c_str());
  return true;
}

bool MapoiGazeboBridge::respawn_robot(double x, double y, double yaw)
{
  // preflight: 失敗するなら delete もしない
  if (robot_sdf_path_.empty()) {
    RCLCPP_WARN(this->get_logger(),
      "robot_sdf_path is empty; skipping respawn (robot stays at old pose)");
    return false;
  }
  if (!std::filesystem::exists(robot_sdf_path_)) {
    RCLCPP_ERROR(this->get_logger(),
      "robot_sdf_path not found: %s", robot_sdf_path_.c_str());
    return false;
  }
  if (!spawn_client_->wait_for_service(10s)) {
    RCLCPP_WARN(this->get_logger(),
      "spawn_entity service not available; skipping robot respawn");
    return false;
  }
  std::ifstream f(robot_sdf_path_);
  if (!f.is_open()) {
    RCLCPP_ERROR(this->get_logger(),
      "Failed to open robot SDF: %s", robot_sdf_path_.c_str());
    return false;
  }
  std::stringstream buf;
  buf << f.rdbuf();
  const std::string sdf = buf.str();

  // delete は失敗しても spawn を試みる
  // (前回 delete 成功 + spawn 失敗で robot 不在の状態からの retry 対応。
  // 重複があれば spawn 側が success=false で検知)
  if (!delete_entity(robot_name_)) {
    RCLCPP_INFO(this->get_logger(),
      "delete robot returned not-success; entity may already be gone. "
      "will still attempt spawn at new pose");
  }

  auto req = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
  req->name = robot_name_;
  req->xml = sdf;
  req->initial_pose.position.x = x;
  req->initial_pose.position.y = y;
  req->initial_pose.position.z = 0.01;
  req->initial_pose.orientation.z = std::sin(yaw / 2.0);
  req->initial_pose.orientation.w = std::cos(yaw / 2.0);
  auto future = spawn_client_->async_send_request(req);
  if (future.wait_for(10s) != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(),
      "respawn_robot(%s) timed out (robot may be deleted)",
      robot_name_.c_str());
    return false;
  }
  auto res = future.get();
  if (!res->success) {
    RCLCPP_WARN(this->get_logger(),
      "respawn_robot(%s) failed: %s",
      robot_name_.c_str(), res->status_message.c_str());
    return false;
  }
  RCLCPP_INFO(this->get_logger(),
    "respawn_robot(%s, %.2f, %.2f, yaw=%.2f): success",
    robot_name_.c_str(), x, y, yaw);
  return true;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapoiGazeboBridge>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
