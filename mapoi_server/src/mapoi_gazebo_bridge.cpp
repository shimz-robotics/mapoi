// mapoi_gazebo_bridge: SwitchMap 時に Gazebo Classic (gazebo_msgs) の entity を入れ替える。
//
// mapoi_config_path topic を購読し、map_name 変化を検知すると:
// - 旧マップの world_model entity を /delete_entity で削除
// - 新マップの world_model entity を /spawn_entity で生成 (model://... URI で)
// - ロボットも /delete_entity + /spawn_entity で initial_pose POI 座標に再生成
// Gazebo 本体は無停止で /clock, /tf, /odom, /scan の継続性を保つ。
//
// map 依存情報 (world_model の URI / name) は各 map の mapoi_config.yaml の
// gazebo: セクションに置き、mapoi_config_path 受信時に本 node が直接 parse する。
// ロボット依存情報 (entity_name / SDF path) は launch から parameter で渡す。
//
// NOTE: Humble (Gazebo Classic) 専用。Jazzy (gz-sim) 用は別 node
// mapoi_gz_bridge を参照。API が異なるため (gazebo_msgs vs ros_gz_interfaces)
// 同一コードでの distro 分岐はせず、別 node に分離している。

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
  // publisher (mapoi_server の config_path_publisher_) は transient_local なので、
  // subscriber も transient_local にして、bridge が後起動/再起動した時に
  // 直近の config_path を latched 値として受け取れるようにする。
  auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();
  config_path_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_config_path", sub_qos,
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
  // mapoi_server は config_path を 500ms 周期で re-publish するため、worker が
  // service 不達などで blocking 中だと queue が際限なく膨らむ。常に latest 1 件のみ
  // 保持する coalesce 戦略で、stale な path は破棄する。
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    while (!queue_.empty()) {
      queue_.pop();
    }
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
  const auto status = load_gazebo_info(path, info);
  if (status == ConfigLoadStatus::ParseError) {
    // transient な失敗 (YAML parse / I/O)。state 進めず、mapoi_server の
    // 周期 re-publish (500ms) で retry する。
    RCLCPP_WARN(this->get_logger(),
      "load_gazebo_info parse error for %s; keeping state for retry",
      path.c_str());
    return;
  }
  if (status == ConfigLoadStatus::NoGazeboSection) {
    // legitimate: gazebo セクション無し map (例: 実機用 map config)。
    // sim 側に旧 world_model が残っていれば cleanup してから state を進める
    // (sim 側 state と map state の乖離防止)。delete に失敗したら state を進めず retry。
    if (!current_world_model_name_.empty()) {
      RCLCPP_INFO(this->get_logger(),
        "No gazebo section in %s; cleaning up stale world_model=%s",
        path.c_str(), current_world_model_name_.c_str());
      if (!delete_entity(current_world_model_name_)) {
        RCLCPP_WARN(this->get_logger(),
          "delete world_model failed during NoGazeboSection cleanup; "
          "keeping state for retry");
        return;
      }
      current_world_model_name_.clear();
    } else {
      RCLCPP_INFO(this->get_logger(),
        "No gazebo section in %s; skipping entity swap", path.c_str());
    }
    current_map_name_ = map_name;
    current_config_path_ = path;
    return;
  }
  // status == Ok: 以下通常処理

  if (prev_map_name.empty()) {
    // 起動時の最初の通知: launch が既に対応する world で起動済みと仮定。
    // 内部 state のみ更新、entity 入れ替えはしない。
    current_world_model_name_ = info.world_model.name;
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

ConfigLoadStatus MapoiGazeboBridge::load_gazebo_info(
  const std::string & config_path, MapGazeboInfo & out)
{
  try {
    YAML::Node cfg = YAML::LoadFile(config_path);
    if (cfg["gazebo"] && cfg["gazebo"]["world_model"]) {
      const auto & wm = cfg["gazebo"]["world_model"];
      out.world_model.uri = wm["uri"] ? wm["uri"].as<std::string>() : "";
      out.world_model.name = wm["name"] ? wm["name"].as<std::string>() : "";
      out.has_gazebo = !out.world_model.uri.empty();
    }
    // initial pose は POI list の先頭 (landmark タグは到達不可なので除外) を採用 (#144)。
    // 旧 `initial_pose` system tag を yaml 順序で表現する semantics に移行した。
    if (cfg["poi"] && cfg["poi"].IsSequence()) {
      for (const auto & poi : cfg["poi"]) {
        bool is_landmark = false;
        if (poi["tags"] && poi["tags"].IsSequence()) {
          for (const auto & tag : poi["tags"]) {
            if (tag.as<std::string>() == "landmark") { is_landmark = true; break; }
          }
        }
        if (is_landmark) continue;
        // pose 妥当性 check: mapoi_server::compute_initial_poi_name と挙動一致 (#149 round 5 high)。
        // pose ノード or x/y/yaw が欠落 / numeric 不可なら次の POI を試す。
        // (旧仕様の 0.0 fallback は意図しない (0,0) spawn の原因になり得るため撤廃。
        // 共通 helper 化は #150 follow-up。)
        if (!poi["pose"] || !poi["pose"].IsMap()) continue;
        const auto & pose = poi["pose"];
        bool pose_ok = true;
        for (const char * k : {"x", "y", "yaw"}) {
          if (!pose[k]) { pose_ok = false; break; }
          try { (void)pose[k].as<double>(); }
          catch (const YAML::Exception &) { pose_ok = false; break; }
        }
        if (!pose_ok) continue;
        out.initial_x = pose["x"].as<double>();
        out.initial_y = pose["y"].as<double>();
        out.initial_yaw = pose["yaw"].as<double>();
        out.has_initial_pose = true;
        break;
      }
    }
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(this->get_logger(),
      "Failed to parse %s: %s", config_path.c_str(), e.what());
    return ConfigLoadStatus::ParseError;
  }
  return out.has_gazebo ? ConfigLoadStatus::Ok : ConfigLoadStatus::NoGazeboSection;
}

bool MapoiGazeboBridge::switch_world(
  const std::string & prev_map, const std::string & new_map,
  const MapGazeboInfo & info)
{
  (void)prev_map;
  (void)new_map;

  // 旧 world_model delete
  if (!current_world_model_name_.empty()) {
    if (!delete_entity(current_world_model_name_)) {
      RCLCPP_WARN(this->get_logger(),
        "delete world_model failed; aborting world switch (旧 world_model stays)");
      return false;
    }
    current_world_model_name_.clear();
  }

  bool all_ok = true;

  // 新 world_model spawn
  if (!info.world_model.uri.empty()) {
    if (spawn_entity_from_uri(info.world_model.name, info.world_model.uri)) {
      current_world_model_name_ = info.world_model.name;
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
