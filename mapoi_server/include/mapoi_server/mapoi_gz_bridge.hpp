#ifndef MAPOI_SERVER__MAPOI_GZ_BRIDGE_HPP_
#define MAPOI_SERVER__MAPOI_GZ_BRIDGE_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>

#include "mapoi_interfaces/msg/initial_pose_request.hpp"


struct GzWorldModelInfo
{
  std::string uri;
  std::string name;
};

struct MapGzInfo
{
  GzWorldModelInfo world_model;
  double initial_x = 0.0;
  double initial_y = 0.0;
  double initial_yaw = 0.0;
  bool has_gazebo = false;
  bool has_initial_pose = false;
};

enum class ConfigLoadStatus
{
  Ok,               // parse 成功 + gazebo section あり
  NoGazeboSection,  // parse 成功 + gazebo section 無し (legitimate)
  ParseError,       // YAML parse or file I/O 失敗 (transient、retry すべき)
};


class MapoiGzBridge : public rclcpp::Node
{
public:
  MapoiGzBridge();
  ~MapoiGzBridge() override;

private:
  void on_config_path(const std_msgs::msg::String::SharedPtr msg);
  void worker_loop();
  void process_config_path(const std::string & path);
  ConfigLoadStatus load_gazebo_info(const std::string & config_path, MapGzInfo & out);
  bool switch_world(
    const std::string & prev_map, const std::string & new_map,
    const MapGzInfo & info);
  bool delete_model_entity(const std::string & name);
  bool spawn_model_from_uri(const std::string & name, const std::string & uri);
  bool teleport_robot(double x, double y, double yaw);
  // teleport 後に bridge から /initialpose を late publish する (#202、Classic 側 #91 / #200 と API 一貫)。
  // gz-sim は SetEntityPose で atomic teleport なので Classic のような laser scan 不整合 race は無いが、
  // AMCL の /initialpose 受信時の covariance 拡散ぶれ (σ=0.5m) は両 simulator 共通の AMCL 仕様。
  // teleport 後の AMCL 状態を「正解 pose」で確実に上書きする preventive measure として導入。
  // gz-sim では teleport が atomic なので delay 不要 (即時 publish)、count/interval は Classic と同じ
  // constexpr 固定。
  void publish_initialpose_after_teleport(double x, double y, double yaw);

  // parameters
  std::string robot_name_;
  std::string init_world_name_;
  // /initialpose late publish parameters (#202)。default は declare_parameter (cpp) 側を SSOT。
  // gz-sim では delay parameter は持たない (atomic teleport で laser scan race 無し、即時 publish で十分)。
  std::string initial_pose_topic_;

  // state (worker thread のみ touch)
  std::string current_map_name_;
  std::string current_world_model_name_;
  std::string current_config_path_;

  // service clients (Reentrant callback group で worker thread から wait_for 可能に)
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr spawn_client_;
  rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr delete_client_;
  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_pose_client_;

  // /initialpose late publisher (#202、Classic 側 #200 と API 一貫)
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;

  // subscription
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_path_sub_;
  // mapoi/initialpose_poi (#149 round 7-8 ヘビー high 対応): SelectMap.initial_poi_name 指定時に
  // mapoi_server がここに {map_name, poi_name} を publish するので、bridge も map 一致時に
  // 同じ POI を spawn 位置に採用する。
  rclcpp::Subscription<mapoi_interfaces::msg::InitialPoseRequest>::SharedPtr initialpose_poi_sub_;
  void on_initialpose_poi(
    const mapoi_interfaces::msg::InitialPoseRequest::SharedPtr msg);
  std::string requested_initial_pose_map_;
  std::string requested_initial_pose_poi_;
  std::mutex requested_initial_pose_mutex_;

  // worker thread + queue (subscribe callback は queue.push のみ、worker が直列処理)
  std::thread worker_;
  std::atomic<bool> stop_worker_{false};
  std::queue<std::string> queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
};

#endif  // MAPOI_SERVER__MAPOI_GZ_BRIDGE_HPP_
