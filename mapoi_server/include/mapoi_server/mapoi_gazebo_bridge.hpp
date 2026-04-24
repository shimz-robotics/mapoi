#ifndef MAPOI_SERVER__MAPOI_GAZEBO_BRIDGE_HPP_
#define MAPOI_SERVER__MAPOI_GAZEBO_BRIDGE_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>


struct GazeboWorldModelInfo
{
  std::string uri;
  std::string name;
};

struct MapGazeboInfo
{
  GazeboWorldModelInfo world_model;
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


class MapoiGazeboBridge : public rclcpp::Node
{
public:
  MapoiGazeboBridge();
  ~MapoiGazeboBridge() override;

private:
  void on_config_path(const std_msgs::msg::String::SharedPtr msg);
  void worker_loop();
  void process_config_path(const std::string & path);
  ConfigLoadStatus load_gazebo_info(const std::string & config_path, MapGazeboInfo & out);
  bool switch_world(
    const std::string & prev_map, const std::string & new_map,
    const MapGazeboInfo & info);
  bool delete_entity(const std::string & name);
  bool spawn_entity_from_uri(const std::string & name, const std::string & uri);
  bool respawn_robot(double x, double y, double yaw);

  // parameters
  std::string robot_name_;
  std::string robot_sdf_path_;

  // state (worker thread のみが touch)
  std::string current_map_name_;
  std::string current_world_model_name_;
  std::string current_config_path_;

  // service clients (Reentrant callback group で worker thread から wait_for 可能に)
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;

  // subscription
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_path_sub_;

  // worker thread + queue (subscribe callback は queue.push のみ、worker が直列処理)
  std::thread worker_;
  std::atomic<bool> stop_worker_{false};
  std::queue<std::string> queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
};

#endif  // MAPOI_SERVER__MAPOI_GAZEBO_BRIDGE_HPP_
