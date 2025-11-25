#pragma once

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <mapoi_interfaces/msg/point_of_interest.hpp>
#include <mapoi_interfaces/srv/switch_map.hpp>
#include <mapoi_interfaces/srv/get_tagged_pois.hpp>
#include <mapoi_interfaces/srv/get_map_info.hpp>
#include <cmd_vel_manager/srv/set_cmd_vel_mode.hpp>
#include <cmd_vel_manager/msg/cmd_vel_mode.hpp>


namespace Ui {
class ScUI;
}


namespace mapoi_rviz_plugins
{

class MapoiPanel: public rviz_common::Panel
{
  Q_OBJECT
public:
  MapoiPanel(QWidget* parent = nullptr);
    ~MapoiPanel() override;

  void onInitialize() override;
  void onEnable();
  void onDisable();

private Q_SLOTS:
  void MapComboBox();
  void DestinationComboBox();
  void FootprintComboBox();

  void LocalizationButton();
  void RunButton();
  void StopButton();

protected:
  Ui::ScUI* ui_;
  int dest_ind_;
  std::vector<mapoi_interfaces::msg::PointOfInterest> pois_;
  std::vector<std::string> map_name_list_;
  std::string current_map_;
  std::string sim_real_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_name_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cancel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr footprint_mode_pub_;

  using CmdVelFrom = cmd_vel_manager::msg::CmdVelMode;
  rclcpp::Subscription<CmdVelFrom>::SharedPtr cmd_vel_mode_sub_;
  std::string cmd_vel_mode_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_name_sub_;
  void MapNameCallback(std_msgs::msg::String::SharedPtr msg);

  void RequestSetCmdVelMode(std::string cm);
  void SetMapComboBox(std::string map_name);
  void SetDestComboBox();
};
} // end namespace mapoi_rviz_plugins
