#pragma once

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include "mapoi_interfaces/msg/point_of_interest.hpp"
#include "mapoi_interfaces/srv/switch_map.hpp"
#include "mapoi_interfaces/srv/get_pois_info.hpp"
#include "mapoi_interfaces/srv/get_maps_info.hpp"
#include "mapoi_interfaces/srv/get_route_pois.hpp"
#include "mapoi_interfaces/srv/get_routes_info.hpp"

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
  void Nav2GoalComboBox();
  void MapoiRouteComboBox();

  void LocalizationButton();
  void RunGoalButton();
  void RunRouteButton();
  void StopButton();

protected:
  Ui::ScUI* ui_;
  int goal_combobox_ind_;
  std::vector<mapoi_interfaces::msg::PointOfInterest> pois_;
  std::vector<std::string> map_name_list_;
  std::string current_map_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr nav2_initialpose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav2_goal_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapoi_cancel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapoi_route_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapoi_highlight_goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapoi_highlight_route_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_path_sub_;
  void ConfigPathCallback(std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_status_sub_;
  void NavStatusCallback(std_msgs::msg::String::SharedPtr msg);
  std::string current_nav_mode_;
  std::string current_nav_target_;

  void RequestSetCmdVelMode(std::string cm);
  void SetMapComboBox(std::string map_name);
  void SetNav2GoalComboBox();
  void SetMapoiRouteComboBox();
  void PublishHighlightPois();

  std::vector<std::string> route_name_list_;
  int route_combobox_ind_;

  std::string highlighted_goal_name_;
  std::vector<std::string> highlighted_route_poi_names_;
};
}
