#pragma once

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <set>

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mapoi_interfaces/msg/point_of_interest.hpp>
#include <mapoi_interfaces/srv/switch_map.hpp>
#include <mapoi_interfaces/srv/get_pois_info.hpp>
#include <mapoi_interfaces/srv/get_maps_info.hpp>
#include <mapoi_interfaces/srv/get_tag_definitions.hpp>

#include <yaml-cpp/yaml.h>
#endif

#include <math.h>

#include <QMessageBox>
#include <QLabel>

namespace Ui {
class PoiEditorUi;
}


namespace mapoi_rviz_plugins
{

class PoiEditorPanel: public rviz_common::Panel
{
  Q_OBJECT
public:
  PoiEditorPanel(QWidget* parent = nullptr);
    ~PoiEditorPanel() override;

  void onInitialize() override;
  void onEnable();
  void onDisable();

private Q_SLOTS:
  void MapComboBox();
  void ResetButton();
  void TableChanged(int row, int column);
  void RowMoved(int logicalIndex, int oldVisualIndex, int newVisualIndex);
  void NewButton();
  void CopyButton();
  void DeleteButton();
  void FileComboBox();
  void SaveButton();
  void TagFilterChanged(int index);
  void TagHelperSelected(int index);

protected:
  Ui::PoiEditorUi* ui_;
  std::string current_map_;
  std::string config_path_;
  std::vector<std::string> map_name_list_;
  bool is_table_color_;

  // Tag filter: store all POIs to restore when filter is cleared
  std::vector<mapoi_interfaces::msg::PointOfInterest> all_pois_;

  // Tag definitions from server
  std::vector<std::string> known_tag_names_;
  std::vector<bool> known_tag_is_system_;

  // ROS
  rclcpp::Node::SharedPtr node_;

  // Shared service node and persistent clients
  rclcpp::Node::SharedPtr service_node_;
  rclcpp::Client<mapoi_interfaces::srv::SwitchMap>::SharedPtr switch_map_client_;
  rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedPtr get_pois_info_client_;
  rclcpp::Client<mapoi_interfaces::srv::GetMapsInfo>::SharedPtr get_maps_info_client_;
  rclcpp::Client<mapoi_interfaces::srv::GetTagDefinitions>::SharedPtr get_tag_defs_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reload_map_info_client_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poi_pose_sub_;
  void PoiPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_path_sub_;
  void ConfigPathCallback(std_msgs::msg::String::SharedPtr msg);

  // Functions
  void InitConfigs(std::string map_name);
  void UpdatePoiTable();
  void UpdatePoiCount();
  void PopulateTagFilter();
  void LoadTagDefinitions();
  bool ValidatePois();
  double calcYaw(geometry_msgs::msg::Pose pose);

  std::string join(const std::vector<std::string>& v, const char* delim);
  std::vector<std::string> SplitSentence(std::string sentence,
                                         std::string delimiter);
};
} // end namespace mapoi_rviz_plugins
