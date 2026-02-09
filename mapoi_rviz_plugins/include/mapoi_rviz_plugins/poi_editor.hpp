#pragma once

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif

#include <math.h>

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mapoi_interfaces/msg/point_of_interest.hpp>
#include <mapoi_interfaces/srv/switch_map.hpp>
#include <mapoi_interfaces/srv/get_tagged_pois.hpp>
#include <mapoi_interfaces/srv/get_maps_info.hpp>

#include <yaml-cpp/yaml.h>

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

protected:
  Ui::PoiEditorUi* ui_;
  std::string current_map_;
  std::vector<std::string> map_name_list_;
  bool is_table_color_;

  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poi_pose_sub_;
  void PoiPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_name_sub_;
  void MapNameCallback(std_msgs::msg::String::SharedPtr msg);

  // Functions
  void InitConfigs(std::string map_name);
  void UpdatePoiTable(std::string tag);
  double calcYaw(geometry_msgs::msg::Pose pose);

  std::string join(const std::vector<std::string>& v, const char* delim);
  std::vector<std::string> SplitSentence(std::string sentence,
                                         std::string delimiter);
};
} // end namespace mapoi_rviz_plugins