#pragma once

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include "mapoi_interfaces/msg/point_of_interest.hpp"
#include "mapoi_interfaces/msg/initial_pose_request.hpp"
#include "mapoi_interfaces/msg/navigation_backend_status.hpp"
#include "mapoi_interfaces/msg/localization_backend_status.hpp"
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
  void PauseButton();
  void ResumeButton();
  void StopButton();

protected:
  Ui::ScUI* ui_;
  int goal_combobox_ind_;
  std::vector<mapoi_interfaces::msg::PointOfInterest> pois_;
  std::vector<std::string> map_name_list_;
  std::string current_map_;

  rclcpp::Node::SharedPtr node_;

  // Shared service node and persistent clients
  rclcpp::Node::SharedPtr service_node_;
  rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedPtr get_pois_info_client_;
  rclcpp::Client<mapoi_interfaces::srv::GetMapsInfo>::SharedPtr get_maps_info_client_;
  rclcpp::Client<mapoi_interfaces::srv::GetRoutesInfo>::SharedPtr get_routes_info_client_;
  rclcpp::Client<mapoi_interfaces::srv::GetRoutePois>::SharedPtr get_route_pois_client_;

  // mapoi/initialpose_poi (transient_local) publisher: LocalizationButton クリック時に
  // {map_name, poi_name} を publish し、`mapoi_amcl_localization_bridge` (or 自作 bridge) が
  // POI resolve / `/initialpose` 配信 / retry を担当する (#209)。直接 `/initialpose` には publish しない。
  rclcpp::Publisher<mapoi_interfaces::msg::InitialPoseRequest>::SharedPtr mapoi_initialpose_poi_pub_;
  // mapoi/nav/goal_pose_poi (std_msgs/String) publisher: RunGoalButton クリック時に POI 名を
  // publish し、navigation bridge (mapoi_nav2_bridge / 自作 bridge) が POI resolve / Nav2 action
  // 起動 / status 配信を担当する (#209 review #3)。直接 `goal_pose` には publish しない。
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapoi_goal_pose_poi_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapoi_cancel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapoi_switch_map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapoi_pause_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapoi_resume_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapoi_route_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapoi_highlight_goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapoi_highlight_route_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_path_sub_;
  void ConfigPathCallback(std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_status_sub_;
  void NavStatusCallback(std_msgs::msg::String::SharedPtr msg);
  std::string current_nav_mode_;
  std::string current_nav_target_;

  // Navigation backend readiness subscribe (#198, #205 minimal contract):
  // bridge が `mapoi/nav/backend_status` を publish する場合は backend_ready で navigation
  // 操作ボタンと MapComboBox を一括 gate する。topic 不在 (古い mapoi_nav2_bridge) では「全 enable」の
  // ままで後方互換を保つ。Minimal contract なので per-capability gate は持たない。
  rclcpp::Subscription<mapoi_interfaces::msg::NavigationBackendStatus>::SharedPtr backend_status_sub_;
  void BackendStatusCallback(mapoi_interfaces::msg::NavigationBackendStatus::SharedPtr msg);
  // backend_status 不在 (旧 mapoi_nav2_bridge / editor 構成) では callback が呼ばれず、初期値
  // (true = enable) のままで後方互換を保つ。Minimal contract なので per-capability gate は持たない。
  bool last_navigation_backend_ready_ {true};
  // 一度でも navigation backend_status を受信したか。受信実績なし = contract 未実装の旧 mapoi_nav2_bridge
  // / editor 構成として enable のまま (後方互換)。受信実績ありの publisher が liveliness lost
  // (= bridge 死亡) した場合のみ disable に倒す。
  bool nav_backend_status_received_ {false};
  // MANUAL_BY_TOPIC liveliness (#208) で publisher 生存を track。subscription event_callback
  // が `alive_count > 0` で更新する。`*_received_` と AND して、未受信は alive 判定をバイパス。
  bool nav_backend_alive_ {false};

  // Localization backend readiness subscribe (#209): mapoi_amcl_localization_bridge (or any
  // custom localization bridge) が publish する readiness で LocalizationButton を gate する。
  // navigation backend (#205) と独立した仕様。topic 不在 (bridge 単独不起動 / editor 構成) は
  // 後方互換のため「全 enable のまま」とする (BackendStatusCallback と同じ初期値方針)。
  rclcpp::Subscription<mapoi_interfaces::msg::LocalizationBackendStatus>::SharedPtr
    localization_backend_status_sub_;
  void LocalizationBackendStatusCallback(
    mapoi_interfaces::msg::LocalizationBackendStatus::SharedPtr msg);
  bool last_localization_backend_ready_ {true};
  bool localization_backend_status_received_ {false};
  bool localization_backend_alive_ {false};

  // navigation_ready / localization_ready の最新値を保持し、両 callback / liveliness event から
  // 共通の UpdateNavButtonsEnabled を呼び出す。LocalizationButton は localization、それ以外の
  // navigation 操作 UI は navigation で gate する (#209 で 2 軸に分離)。
  void UpdateNavButtonsEnabled();

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
