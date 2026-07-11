// MapoiPanel の config/ComboBox 系メソッド定義 (#397 step 1 で mapoi_panel.cpp から別 TU へ分離)。
// ConfigPathCallback / SetMapComboBox / SetNav2GoalComboBox / SetMapoiRouteComboBox を収録。
#include "mapoi_rviz_plugins/mapoi_panel.hpp"
#include "mapoi_rviz_plugins/config_path_update_policy.hpp"
#include <filesystem>
#include "ui_mapoi_panel.h"

using namespace std::chrono_literals;

namespace mapoi_rviz_plugins
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mapoi_rviz_plugins.mapoi_panel");

void MapoiPanel::ConfigPathCallback(std_msgs::msg::String::SharedPtr msg)
{
  std::filesystem::path config_path(msg->data);
  std::string map_name = config_path.parent_path().filename().string();
  const auto action = detail::decide_mapoi_panel_config_path_action(current_map_, map_name);
  current_map_ = map_name;

  // 同じ map で path も同じ場合 (= save 後の reload_map_info で再 publish される flow) でも
  // POI / route list が変わっている可能性があるため Nav2GoalComboBox / MapoiRouteComboBox を
  // 再 fetch する (#135)。map 切替時は highlight クリア + MapComboBox 同期も必要なので
  // SetMapComboBox を呼ぶ。
  QMetaObject::invokeMethod(this, [this, map_name, action]() {
    if (action == detail::ConfigPathUpdateAction::ReinitializeMap) {
      SetMapComboBox(map_name);
    } else {
      SetNav2GoalComboBox();
      SetMapoiRouteComboBox();
    }
  }, Qt::QueuedConnection);
}

void MapoiPanel::SetMapComboBox(std::string map_name)
{
  highlighted_goal_name_.clear();
  highlighted_route_poi_names_.clear();
  PublishHighlightPois();

  int i = 0;
  for(const auto & map : map_name_list_){
    if(map == map_name){
      ui_->MapComboBox->setCurrentIndex(i);
      SetNav2GoalComboBox();
      SetMapoiRouteComboBox();
      return;
    }
    i++;
  }
  i = 0;
  for(const auto & map : map_name_list_){
    RCLCPP_ERROR_STREAM(LOGGER, "map" << i << " : " << map << " correctly");
    i++;
  }
  RCLCPP_ERROR_STREAM(LOGGER, "Couldn't get " << map_name << " correctly");
}

void MapoiPanel::SetNav2GoalComboBox()
{
  // 再構築前に現在 selection を保存し、同名項目があれば復元する (#135 副作用対応)。
  // 他クライアント save (POI 追加 / pose 変更など) の reload で goal 選択が消えるのを防ぐ。
  // 削除 / 名前変更で同名項目が消えた場合は currentIndex 0 (default) に fallback。
  QString prev_selection = ui_->Nav2GoalComboBox->currentText();

  auto request_gtp = std::make_shared<mapoi_interfaces::srv::GetPoisInfo::Request>();

  if (!get_pois_info_client_->wait_for_service(3s)) {
    RCLCPP_ERROR(LOGGER, "mapoi/get_pois_info service not available after 3s timeout.");
    return;
  }

  auto result_get_pois_info = get_pois_info_client_->async_send_request(request_gtp);
  if(rclcpp::spin_until_future_complete(service_node_, result_get_pois_info) == rclcpp::FutureReturnCode::SUCCESS)
  {

    auto pois_all = result_get_pois_info.get()->pois_list;
    ui_->Nav2GoalComboBox->clear();
    pois_.clear();

    for(const auto & p : pois_all){
      bool is_waypoint = false;
      bool is_landmark = false;
      for(const auto & tag : p.tags){
        if(tag == "waypoint") is_waypoint = true;
        else if(tag == "landmark") is_landmark = true;
      }
      // waypoint+landmark 併用は意味矛盾だが万一 config に混入しても candidate には載せない (#85)。
      if(is_waypoint && !is_landmark){
        pois_.push_back(p);
        ui_->Nav2GoalComboBox->addItem(QString::fromStdString(p.name));
      }
    }
    int idx = ui_->Nav2GoalComboBox->findText(prev_selection);
    if (idx >= 0) {
      ui_->Nav2GoalComboBox->setCurrentIndex(idx);
    }
    goal_combobox_ind_ = ui_->Nav2GoalComboBox->currentIndex();
  }else{
    RCLCPP_ERROR(LOGGER, "Failed to call service mapoi/get_pois_info");
  }
}

void MapoiPanel::SetMapoiRouteComboBox()
{
  // 再構築前に現在 selection を保存し、同名項目があれば復元する (#135 副作用対応)。
  // 他クライアント save (route 追加 / 内容変更) の reload で route 選択が消えるのを防ぐ。
  QString prev_selection = ui_->MapoiRouteComboBox->currentText();

  if (!get_routes_info_client_->wait_for_service(3s)) {
    RCLCPP_ERROR(LOGGER, "mapoi/get_routes_info service not available after 3s timeout.");
    return;
  }

  auto request = std::make_shared<mapoi_interfaces::srv::GetRoutesInfo::Request>();
  auto result = get_routes_info_client_->async_send_request(request);
  if(rclcpp::spin_until_future_complete(service_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    ui_->MapoiRouteComboBox->clear();
    ui_->MapoiRouteComboBox->addItem(QString::fromStdString("(なし)"));
    route_name_list_ = result.get()->routes_list;
    for(const auto & route : route_name_list_){
      ui_->MapoiRouteComboBox->addItem(QString::fromStdString(route));
    }
    int idx = ui_->MapoiRouteComboBox->findText(prev_selection);
    if (idx >= 0) {
      ui_->MapoiRouteComboBox->setCurrentIndex(idx);
    }
    route_combobox_ind_ = ui_->MapoiRouteComboBox->currentIndex();
  }else{
    RCLCPP_ERROR(LOGGER, "Failed to call service mapoi/get_routes_info");
  }
}

}  // namespace mapoi_rviz_plugins
