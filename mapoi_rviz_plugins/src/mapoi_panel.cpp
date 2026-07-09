#include "mapoi_rviz_plugins/mapoi_panel.hpp"
#include <class_loader/class_loader.hpp>
#include <filesystem>
#include <set>

#include "mapoi_rviz_plugins/config_path_update_policy.hpp"
#include "ui_mapoi_panel.h"

using namespace std::chrono_literals;

namespace mapoi_rviz_plugins
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mapoi_rviz_plugins.mapoi_panel");

MapoiPanel::MapoiPanel(QWidget* parent) : Panel(parent),  ui_(new Ui::ScUI())
{
  ui_->setupUi(this);
}

MapoiPanel::~MapoiPanel() = default;

void MapoiPanel::onInitialize()
{
  node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Create shared service node and persistent clients
  service_node_ = rclcpp::Node::make_shared("mapoi_panel_service_client");
  get_pois_info_client_ = service_node_->create_client<mapoi_interfaces::srv::GetPoisInfo>("mapoi/get_pois_info");
  get_maps_info_client_ = service_node_->create_client<mapoi_interfaces::srv::GetMapsInfo>("mapoi/get_maps_info");
  get_routes_info_client_ = service_node_->create_client<mapoi_interfaces::srv::GetRoutesInfo>("mapoi/get_routes_info");
  get_route_pois_client_ = service_node_->create_client<mapoi_interfaces::srv::GetRoutePois>("mapoi/get_route_pois");
  // #211: initialpose POI は直接 publish せず mapoi_server へ service 経由で依頼する。
  request_initial_pose_client_ =
    service_node_->create_client<mapoi_interfaces::srv::RequestInitialPose>("mapoi/request_initial_pose");

  connect(ui_->MapComboBox, SIGNAL(activated(int)), this, SLOT(MapComboBox()));
  connect(ui_->Nav2GoalComboBox, SIGNAL(activated(int)), this, SLOT(Nav2GoalComboBox()));
  connect(ui_->MapoiRouteComboBox, SIGNAL(activated(int)), this, SLOT(MapoiRouteComboBox()));

  connect(ui_->LocalizationButton, SIGNAL(clicked()), this, SLOT(LocalizationButton()));
  connect(ui_->RunGoalButton, SIGNAL(clicked()), this, SLOT(RunGoalButton()));
  connect(ui_->RunRouteButton, SIGNAL(clicked()), this, SLOT(RunRouteButton()));
  connect(ui_->PauseButton,    SIGNAL(clicked()), this, SLOT(PauseButton()));
  connect(ui_->ResumeButton,   SIGNAL(clicked()), this, SLOT(ResumeButton()));
  connect(ui_->StopButton, SIGNAL(clicked()), this, SLOT(StopButton()));

  parentWidget()->setVisible(true);

  // map ComboBox
  if (!get_maps_info_client_->wait_for_service(3s)) {
    RCLCPP_ERROR(LOGGER, "mapoi/get_maps_info service not available after 3s timeout.");
    return;
  }
  auto request = std::make_shared<mapoi_interfaces::srv::GetMapsInfo::Request>();
  auto result = get_maps_info_client_->async_send_request(request);
  if(rclcpp::spin_until_future_complete(service_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto map_info = result.get();
    current_map_ = map_info->map_name;
    map_name_list_ = map_info->maps_list;
    for (const auto & map : map_name_list_) {
      ui_->MapComboBox->addItem(QString::fromStdString(map));
    }
    SetMapComboBox(map_info->map_name);
  }else{
    RCLCPP_ERROR(LOGGER, "Failed to call service mapoi/get_maps_info");
  }

  // Buttons
  // RunGoalButton は Nav2 native `goal_pose` を直接叩かず、`mapoi/nav/goal_pose_poi` 経由で
  // navigation bridge に POI 名を渡す (#209 review #3)。bridge 側で POI resolve / Nav2 action
  // / `mapoi/nav/status` 配信を一元化することで、custom navigation bridge / pause / cancel /
  // resume の状態管理が WebUI と一貫する。
  mapoi_goal_pose_poi_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "mapoi/nav/goal_pose_poi", 1);

  mapoi_cancel_pub_ = node_->create_publisher<std_msgs::msg::String>("mapoi/nav/cancel", 1);
  mapoi_switch_map_pub_ = node_->create_publisher<std_msgs::msg::String>("mapoi/nav/switch_map", 1);
  mapoi_pause_pub_  = node_->create_publisher<std_msgs::msg::String>("mapoi/nav/pause",  1);
  mapoi_resume_pub_ = node_->create_publisher<std_msgs::msg::String>("mapoi/nav/resume", 1);
  mapoi_route_pub_ = node_->create_publisher<std_msgs::msg::String>("mapoi/nav/route", 1);
  mapoi_highlight_goal_pub_ = node_->create_publisher<std_msgs::msg::String>("mapoi/highlight/goal", 1);
  mapoi_highlight_route_pub_ = node_->create_publisher<std_msgs::msg::String>("mapoi/highlight/route", 1);

  config_path_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "mapoi/config_path", rclcpp::QoS(1).transient_local(),
      std::bind(&MapoiPanel::ConfigPathCallback, this, std::placeholders::_1));

  // QoS は mapoi_nav2_bridge と同じ transient_local。後起動 panel でも latched 値を受信できる。
  nav_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "mapoi/nav/status", rclcpp::QoS(1).transient_local(),
      std::bind(&MapoiPanel::NavStatusCallback, this, std::placeholders::_1));

  // Navigation / Localization backend readiness (#198, #209) の QoS は msg contract (#208)
  // に従う: transient_local + liveliness (publisher=MANUAL_BY_TOPIC, subscriber=AUTOMATIC)
  // + lease 5s (両側必須)。subscriber 側 policy を AUTOMATIC にするのは pub=MANUAL_BY_TOPIC
  // × sub=AUTOMATIC が compatibility 表上 OK だから。lease は `pub.lease <= sub.lease` 制約が
  // あり、両側 5s で揃えるのが運用上シンプル。
  // **msg contract に従わない publisher (例: liveliness QoS 未設定 = lease infinite) は
  // 意図的に QoS incompatible で接続を拒否する**。custom bridge 実装者は msg README の
  // contract 表に従う必要がある (test_backend_status_liveliness.py で incompatibility を pin)。
  // 受信前 (`*_status_received_` flag = false、つまり contract 未実装の旧 publisher も含む) は
  // alive 判定をバイパスして全 enable のまま (UpdateNavButtonsEnabled 内で実現)。
  const auto backend_status_qos = rclcpp::QoS(1)
    .transient_local()
    .liveliness(rclcpp::LivelinessPolicy::Automatic)
    .liveliness_lease_duration(std::chrono::seconds(5));

  rclcpp::SubscriptionOptions nav_sub_opts;
  nav_sub_opts.event_callbacks.liveliness_callback =
    [this](rclcpp::QOSLivelinessChangedInfo & event) {
      nav_backend_alive_ = event.alive_count > 0;
      QMetaObject::invokeMethod(this, [this]() {
        UpdateNavButtonsEnabled();
      }, Qt::QueuedConnection);
    };
  backend_status_sub_ = node_->create_subscription<mapoi_interfaces::msg::NavigationBackendStatus>(
      "mapoi/nav/backend_status", backend_status_qos,
      std::bind(&MapoiPanel::BackendStatusCallback, this, std::placeholders::_1),
      nav_sub_opts);

  rclcpp::SubscriptionOptions loc_sub_opts;
  loc_sub_opts.event_callbacks.liveliness_callback =
    [this](rclcpp::QOSLivelinessChangedInfo & event) {
      localization_backend_alive_ = event.alive_count > 0;
      QMetaObject::invokeMethod(this, [this]() {
        UpdateNavButtonsEnabled();
      }, Qt::QueuedConnection);
    };
  localization_backend_status_sub_ = node_->create_subscription<
    mapoi_interfaces::msg::LocalizationBackendStatus>(
      "mapoi/localization/backend_status", backend_status_qos,
      std::bind(&MapoiPanel::LocalizationBackendStatusCallback, this, std::placeholders::_1),
      loc_sub_opts);

  current_nav_mode_ = "idle";

  // Mapoi Route ComboBox
  SetMapoiRouteComboBox();
}

void MapoiPanel::onEnable()
{
  show();
  parentWidget()->show();
}

void MapoiPanel::onDisable()
{
  hide();
  parentWidget()->hide();
}

void MapoiPanel::MapComboBox()
{
  const int index = ui_->MapComboBox->currentIndex();
  if (index < 0 || index >= static_cast<int>(map_name_list_.size())) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = map_name_list_[index];
  if (mapoi_switch_map_pub_->get_subscription_count() == 0) {
    RCLCPP_WARN(LOGGER, "mapoi/nav/switch_map has no subscribers; mapoi_nav2_bridge may not be running.");
  }
  mapoi_switch_map_pub_->publish(msg);
  RCLCPP_INFO(LOGGER, "Published map switch request: %s", msg.data.c_str());
}

void MapoiPanel::Nav2GoalComboBox()
{
  goal_combobox_ind_ = ui_->Nav2GoalComboBox->currentIndex();
  if (goal_combobox_ind_ >= 0 && goal_combobox_ind_ < static_cast<int>(pois_.size())) {
    highlighted_goal_name_ = pois_[goal_combobox_ind_].name;
  } else {
    highlighted_goal_name_.clear();
  }
  PublishHighlightPois();
}

void MapoiPanel::MapoiRouteComboBox()
{
  route_combobox_ind_ = ui_->MapoiRouteComboBox->currentIndex();
  highlighted_route_poi_names_.clear();
  if (route_combobox_ind_ < 1) {
    PublishHighlightPois();
    return;
  }
  int route_index = route_combobox_ind_ - 1;
  if (route_index >= 0 && route_index < static_cast<int>(route_name_list_.size())) {
    if (get_route_pois_client_->wait_for_service(3s)) {
      auto request = std::make_shared<mapoi_interfaces::srv::GetRoutePois::Request>();
      request->route_name = route_name_list_[route_index];
      auto result = get_route_pois_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(service_node_, result, 5s) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result.get();
        if (response && response->success) {
          for (const auto & poi : response->pois_list) {
            highlighted_route_poi_names_.push_back(poi.name);
          }
        } else if (response) {
          // #342: route が見つからない (typo 等)。highlight は設定しない
          // (highlighted_route_poi_names_ は上で既に clear 済み)。
          RCLCPP_ERROR(LOGGER, "get_route_pois failed for '%s': %s",
                       route_name_list_[route_index].c_str(), response->error_message.c_str());
        }
      }
    } else {
      RCLCPP_ERROR(LOGGER, "mapoi/get_route_pois service not available after 3s timeout.");
    }
  }
  PublishHighlightPois();
}

void MapoiPanel::LocalizationButton()
{
  // POI 選択ガード: GoalComboBox で選択されていない / 範囲外なら何もしない。
  // (LocalizationButton は GoalComboBox の POI を流用する仕様、cf. PublishHighlightPois)
  if (goal_combobox_ind_ < 0 || goal_combobox_ind_ >= static_cast<int>(pois_.size())) {
    RCLCPP_WARN(LOGGER, "No POI selected for initial pose; pick one in the goal ComboBox first.");
    return;
  }
  // #211: `/initialpose` も `mapoi/initialpose_poi` も直接叩かず、唯一の writer である mapoi_server に
  // request_initial_pose service 経由で publish を依頼する。これにより transient_local の
  // per-writer latched cache が単一化され stale POI を構造的に排除する。bridge が POI resolve /
  // landmark 排他 / `/initialpose` 配信 / retry を一元処理する点は不変 (#209)。
  if (!request_initial_pose_client_->wait_for_service(3s)) {
    RCLCPP_ERROR(LOGGER, "mapoi/request_initial_pose service not available after 3s timeout.");
    return;
  }
  auto request = std::make_shared<mapoi_interfaces::srv::RequestInitialPose::Request>();
  request->map_name = current_map_;
  request->poi_name = pois_[goal_combobox_ind_].name;
  auto result = request_initial_pose_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(service_node_, result, 5s) == rclcpp::FutureReturnCode::SUCCESS) {
    // #299: server は非空 map_name が現在 map と不一致な要求を publish せず success=false で
    // 返すようになった (panel の current_map_ が stale な窓など)。future 完了 = 成功ではない
    // ので response を確認し、reject を operator に成功と誤認させない。
    const auto response = result.get();
    if (response->success) {
      RCLCPP_INFO(LOGGER, "Requested initial pose: %s (map: %s).",
                  request->poi_name.c_str(), current_map_.c_str());
    } else {
      RCLCPP_ERROR(LOGGER, "request_initial_pose rejected: %s",
                   response->error_message.c_str());
    }
  } else {
    RCLCPP_ERROR(LOGGER, "request_initial_pose call failed or timed out.");
  }
}

void MapoiPanel::RunGoalButton()
{
  if (goal_combobox_ind_ < 0 || goal_combobox_ind_ >= static_cast<int>(pois_.size())) {
    return;
  }
  // Nav2 native `goal_pose` を直接叩かず、`mapoi/nav/goal_pose_poi` 経由で bridge に POI 名を
  // 渡す (#209 review #3)。bridge が POI resolve → Nav2 action 起動 → `mapoi/nav/status` 配信を
  // 担当するので、status / cancel / pause / resume の状態管理が WebUI と一貫する。
  std_msgs::msg::String msg;
  msg.data = pois_[goal_combobox_ind_].name;
  mapoi_goal_pose_poi_pub_->publish(msg);

  current_nav_mode_ = "goal";
  current_nav_target_ = msg.data;
  ui_->NavStatusLabel->setText(
      QString::fromStdString("目的地走行中: " + current_nav_target_));
  RCLCPP_INFO(LOGGER, "Published goal POI request: %s", current_nav_target_.c_str());
}

void MapoiPanel::RunRouteButton()
{
  if (route_combobox_ind_ < 1) {
    return;
  }
  int route_index = route_combobox_ind_ - 1;
  if (route_index >= static_cast<int>(route_name_list_.size())) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = route_name_list_[route_index];
  mapoi_route_pub_->publish(msg);

  current_nav_mode_ = "route";
  current_nav_target_ = route_name_list_[route_index];
  ui_->NavStatusLabel->setText(
      QString::fromStdString("ルート走行中: " + current_nav_target_));
  RCLCPP_INFO(LOGGER, "A route was set: %s", msg.data.c_str());
}

void MapoiPanel::PauseButton()
{
  std_msgs::msg::String msg;
  msg.data = "mapoi_panel";
  mapoi_pause_pub_->publish(msg);
  ui_->NavStatusLabel->setText(QString::fromStdString("一時停止中"));
  RCLCPP_INFO(LOGGER, "Pause requested");
}

void MapoiPanel::ResumeButton()
{
  std_msgs::msg::String msg;
  msg.data = "mapoi_panel";
  mapoi_resume_pub_->publish(msg);
  ui_->NavStatusLabel->setText(QString::fromStdString("再開中..."));
  RCLCPP_INFO(LOGGER, "Resume requested");
}

void MapoiPanel::StopButton()
{
  std_msgs::msg::String msg;
  msg.data = "mapoi_panel";
  mapoi_cancel_pub_->publish(msg);

  current_nav_mode_ = "idle";
  ui_->NavStatusLabel->setText(QString::fromStdString("走行キャンセル"));
  RCLCPP_INFO(LOGGER, "The robot was stopped");
}

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

void MapoiPanel::NavStatusCallback(std_msgs::msg::String::SharedPtr msg)
{
  // payload は "status" または "status:target" 形式 (#104)。
  // : split で target を抽出し、有れば後起動 panel / 外部ノード発行 nav でも
  // target 復元できるようにする。
  const auto & raw = msg->data;
  const auto colon = raw.find(':');
  const std::string status = (colon == std::string::npos) ? raw : raw.substr(0, colon);
  const std::string target = (colon == std::string::npos) ? std::string{} : raw.substr(colon + 1);
  QMetaObject::invokeMethod(this, [this, status, target]() {
    if (status == "navigating") {
      if (!target.empty() && current_nav_mode_ == "idle") {
        // 後起動 panel / 外部ノード発行 nav: payload の target を復元して表示。
        current_nav_target_ = target;
        ui_->NavStatusLabel->setText(
            QString::fromStdString("走行中: " + target));
      } else if (current_nav_mode_ == "route") {
        ui_->NavStatusLabel->setText(
            QString::fromStdString("ルート走行中: " + current_nav_target_));
      } else if (current_nav_mode_ == "idle") {
        // target も無い (旧 publisher 等) → 汎用表示にフォールバック。
        ui_->NavStatusLabel->setText(QString::fromStdString("走行中"));
      }
      // goal mode は RunGoalButton で即時表示済みのため何もしない
    } else if (status == "succeeded") {
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("到着")
                         : QString::fromStdString("到着: " + target));
    } else if (status == "aborted") {
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("走行失敗")
                         : QString::fromStdString("走行失敗: " + target));
    } else if (status == "canceled") {
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("走行キャンセル")
                         : QString::fromStdString("走行キャンセル: " + target));
    } else if (status == "paused") {
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("一時停止中")
                         : QString::fromStdString("一時停止中: " + target));
    } else if (status == "map_switching") {
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("地図切替中")
                         : QString::fromStdString("地図切替中: " + target));
    } else if (status == "map_switch_succeeded") {
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("地図切替完了")
                         : QString::fromStdString("地図切替完了: " + target));
    } else if (status == "map_switch_failed") {
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("地図切替失敗")
                         : QString::fromStdString("地図切替失敗: " + target));
    } else if (status == "backend_unavailable") {
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("ナビゲーション利用不可")
                         : QString::fromStdString("ナビゲーション利用不可: " + target));
    } else if (status == "rejected") {
      // #339: 受理前に拒否されたコマンド (存在しない POI 名、landmark POI を goal 指定、
      // 空 route 等)。直前の status が居座って誤操作に気づけない事態を防ぐ。
      current_nav_mode_ = "idle";
      ui_->NavStatusLabel->setText(
          target.empty() ? QString::fromStdString("コマンド拒否")
                         : QString::fromStdString("コマンド拒否: " + target));
    }
  }, Qt::QueuedConnection);
}

void MapoiPanel::BackendStatusCallback(
  mapoi_interfaces::msg::NavigationBackendStatus::SharedPtr msg)
{
  // RViz panel 側でも backend_ready を見て操作ボタンを gate する (#198, #205 minimal)。
  // backend_status 不在 (旧 mapoi_nav_server build (#208 以前 backend_status contract 未実装、#204 で nav2_bridge へ rename) / panel 単独起動) では callback が呼ばれず、
  // navigation 軸は初期値 (true = enable) を使い続ける。
  nav_backend_status_received_ = true;
  last_navigation_backend_ready_ = msg->backend_ready;
  QMetaObject::invokeMethod(this, [this]() {
    UpdateNavButtonsEnabled();
  }, Qt::QueuedConnection);
}

void MapoiPanel::LocalizationBackendStatusCallback(
  mapoi_interfaces::msg::LocalizationBackendStatus::SharedPtr msg)
{
  // localization bridge の readiness で LocalizationButton を gate する (#209)。
  // 受信前は last_localization_backend_ready_ が初期値 true のままで、互換的に enable のまま。
  localization_backend_status_received_ = true;
  last_localization_backend_ready_ = msg->backend_ready;
  QMetaObject::invokeMethod(this, [this]() {
    UpdateNavButtonsEnabled();
  }, Qt::QueuedConnection);
}

void MapoiPanel::UpdateNavButtonsEnabled()
{
  // Qt main thread で呼ぶこと (BackendStatusCallback / LocalizationBackendStatusCallback /
  // liveliness event_callback から QueuedConnection 経由で invoke される)。
  // 2 軸の minimal 仕様 (#209): navigation 操作 UI と MapComboBox は navigation backend、
  // LocalizationButton は localization backend で gate する。MapComboBox は Nav2 LoadMap +
  // AMCL initial pose の両方を必要とするが、Nav2 LoadMap が走らないと localization 側に意味のある
  // initial pose を送れないので最低限 navigation_ready を要求する。両方を AND する厳格化は
  // future work で UX 検討する。
  // Staleness gating (#208): 受信前 (`*_received_` false) は alive 判定をバイパスして
  // `last_*_backend_ready_` の初期値 (true) を使う = 旧 mapoi_nav_server build (#204 で nav2_bridge へ rename) / panel 単独起動の後方互換。
  // 受信後は MANUAL_BY_TOPIC liveliness で得た `*_alive_` flag と AND し、bridge 死亡時に
  // 自動 disable する。
  const bool nav_ready = nav_backend_status_received_
    ? (last_navigation_backend_ready_ && nav_backend_alive_)
    : last_navigation_backend_ready_;
  const bool loc_ready = localization_backend_status_received_
    ? (last_localization_backend_ready_ && localization_backend_alive_)
    : last_localization_backend_ready_;
  ui_->LocalizationButton->setEnabled(loc_ready);
  ui_->RunGoalButton->setEnabled(nav_ready);
  ui_->RunRouteButton->setEnabled(nav_ready);
  ui_->PauseButton->setEnabled(nav_ready);
  ui_->ResumeButton->setEnabled(nav_ready);
  ui_->StopButton->setEnabled(nav_ready);
  ui_->MapComboBox->setEnabled(nav_ready);
}

void MapoiPanel::PublishHighlightPois()
{
  if (mapoi_highlight_goal_pub_) {
    std_msgs::msg::String goal_msg;
    goal_msg.data = highlighted_goal_name_;
    mapoi_highlight_goal_pub_->publish(goal_msg);
  }

  if (mapoi_highlight_route_pub_) {
    std_msgs::msg::String route_msg;
    for (const auto & name : highlighted_route_poi_names_) {
      if (!route_msg.data.empty()) {
        route_msg.data += ",";
      }
      route_msg.data += name;
    }
    mapoi_highlight_route_pub_->publish(route_msg);
  }
}

}  // mapoi_rviz_plugins
CLASS_LOADER_REGISTER_CLASS(mapoi_rviz_plugins::MapoiPanel, rviz_common::Panel)
