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
  get_pois_info_client_ = service_node_->create_client<mapoi_interfaces::srv::GetPoisInfo>("get_pois_info");
  get_maps_info_client_ = service_node_->create_client<mapoi_interfaces::srv::GetMapsInfo>("get_maps_info");
  get_routes_info_client_ = service_node_->create_client<mapoi_interfaces::srv::GetRoutesInfo>("get_routes_info");
  get_route_pois_client_ = service_node_->create_client<mapoi_interfaces::srv::GetRoutePois>("get_route_pois");

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
    RCLCPP_ERROR(LOGGER, "get_maps_info service not available after 3s timeout.");
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
    RCLCPP_ERROR(LOGGER, "Failed to call service get_maps_info");
  }

  // Buttons
  // LocalizationButton は `/initialpose` を直接叩かず、`mapoi/initialpose_poi` 経由で bridge に
  // POI 名を渡す (#209)。bridge 側で POI 名 → pose resolve / retry / `initial_pose_topic` parameter
  // による別 topic 対応を一元化するため、panel と WebUI で同じ flow に揃える。
  mapoi_initialpose_poi_pub_ = node_->create_publisher<mapoi_interfaces::msg::InitialPoseRequest>(
    "mapoi/initialpose_poi", rclcpp::QoS(1).transient_local());
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

  // QoS は mapoi_nav_server と同じ transient_local。後起動 panel でも latched 値を受信できる。
  nav_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "mapoi/nav/status", rclcpp::QoS(1).transient_local(),
      std::bind(&MapoiPanel::NavStatusCallback, this, std::placeholders::_1));

  // Navigation backend readiness (#198 review medium): bridge が publish する readiness で
  // ボタンを gate する。受信前 (古い nav_server / panel 単独起動) は全 enable のまま。
  backend_status_sub_ = node_->create_subscription<mapoi_interfaces::msg::NavigationBackendStatus>(
      "mapoi/nav/backend_status", rclcpp::QoS(1).transient_local(),
      std::bind(&MapoiPanel::BackendStatusCallback, this, std::placeholders::_1));

  // Localization backend readiness (#209): localization bridge が publish する readiness で
  // LocalizationButton を gate する。Navigation 軸とは独立。受信前 (bridge 不在 / editor 構成)
  // は全 enable のまま (BackendStatusCallback と同じフォールバック方針)。
  localization_backend_status_sub_ = node_->create_subscription<
    mapoi_interfaces::msg::LocalizationBackendStatus>(
      "mapoi/localization/backend_status", rclcpp::QoS(1).transient_local(),
      std::bind(&MapoiPanel::LocalizationBackendStatusCallback, this, std::placeholders::_1));

  // Bridge プロセス死亡時の staleness 検出 (#209 review、#208 軽量代替)。
  // transient_local の cache は callback が呼ばれない限り古い値を保持し続けるため、
  // 1Hz で publisher 数を polling し、0 のときは ready=false 相当に倒す。
  backend_staleness_timer_ = node_->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&MapoiPanel::BackendStalenessTick, this));

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
    RCLCPP_WARN(LOGGER, "mapoi/nav/switch_map has no subscribers; mapoi_nav_server may not be running.");
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
        if (response) {
          for (const auto & poi : response->pois_list) {
            highlighted_route_poi_names_.push_back(poi.name);
          }
        }
      }
    } else {
      RCLCPP_ERROR(LOGGER, "get_route_pois service not available after 3s timeout.");
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
  // /initialpose は叩かず、`mapoi/initialpose_poi` 経由で bridge に POI 名を渡す (#209 review fix)。
  // bridge が POI を resolve して `/initialpose` (または `initial_pose_topic` で指定された topic) に
  // 配信する。これにより landmark 排他 / subscriber 後起動 retry / custom localization bridge への
  // 切替が WebUI と同一 flow で行われる。
  mapoi_interfaces::msg::InitialPoseRequest msg;
  msg.map_name = current_map_;
  msg.poi_name = pois_[goal_combobox_ind_].name;
  mapoi_initialpose_poi_pub_->publish(msg);
  RCLCPP_INFO(LOGGER, "Published initial pose request: %s (map: %s).",
              msg.poi_name.c_str(), msg.map_name.c_str());
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
    RCLCPP_ERROR(LOGGER, "get_pois_info service not available after 3s timeout.");
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
    RCLCPP_ERROR(LOGGER, "Failed to call service get_pois_info");
  }
}

void MapoiPanel::SetMapoiRouteComboBox()
{
  // 再構築前に現在 selection を保存し、同名項目があれば復元する (#135 副作用対応)。
  // 他クライアント save (route 追加 / 内容変更) の reload で route 選択が消えるのを防ぐ。
  QString prev_selection = ui_->MapoiRouteComboBox->currentText();

  if (!get_routes_info_client_->wait_for_service(3s)) {
    RCLCPP_ERROR(LOGGER, "get_routes_info service not available after 3s timeout.");
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
    RCLCPP_ERROR(LOGGER, "Failed to call service get_routes_info");
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
    }
  }, Qt::QueuedConnection);
}

void MapoiPanel::BackendStatusCallback(
  mapoi_interfaces::msg::NavigationBackendStatus::SharedPtr msg)
{
  // RViz panel 側でも backend_ready を見て操作ボタンを gate する (#198, #205 minimal)。
  // backend_status 不在 (古い nav_server / panel 単独起動) では callback が呼ばれず、
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

void MapoiPanel::BackendStalenessTick()
{
  // bridge プロセス死亡時の staleness 検出 (#209 review、#208 軽量代替)。
  // publisher 数 = 0 を検知したら、cache された ready=true を強制的に false に倒す
  // (callback は呼ばれないので last_*_backend_ready_ を上書きする必要がある)。
  // publisher が再び現れて新しい ready 値を送ってくれば、その callback で last_* が上書きされる。
  // 重要 (review fix): `*_received_` flag が false の間は何もしない。これは「contract 未実装の
  // 旧 nav_server / editor 構成」では publisher 数が常に 0 でも UI を disable しない、という
  // 後方互換の保証。一度でも status を受け取った publisher の死亡だけを staleness として扱う。
  bool changed = false;
  if (nav_backend_status_received_ &&
      node_->count_publishers("mapoi/nav/backend_status") == 0 &&
      last_navigation_backend_ready_) {
    last_navigation_backend_ready_ = false;
    changed = true;
  }
  if (localization_backend_status_received_ &&
      node_->count_publishers("mapoi/localization/backend_status") == 0 &&
      last_localization_backend_ready_) {
    last_localization_backend_ready_ = false;
    changed = true;
  }
  if (changed) {
    QMetaObject::invokeMethod(this, [this]() {
      UpdateNavButtonsEnabled();
    }, Qt::QueuedConnection);
  }
}

void MapoiPanel::UpdateNavButtonsEnabled()
{
  // Qt main thread で呼ぶこと (BackendStatusCallback / LocalizationBackendStatusCallback /
  // BackendStalenessTick から QueuedConnection 経由で invoke される)。
  // 2 軸の minimal 仕様 (#209): navigation 操作 UI と MapComboBox は navigation backend、
  // LocalizationButton は localization backend で gate する。MapComboBox は Nav2 LoadMap +
  // AMCL initial pose の両方を必要とするが、Nav2 LoadMap が走らないと localization 側に意味のある
  // initial pose を送れないので最低限 navigation_ready を要求する。両方を AND する厳格化は
  // future work で UX 検討する。
  ui_->LocalizationButton->setEnabled(last_localization_backend_ready_);
  ui_->RunGoalButton->setEnabled(last_navigation_backend_ready_);
  ui_->RunRouteButton->setEnabled(last_navigation_backend_ready_);
  ui_->PauseButton->setEnabled(last_navigation_backend_ready_);
  ui_->ResumeButton->setEnabled(last_navigation_backend_ready_);
  ui_->StopButton->setEnabled(last_navigation_backend_ready_);
  ui_->MapComboBox->setEnabled(last_navigation_backend_ready_);
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
