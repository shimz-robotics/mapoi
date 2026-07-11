#include "mapoi_rviz_plugins/mapoi_panel.hpp"
#include <class_loader/class_loader.hpp>

#include "ui_mapoi_panel.h"
// ConfigPathCallback / SetMapComboBox / SetNav2GoalComboBox / SetMapoiRouteComboBox は
// mapoi_panel_config.cpp (#397 step 1) へ移動。

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

  // reject_clear_timer_: 受信から 5 秒後に CommandRejectedLabel をクリアする。
  // singleShot=true かつ再受信時に start() を呼ぶことで連続 reject でも先行タイマーが
  // 後続表示を早期クリアする race を防ぐ (start() は未満了タイマーを自動リスタートする)。
  // 購読より先に生成しておく (callback の queued lambda が timer を参照するため、
  // 将来 onInitialize 内に Qt イベント処理が入っても null 参照にならない順序を保つ)。
  reject_clear_timer_ = new QTimer(this);
  reject_clear_timer_->setSingleShot(true);
  reject_clear_timer_->setInterval(5000);
  connect(reject_clear_timer_, &QTimer::timeout, this, [this]() {
    ui_->CommandRejectedLabel->setText(QString{});
  });

  // #398: command_rejected は volatile depth 10 (publisher と整合)。latched 不要 (イベント通知)。
  command_rejected_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "mapoi/nav/command_rejected", rclcpp::QoS(10),
      std::bind(&MapoiPanel::CommandRejectedCallback, this, std::placeholders::_1));

  // #406: mapoi/events は volatile depth 10 (publisher と整合)。ROUTE 走行時のみ受信される。
  poi_event_sub_ = node_->create_subscription<mapoi_interfaces::msg::PoiEvent>(
      "mapoi/events", rclcpp::QoS(10),
      std::bind(&MapoiPanel::PoiEventCallback, this, std::placeholders::_1));

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

// BackendStatusCallback / LocalizationBackendStatusCallback / UpdateNavButtonsEnabled は
// mapoi_panel_backend_status.cpp (#397 step 2) へ移動。

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
