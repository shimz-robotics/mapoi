#include "mapoi_server/mapoi_server.hpp"

#include <cmath>

using namespace std::chrono_literals;

MapoiServer::MapoiServer() : Node("mapoi_server") {
  // parameters
  mapoi_server_pkg_ = ament_index_cpp::get_package_share_directory("mapoi_server");
  // maps_path は REQUIRED (#163 で sample maps を mapoi_server から削除したため default 廃止)。
  this->declare_parameter<std::string>("maps_path", "");
  this->declare_parameter<std::string>("map_name", "turtlebot3_world");
  this->declare_parameter<std::string>("config_file", "mapoi_config.yaml");

  // initial maps path
  maps_path_ = this->get_parameter("maps_path").as_string();
  map_name_ = this->get_parameter("map_name").as_string();
  config_file_ = this->get_parameter("config_file").as_string();
  if (maps_path_.empty()) {
    RCLCPP_FATAL(this->get_logger(),
      "maps_path parameter is REQUIRED. mapoi_server は #163 から sample maps を "
      "提供しなくなったため、起動時に必ず maps_path を指定する必要があります。");
    throw std::runtime_error("maps_path parameter is required");
  }

  load_tag_definitions();
  load_mapoi_config_file();

  // Publish config_path_ (transient_local QoS で latched、subscriber も transient_local に揃え
  // たので定期 publish は廃止、起動時 / SwitchMap / reload_map_info で明示 publish する仕様に
  // 移行 (#135))。
  config_path_publisher_ = this->create_publisher<std_msgs::msg::String>(
    "mapoi_config_path", rclcpp::QoS(1).transient_local());
  publish_config_path();  // 起動時に latched 値として publish

  // Publish initial pose POI name (#144): mapoi_nav_server がこれを受けて /initialpose を流す。
  // 起動時 / SwitchMap / reload で publish する。後起動 subscriber でも受信できるよう transient_local。
  initialpose_poi_publisher_ = this->create_publisher<mapoi_interfaces::msg::InitialPoseRequest>(
    "mapoi_initialpose_poi", rclcpp::QoS(1).transient_local());

  // 起動時の最初の map に対する initial pose POI を publish (#144)。
  // 順序保証: load_mapoi_config_file() (line 21) → publish_initial_poi_name() の順に実行。
  // pois_list_ は load 後に最新化されているので、compute_initial_poi_name の lookup は安全。
  // mapoi_initialpose_poi は transient_local QoS なので mapoi_nav_server 後起動でも latched 値を
  // 受信できる。mapoi_nav_server cb 側は新たに get_pois_info を fetch して名前 lookup するので、
  // mapoi_server 内の pois_list_ 更新と nav_server 側の cb 処理は順序非依存 (#149 review 補足)。
  // 起動時は requested_name = empty で default (POI list 先頭) を採用。
  publish_initial_poi_name("");

  get_pois_info_service_ = this->create_service<mapoi_interfaces::srv::GetPoisInfo>("get_pois_info",
    std::bind(&MapoiServer::get_pois_info_service, this, std::placeholders::_1, std::placeholders::_2));

  get_route_pois_service_ = this->create_service<mapoi_interfaces::srv::GetRoutePois>("get_route_pois",
    std::bind(&MapoiServer::get_route_pois_service, this, std::placeholders::_1, std::placeholders::_2));

  get_maps_info_service_ = this->create_service<mapoi_interfaces::srv::GetMapsInfo>("get_maps_info",
    std::bind(&MapoiServer::get_maps_info_service, this, std::placeholders::_1, std::placeholders::_2));

  get_routes_info_service_ = this->create_service<mapoi_interfaces::srv::GetRoutesInfo>("get_routes_info",
    std::bind(&MapoiServer::get_routes_info_service, this, std::placeholders::_1, std::placeholders::_2));

  switch_map_service_ = this->create_service<mapoi_interfaces::srv::SwitchMap>("switch_map",
    std::bind(&MapoiServer::switch_map_service, this, std::placeholders::_1, std::placeholders::_2));

  reload_map_info_service_ = this->create_service<std_srvs::srv::Trigger>("reload_map_info",
    std::bind(&MapoiServer::reload_map_info_service, this, std::placeholders::_1, std::placeholders::_2));

  get_tag_definitions_srv_ = this->create_service<mapoi_interfaces::srv::GetTagDefinitions>("get_tag_definitions",
    std::bind(&MapoiServer::get_tag_definitions_service, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Ready to serve. The current map_name is %s", config_path_.c_str());
}

mapoi_interfaces::msg::PointOfInterest MapoiServer::yaml_to_poi_msg(const YAML::Node & poi)
{
  mapoi_interfaces::msg::PointOfInterest msg;
  msg.name = poi["name"].as<std::string>("");
  msg.pose.position.x = poi["pose"]["x"].as<double>(0.0);
  msg.pose.position.y = poi["pose"]["y"].as<double>(0.0);
  auto yaw = poi["pose"]["yaw"].as<double>(0.0);
  msg.pose.orientation.z = std::sin(yaw / 2.0);
  msg.pose.orientation.w = std::cos(yaw / 2.0);
  // tolerance struct (Nav2 align、xy / yaw 同時に指定可能)。v0.3.0 で旧 `radius` フィールドから
  // 破壊変更で移行 (#87)。tolerance.xy は POI radius (進入判定距離) としても使われる。
  // 仕様 (#138): xy / yaw 共に >= 0.001、0 / 負値は禁止 (実用上「無反応 POI」を防ぐ)。
  // 違反時は安全側 default に補正 + WARN ログ:
  //   - tolerance struct 不在 / non-map / xy or yaw 欠落 / < 0.001: WARN + default (xy=0.5, yaw=π/4)
  //   - default は sample yaml と整合 (45° = π/4)
  constexpr double TOL_MIN = 0.001;
  constexpr double DEFAULT_TOL_XY = 0.5;
  const double DEFAULT_TOL_YAW = M_PI / 4.0;  // = 45°、sample yaml 統一値と整合
  msg.tolerance.xy = DEFAULT_TOL_XY;
  msg.tolerance.yaw = DEFAULT_TOL_YAW;
  if (!poi["tolerance"]) {
    RCLCPP_WARN(this->get_logger(),
      "POI '%s': 'tolerance' field missing; using default (xy=%.3f, yaw=%.3f rad). "
      "Old 'radius' field is no longer supported (#87).",
      msg.name.c_str(), DEFAULT_TOL_XY, DEFAULT_TOL_YAW);
  } else if (!poi["tolerance"].IsMap()) {
    RCLCPP_WARN(this->get_logger(),
      "POI '%s': 'tolerance' must be a mapping {xy, yaw}; using default (xy=%.3f, yaw=%.3f rad).",
      msg.name.c_str(), DEFAULT_TOL_XY, DEFAULT_TOL_YAW);
  } else {
    const auto & tol = poi["tolerance"];
    // NaN / +-Inf も clamp 対象 (Codex review #139 medium 対応)。yaml-cpp は ".nan" / ".inf"
    // を double に解釈するため、有限性チェックを明示的に入れる。
    if (tol["xy"]) {
      const double v = tol["xy"].as<double>(DEFAULT_TOL_XY);
      if (!std::isfinite(v) || v < TOL_MIN) {
        RCLCPP_WARN(this->get_logger(),
          "POI '%s': 'tolerance.xy' (%.6f) is non-finite or < %.3f m; clamping to %.3f m.",
          msg.name.c_str(), v, TOL_MIN, TOL_MIN);
        msg.tolerance.xy = TOL_MIN;
      } else {
        msg.tolerance.xy = v;
      }
    } else {
      RCLCPP_WARN(this->get_logger(),
        "POI '%s': 'tolerance.xy' missing; using default (%.3f).",
        msg.name.c_str(), DEFAULT_TOL_XY);
    }
    if (tol["yaw"]) {
      const double v = tol["yaw"].as<double>(DEFAULT_TOL_YAW);
      if (!std::isfinite(v) || v < TOL_MIN) {
        RCLCPP_WARN(this->get_logger(),
          "POI '%s': 'tolerance.yaw' (%.6f rad) is non-finite or < %.3f rad; clamping to %.3f rad.",
          msg.name.c_str(), v, TOL_MIN, TOL_MIN);
        msg.tolerance.yaw = TOL_MIN;
      } else {
        msg.tolerance.yaw = v;
      }
    } else {
      RCLCPP_WARN(this->get_logger(),
        "POI '%s': 'tolerance.yaw' missing; using default (%.3f rad ≒ 45 deg).",
        msg.name.c_str(), DEFAULT_TOL_YAW);
    }
  }
  msg.tags = poi["tags"].as<std::vector<std::string>>(std::vector<std::string>{});
  msg.description = poi["description"].as<std::string>("");
  return msg;
}

void MapoiServer::publish_config_path()
{
  auto msg = std_msgs::msg::String();
  msg.data = config_path_;
  config_path_publisher_->publish(msg);
}

void MapoiServer::load_mapoi_config_file()
{
  config_path_ = maps_path_ + "/" + map_name_ + "/" + config_file_;
  RCLCPP_INFO(this->get_logger(), "Loading mapoi config file: %s", config_path_.c_str());
  try {
    YAML::Node mapoi_config = YAML::LoadFile(config_path_);
    pois_list_ = mapoi_config["poi"] ? mapoi_config["poi"] : YAML::Node(YAML::NodeType::Sequence);
    routes_list_ = mapoi_config["route"] ? mapoi_config["route"] : YAML::Node(YAML::NodeType::Sequence);
    nav2_map_list_ = mapoi_config["map"] ? mapoi_config["map"] : YAML::Node(YAML::NodeType::Sequence);

    // Rebuild tag list: system tags + user tags from config
    tag_definitions_.clear();
    for (const auto & tag : system_tags_) {
      tag_definitions_.push_back(tag);
    }
    if (mapoi_config["custom_tags"]) {
      for (const auto & tag : mapoi_config["custom_tags"]) {
        mapoi_interfaces::msg::TagDefinition td;
        td.name = tag["name"].as<std::string>();
        td.description = tag["description"].as<std::string>("");
        td.is_system = false;
        tag_definitions_.push_back(td);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Tag definitions: %zu system + %zu user",
                system_tags_.size(), tag_definitions_.size() - system_tags_.size());
  } catch (const YAML::BadFile & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load mapoi config file: %s", e.what());
    rclcpp::shutdown();
  }
}

void MapoiServer::get_pois_info_service(const std::shared_ptr<mapoi_interfaces::srv::GetPoisInfo::Request> request,
          std::shared_ptr<mapoi_interfaces::srv::GetPoisInfo::Response> response)
{
  (void)request;
  if (!pois_list_ || !pois_list_.IsSequence()) {
    return;
  }
  for (const auto & poi : pois_list_) {
    response->pois_list.push_back(yaml_to_poi_msg(poi));
  }
  RCLCPP_DEBUG(this->get_logger(), "sending back response '%zu'", response->pois_list.size());
}

void MapoiServer::get_route_pois_service(
    const std::shared_ptr<mapoi_interfaces::srv::GetRoutePois::Request> request,
    std::shared_ptr<mapoi_interfaces::srv::GetRoutePois::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "Incoming get_route_pois request: %s", request->route_name.c_str());

  if (!routes_list_ || !routes_list_.IsSequence()) {
    return;
  }

  // Helper to look up a POI by name in pois_list_ and append the converted msg
  // into the given vector (#143).
  auto append_poi_by_name = [this](const std::string & name,
                                    std::vector<mapoi_interfaces::msg::PointOfInterest> & target) {
    if (!pois_list_ || !pois_list_.IsSequence()) return;
    for (const auto &poi : pois_list_) {
      if (poi["name"].as<std::string>("") == name) {
        target.push_back(yaml_to_poi_msg(poi));
        return;
      }
    }
    RCLCPP_WARN(this->get_logger(),
      "Route POI '%s' not found in pois_list_; skipping.", name.c_str());
  };

  for (const auto &route : routes_list_) {
    if (route["name"].as<std::string>("") != request->route_name) continue;

    // Ordered waypoints (sent to Nav2 FollowWaypoints).
    if (route["waypoints"] && route["waypoints"].IsSequence()) {
      for (const auto &wp : route["waypoints"]) {
        append_poi_by_name(wp.as<std::string>(), response->pois_list);
      }
    }

    // route.landmarks (#143): NOT navigated, but radius-monitored while this
    // route is active. Order is informational only.
    // Codex review #147 low: 参照先 POI が `landmark` tag を持つかも検証し、
    // 持たない POI を route.landmarks に書いた場合は WARN ログ (typo / spec 誤解の検知)。
    if (route["landmarks"] && route["landmarks"].IsSequence()) {
      for (const auto &lm : route["landmarks"]) {
        const std::string lm_name = lm.as<std::string>();
        const size_t before = response->landmark_pois.size();
        append_poi_by_name(lm_name, response->landmark_pois);
        if (response->landmark_pois.size() > before) {
          const auto & lm_poi = response->landmark_pois.back();
          bool has_landmark_tag = false;
          for (const auto & t : lm_poi.tags) {
            if (t == "landmark") { has_landmark_tag = true; break; }
          }
          if (!has_landmark_tag) {
            RCLCPP_WARN(this->get_logger(),
              "Route '%s' references '%s' as landmark but POI does not have 'landmark' tag.",
              request->route_name.c_str(), lm_name.c_str());
          }
        }
      }
    }

    break;
  }

  // debug log
  std::string wp_names;
  bool first = true;
  for (const auto &poi : response->pois_list) {
    if (!first) wp_names += ", ";
    first = false;
    wp_names += poi.name;
  }
  std::string lm_names;
  first = true;
  for (const auto &poi : response->landmark_pois) {
    if (!first) lm_names += ", ";
    first = false;
    lm_names += poi.name;
  }
  RCLCPP_DEBUG(this->get_logger(),
    "Collected route '%s': waypoints=[%s], landmarks=[%s]",
    request->route_name.c_str(), wp_names.c_str(), lm_names.c_str());
}

void MapoiServer::get_maps_info_service(
    const std::shared_ptr<mapoi_interfaces::srv::GetMapsInfo::Request> request,
    std::shared_ptr<mapoi_interfaces::srv::GetMapsInfo::Response> response)
{
  (void)request;
  RCLCPP_DEBUG(this->get_logger(), "Incoming get_maps_info request");

  std::vector<std::string> maps_list;
  for (const auto & entry : std::filesystem::directory_iterator(maps_path_)) {
    if (entry.is_directory()) {
      maps_list.push_back(entry.path().filename().string());
    }
  }

  response->maps_list = maps_list;
  response->map_name = map_name_;

  RCLCPP_DEBUG(this->get_logger(), "sending back response '%zu' maps", response->maps_list.size());
}

void MapoiServer::get_routes_info_service(
    const std::shared_ptr<mapoi_interfaces::srv::GetRoutesInfo::Request> request,
    std::shared_ptr<mapoi_interfaces::srv::GetRoutesInfo::Response> response)
{
  (void)request;
  RCLCPP_DEBUG(this->get_logger(), "Incoming get_routes_info request");

  std::vector<std::string> routes_list;
  if (routes_list_ && routes_list_.IsSequence()) {
    for (const auto &route : routes_list_) {
      routes_list.push_back(route["name"].as<std::string>(""));
    }
  }

  response->routes_list = routes_list;
  RCLCPP_DEBUG(this->get_logger(), "sending back response '%zu' routes", response->routes_list.size());
}

// Map
bool MapoiServer::send_load_map_request(rclcpp::Node::SharedPtr node, const std::string& server_name, const std::string& map_file)
{
  auto load_map_client = node->create_client<nav2_msgs::srv::LoadMap>(server_name + "/load_map");
  auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  req->map_url = map_file;

  if (!load_map_client->wait_for_service(10s)) {
    RCLCPP_INFO(node->get_logger(), "%s/load_map service could not available.", server_name.c_str());
    return false;
  }
  auto future = load_map_client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node, future, 1s) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "[%s] Request has been accepted.", __func__);
    return true;
  } else {
    RCLCPP_ERROR(node->get_logger(), "[%s] Didn't return request.", __func__);
    return false;
  }
}

void MapoiServer::switch_map_service(const std::shared_ptr<mapoi_interfaces::srv::SwitchMap::Request> request,
          std::shared_ptr<mapoi_interfaces::srv::SwitchMap::Response> response)
{
  auto map_name_new = request->map_name;
  RCLCPP_INFO(this->get_logger(), "Incoming request: %s", map_name_new.c_str());
  if (map_name_ == map_name_new) {
    response->success = false;
    response->error_message = "The map is already " + map_name_new;
    RCLCPP_INFO(this->get_logger(), "The map is already %s", map_name_new.c_str());
    return;
  }

  map_name_ = map_name_new;
  load_mapoi_config_file();

  auto node = rclcpp::Node::make_shared("sc_map_client");
  if (!nav2_map_list_ || !nav2_map_list_.IsSequence()) {
    response->success = false;
    response->error_message = "No map entries defined in config for " + map_name_new;
    return;
  }
  for (const auto& map : nav2_map_list_) {
    auto map_url = maps_path_ + "/" + map_name_new + "/" + map["map_file"].as<std::string>("");
    bool result = MapoiServer::send_load_map_request(node, map["node_name"].as<std::string>(""), map_url);
    if (!result) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load map for %s", map_url.c_str());
      response->success = false;
      response->error_message = "Failed to load map: " + map_url;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded map for %s", map_url.c_str());
  }

  // Nav2 LoadMap 完了後に initial pose POI 名を publish する (#144 + Cursor review #149 round 2 high
  // 対応)。Nav2 が新 map に切り替わってから AMCL に initial pose が届くようにするため、必ず
  // send_load_map_request 全件成功後にここへ来る。失敗 (上の return) では publish しない。
  publish_initial_poi_name(request->initial_poi_name);

  // mapoi_config_path を再 publish (transient_local で latched 値更新)。subscriber は新 path を
  // 受け取って table / ComboBox を再 fetch する (#135)。
  publish_config_path();

  RCLCPP_INFO(this->get_logger(), "The map was switched into %s", map_name_.c_str());
  response->success = true;
}


void MapoiServer::reload_map_info_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  load_mapoi_config_file();
  // mapoi_config_path を再 publish (transient_local で latched 値更新)。subscriber が
  // table / ComboBox を再 fetch することで save 後の内容更新が反映される (#135)。
  publish_config_path();
  // reload は POI 編集後の再読込 trigger。ここで `mapoi_initialpose_poi` を再 publish すると、
  // mapoi_nav_server 側で /initialpose が再配信され **運用中の自己位置が巻き戻される** 回帰がある
  // (Cursor review #149 round 4 medium 対応)。
  // 初期姿勢の (再) 設定は SwitchMap (= 地図切替) または手動経路 (RViz / WebUI / mapoi_initialpose_poi
  // 直接 publish) を使うのが正しい運用。ここでは publish_initial_poi_name を呼ばない。
  response->success = true;
  response->message = config_path_;
  RCLCPP_INFO(this->get_logger(), "Reloaded mapoi config: %s", config_path_.c_str());
}

// static (純関数). pois_list と requested_name から initial POI 名を決定。
// state を持たないため unit test で直接呼び出せる (#149 round 4 high 対応)。
std::string MapoiServer::compute_initial_poi_name(
  const YAML::Node & pois_list, const std::string & requested_name)
{
  if (!pois_list || !pois_list.IsSequence()) {
    return "";
  }
  auto is_landmark_poi = [](const YAML::Node & poi) {
    if (!poi["tags"] || !poi["tags"].IsSequence()) return false;
    for (const auto & t : poi["tags"]) {
      if (t.as<std::string>() == "landmark") return true;
    }
    return false;
  };
  // pose ノード + x / y / yaw 全てを numeric として読めるかを確認する。
  // bridge 側は欠落時 0.0 fallback なので、ここで弾けば「意図しない (0,0) spawn」を防げる。
  auto has_valid_pose = [](const YAML::Node & poi) {
    if (!poi["pose"]) return false;
    const auto & pose = poi["pose"];
    if (!pose.IsMap()) return false;
    for (const char * k : {"x", "y", "yaw"}) {
      if (!pose[k]) return false;
      try {
        (void)pose[k].as<double>();
      } catch (const YAML::Exception &) {
        return false;
      }
    }
    return true;
  };
  if (!requested_name.empty()) {
    bool found = false;
    for (const auto & poi : pois_list) {
      if (poi["name"].as<std::string>("") != requested_name) continue;
      found = true;
      if (is_landmark_poi(poi)) break;     // landmark → fall back
      if (!has_valid_pose(poi)) break;     // pose 欠落 → fall back
      return requested_name;
    }
    (void)found;  // not-found 警告は呼び出し側で扱う (state-less 化のため log は出さない)
  }
  for (const auto & poi : pois_list) {
    if (is_landmark_poi(poi)) continue;     // landmark は到達不可、initial pose 候補から除外
    if (!has_valid_pose(poi)) continue;     // pose 欠落 POI もスキップ
    const std::string n = poi["name"].as<std::string>("");
    if (!n.empty()) return n;
  }
  return "";
}

void MapoiServer::publish_initial_poi_name(const std::string & requested_name)
{
  // requested_name は SwitchMap.initial_poi_name (空 = default、POI list 先頭採用)。
  // shared state を持たず、呼び出し側が直接渡す形にして thread safety / lifecycle race を排除
  // (Cursor review #149 round 2 high 対応)。
  const std::string target = compute_initial_poi_name(pois_list_, requested_name);
  // ログ順序: target.empty() の WARN は下流に任せ、ここでは「requested 不採用かつ fallback 候補が
  // 存在する」ケースのみ WARN を出す (#149 round 5 low: target.empty() 時に紛らわしい両 WARN が
  // 出るのを避ける)。
  if (!requested_name.empty() && !target.empty() && target != requested_name) {
    RCLCPP_WARN(this->get_logger(),
      "Requested initial_poi_name '%s' was not adopted (not found / landmark / invalid pose); "
      "falling back to POI list first ('%s').",
      requested_name.c_str(), target.c_str());
  }
  if (target.empty()) {
    RCLCPP_WARN(this->get_logger(),
      "No POI available for initial pose in map '%s'; skipping mapoi_initialpose_poi publish.",
      map_name_.c_str());
    return;
  }
  auto msg = mapoi_interfaces::msg::InitialPoseRequest();
  msg.map_name = map_name_;
  msg.poi_name = target;
  initialpose_poi_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(),
    "Published initial pose POI name '%s' for map '%s' (#144).",
    target.c_str(), map_name_.c_str());
}

void MapoiServer::load_tag_definitions()
{
  std::string tag_defs_path = mapoi_server_pkg_ + "/maps/tag_definitions.yaml";
  system_tags_.clear();

  try {
    YAML::Node tag_config = YAML::LoadFile(tag_defs_path);
    if (tag_config["tags"]) {
      for (const auto& tag : tag_config["tags"]) {
        mapoi_interfaces::msg::TagDefinition td;
        td.name = tag["name"].as<std::string>();
        td.description = tag["description"].as<std::string>("");
        td.is_system = true;
        system_tags_.push_back(td);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu system tag definitions from %s",
                system_tags_.size(), tag_defs_path.c_str());
  } catch (const YAML::BadFile& e) {
    RCLCPP_WARN(this->get_logger(), "System tag definitions file not found: %s. Continuing with empty list.",
                tag_defs_path.c_str());
  }
}

void MapoiServer::get_tag_definitions_service(
    const std::shared_ptr<mapoi_interfaces::srv::GetTagDefinitions::Request> request,
    std::shared_ptr<mapoi_interfaces::srv::GetTagDefinitions::Response> response)
{
  (void)request;
  response->definitions = tag_definitions_;
  RCLCPP_DEBUG(this->get_logger(), "Sending %zu tag definitions", tag_definitions_.size());
}

#ifndef UNIT_TEST
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapoiServer>());
  rclcpp::shutdown();
  return 0;
}
#endif
