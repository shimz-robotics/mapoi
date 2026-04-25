#include "mapoi_server/mapoi_rviz2_publisher.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;


MapoiRviz2Publisher::MapoiRviz2Publisher() : Node("mapoi_rviz2_publisher") {
  id_buf_ = 0;

  // POI 矢印の大きさを radius に対する比率で指定する (arrow_size = radius × ratio)。
  // default 1.0 = radius と同じ長さ。Route 矢印 (waypoint 間) は radius 概念を持たないので対象外。
  this->declare_parameter<double>("arrow_size_ratio", 1.0);

  // POI label の表示形式: "index" (POI Editor 行番号、1-based) / "name" / "both" (= "<index>: <name>") / "none"。
  // default "index" は WebUI 上の行と RViz label を直接対応させ、文字長を抑えて重なりを減らす。
  // ("off" は ros2 param CLI で bool として解釈されるため "none" を採用)
  this->declare_parameter<std::string>("poi_label_format", "index");

  marker_waypoints_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mapoi_goal_marks", 10);
  marker_events_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mapoi_event_marks", 10);

  this->poi_client_ = this->create_client<mapoi_interfaces::srv::GetPoisInfo>("get_pois_info");

  highlight_goal_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_highlight_goal", 10,
    std::bind(&MapoiRviz2Publisher::on_highlight_goal_received, this, _1));
  highlight_route_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_highlight_route", 10,
    std::bind(&MapoiRviz2Publisher::on_highlight_route_received, this, _1));

  // mapoi_config_path 変化検出で POI list を再取得 (WebUI / Panel 並び替え保存 / SwitchMap 対応)。
  // QoS は mapoi_nav_server と同じ transient_local。後起動でも latched 値を受信できる。
  config_path_sub_ = this->create_subscription<std_msgs::msg::String>(
    "mapoi_config_path", rclcpp::QoS(1).transient_local(),
    std::bind(&MapoiRviz2Publisher::on_config_path_changed, this, _1));

  // 初期化シーケンスをデッドロック回避のため少し遅延させて開始
  this->init_timer_ = this->create_wall_timer(100ms, std::bind(&MapoiRviz2Publisher::start_sequence, this));

  timer_ = this->create_wall_timer(1s, std::bind(&MapoiRviz2Publisher::timer_callback, this));
}


void MapoiRviz2Publisher::start_sequence()
{
  this->init_timer_->cancel();

  if (!poi_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for services.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Services not available, retrying...");
    this->init_timer_->reset();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Requesting POI Info...");
  request_pois_list();
}

void MapoiRviz2Publisher::request_pois_list()
{
  auto request = std::make_shared<mapoi_interfaces::srv::GetPoisInfo::Request>();
  poi_client_->async_send_request(
    request, std::bind(&MapoiRviz2Publisher::on_poi_received, this, _1));
}

void MapoiRviz2Publisher::on_config_path_changed(const std_msgs::msg::String::SharedPtr msg)
{
  // mapoi_server は config path 文字列を周期 publish (default 5s) する。path だけで dedup すると
  // SwitchMap (path 変更) は拾えるが、WebUI/Panel Save (path 不変、内容のみ変更) を取りこぼす。
  // YAML ファイルの mtime も併せて比較し、両 case を検出する。
  const std::string & current_path = msg->data;
  std::filesystem::file_time_type current_mtime{};
  std::error_code ec;
  auto stat_mtime = std::filesystem::last_write_time(current_path, ec);
  if (!ec) {
    current_mtime = stat_mtime;
    if (current_path == last_config_path_ && current_mtime == last_config_mtime_) {
      return;  // 周期 publish (path も内容も不変) → skip
    }
  }
  // stat 失敗時は dedup 不能とみなし fetch を試みる (起動 race などで一時的に発生し得る)

  RCLCPP_INFO(this->get_logger(), "Map config changed: %s — refreshing POI list.", current_path.c_str());

  // 起動直後は service が未起動の場合がある。guard 値は更新前に readiness 判定して、未起動なら skip。
  // last_config_path_ / last_config_mtime_ を未更新のまま return するため、次回 publish で再試行される。
  if (!poi_client_->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "get_pois_info service not ready, skipping refresh.");
    return;
  }

  last_config_path_ = current_path;
  last_config_mtime_ = current_mtime;
  request_pois_list();
}

void MapoiRviz2Publisher::on_poi_received(rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedFuture future)
{
  auto result = future.get();
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get Pois Info.");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    pois_list_ = result->pois_list;
  }
  RCLCPP_INFO(this->get_logger(), "Received %zu POIs.", pois_list_.size());
}


void MapoiRviz2Publisher::on_highlight_goal_received(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  highlighted_goal_names_.clear();
  if (!msg->data.empty()) {
    highlighted_goal_names_.insert(msg->data);
  }
}

void MapoiRviz2Publisher::on_highlight_route_received(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  highlighted_route_names_.clear();
  highlighted_route_ordered_.clear();
  if (!msg->data.empty()) {
    std::istringstream ss(msg->data);
    std::string token;
    int order = 1;
    while (std::getline(ss, token, ',')) {
      if (!token.empty()) {
        highlighted_route_names_[token] = order++;
        highlighted_route_ordered_.push_back(token);
      }
    }
  }
}

void MapoiRviz2Publisher::timer_callback(){
  std::lock_guard<std::mutex> lock(data_mutex_);
  // publish markers on rviz
  visualization_msgs::msg::Marker default_arrow_marker;
  default_arrow_marker.header.frame_id = "map";
  default_arrow_marker.header.stamp = rclcpp::Clock().now();
  default_arrow_marker.action = visualization_msgs::msg::Marker::ADD;
  default_arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
  // Default scale (radius が無い / 0 の POI 用 fallback)。比率 (length:shaft:head) = 0.3:0.2:0.1
  default_arrow_marker.scale.x = 0.3; default_arrow_marker.scale.y = 0.2; default_arrow_marker.scale.z = 0.1;
  default_arrow_marker.color.r = 0.0; default_arrow_marker.color.g = 1.0; default_arrow_marker.color.b = 0.0; default_arrow_marker.color.a = 0.7;
  default_arrow_marker.lifetime.sec = 2.0;

  // POI radius に基づく arrow scale 適用 helper。
  // 既存 default 比率 (length:shaft:head = 0.3:0.2:0.1) を保ち、length を radius × ratio に。
  const double arrow_size_ratio =
    this->get_parameter("arrow_size_ratio").as_double();
  auto apply_radius_scale = [&](visualization_msgs::msg::Marker & m, double radius) {
    if (radius <= 0.0 || arrow_size_ratio <= 0.0) {
      return;  // default scale を維持
    }
    const double length = radius * arrow_size_ratio;
    m.scale.x = length;
    m.scale.y = length * (0.2 / 0.3);  // shaft diameter
    m.scale.z = length * (0.1 / 0.3);  // head diameter
  };

  // POI label の format ("index" / "name" / "both" / "none") を runtime parameter で切替。
  // 空文字列を返した場合は label を生成しない (none モード)。
  const std::string label_format =
    this->get_parameter("poi_label_format").as_string();
  auto build_label = [&label_format](size_t index_one_based, const std::string & name) -> std::string {
    if (label_format == "none") return "";
    if (label_format == "name") return name;
    if (label_format == "both") return std::to_string(index_one_based) + ": " + name;
    // default "index" (および未知の値)
    return std::to_string(index_one_based);
  };

  visualization_msgs::msg::Marker default_text_marker = default_arrow_marker;
  default_text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  default_text_marker.scale.x = 0.2; default_text_marker.scale.y = 0.2; default_text_marker.scale.z = 0.2;
  default_text_marker.color.r = 0.0; default_text_marker.color.g = 0.0; default_text_marker.color.b = 0.0; default_text_marker.color.a = 1.0;

  // POI radius を床面の円 (LINE_STRIP) で描画する template。
  // CYLINDER と異なり top-down view で map / costmap と Z 重なりにくく、視界を遮らない。
  visualization_msgs::msg::Marker default_circle_marker = default_arrow_marker;
  default_circle_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  default_circle_marker.scale.x = 0.03;  // line width (m)
  default_circle_marker.scale.y = 0.0; default_circle_marker.scale.z = 0.0;
  default_circle_marker.pose.orientation.w = 1.0;  // identity (LINE_STRIP の point は world 座標で指定するため)

  visualization_msgs::msg::MarkerArray ma_waypoints;
  visualization_msgs::msg::MarkerArray ma_events;
  int id = 0;

  // 円の頂点数 (滑らかさと marker サイズの trade-off、36 角形 = 10度刻みで十分)
  constexpr int CIRCLE_SEGMENTS = 36;
  // 円を描く床面の z (map / costmap よりわずかに上、0.01m)
  constexpr double CIRCLE_Z = 0.01;
  // 円の透明度 (薄めにして他 marker や map を遮らない)
  constexpr float CIRCLE_ALPHA = 0.5f;
  auto add_radius_circle = [&](const geometry_msgs::msg::Pose & p, double radius,
                               float r, float g, float b,
                               int marker_id,
                               visualization_msgs::msg::MarkerArray & target) {
    visualization_msgs::msg::Marker m = default_circle_marker;
    m.id = marker_id;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = CIRCLE_ALPHA;
    m.points.reserve(CIRCLE_SEGMENTS + 1);
    for (int i = 0; i <= CIRCLE_SEGMENTS; ++i) {
      const double angle = 2.0 * M_PI * static_cast<double>(i) / CIRCLE_SEGMENTS;
      geometry_msgs::msg::Point pt;
      pt.x = p.position.x + radius * std::cos(angle);
      pt.y = p.position.y + radius * std::sin(angle);
      pt.z = CIRCLE_Z;
      m.points.push_back(pt);
    }
    target.markers.push_back(m);
  };

  size_t poi_index_one_based = 0;
  for (const auto & poi : pois_list_) {
    poi_index_one_based += 1;  // POI Editor (mapoi_config.yaml の poi: 順) 行番号、1-based、tag フィルタ非依存
    geometry_msgs::msg::Pose pose = poi.pose;

    // POI radius を床面の円で描画 (全 POI 共通)。
    // primary tag の color を採用。優先順位は tags 配列の順序ではなく
    // goal > event > origin で固定 (poi.tags が ["event", "goal"] のような順でも goal が勝つ)。
    if (poi.radius > 0.0) {
      const auto has_tag = [&poi](const std::string & target) {
        for (const auto & t : poi.tags) {
          if (t == target) return true;
        }
        return false;
      };

      float cr = 0.5f, cg = 0.5f, cb = 0.5f;  // default gray (recognized tag none)
      visualization_msgs::msg::MarkerArray * circle_target = &ma_events;
      if (has_tag("goal")) {
        cr = 0.0f; cg = 1.0f; cb = 0.0f;
        circle_target = &ma_waypoints;
      } else if (has_tag("event")) {
        cr = 0.0f; cg = 0.0f; cb = 1.0f;
        circle_target = &ma_events;
      } else if (has_tag("origin")) {
        cr = 1.0f; cg = 0.0f; cb = 0.0f;
        circle_target = &ma_events;
      }
      add_radius_circle(pose, poi.radius, cr, cg, cb, id, *circle_target);
      id += 1;
    }

    for(const auto & tag : poi.tags){
      if(tag == "goal"){
        visualization_msgs::msg::Marker m_waypoint = default_arrow_marker;
        m_waypoint.pose = pose;
        m_waypoint.pose.position.z = 0.1;
        apply_radius_scale(m_waypoint, poi.radius);
        {
          bool is_goal = highlighted_goal_names_.count(poi.name) > 0;
          if (is_goal) {
            m_waypoint.color.r = 1.0; m_waypoint.color.g = 0.6; m_waypoint.color.b = 0.0; m_waypoint.color.a = 0.7;
          }
        }
        m_waypoint.id = id;
        ma_waypoints.markers.push_back(m_waypoint);
        id += 1;

        const std::string label_text = build_label(poi_index_one_based, poi.name);
        if (!label_text.empty()) {
          visualization_msgs::msg::Marker m_text = default_text_marker;
          m_text.text = label_text;
          m_text.pose = pose;
          m_text.pose.position.z = 0.1;
          m_text.id = id;
          ma_waypoints.markers.push_back(m_text);
          id += 1;
        }
      }
      else if(tag == "event"){
        visualization_msgs::msg::Marker m_event = default_arrow_marker;
        m_event.pose = pose;
        m_event.pose.position.z = 0.1;
        apply_radius_scale(m_event, poi.radius);
        m_event.color.r = 0.0; m_event.color.g = 0.0; m_event.color.b = 1.0; m_event.color.a = 0.7;
        m_event.id = id;
        ma_events.markers.push_back(m_event);
        id += 1;

        const std::string label_text = build_label(poi_index_one_based, poi.name);
        if (!label_text.empty()) {
          visualization_msgs::msg::Marker m_text = default_text_marker;
          m_text.text = label_text;
          m_text.pose = pose;
          m_text.pose.position.z = 0.1;
          m_text.id = id;
          ma_events.markers.push_back(m_text);
          id += 1;
        }
      }
      else if(tag == "origin"){
        visualization_msgs::msg::Marker m_event = default_arrow_marker;
        m_event.pose = pose;
        m_event.pose.position.z = 0.1;
        apply_radius_scale(m_event, poi.radius);
        m_event.color.r = 1.0; m_event.color.g = 0.0; m_event.color.b = 0.0; m_event.color.a = 0.7;
        m_event.id = id;
        ma_events.markers.push_back(m_event);
        id += 1;
      }
    }
  }

  // ルート経由点間の矢印マーカー生成
  if (highlighted_route_ordered_.size() >= 2) {
    // POI名→poseのルックアップマップを構築
    std::map<std::string, geometry_msgs::msg::Pose> poi_pose_map;
    for (const auto & poi : pois_list_) {
      poi_pose_map[poi.name] = poi.pose;
    }

    for (size_t i = 0; i + 1 < highlighted_route_ordered_.size(); ++i) {
      auto it_start = poi_pose_map.find(highlighted_route_ordered_[i]);
      auto it_end = poi_pose_map.find(highlighted_route_ordered_[i + 1]);
      if (it_start == poi_pose_map.end() || it_end == poi_pose_map.end()) {
        continue;
      }

      visualization_msgs::msg::Marker m_arrow;
      m_arrow.header.frame_id = "map";
      m_arrow.header.stamp = rclcpp::Clock().now();
      m_arrow.action = visualization_msgs::msg::Marker::ADD;
      m_arrow.type = visualization_msgs::msg::Marker::ARROW;
      m_arrow.scale.x = 0.05;  // shaft diameter
      m_arrow.scale.y = 0.1;   // head diameter
      m_arrow.scale.z = 0.1;   // head length
      m_arrow.color.r = 0.6; m_arrow.color.g = 0.2; m_arrow.color.b = 1.0; m_arrow.color.a = 0.7;
      m_arrow.lifetime.sec = 2.0;

      geometry_msgs::msg::Point p_start;
      p_start.x = it_start->second.position.x;
      p_start.y = it_start->second.position.y;
      p_start.z = 0.15;
      geometry_msgs::msg::Point p_end;
      p_end.x = it_end->second.position.x;
      p_end.y = it_end->second.position.y;
      p_end.z = 0.15;

      m_arrow.points.push_back(p_start);
      m_arrow.points.push_back(p_end);
      m_arrow.id = id;
      ma_waypoints.markers.push_back(m_arrow);
      id += 1;
    }
  }

  // delete remaining incorrect marker
  if(id_buf_ > id) {
    visualization_msgs::msg::Marker m_del;
    m_del.action = visualization_msgs::msg::Marker::DELETEALL;
    visualization_msgs::msg::MarkerArray ma_del;
    ma_del.markers.push_back(m_del);
    marker_waypoints_pub_->publish(ma_del);
    marker_events_pub_->publish(ma_del);
  }
  id_buf_ = id;

  marker_waypoints_pub_->publish(ma_waypoints);
  marker_events_pub_->publish(ma_events);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapoiRviz2Publisher>());
  rclcpp::shutdown();
  return 0;
}