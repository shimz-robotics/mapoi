#include "mapoi_server/mapoi_rviz2_publisher.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;


MapoiRviz2Publisher::MapoiRviz2Publisher() : Node("mapoi_rviz2_publisher") {
  id_buf_ = 0;

  // POI tolerance visualization (#136 / #179) を表示するか。
  // 描画 layer (false で全 POI 抑制):
  //   - xy 判定円 outline (細い実線、薄め): tolerance.xy = ロボット進入判定の境界
  //   - yaw 制約扇形 (塗り or 中抜き): 0 < tolerance.yaw < π の時のみ重ね描き
  //   - pause overlay (点線 dot): pause tag POI の xy 円沿いに dot pattern
  // Editor 中心の使い方や RViz が情報過多な時に false にする想定。default true。
  this->declare_parameter<bool>("show_tolerance_sector", true);

  // POI label の表示形式: "index" (POI Editor 行番号、1-based) / "name" / "both" (= "<index>: <name>") / "none"。
  // default "index" は WebUI 上の行と RViz label を直接対応させ、文字長を抑えて重なりを減らす。
  // ("off" は ros2 param CLI で bool として解釈されるため "none" を採用)
  this->declare_parameter<std::string>("poi_label_format", "index");

  // Route polyline の表示形式: "all" (全 route 表示、active は強調) / "selected" (active のみ) / "none"。
  // default "selected": RViz 起動時の clutter を抑え、user が選択した route だけを示す。
  // 全 route を見たい場合は ros2 param set /mapoi_rviz2_publisher route_display_mode all で切替。
  this->declare_parameter<std::string>("route_display_mode", "selected");

  marker_goals_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mapoi_goal_marks", 10);
  marker_events_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mapoi_event_marks", 10);
  marker_routes_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mapoi_route_marks", 10);

  this->poi_client_ = this->create_client<mapoi_interfaces::srv::GetPoisInfo>("get_pois_info");
  this->routes_info_client_ = this->create_client<mapoi_interfaces::srv::GetRoutesInfo>("get_routes_info");
  this->route_pois_client_ = this->create_client<mapoi_interfaces::srv::GetRoutePois>("get_route_pois");

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
  request_routes_info();
}

void MapoiRviz2Publisher::request_pois_list()
{
  auto request = std::make_shared<mapoi_interfaces::srv::GetPoisInfo::Request>();
  poi_client_->async_send_request(
    request, std::bind(&MapoiRviz2Publisher::on_poi_received, this, _1));
}

void MapoiRviz2Publisher::request_routes_info()
{
  // get_routes_info → 各 route について get_route_pois の 2 段 fetch。
  // 世代番号 (routes_fetch_generation_) を increment して capture することで、後続 fan-out が始まった
  // 時点で旧世代の callback を stale 判定し all_routes_ への書き込みを drop する
  // (Codex round 1 high 対策: stale callback による旧 map / 新 map の route 混在防止)。
  // service 未起動なら skip (config_path 通知の度に再試行されるので自然回復)。
  if (!routes_info_client_->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "get_routes_info service not ready, skipping route fetch.");
    return;
  }
  const size_t my_gen = ++routes_fetch_generation_;
  auto request = std::make_shared<mapoi_interfaces::srv::GetRoutesInfo::Request>();
  routes_info_client_->async_send_request(
    request,
    [this, my_gen](rclcpp::Client<mapoi_interfaces::srv::GetRoutesInfo>::SharedFuture f) {
      on_routes_info_received(my_gen, f);
    });
}

void MapoiRviz2Publisher::on_routes_info_received(
  size_t my_generation,
  rclcpp::Client<mapoi_interfaces::srv::GetRoutesInfo>::SharedFuture future)
{
  // stale check: 後続 fan-out が走っていたら旧世代の応答を捨てる
  if (my_generation != routes_fetch_generation_) {
    return;
  }
  auto result = future.get();
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get Routes Info.");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Received %zu route names (gen=%zu).",
    result->routes_list.size(), my_generation);

  // 空 route list は all_routes_ を即時 clear (削除された route の取り残し防止)
  if (result->routes_list.empty()) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    all_routes_.clear();
    return;
  }

  // pending map に集約してから lock 下で all_routes_ に swap (timer_callback への部分公開を防ぐ)。
  // shared_ptr で per-route lambda 間で共有、最後の callback で swap する。
  using RouteMap = std::map<std::string, std::vector<mapoi_interfaces::msg::PointOfInterest>>;
  auto pending = std::make_shared<RouteMap>();
  auto remaining = std::make_shared<size_t>(result->routes_list.size());

  for (const auto & route_name : result->routes_list) {
    auto req = std::make_shared<mapoi_interfaces::srv::GetRoutePois::Request>();
    req->route_name = route_name;
    route_pois_client_->async_send_request(
      req,
      [this, route_name, my_generation, pending, remaining]
      (rclcpp::Client<mapoi_interfaces::srv::GetRoutePois>::SharedFuture f) {
        // stale check (per-route): 後続 fan-out が走っていたら結果を捨てる
        if (my_generation != routes_fetch_generation_) {
          return;
        }
        auto r = f.get();
        if (r) {
          (*pending)[route_name] = r->pois_list;
          RCLCPP_INFO(this->get_logger(), "Route '%s': %zu waypoints (gen=%zu).",
            route_name.c_str(), r->pois_list.size(), my_generation);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to get route pois for '%s' (gen=%zu).",
            route_name.c_str(), my_generation);
        }
        // 全 route 集約済みになったら lock 下で all_routes_ に swap
        if (--(*remaining) == 0) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          all_routes_ = std::move(*pending);
        }
      });
  }
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

  // config 変更を検出したら fan-out 実行可否に関わらず routes_fetch_generation_ を進めて
  // 旧 fan-out の pending callback を全て stale 化する (Codex round 2 high 対策: service 未 ready で
  // 早期 return する経路で gen が進まないと、旧 config の callback が my_gen == current_gen を満たして
  // 旧 route set を swap してしまう窓が残る)。
  // request_routes_info() 側でも increment するため fan-out 成功時は double increment になるが、
  // 単調増加のため意味的には正しく、無害。
  ++routes_fetch_generation_;

  // 起動直後は service が未起動の場合がある。POI / routes_info / route_pois 全てが ready の時のみ
  // guard を更新して fan-out 開始する (Codex round 1 medium 対策: route 系 service だけ未 ready の
  // 状態で guard が進むと、同じ path+mtime が dedup で skip され route fetch の自然 retry が無くなる)。
  // 一つでも未 ready なら guard 据え置き → 次回 publish で再試行。
  if (!poi_client_->service_is_ready() ||
      !routes_info_client_->service_is_ready() ||
      !route_pois_client_->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(),
      "Some service not ready (poi=%d, routes_info=%d, route_pois=%d), skipping refresh.",
      poi_client_->service_is_ready(),
      routes_info_client_->service_is_ready(),
      route_pois_client_->service_is_ready());
    return;
  }

  last_config_path_ = current_path;
  last_config_mtime_ = current_mtime;
  request_pois_list();
  request_routes_info();  // POI と同様、SwitchMap / Save で route 構成も変わる可能性あり
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
  // 矢印は扇形 (#136) の方向補助なので細く短く固定 (POI 中心線として yaw のみ示す)。
  // 比率 (length:shaft:head) = 0.15:0.04:0.04。radius 連動の倍率は廃止 (arrow_size_ratio 廃止)。
  default_arrow_marker.scale.x = 0.15; default_arrow_marker.scale.y = 0.04; default_arrow_marker.scale.z = 0.04;
  default_arrow_marker.color.r = 0.0; default_arrow_marker.color.g = 1.0; default_arrow_marker.color.b = 0.0; default_arrow_marker.color.a = 0.7;
  default_arrow_marker.lifetime.sec = 2.0;

  // tolerance visualization (#136 / #179) の表示制御。false で全 POI の円 + 扇形 + pause overlay を抑制。
  // parameter 名は backward compat のため `show_tolerance_sector` のまま (制御対象は scope 拡張、
  // CHANGELOG 参照)。
  const bool show_sector = this->get_parameter("show_tolerance_sector").as_bool();

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

  visualization_msgs::msg::MarkerArray ma_waypoints;
  visualization_msgs::msg::MarkerArray ma_events;
  int id = 0;

  // 扇形 (sector) の床面 z (map / costmap よりわずかに上、0.01m)
  constexpr double SECTOR_Z = 0.01;

  // Pose の orientation (quaternion) から 2D yaw を取り出す。
  // tf2 を引かずに済むよう atan2(2(wz+xy), 1-2(y^2+z^2)) を直書き。
  // 前提: POI は 2D (roll/pitch=0)、quaternion は正規化済 (mapoi_server / Editor 側で保証)。
  // 3D POI 対応や未正規化入力が必要になれば tf2 経由に書き換える。
  auto get_yaw_from_pose = [](const geometry_msgs::msg::Pose & p) -> double {
    const double w = p.orientation.w;
    const double x = p.orientation.x;
    const double y = p.orientation.y;
    const double z = p.orientation.z;
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  };

  // xy 判定円 outline を細い実線で描画する helper (#179)。
  // - radius = tolerance.xy = ロボット進入判定の境界 (yaw 不問)
  // - 扇形 (yaw 制約) と重ね描きする前提で、薄めの alpha + 細い実線で控えめに
  // - LINE_STRIP の頂点列で 36 角形 (10 度刻み) を構成
  auto add_radius_circle = [&](const geometry_msgs::msg::Pose & pose, double radius,
                               float r, float g, float b, float a,
                               int marker_id,
                               visualization_msgs::msg::MarkerArray & target) {
    if (radius <= 0.0) return;
    constexpr int N_SEG = 36;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = rclcpp::Clock().now();
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.id = marker_id;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.02;  // 細い実線 (扇形 stroke と同じ太さ)
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    m.lifetime.sec = 2;
    m.points.reserve(N_SEG + 1);
    for (int i = 0; i <= N_SEG; ++i) {
      const double angle = 2.0 * M_PI * i / N_SEG;
      geometry_msgs::msg::Point p;
      p.x = pose.position.x + radius * std::cos(angle);
      p.y = pose.position.y + radius * std::sin(angle);
      p.z = SECTOR_Z;
      m.points.push_back(p);
    }
    target.markers.push_back(m);
  };

  // 扇形 (sector) marker を描画する helper (#136 / #179)。
  // - radius = tolerance.xy、扇角 = 2 * yaw_tolerance、中心線 = pose.yaw
  // - 0 < yaw_tolerance < π の時のみ描画。yaw 不問 (それ以外の値) は呼び出し側で
  //   add_radius_circle に分岐させる (#179: 円 + 扇形重ね描き)
  // - fill_alpha > 0 → TRIANGLE_LIST で塗り (waypoint 用)
  // - stroke_alpha > 0 → LINE_STRIP で境界線 (中心 → 弧 → 中心、yaw 範囲を半径線で強調)
  // 戻り値: target.markers に push した marker 数 (id 消費数、yaw 不問入力は 0)
  auto add_sector = [&](const geometry_msgs::msg::Pose & pose,
                        double radius, double yaw_tolerance,
                        float r, float g, float b,
                        float fill_alpha, float stroke_alpha,
                        int marker_id_base,
                        visualization_msgs::msg::MarkerArray & target) -> int {
    if (radius <= 0.0) return 0;
    if (yaw_tolerance <= 0.0 || yaw_tolerance >= M_PI) return 0;

    const double half_angle = yaw_tolerance;
    const double yaw_center = get_yaw_from_pose(pose);
    const double total_angle = 2.0 * half_angle;
    // 弧の頂点数: 0.1 rad 刻み相当 (約 5.7 度)、最低 8 (扇形でも視認可能)
    const int n_seg = std::max(8, static_cast<int>(std::ceil(total_angle / 0.1)));

    const double start_angle = yaw_center - half_angle;
    std::vector<geometry_msgs::msg::Point> arc_pts;
    arc_pts.reserve(n_seg + 1);
    for (int i = 0; i <= n_seg; ++i) {
      const double a = start_angle + total_angle * i / n_seg;
      geometry_msgs::msg::Point p;
      p.x = pose.position.x + radius * std::cos(a);
      p.y = pose.position.y + radius * std::sin(a);
      p.z = SECTOR_Z;
      arc_pts.push_back(p);
    }

    geometry_msgs::msg::Point center;
    center.x = pose.position.x;
    center.y = pose.position.y;
    center.z = SECTOR_Z;

    int n_added = 0;

    if (fill_alpha > 0.0f) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = rclcpp::Clock().now();
      m.action = visualization_msgs::msg::Marker::ADD;
      m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      m.id = marker_id_base + n_added;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 1.0;
      m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = fill_alpha;
      m.lifetime.sec = 2;
      m.points.reserve(static_cast<size_t>(n_seg) * 3);
      for (int i = 0; i < n_seg; ++i) {
        m.points.push_back(center);
        m.points.push_back(arc_pts[i]);
        m.points.push_back(arc_pts[i + 1]);
      }
      target.markers.push_back(m);
      n_added += 1;
    }

    if (stroke_alpha > 0.0f) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = rclcpp::Clock().now();
      m.action = visualization_msgs::msg::Marker::ADD;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.id = marker_id_base + n_added;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.02;  // line width (m)、xy 円 outline と同じ太さ
      m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = stroke_alpha;
      m.lifetime.sec = 2;
      // 扇形の境界: 中心 → 弧端 → 弧上 → 弧端 → 中心 (半径線で yaw 範囲を強調)
      m.points.reserve(arc_pts.size() + 2);
      m.points.push_back(center);
      for (const auto & p : arc_pts) m.points.push_back(p);
      m.points.push_back(center);
      target.markers.push_back(m);
      n_added += 1;
    }

    return n_added;
  };

  // pause overlay を xy 円沿いに点線 (dot 形式) で重ね描き (#179)。
  // LINE_LIST で短い segment + 長い gap で dot 表現:
  //   旧 add_dashed_outline (#136) は segment 長 0.05m + 1:1 比率の dash で「点と感じない、
  //   潰れて見える」user feedback (#178 PR コメント) を受け、dot 長を短くし on:off 比率を 1:4 に拡大。
  // pause 発火条件は xy 円内 (#84 hysteresis) なので xy 境界 (= add_radius_circle と同じ円) に重畳する。
  // 主 glyph より僅かに上 (z + 0.001) に置いて隠れ防止。
  auto add_dotted_outline = [&](const geometry_msgs::msg::Pose & pose, double radius,
                                float r, float g, float b, float a,
                                int marker_id,
                                visualization_msgs::msg::MarkerArray & target) {
    if (radius <= 0.0) return;

    // dot 中心間隔 0.10m、dot 長 0.02m → cycle 比 20% on (1:4 on:off)。
    // WebUI 側 (dashArray '2, 6') は 25% on (1:3 on:off) で厳密には僅差あるが、
    // どちらも sparse dot pattern として視覚的に同方向で整合 (#179 cursor review round 2)。
    // typical radius (0.1-2m) で dot として識別可能な粒度。
    constexpr double DOT_STEP = 0.10;
    constexpr double DOT_LENGTH = 0.02;
    const double total_arc_len = 2.0 * M_PI * radius;
    const int n_dots = std::max(12, static_cast<int>(std::ceil(total_arc_len / DOT_STEP)));
    // dot 1 個分の弧角度。極小半径 (Tolerance min 0.001m など) で DOT_LENGTH/radius が
    // セル角を超えると dot が連結して dash 化するため、cell の 40% を cap として保つ
    // (1:4 比率を維持できなくなる場合でも dot 識別性 ≧ on:off 分離を優先)。
    const double cell_angle = (2.0 * M_PI) / n_dots;
    const double dot_angle_span = std::min(DOT_LENGTH / radius, 0.4 * cell_angle);

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = rclcpp::Clock().now();
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.id = marker_id;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.04;  // dot 線幅 (旧 dash の 0.06m から短縮、dot として丸みを出す)
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    m.lifetime.sec = 2;

    m.points.reserve(static_cast<size_t>(n_dots) * 2);
    for (int i = 0; i < n_dots; ++i) {
      const double a1 = (2.0 * M_PI) * i / n_dots;
      const double a2 = a1 + dot_angle_span;
      geometry_msgs::msg::Point p1, p2;
      p1.x = pose.position.x + radius * std::cos(a1);
      p1.y = pose.position.y + radius * std::sin(a1);
      p1.z = SECTOR_Z + 0.001;
      p2.x = pose.position.x + radius * std::cos(a2);
      p2.y = pose.position.y + radius * std::sin(a2);
      p2.z = SECTOR_Z + 0.001;
      m.points.push_back(p1);
      m.points.push_back(p2);
    }
    target.markers.push_back(m);
  };

  size_t poi_index_one_based = 0;
  for (const auto & poi : pois_list_) {
    poi_index_one_based += 1;  // POI Editor (mapoi_config.yaml の poi: 順) 行番号、1-based、tag フィルタ非依存
    geometry_msgs::msg::Pose pose = poi.pose;

    // POI tolerance を 円 (xy 判定領域) + 扇形 (yaw 制約) で重ね描き (#136 / #179、全 POI 共通)。
    // 主 glyph は waypoint > landmark の優先順:
    //   - waypoint: 塗り扇形 (緑)
    //   - landmark: 中抜き扇形 (灰)
    //   - その他 (旧 event 等): scope 外 (色 / glyph 整理は #70)
    // 円 outline は判定 semantics (xy 境界) を控えめに常時表示し、扇形は yaw 制約あり時のみ
    // 重ね描きする。pause tag があれば xy 円沿いに dot pattern overlay を追加。
    // 色は既存 hardcode を継承 (整理は #70)。
    if (show_sector && poi.tolerance.xy > 0.0) {
      const auto has_tag = [&poi](const std::string & target) {
        for (const auto & t : poi.tags) {
          if (t == target) return true;
        }
        return false;
      };

      float sr = 0.5f, sg = 0.5f, sb = 0.5f;
      float fill_a = 0.0f, stroke_a = 0.0f;
      visualization_msgs::msg::MarkerArray * sector_target = nullptr;
      if (has_tag("waypoint")) {
        sr = 0.0f; sg = 1.0f; sb = 0.0f;
        fill_a = 0.4f; stroke_a = 0.7f;
        sector_target = &ma_waypoints;
      } else if (has_tag("landmark")) {
        sr = 0.5f; sg = 0.5f; sb = 0.5f;
        fill_a = 0.15f; stroke_a = 0.7f;  // 透過度高めの薄塗り (#179 ユーザー確認 2: 中抜きから薄塗りに変更)
        sector_target = &ma_events;
      }

      if (sector_target != nullptr) {
        // 円 outline (xy 判定領域、#179): 常時描画。半透明 + 細実線で控えめに「進入判定境界」を示す。
        add_radius_circle(pose, poi.tolerance.xy, sr, sg, sb, 0.4f, id, *sector_target);
        id += 1;

        // 扇形 (yaw 制約、#136): 0 < yaw < π の時のみ重ね描き。
        // それ以外 (yaw 不問、扇形 = 円となり情報冗長) は円 outline のみで表現。
        const bool has_yaw_constraint =
          (poi.tolerance.yaw > 0.0 && poi.tolerance.yaw < M_PI);
        if (has_yaw_constraint) {
          id += add_sector(pose, poi.tolerance.xy, poi.tolerance.yaw,
                           sr, sg, sb,
                           fill_a, stroke_a,
                           id, *sector_target);
        }

        if (has_tag("pause")) {
          // pause overlay は xy 円沿いに dot pattern (#179)。
          // pause 発火条件 (xy 円内) と境界が一致するため自然な視覚的根拠になる。
          add_dotted_outline(pose, poi.tolerance.xy,
                             sr, sg, sb, 0.9f,
                             id, *sector_target);
          id += 1;
        }
      }
    }

    for(const auto & tag : poi.tags){
      if(tag == "waypoint"){
        visualization_msgs::msg::Marker m_waypoint = default_arrow_marker;
        m_waypoint.pose = pose;
        m_waypoint.pose.position.z = 0.1;
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
      else if(tag == "landmark"){
        // landmark POI も矢印 + label を描画する (#85)。reference 専用なので
        // ma_events 側に置く。color は default gray、tag 別 color の整理は #70 で扱う。
        visualization_msgs::msg::Marker m_landmark = default_arrow_marker;
        m_landmark.pose = pose;
        m_landmark.pose.position.z = 0.1;
        m_landmark.color.r = 0.5; m_landmark.color.g = 0.5; m_landmark.color.b = 0.5; m_landmark.color.a = 0.7;
        m_landmark.id = id;
        ma_events.markers.push_back(m_landmark);
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
    marker_goals_pub_->publish(ma_del);
    marker_events_pub_->publish(ma_del);
  }
  id_buf_ = id;

  marker_goals_pub_->publish(ma_waypoints);
  marker_events_pub_->publish(ma_events);

  // Route LINE_STRIP markers (mapoi_route_marks topic)。
  // route_display_mode parameter で表示制御:
  //   "all"      : 全 route 表示、active route (highlighted_route_names_ に含む) は太線+不透明で強調
  //   "selected" : active route のみ表示
  //   "none"     : 表示しない (DELETEALL で既存も消す)
  // POI marker (goal=green / event=blue / landmark=gray) と被らない palette を使う。
  static constexpr std::array<std::array<float, 3>, 6> ROUTE_PALETTE = {{
    {1.0f, 0.5f, 0.0f},   // orange
    {1.0f, 0.0f, 1.0f},   // magenta
    {0.0f, 1.0f, 1.0f},   // cyan
    {1.0f, 1.0f, 0.0f},   // yellow
    {0.7f, 0.3f, 1.0f},   // purple
    {0.0f, 0.7f, 0.3f},   // teal
  }};
  auto pick_route_color = [](const std::string & name) {
    std::hash<std::string> h;
    return ROUTE_PALETTE[h(name) % ROUTE_PALETTE.size()];
  };

  const std::string route_mode = this->get_parameter("route_display_mode").as_string();
  visualization_msgs::msg::MarkerArray ma_routes;
  int route_id = 0;

  if (route_mode != "none") {
    constexpr double ROUTE_LINE_Z = 0.05;  // map / costmap よりわずかに上
    for (const auto & [name, waypoints] : all_routes_) {
      if (waypoints.size() < 2) continue;  // 1 点だけの route は polyline にできない
      const bool is_active = highlighted_route_names_.count(name) > 0;
      if (route_mode == "selected" && !is_active) continue;

      const auto color = pick_route_color(name);
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = rclcpp::Clock().now();
      m.action = visualization_msgs::msg::Marker::ADD;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.id = route_id++;
      m.lifetime.sec = 2.0;
      m.pose.orientation.w = 1.0;  // identity (LINE_STRIP の point は world 座標)
      m.scale.x = is_active ? 0.08 : 0.04;  // active を太く (line width)
      m.color.r = color[0]; m.color.g = color[1]; m.color.b = color[2];
      m.color.a = is_active ? 0.9f : 0.4f;  // active を不透明、他は薄く

      m.points.reserve(waypoints.size());
      for (const auto & wp : waypoints) {
        geometry_msgs::msg::Point p;
        p.x = wp.pose.position.x;
        p.y = wp.pose.position.y;
        p.z = ROUTE_LINE_Z;
        m.points.push_back(p);
      }
      ma_routes.markers.push_back(m);
    }
  }

  if (route_id_buf_ > route_id) {
    visualization_msgs::msg::Marker m_del;
    m_del.action = visualization_msgs::msg::Marker::DELETEALL;
    visualization_msgs::msg::MarkerArray ma_del;
    ma_del.markers.push_back(m_del);
    marker_routes_pub_->publish(ma_del);
  }
  route_id_buf_ = route_id;
  marker_routes_pub_->publish(ma_routes);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapoiRviz2Publisher>());
  rclcpp::shutdown();
  return 0;
}