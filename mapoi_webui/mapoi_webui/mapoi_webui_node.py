#!/usr/bin/env python3
"""mapoi_webui_node: ROS2 node with embedded Flask server for mapoi Web UI."""

import math
import os
import json
import logging
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mapoi_interfaces.srv import GetMapsInfo, GetTagDefinitions, SelectMap
from mapoi_interfaces.msg import InitialPoseRequest
import tf2_ros

from flask import Flask, jsonify, request, send_from_directory, Response

from mapoi_webui.yaml_handler import load_config, save_pois, save_routes, save_custom_tags, get_pois, get_routes
from mapoi_webui.map_image import get_map_metadata, get_map_png


def _validate_unique_names(items, label):
    """POI / route の name list が空 / 重複を含まないか検査する (#109)。

    frontend (poi-editor / route-editor) は formOk で同じ check を持つが、
    yaml 直編集や別 client からの POST に対する保険として backend でも reject。
    return: エラーメッセージ文字列 (issue あり) または None (OK)。case-sensitive 判定。
    """
    # Top-level 型 check: list でないと {"pois": null} や数値が validate を
    # 素通りして save_* で 500 になる (Codex PR #120 round 1 medium)。
    if not isinstance(items, list):
        return f'{label} list must be an array'
    seen = set()
    for i, it in enumerate(items):
        name = (it.get('name') if isinstance(it, dict) else None)
        if not name or not isinstance(name, str) or not name.strip():
            return f'{label} #{i} has empty name'
        name = name.strip()
        if name in seen:
            return f'{label} name "{name}" is duplicated'
        seen.add(name)
    return None


# tolerance 最小値 (msg spec #138): xy = 1 mm、yaw = 約 0.057°。
# 0 / 負値は「無反応 POI」を許してしまうため明示的に禁止。
_TOLERANCE_MIN = 0.001


def _validate_pois_tag_exclusivity(pois):
    """POI list の tag 排他組合せを backend でも reject する (#85, #143)。

    frontend (poi-editor.formOk → MapoiPoiFilter.validatePoiTags) で reject 済みだが、
    yaml 直編集や別 client からの POST に対する保険として backend でも検査。
    return: エラーメッセージ文字列 (issue あり) または None (OK)。
    """
    for i, poi in enumerate(pois):
        tags = []
        if isinstance(poi, dict) and isinstance(poi.get('tags'), list):
            tags = [str(t).lower() for t in poi['tags']]
        has_waypoint = 'waypoint' in tags
        has_landmark = 'landmark' in tags
        has_pause = 'pause' in tags
        name = (poi.get('name', '?') if isinstance(poi, dict) else '?')
        if has_waypoint and has_landmark:
            return (f'POI #{i} ({name}): "waypoint" と "landmark" は併用できません '
                    '(landmark は Nav2 navigation 不可な reference 専用)')
        if has_pause and has_landmark:
            return (f'POI #{i} ({name}): "pause" と "landmark" は併用できません '
                    '(landmark は到達不可な reference のため pause 動作が成立しません)')
        # (initial_pose × landmark 排他は #144 で initial_pose system tag を廃止したため不要に。)
    return None


def _validate_pois_tolerance(pois):
    """POI list の tolerance.{xy,yaw} が finite な number で >= 0.001 を満たすかを検査する (#138)。

    frontend (poi-editor の HTML min / parseTolerance) で reject 済みだが、
    yaml 直編集や別 client からの POST に対する保険として backend でも reject。
    bool は int subclass のため明示除外、NaN / +-Inf は math.isfinite で reject
    (Codex review #139 medium 対応)。
    return: エラーメッセージ文字列 (issue あり) または None (OK)。
    """
    for i, poi in enumerate(pois):
        if not isinstance(poi, dict):
            return f'POI #{i} must be an object'
        tol = poi.get('tolerance')
        if not isinstance(tol, dict):
            return f'POI #{i} ({poi.get("name", "?")}): "tolerance" must be a mapping {{xy, yaw}}'
        for key in ('xy', 'yaw'):
            v = tol.get(key)
            if (isinstance(v, bool)
                    or not isinstance(v, (int, float))
                    or not math.isfinite(v)
                    or v < _TOLERANCE_MIN):
                return (f'POI #{i} ({poi.get("name", "?")}): '
                        f'"tolerance.{key}" must be a finite number >= {_TOLERANCE_MIN}')
    return None


class MapoiWebNode(Node):
    def __init__(self):
        super().__init__('mapoi_webui_node')

        # Parameters (same as mapoi_server)
        self.declare_parameter('maps_path', '')
        self.declare_parameter('map_name', 'turtlebot3_world')
        self.declare_parameter('config_file', 'mapoi_config.yaml')
        self.declare_parameter('web_port', 8765)
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        # ロボットの実寸 (m)。frontend が connector 到達閾値 / robot marker サイズ
        # に使う (#116, #117)。Nav2 の `robot_radius` と意味は同じだが、本 node
        # は Nav2 非依存運用 (Editor mode 等) も想定するため、launch param で
        # 個別に渡す。Nav2 と必ずセットで使う場合は controller_server 側を
        # 直接 pull する案 (#117 案 B) を別途検討。
        self.declare_parameter('robot_radius', 0.15)

        self.maps_path_ = self.get_parameter('maps_path').get_parameter_value().string_value
        self.map_name_ = self.get_parameter('map_name').get_parameter_value().string_value
        self.config_file_ = self.get_parameter('config_file').get_parameter_value().string_value
        self.web_port_ = self.get_parameter('web_port').get_parameter_value().integer_value
        self.web_host_ = self.get_parameter('web_host').get_parameter_value().string_value
        self.map_frame_ = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame_ = self.get_parameter('base_frame').get_parameter_value().string_value
        # 値検証: 設定ミス (typo / yaml で `0` / 負値) を silent に飲み込まないよう
        # finite かつ正値を要求する。invalid なら warn して default 0.15 に fallback
        # (frontend 側 `setRobotRadius` ガードと defense in depth、
        #  Codex PR #126 round 1 low)。
        raw_radius = float(
            self.get_parameter('robot_radius').get_parameter_value().double_value)
        if not math.isfinite(raw_radius) or raw_radius <= 0.0:
            self.get_logger().warn(
                f'robot_radius={raw_radius!r} は不正値です。default 0.15m を使います。'
                ' (連動する Nav2 robot_radius と一致するように launch param を見直してください)')
            self.robot_radius_ = 0.15
        else:
            self.robot_radius_ = raw_radius

        # ROS2 service clients
        self.reload_client_ = self.create_client(Trigger, 'reload_map_info')
        self.get_maps_client_ = self.create_client(GetMapsInfo, 'get_maps_info')
        self.tag_defs_client_ = self.create_client(GetTagDefinitions, 'get_tag_definitions')
        self.select_map_client_ = self.create_client(SelectMap, 'select_map')

        # Navigation publishers
        self.goal_poi_pub_ = self.create_publisher(String, 'mapoi/nav/goal_pose_poi', 10)
        self.route_pub_ = self.create_publisher(String, 'mapoi/nav/route', 10)
        self.cancel_pub_ = self.create_publisher(String, 'mapoi/nav/cancel', 10)
        self.pause_pub_  = self.create_publisher(String, 'mapoi/nav/pause',  10)
        self.resume_pub_ = self.create_publisher(String, 'mapoi/nav/resume', 10)
        # mapoi/initialpose_poi は #149 round 8 で {map_name, poi_name} 型に変更。
        # transient_local QoS で後起動 subscriber (bridge / nav_server) も latched 値を拾える。
        initialpose_poi_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.initialpose_poi_pub_ = self.create_publisher(
            InitialPoseRequest, 'mapoi/initialpose_poi', initialpose_poi_qos)

        # TF for robot pose
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)
        self.robot_pose_ = None
        self.create_timer(0.2, self.update_robot_pose)

        # Navigation status
        # QoS は mapoi_nav_server と同じ transient_local。後起動 webui でも latched 値を受信できる。
        self.nav_status_ = 'idle'
        self.nav_status_target_ = ''
        nav_status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.nav_status_sub_ = self.create_subscription(
            String, 'mapoi/nav/status', self.nav_status_callback, nav_status_qos)

        # Subscribe to mapoi/config_path for external map switches.
        # transient_local QoS で publisher (mapoi_server) の latched 値を受信する (#135)。
        config_path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.config_path_sub_ = self.create_subscription(
            String, 'mapoi/config_path', self.config_path_callback, config_path_qos)

        # SSE clients (#135 (B)): rviz / 外部 save 由来の config 変更を frontend に push するための
        # client queue 集合。/api/events で接続された client ごとに queue を 1 つ持ち、
        # config_path_callback で全 client queue に broadcast する。
        self._sse_clients = set()
        self._sse_lock = threading.Lock()

        # If maps_path not set, try to get it from get_maps_info service or use default
        if not self.maps_path_:
            self.get_logger().warn('maps_path parameter not set. Set it to use the web editor.')

        self.get_logger().info(
            f'mapoi_webui_node started: maps_path={self.maps_path_}, '
            f'map_name={self.map_name_}, port={self.web_port_}')

        # Start Flask in daemon thread
        self.flask_app_ = self.create_flask_app()
        flask_thread = threading.Thread(
            target=self.run_flask, daemon=True)
        flask_thread.start()

    def nav_status_callback(self, msg):
        """Update navigation status from mapoi/nav/status topic."""
        # Expected format: "status" or "status:target"
        parts = msg.data.split(':', 1)
        self.nav_status_ = parts[0]
        # Only overwrite target if explicitly provided; keep previous target
        # so that succeeded/aborted/canceled still show the target name.
        if len(parts) > 1:
            self.nav_status_target_ = parts[1]

    def update_robot_pose(self):
        """Lookup TF map->base_link and cache robot pose."""
        try:
            t = self.tf_buffer_.lookup_transform(
                self.map_frame_, self.base_frame_, rclpy.time.Time())
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.robot_pose_ = {
                'x': t.transform.translation.x,
                'y': t.transform.translation.y,
                'yaw': yaw,
            }
        except Exception:
            self.robot_pose_ = None

    def config_path_callback(self, msg):
        """Detect external map switches via mapoi/config_path topic + frontend に push (#135 (B))."""
        # Extract map_name from path: .../maps/<map_name>/mapoi_config.yaml
        path = msg.data
        try:
            parts = path.replace('\\', '/').split('/')
            # Find config_file in path, map_name is the directory before it
            map_name_parsed = None
            for i, part in enumerate(parts):
                if part == self.config_file_ and i > 0:
                    map_name_parsed = parts[i - 1]
                    if map_name_parsed != self.map_name_:
                        self.map_name_ = map_name_parsed
                        self.get_logger().info(f'Map switched externally to: {map_name_parsed}')
                    break
            if map_name_parsed is not None:
                # 内容変更 (POI / route の save 後 reload による再 publish) でも frontend に通知して
                # loadPois / loadRoutes / loadTagDefinitions を再実行させる (#135 (B))。
                # map 名が正しく解釈できた時のみ broadcast: 期待形式に部分一致するが map 特定でき
                # ないケースで無駄な reload を避ける (#173 Round 1 / 2 medium)。
                self._broadcast_sse_event('config_changed')
        except Exception as e:
            self.get_logger().warn(f'Failed to parse config path: {e}')

    def _broadcast_sse_event(self, event_type, payload=None):
        """SSE で接続中の全 frontend client にイベントを broadcast する (#135 (B))。

        各 client の queue.Queue に put_nowait で event を流す。client 切断は
        /api/events generator の finally で自動 discard されるので queue 漏れなし。
        queue は unbounded (現状) なので Full 例外は通常発火しない。
        """
        data = {'type': event_type}
        if payload is not None:
            data['payload'] = payload
        with self._sse_lock:
            for q in self._sse_clients:
                try:
                    q.put_nowait(data)
                except queue.Full:
                    pass  # bounded queue にした場合の安全弁、現在は unbounded で発火しない

    def get_config_path(self, map_name=None):
        """Get the full path to mapoi_config.yaml for a given map."""
        name = map_name or self.map_name_
        return os.path.join(self.maps_path_, name, self.config_file_)

    def get_map_dir(self, map_name=None):
        """Get the map directory path."""
        name = map_name or self.map_name_
        return os.path.join(self.maps_path_, name)

    def call_reload_map_info(self):
        """Call reload_map_info service and await response.success.

        Returns True only when server reports success; False on
        unavailable / timeout / server-side failure.
        """
        response = self._call_service_sync(
            self.reload_client_, Trigger.Request(), 'reload_map_info', timeout_sec=3.0)
        if response is None:
            return False  # service unavailable / timeout (logged in helper)
        if not response.success:
            self.get_logger().warn(f'reload_map_info returned failure: {response.message}')
            return False
        self.get_logger().info('reload_map_info succeeded')
        return True

    def publish_with_subscriber_check(self, pub, msg, topic_name):
        """Publish with best-effort subscriber-count check.

        ROS 2 `publisher.publish()` は subscriber がいなくても成功扱いになる。
        UI ボタン経由の publish では subscriber 不在 (相手 node 未起動等) を
        silent failure にせず warning として user に返したい場面がある。

        **best-effort**: `get_subscription_count()` と `publish()` の間で
        subscriber 状態が変わる race を排除できない (check 通過後に消える /
        check 失敗後に直前に現れる)。「明らかな未起動」検出が目的で、厳密な
        到達確認が必要なら service / action / ack 設計に切り替える。

        Args:
            pub: rclpy publisher
            msg: message to publish
            topic_name: topic name for warning message

        Returns:
            (published, warning) where warning is None or human-readable string.
            published is always True since publish itself doesn't fail locally.
        """
        sub_count = pub.get_subscription_count()
        pub.publish(msg)
        if sub_count == 0:
            warning = (f"{topic_name} に subscriber が見つかりません "
                       "(mapoi_nav_server などの listener が起動していない可能性)")
            self.get_logger().warn(warning)
            return True, warning
        return True, None

    def _call_service_sync(self, client, request, service_name, timeout_sec=3.0,
                           wait_for_service_sec=2.0):
        """Call ROS 2 service from Flask thread and wait for the response.

        SingleThreadedExecutor で main thread の rclpy.spin が future を resolve するため、
        Flask thread はここで future.done() を polling する。spin_until_future_complete を
        Flask thread から呼ぶと main thread の spin と executor を競合させるので避ける。

        実効最長待ち時間 ≒ wait_for_service_sec + timeout_sec (default 5s)。
        timeout 時は future.cancel() で pending request を解放する (helper の
        繰り返し呼び出しで cancel 忘れ future が蓄積しないように)。

        Args:
            client: rclpy service client
            request: service request message
            service_name: service name for logging
            timeout_sec: 全体の wait timeout (default 3.0s)
            wait_for_service_sec: service discovery の wait timeout (default 2.0s)

        Returns:
            response object on success, None on timeout / unavailability / exception.
        """
        if not client.wait_for_service(timeout_sec=wait_for_service_sec):
            self.get_logger().warn(f'{service_name} service not available')
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if time.monotonic() > deadline:
                future.cancel()
                self.get_logger().warn(f'{service_name} call timed out after {timeout_sec}s')
                return None
            time.sleep(0.05)
        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f'{service_name} call exception: {e}')
            return None

    def get_maps_list(self):
        """Get list of available maps from maps_path directory."""
        maps = []
        if self.maps_path_ and os.path.isdir(self.maps_path_):
            for entry in sorted(os.listdir(self.maps_path_)):
                full = os.path.join(self.maps_path_, entry)
                if os.path.isdir(full):
                    maps.append(entry)
        return maps

    def run_flask(self):
        """Run Flask server (called in daemon thread)."""
        self.get_logger().info(f'Flask server starting on {self.web_host_}:{self.web_port_}')
        self.flask_app_.run(
            host=self.web_host_,
            port=self.web_port_,
            debug=False,
            use_reloader=False,
            threaded=True)  # SSE long-lived connection と他 request の並行のため明示 (#135 (B))

    def create_flask_app(self):
        """Create and configure the Flask application."""
        app = Flask(__name__)
        logging.getLogger('werkzeug').setLevel(logging.WARNING)
        node = self  # capture for closures

        # Static files directory
        web_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'web')
        # For installed package, try share directory
        try:
            from ament_index_python.packages import get_package_share_directory
            share_web_dir = os.path.join(
                get_package_share_directory('mapoi_webui'), 'web')
            if os.path.isdir(share_web_dir):
                web_dir = share_web_dir
        except Exception:
            pass

        @app.route('/')
        def index():
            return send_from_directory(web_dir, 'index.html')

        @app.route('/css/<path:filename>')
        def css_static(filename):
            return send_from_directory(os.path.join(web_dir, 'css'), filename)

        @app.route('/js/<path:filename>')
        def js_static(filename):
            return send_from_directory(os.path.join(web_dir, 'js'), filename)

        @app.route('/api/maps')
        def api_maps():
            maps = node.get_maps_list()
            return jsonify({
                'maps': maps,
                'current_map': node.map_name_,
            })

        @app.route('/api/maps/<name>/image')
        def api_map_image(name):
            map_dir = node.get_map_dir(name)
            png_bytes, content_type = get_map_png(map_dir)
            if png_bytes is None:
                return jsonify({'error': 'Map image not found'}), 404
            return Response(png_bytes, mimetype=content_type)

        @app.route('/api/maps/<name>/metadata')
        def api_map_metadata(name):
            map_dir = node.get_map_dir(name)
            meta = get_map_metadata(map_dir)
            if meta is None:
                return jsonify({'error': 'Map metadata not found'}), 404
            return jsonify({
                'resolution': meta['resolution'],
                'origin': meta['origin'],
                'width': meta['width'],
                'height': meta['height'],
            })

        @app.route('/api/maps/select', methods=['POST'])
        def api_select_map():
            data = request.get_json()
            if not data or 'map_name' not in data:
                return jsonify({'error': 'map_name required'}), 400
            map_name = str(data['map_name']).strip()
            if not map_name:
                return jsonify({'error': 'map_name required'}), 400
            req = SelectMap.Request()
            req.map_name = map_name
            response = node._call_service_sync(
                node.select_map_client_, req, 'select_map', timeout_sec=3.0)
            if response is None:
                return jsonify({'error': 'select_map service unavailable or timed out'}), 503
            if not response.success:
                return jsonify({'error': response.error_message or 'select_map failed'}), 400
            node.map_name_ = map_name
            return jsonify({
                'success': True,
                'map_name': map_name,
                'config_path': response.config_path,
                'initial_poi_name': response.initial_poi_name,
            })

        @app.route('/api/pois')
        def api_get_pois():
            config_path = node.get_config_path()
            if not os.path.exists(config_path):
                return jsonify({'error': 'Config not found'}), 404
            pois = get_pois(config_path)
            return jsonify({'pois': pois, 'map_name': node.map_name_})

        @app.route('/api/pois', methods=['POST'])
        def api_save_pois():
            data = request.get_json()
            if data is None or 'pois' not in data:
                return jsonify({'error': 'Invalid request body'}), 400
            # name の uniqueness / 空 を backend でも validate (#109)。
            # frontend 経由なら poi-editor が reject 済みだが、yaml 直編集や
            # 別 client からの POST に対する保険として 400 で reject する。
            err = _validate_unique_names(data['pois'], 'POI')
            if err:
                return jsonify({'error': err}), 400
            err = _validate_pois_tag_exclusivity(data['pois'])
            if err:
                return jsonify({'error': err}), 400
            err = _validate_pois_tolerance(data['pois'])
            if err:
                return jsonify({'error': err}), 400
            config_path = node.get_config_path()
            if not os.path.exists(config_path):
                return jsonify({'error': 'Config not found'}), 404
            try:
                save_pois(config_path, data['pois'])
                reloaded = node.call_reload_map_info()
                if not reloaded:
                    return jsonify({
                        'success': True,
                        'warning': 'YAML は保存しましたが、mapoi_server の reload_map_info '
                                   'service が応答しなかったか失敗しました。詳細はログを確認してください'
                    })
                return jsonify({'success': True})
            except Exception as e:
                node.get_logger().error(f'Failed to save POIs: {e}')
                return jsonify({'error': str(e)}), 500

        @app.route('/api/tag_definitions')
        def api_tag_definitions():
            response = node._call_service_sync(
                node.tag_defs_client_, GetTagDefinitions.Request(), 'get_tag_definitions')
            if response is None:
                return jsonify({'error': 'get_tag_definitions service unavailable or timed out'}), 503
            tags = [
                {'name': d.name, 'description': d.description, 'is_system': d.is_system}
                for d in response.definitions
            ]
            return jsonify({'tags': tags})

        @app.route('/api/custom_tags', methods=['POST'])
        def api_save_custom_tags():
            data = request.get_json()
            if data is None or 'custom_tags' not in data:
                return jsonify({'error': 'Invalid request body'}), 400
            config_path = node.get_config_path()
            if not os.path.exists(config_path):
                return jsonify({'error': 'Config not found'}), 404
            try:
                save_custom_tags(config_path, data['custom_tags'])
                reloaded = node.call_reload_map_info()
                if not reloaded:
                    return jsonify({
                        'success': True,
                        'warning': 'YAML は保存しましたが、mapoi_server の reload_map_info '
                                   'service が応答しなかったか失敗しました。詳細はログを確認してください'
                    })
                return jsonify({'success': True})
            except Exception as e:
                node.get_logger().error(f'Failed to save custom tags: {e}')
                return jsonify({'error': str(e)}), 500

        @app.route('/api/routes')
        def api_get_routes():
            config_path = node.get_config_path()
            if not os.path.exists(config_path):
                return jsonify({'error': 'Config not found'}), 404
            routes = get_routes(config_path)
            return jsonify({'routes': routes, 'map_name': node.map_name_})

        @app.route('/api/routes', methods=['POST'])
        def api_save_routes():
            data = request.get_json()
            if data is None or 'routes' not in data:
                return jsonify({'error': 'Invalid request body'}), 400
            err = _validate_unique_names(data['routes'], 'Route')
            if err:
                return jsonify({'error': err}), 400
            config_path = node.get_config_path()
            if not os.path.exists(config_path):
                return jsonify({'error': 'Config not found'}), 404
            try:
                save_routes(config_path, data['routes'])
                reloaded = node.call_reload_map_info()
                if not reloaded:
                    return jsonify({
                        'success': True,
                        'warning': 'YAML は保存しましたが、mapoi_server の reload_map_info '
                                   'service が応答しなかったか失敗しました。詳細はログを確認してください'
                    })
                return jsonify({'success': True})
            except Exception as e:
                node.get_logger().error(f'Failed to save routes: {e}')
                return jsonify({'error': str(e)}), 500

        @app.route('/api/nav/goal', methods=['POST'])
        def api_nav_goal():
            data = request.get_json()
            if not data or 'poi_name' not in data:
                return jsonify({'error': 'poi_name required'}), 400
            msg = String()
            msg.data = data['poi_name']
            _, warning = node.publish_with_subscriber_check(
                node.goal_poi_pub_, msg, 'mapoi/nav/goal_pose_poi')
            node.nav_status_ = 'navigating'
            node.nav_status_target_ = data['poi_name']
            node.get_logger().info(f'Nav goal: {data["poi_name"]}')
            return jsonify({'success': True, 'warning': warning} if warning else {'success': True})

        @app.route('/api/nav/route', methods=['POST'])
        def api_nav_route():
            data = request.get_json()
            if not data or 'route_name' not in data:
                return jsonify({'error': 'route_name required'}), 400
            msg = String()
            msg.data = data['route_name']
            _, warning = node.publish_with_subscriber_check(
                node.route_pub_, msg, 'mapoi/nav/route')
            node.nav_status_ = 'navigating'
            node.nav_status_target_ = data['route_name']
            node.get_logger().info(f'Nav route: {data["route_name"]}')
            return jsonify({'success': True, 'warning': warning} if warning else {'success': True})

        @app.route('/api/nav/cancel', methods=['POST'])
        def api_nav_cancel():
            msg = String()
            msg.data = 'cancel'
            _, warning = node.publish_with_subscriber_check(
                node.cancel_pub_, msg, 'mapoi/nav/cancel')
            node.nav_status_ = 'canceled'
            node.get_logger().info('Nav canceled')
            return jsonify({'success': True, 'warning': warning} if warning else {'success': True})

        @app.route('/api/nav/pause', methods=['POST'])
        def api_nav_pause():
            msg = String()
            msg.data = 'webui'
            _, warning = node.publish_with_subscriber_check(
                node.pause_pub_, msg, 'mapoi/nav/pause')
            node.get_logger().info('Nav pause requested')
            return jsonify({'success': True, 'warning': warning} if warning else {'success': True})

        @app.route('/api/nav/resume', methods=['POST'])
        def api_nav_resume():
            msg = String()
            msg.data = 'webui'
            _, warning = node.publish_with_subscriber_check(
                node.resume_pub_, msg, 'mapoi/nav/resume')
            node.get_logger().info('Nav resume requested')
            return jsonify({'success': True, 'warning': warning} if warning else {'success': True})

        @app.route('/api/nav/status')
        def api_nav_status():
            return jsonify({
                'status': node.nav_status_,
                'target': node.nav_status_target_,
                'robot_pose': node.robot_pose_,
                # robot_radius は launch 起動時固定だが、frontend が boot 時に
                # 別 endpoint を叩かなくて済むよう poll 結果に相乗りさせる。
                # 値は float (m)、payload size 影響は無視できる (#117)。
                'robot_radius': node.robot_radius_,
            })

        @app.route('/api/nav/initialpose', methods=['POST'])
        def api_nav_initialpose():
            data = request.get_json()
            if not data or 'poi_name' not in data:
                return jsonify({'error': 'poi_name required'}), 400
            msg = InitialPoseRequest()
            msg.map_name = node.map_name_
            msg.poi_name = data['poi_name']
            _, warning = node.publish_with_subscriber_check(
                node.initialpose_poi_pub_, msg, 'mapoi/initialpose_poi')
            node.get_logger().info(f'Initial pose: {data["poi_name"]}')
            return jsonify({'success': True, 'warning': warning} if warning else {'success': True})

        @app.route('/api/events')
        def api_events():
            """SSE endpoint: rviz / 外部 save 由来の config 変更を frontend に push (#135 (B))。

            EventSource (frontend) で接続して、{"type": "config_changed"} 等のイベントを受信する。
            client 切断時は heartbeat (30s 周期) の write error で generator が例外を受け、
            finally で client queue を discard する (#173 Round 1 high)。
            """
            def gen():
                # bounded queue: 遅いクライアントで event が積み上がる場合の DoS 対策 (#173 Round 1 high)。
                # 通常は frontend が即座に drain するので 10 件以内に収まる、overflow は drop。
                q = queue.Queue(maxsize=10)
                with node._sse_lock:
                    node._sse_clients.add(q)
                try:
                    while True:
                        try:
                            # 30s timeout + heartbeat: client 切断検知 + プロキシバッファ flush。
                            # timeout 時は SSE comment (`:\n\n`) を yield することで socket write を発火し、
                            # client 切断時は ConnectionResetError → finally で discard される。
                            data = q.get(timeout=30)
                            yield f'data: {json.dumps(data)}\n\n'
                        except queue.Empty:
                            yield ': heartbeat\n\n'
                finally:
                    with node._sse_lock:
                        node._sse_clients.discard(q)
            # SSE response headers: プロキシ (nginx 等) のバッファリング無効化 + cache 無効化
            # (#173 Round 1 medium)。
            return Response(gen(), mimetype='text/event-stream',
                            headers={
                                'Cache-Control': 'no-cache',
                                'X-Accel-Buffering': 'no',
                            })

        return app


def main(args=None):
    rclpy.init(args=args)
    node = MapoiWebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # launch 経由 SIGINT では rclpy context が既に shutdown 済みの場合があり、
        # rclpy.shutdown() を直接呼ぶと RCLError になる。try_shutdown() は
        # 状態確認と shutdown を 1 つの API に閉じ込めてある。
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
