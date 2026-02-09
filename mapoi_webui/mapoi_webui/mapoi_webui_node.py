#!/usr/bin/env python3
"""mapoi_webui_node: ROS2 node with embedded Flask server for mapoi Web UI."""

import math
import os
import logging
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mapoi_interfaces.srv import GetMapsInfo
import tf2_ros

from flask import Flask, jsonify, request, send_from_directory, Response

from mapoi_webui.yaml_handler import load_config, save_pois, get_pois, get_routes, get_tag_definitions
from mapoi_webui.map_image import get_map_metadata, get_map_png


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

        self.maps_path_ = self.get_parameter('maps_path').get_parameter_value().string_value
        self.map_name_ = self.get_parameter('map_name').get_parameter_value().string_value
        self.config_file_ = self.get_parameter('config_file').get_parameter_value().string_value
        self.web_port_ = self.get_parameter('web_port').get_parameter_value().integer_value
        self.web_host_ = self.get_parameter('web_host').get_parameter_value().string_value
        self.map_frame_ = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame_ = self.get_parameter('base_frame').get_parameter_value().string_value

        # ROS2 service clients
        self.reload_client_ = self.create_client(Trigger, 'reload_map_info')
        self.get_maps_client_ = self.create_client(GetMapsInfo, 'get_maps_info')

        # Navigation publishers
        self.goal_poi_pub_ = self.create_publisher(String, 'mapoi_goal_pose_poi', 10)
        self.route_pub_ = self.create_publisher(String, 'mapoi_route', 10)
        self.cancel_pub_ = self.create_publisher(String, 'mapoi_cancel', 10)
        self.initialpose_poi_pub_ = self.create_publisher(String, 'mapoi_initialpose_poi', 10)

        # TF for robot pose
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)
        self.robot_pose_ = None
        self.create_timer(0.2, self.update_robot_pose)

        # Navigation status
        self.nav_status_ = 'idle'
        self.nav_status_target_ = ''
        self.nav_status_sub_ = self.create_subscription(
            String, 'mapoi_nav_status', self.nav_status_callback, 10)

        # Subscribe to mapoi_config_path for external map switches
        self.config_path_sub_ = self.create_subscription(
            String, 'mapoi_config_path', self.config_path_callback, 10)

        # Resolve system tag_definitions.yaml from mapoi_server package
        self.system_tags_path_ = ''
        try:
            from ament_index_python.packages import get_package_share_directory
            self.system_tags_path_ = os.path.join(
                get_package_share_directory('mapoi_server'), 'maps', 'tag_definitions.yaml')
        except Exception:
            # Fallback: try sibling directory
            pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            fallback = os.path.join(os.path.dirname(pkg_dir), 'mapoi_server', 'maps', 'tag_definitions.yaml')
            if os.path.exists(fallback):
                self.system_tags_path_ = fallback

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
        """Update navigation status from mapoi_nav_status topic."""
        # Expected format: "status" or "status:target"
        parts = msg.data.split(':', 1)
        self.nav_status_ = parts[0]
        self.nav_status_target_ = parts[1] if len(parts) > 1 else ''

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
        """Detect external map switches via mapoi_config_path topic."""
        # Extract map_name from path: .../maps/<map_name>/mapoi_config.yaml
        path = msg.data
        try:
            parts = path.replace('\\', '/').split('/')
            # Find config_file in path, map_name is the directory before it
            for i, part in enumerate(parts):
                if part == self.config_file_ and i > 0:
                    new_map = parts[i - 1]
                    if new_map != self.map_name_:
                        self.map_name_ = new_map
                        self.get_logger().info(f'Map switched externally to: {new_map}')
                    break
        except Exception as e:
            self.get_logger().warn(f'Failed to parse config path: {e}')

    def get_config_path(self, map_name=None):
        """Get the full path to mapoi_config.yaml for a given map."""
        name = map_name or self.map_name_
        return os.path.join(self.maps_path_, name, self.config_file_)

    def get_map_dir(self, map_name=None):
        """Get the map directory path."""
        name = map_name or self.map_name_
        return os.path.join(self.maps_path_, name)

    def call_reload_map_info(self):
        """Call reload_map_info service on mapoi_server."""
        if not self.reload_client_.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('reload_map_info service not available')
            return False
        req = Trigger.Request()
        future = self.reload_client_.call_async(req)
        # We don't block here since we're in Flask thread; fire and forget
        self.get_logger().info('Called reload_map_info')
        return True

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
            use_reloader=False)

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
            config_path = node.get_config_path()
            if not os.path.exists(config_path):
                return jsonify({'error': 'Config not found'}), 404
            try:
                save_pois(config_path, data['pois'])
                node.call_reload_map_info()
                return jsonify({'success': True})
            except Exception as e:
                node.get_logger().error(f'Failed to save POIs: {e}')
                return jsonify({'error': str(e)}), 500

        @app.route('/api/tag_definitions')
        def api_tag_definitions():
            config_path = node.get_config_path()
            tags = get_tag_definitions(node.system_tags_path_, config_path)
            return jsonify({'tags': tags})

        @app.route('/api/routes')
        def api_get_routes():
            config_path = node.get_config_path()
            if not os.path.exists(config_path):
                return jsonify({'error': 'Config not found'}), 404
            routes = get_routes(config_path)
            return jsonify({'routes': routes, 'map_name': node.map_name_})

        @app.route('/api/nav/goal', methods=['POST'])
        def api_nav_goal():
            data = request.get_json()
            if not data or 'poi_name' not in data:
                return jsonify({'error': 'poi_name required'}), 400
            msg = String()
            msg.data = data['poi_name']
            node.goal_poi_pub_.publish(msg)
            node.nav_status_ = 'navigating'
            node.nav_status_target_ = data['poi_name']
            node.get_logger().info(f'Nav goal: {data["poi_name"]}')
            return jsonify({'success': True})

        @app.route('/api/nav/route', methods=['POST'])
        def api_nav_route():
            data = request.get_json()
            if not data or 'route_name' not in data:
                return jsonify({'error': 'route_name required'}), 400
            msg = String()
            msg.data = data['route_name']
            node.route_pub_.publish(msg)
            node.nav_status_ = 'navigating'
            node.nav_status_target_ = data['route_name']
            node.get_logger().info(f'Nav route: {data["route_name"]}')
            return jsonify({'success': True})

        @app.route('/api/nav/cancel', methods=['POST'])
        def api_nav_cancel():
            msg = String()
            msg.data = 'cancel'
            node.cancel_pub_.publish(msg)
            node.nav_status_ = 'canceled'
            node.get_logger().info('Nav canceled')
            return jsonify({'success': True})

        @app.route('/api/nav/status')
        def api_nav_status():
            return jsonify({
                'status': node.nav_status_,
                'target': node.nav_status_target_,
                'robot_pose': node.robot_pose_,
            })

        @app.route('/api/nav/initialpose', methods=['POST'])
        def api_nav_initialpose():
            data = request.get_json()
            if not data or 'poi_name' not in data:
                return jsonify({'error': 'poi_name required'}), 400
            msg = String()
            msg.data = data['poi_name']
            node.initialpose_poi_pub_.publish(msg)
            node.get_logger().info(f'Initial pose: {data["poi_name"]}')
            return jsonify({'success': True})

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
