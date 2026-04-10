import os
import unittest
import time

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers

import rclpy
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from mapoi_interfaces.msg import PoiEvent
from ament_index_python.packages import get_package_share_directory


@launch_testing.markers.keep_alive
def generate_test_description():
    pkg_share = get_package_share_directory('mapoi_server')
    test_config_dir = os.path.join(pkg_share, 'test')

    mapoi_server_node = launch_ros.actions.Node(
        package='mapoi_server',
        executable='mapoi_server',
        name='mapoi_server',
        parameters=[{
            'maps_path': test_config_dir,
            'map_name': '.',
            'config_file': 'test_mapoi_config.yaml',
        }],
    )

    mapoi_nav_server_node = launch_ros.actions.Node(
        package='mapoi_server',
        executable='mapoi_nav_server',
        name='mapoi_nav_server',
        parameters=[{
            'radius_check_hz': 10.0,
            'map_frame': 'map',
            'base_frame': 'base_link',
        }],
    )

    return launch.LaunchDescription([
        mapoi_server_node,
        mapoi_nav_server_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'mapoi_server': mapoi_server_node, 'mapoi_nav_server': mapoi_nav_server_node}


class TestPoiEventIntegration(unittest.TestCase):
    """mapoi_poi_events トピックの発行を検証する統合テスト"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_poi_event_node')
        cls.received_events = []
        cls.sub = cls.node.create_subscription(
            PoiEvent, 'mapoi_poi_events',
            lambda msg: cls.received_events.append(msg), 10)
        cls.tf_broadcaster = StaticTransformBroadcaster(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.received_events.clear()

    def _publish_tf(self, x, y):
        """map -> base_link のTFを発行"""
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _spin_and_wait(self, timeout_sec=3.0):
        """指定秒数spinしてイベントを待つ"""
        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def test_enter_event_goal_only_poi(self):
        """goalタグのみのPOI (1.0, 0.0) 付近 → ENTER イベント発行"""
        self._publish_tf(1.0, 0.0)
        self._spin_and_wait(3.0)
        enter_events = [e for e in self.received_events
                        if e.event_type == PoiEvent.EVENT_ENTER
                        and e.poi.name == 'poi_goal_only']
        self.assertGreater(len(enter_events), 0,
                           "goal-only POI の ENTER イベントが発行されるべき")

    def test_enter_event_pause_poi(self):
        """pauseタグ付きPOI (0.0, 2.0) 付近 → ENTER イベント発行"""
        self._publish_tf(0.0, 2.0)
        self._spin_and_wait(3.0)
        enter_events = [e for e in self.received_events
                        if e.event_type == PoiEvent.EVENT_ENTER
                        and e.poi.name == 'poi_with_pause']
        self.assertGreater(len(enter_events), 0,
                           "pause POI の ENTER イベントが発行されるべき")

    def test_enter_event_custom_poi(self):
        """custom_tag付きPOI (3.0, 0.0) 付近 → ENTER イベント発行"""
        self._publish_tf(3.0, 0.0)
        self._spin_and_wait(3.0)
        enter_events = [e for e in self.received_events
                        if e.event_type == PoiEvent.EVENT_ENTER
                        and e.poi.name == 'poi_with_custom']
        self.assertGreater(len(enter_events), 0,
                           "custom_tag POI の ENTER イベントが発行されるべき")

    def test_exit_event(self):
        """POI内 → radius外 に移動 → EXIT イベント発行"""
        self._publish_tf(1.0, 0.0)
        self._spin_and_wait(2.0)
        self.received_events.clear()
        # radius=0.5, hysteresis=1.15 → 0.575m以上離れる
        self._publish_tf(5.0, 5.0)
        self._spin_and_wait(2.0)
        exit_events = [e for e in self.received_events
                       if e.event_type == PoiEvent.EVENT_EXIT]
        self.assertGreater(len(exit_events), 0,
                           "EXIT イベントが発行されるべき")
