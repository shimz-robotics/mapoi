import os
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers

import rclpy
from tf2_ros import TransformBroadcaster
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

    # event 待ちの timeout。CI の CPU 負荷時にも `radius_check_hz=10` × 数周期分の
    # マージンが取れる長さ。低めにすると nav_server の初期 fetch race で flaky 化する (#153)。
    EVENT_WAIT_TIMEOUT = 5.0
    # dynamic TF publish 周期。radius_check_hz=10 (=100ms) より十分速くして取りこぼしを防ぐ。
    TF_PUBLISH_INTERVAL = 0.05

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_poi_event_node')
        cls.received_events = []
        cls.sub = cls.node.create_subscription(
            PoiEvent, 'mapoi_poi_events',
            lambda msg: cls.received_events.append(msg), 10)
        # static TF だと同 child_frame_id の再 publish で buffer 内 cache の更新が
        # 伝播しないことがあり flaky 化する (#153)。dynamic TF を別スレッドで周期 publish
        # することで、nav_server の lookupTransform(TimePointZero) が常に最新を取れる
        # ようにする。
        cls.tf_broadcaster = TransformBroadcaster(cls.node)
        cls._robot_xy = (100.0, 100.0)  # 初期は全 POI から十分遠い座標
        cls._tf_lock = threading.Lock()
        cls._tf_stop = threading.Event()
        cls._tf_thread = threading.Thread(target=cls._tf_publish_loop, daemon=True)
        cls._tf_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls._tf_stop.set()
        cls._tf_thread.join(timeout=2.0)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _tf_publish_loop(cls):
        while not cls._tf_stop.is_set():
            with cls._tf_lock:
                x, y = cls._robot_xy
            t = TransformStamped()
            t.header.stamp = cls.node.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            cls.tf_broadcaster.sendTransform(t)
            cls._tf_stop.wait(cls.TF_PUBLISH_INTERVAL)

    def setUp(self):
        self.received_events.clear()

    def _set_robot_pose(self, x, y):
        """publish スレッドが次周期から拾う robot 位置を更新する"""
        with self._tf_lock:
            type(self)._robot_xy = (x, y)

    def _wait_for_event(self, predicate, timeout_sec=None):
        """述語を満たす PoiEvent を受信するまで spin する。

        受信時点で即 True を返し、timeout したら False。固定時間 spin だと
        nav_server の TF buffer 反映や初期 POI fetch のタイミング次第で取りこぼし
        flaky 化するため、event 駆動で抜ける (#153)。
        """
        if timeout_sec is None:
            timeout_sec = self.EVENT_WAIT_TIMEOUT
        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if any(predicate(e) for e in self.received_events):
                return True
        return False

    def test_enter_event_goal_only_poi(self):
        """goalタグのみのPOI (1.0, 0.0) 付近 → ENTER イベント発行"""
        self._set_robot_pose(1.0, 0.0)
        found = self._wait_for_event(
            lambda e: (e.event_type == PoiEvent.EVENT_ENTER
                       and e.poi.name == 'poi_goal_only'))
        self.assertTrue(found,
                        "goal-only POI の ENTER イベントが発行されるべき")

    def test_enter_event_pause_poi(self):
        """pauseタグ付きPOI (0.0, 2.0) 付近 → ENTER イベント発行"""
        self._set_robot_pose(0.0, 2.0)
        found = self._wait_for_event(
            lambda e: (e.event_type == PoiEvent.EVENT_ENTER
                       and e.poi.name == 'poi_with_pause'))
        self.assertTrue(found,
                        "pause POI の ENTER イベントが発行されるべき")

    def test_enter_event_custom_poi(self):
        """custom_tag付きPOI (3.0, 0.0) 付近 → ENTER イベント発行"""
        self._set_robot_pose(3.0, 0.0)
        found = self._wait_for_event(
            lambda e: (e.event_type == PoiEvent.EVENT_ENTER
                       and e.poi.name == 'poi_with_custom'))
        self.assertTrue(found,
                        "custom_tag POI の ENTER イベントが発行されるべき")

    def test_exit_event(self):
        """POI内 → radius外 に移動 → EXIT イベント発行"""
        # 前段: (1.0, 0.0) の poi_goal_only を ENTER 状態にする。
        # 単に固定 spin するだけだと TF 反映や radius_check の周期次第で was_inside が
        # false のまま後段に進み、後段で EXIT 条件 (was_inside && dist > radius*hyst) が
        # 永遠に成立せず flaky 化していた (#153)。ここで ENTER を assert で確認する。
        self._set_robot_pose(1.0, 0.0)
        enter_found = self._wait_for_event(
            lambda e: (e.event_type == PoiEvent.EVENT_ENTER
                       and e.poi.name == 'poi_goal_only'))
        self.assertTrue(enter_found,
                        "前提: poi_goal_only が ENTER 状態になっていること")
        self.received_events.clear()
        # radius=0.5, hysteresis=1.15 → 0.575m以上離れる
        self._set_robot_pose(5.0, 5.0)
        exit_found = self._wait_for_event(
            lambda e: (e.event_type == PoiEvent.EVENT_EXIT
                       and e.poi.name == 'poi_goal_only'))
        self.assertTrue(exit_found,
                        "poi_goal_only の EXIT イベントが発行されるべき")
