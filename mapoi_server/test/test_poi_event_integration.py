import os
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
    """mapoi_poi_events トピックの発行を検証する統合テスト。

    flaky 対策の経緯 (#153) は :meth:`_wait_for_event` の docstring を参照。
    要点: dynamic TF を rclpy timer で周期 publish + event 駆動 wait で
    取りこぼしを防ぐ。
    """

    # event 待ちの timeout。CI の CPU 負荷時にも `radius_check_hz=10` × 数周期分の
    # マージンが取れる長さ。低めにすると nav_server の初期 fetch race で flaky 化する。
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
        cls.tf_broadcaster = TransformBroadcaster(cls.node)
        cls._robot_xy = (100.0, 100.0)  # 初期は全 POI から十分遠い座標
        # rclpy timer で周期 publish。別 thread で sendTransform を呼ぶと
        # rclpy publisher の thread safety 制約に違反し、jazzy 環境で TF buffer
        # の反映が時々壊れて test_exit_event が flaky 化していた (#165 検証時に観測)。
        # executor 内 (spin_once 経由) で publish することで安全側に倒す。
        cls._tf_timer = cls.node.create_timer(
            cls.TF_PUBLISH_INTERVAL, cls._tf_publish_callback)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_timer(cls._tf_timer)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _tf_publish_callback(cls):
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

    def setUp(self):
        self.received_events.clear()

    def tearDown(self):
        """各 test 後に nav_server の poi_inside_state_ をリセットする (#165)。

        pose を全 POI から十分遠い座標に動かし、radius_check の周期で inside POI
        が EXIT 判定されるまで spin する。これで次 test は「全 POI inside=false」
        を前提に始められる。

        この tearDown が無いと、unittest の実行順 (alphabet 順) と「各 test が
        別々の POI を使う」前提に依存していた。順序変更や同 POI test 追加で
        前段 ENTER が出ず flaky 化するリスクがあったため明示的に隔離する。
        """
        self._set_robot_pose(100.0, 100.0)
        # radius_check_hz=10 (= 100ms 周期) で全 inside POI が EXIT に転じるのに
        # 必要な時間。余裕をもって 1.0 秒 (10 周期分)。
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _set_robot_pose(self, x, y):
        """publish callback が次周期から拾う robot 位置を更新する。

        callback と読み出しは同じ executor 上の単一スレッドで動くため lock 不要。
        """
        type(self)._robot_xy = (x, y)

    def _wait_for_event(self, predicate, timeout_sec=None):
        """述語を満たす PoiEvent を受信するまで spin する。受信時点で即 True、
        timeout したら False を返す。

        flaky 対策の背景 (#153):
        - 旧実装は `_spin_and_wait(2.0)` の固定時間 spin で、TF buffer の反映や
          nav_server の初期 POI fetch のタイミング次第で取りこぼしていた。
        - StaticTransformBroadcaster で同 child_frame_id を別座標で再 publish しても
          subscriber 側の TF buffer cache 更新が伝播せず、前 test の pose が残って
          ENTER 判定が成立しないケースがあった。これは setUpClass で dynamic TF を
          rclpy timer で周期 publish することで解消している。
        - timeout は wall clock 調整の影響を受けないよう `time.monotonic()` で計測。
        """
        if timeout_sec is None:
            timeout_sec = self.EVENT_WAIT_TIMEOUT
        end_time = time.monotonic() + timeout_sec
        while time.monotonic() < end_time:
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

    def test_enter_event_goal_only_poi_again(self):
        """isolation 検証 (#165): 直前の test_enter_event_goal_only_poi で
        inside だった poi_goal_only に再 ENTER できることを確認する。

        tearDown で nav_server の inside_state がリセットされていない場合、
        inside_state[poi_goal_only]=true が残って ENTER の再発火条件
        (was_inside=false && dist <= radius) を満たせず timeout する。
        """
        self._set_robot_pose(1.0, 0.0)
        found = self._wait_for_event(
            lambda e: (e.event_type == PoiEvent.EVENT_ENTER
                       and e.poi.name == 'poi_goal_only'))
        self.assertTrue(found,
                        "tearDown isolation: poi_goal_only への再 ENTER が観測されるべき")

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
        # 後段の EXIT 条件 (was_inside && dist > radius*hyst) は前段で ENTER して
        # was_inside=true を成立させないと永遠に満たせない。前提を assert で固定する。
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
