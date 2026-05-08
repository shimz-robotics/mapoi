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
from geometry_msgs.msg import Twist
from mapoi_interfaces.msg import PoiEvent
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String


STOPPED_DWELL_TIME_SEC = 0.3


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

    mapoi_nav2_bridge_node = launch_ros.actions.Node(
        package='mapoi_server',
        executable='mapoi_nav2_bridge',
        name='mapoi_nav2_bridge',
        parameters=[{
            'tolerance_check_hz': 10.0,
            'map_frame': 'map',
            'base_frame': 'base_link',
            'stopped_dwell_time_sec': STOPPED_DWELL_TIME_SEC,
        }],
    )

    return launch.LaunchDescription([
        mapoi_server_node,
        mapoi_nav2_bridge_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'mapoi_server': mapoi_server_node, 'mapoi_nav2_bridge': mapoi_nav2_bridge_node}


class TestPoiEventIntegration(unittest.TestCase):
    """mapoi/events トピックの発行を検証する統合テスト。

    flaky 対策の経緯 (#153) は :meth:`_wait_for_event` の docstring を参照。
    要点: dynamic TF を rclpy timer で周期 publish + event 駆動 wait で
    取りこぼしを防ぐ。
    """

    # event 待ちの timeout。CI の CPU 負荷時にも `tolerance_check_hz=10` × 数周期分の
    # マージンが取れる長さ。低めにすると mapoi_nav2_bridge の初期 fetch race で flaky 化する。
    EVENT_WAIT_TIMEOUT = 5.0
    # dynamic TF publish 周期。tolerance_check_hz=10 (=100ms) より十分速くして取りこぼしを防ぐ。
    TF_PUBLISH_INTERVAL = 0.05
    # cmd_vel も timer publish にして discovery / callback timing の取りこぼしを避ける。
    CMD_VEL_PUBLISH_INTERVAL = 0.05

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_poi_event_node')
        cls.received_events = []
        cls.inside_state = {}
        cls.sub = cls.node.create_subscription(
            PoiEvent, 'mapoi/events',
            cls._event_callback, 10)
        cls.tf_broadcaster = TransformBroadcaster(cls.node)
        cls._robot_xy = (100.0, 100.0)  # 初期は全 POI から十分遠い座標
        cls.cmd_vel_pub = cls.node.create_publisher(Twist, 'cmd_vel', 10)
        cls.goal_pose_poi_pub = cls.node.create_publisher(String, 'mapoi/nav/goal_pose_poi', 1)
        cls._cmd_vel_linear_x = 0.2
        cls._cmd_vel_angular_z = 0.0
        # rclpy timer で周期 publish。別 thread で sendTransform を呼ぶと
        # rclpy publisher の thread safety 制約に違反し、jazzy 環境で TF buffer
        # の反映が時々壊れて test_exit_event が flaky 化していた (#165 検証時に観測)。
        # executor 内 (spin_once 経由) で publish することで安全側に倒す。
        cls._tf_timer = cls.node.create_timer(
            cls.TF_PUBLISH_INTERVAL, cls._tf_publish_callback)
        cls._cmd_vel_timer = cls.node.create_timer(
            cls.CMD_VEL_PUBLISH_INTERVAL, cls._cmd_vel_publish_callback)

    @classmethod
    def tearDownClass(cls):
        try:
            cls.node.destroy_timer(cls._cmd_vel_timer)
            cls.node.destroy_timer(cls._tf_timer)
            cls.node.destroy_node()
        finally:
            # destroy 過程で例外が出ても rclpy context は確実に閉じる。
            # 後続の test class や launch_testing fixture への汚染を防ぐ。
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

    @classmethod
    def _cmd_vel_publish_callback(cls):
        msg = Twist()
        msg.linear.x = cls._cmd_vel_linear_x
        msg.angular.z = cls._cmd_vel_angular_z
        cls.cmd_vel_pub.publish(msg)

    @classmethod
    def _event_callback(cls, msg):
        cls.received_events.append(msg)
        if msg.event_type == PoiEvent.EVENT_ENTER:
            cls.inside_state[msg.poi.name] = True
        elif msg.event_type == PoiEvent.EVENT_EXIT:
            cls.inside_state[msg.poi.name] = False

    def setUp(self):
        self.received_events.clear()
        self._set_cmd_vel(0.2, 0.0)

    def tearDown(self):
        """各 test 後に mapoi_nav2_bridge の poi_inside_state_ をリセットする (#165)。

        unittest の実行順 (alphabet) と「各 test が別 POI を使う」前提への暗黙
        依存を切るため、明示的に隔離する。実体は :meth:`_reset_inside_state` 側。
        """
        self._reset_inside_state()
        # 次 test が inside した瞬間に前 test の zero velocity dwell を拾わないよう、
        # moving cmd_vel を mapoi_nav2_bridge に届けて stopped detector を reset する。
        self._set_cmd_vel(0.2, 0.0)
        self._spin_for(0.2)

    def _set_robot_pose(self, x, y):
        """publish callback が次周期から拾う robot 位置を更新する。

        全アクセスは unittest main thread 上で行われ、timer callback も
        `rclpy.spin_once` 経由でしか実行されない (SingleThreadedExecutor 前提)
        ため lock 不要。MultiThreadedExecutor や background spin に戻す場合は
        改めて同期が必要。
        """
        type(self)._robot_xy = (x, y)

    def _set_cmd_vel(self, linear_x, angular_z):
        type(self)._cmd_vel_linear_x = linear_x
        type(self)._cmd_vel_angular_z = angular_z

    def _spin_for(self, duration_sec):
        end_time = time.monotonic() + duration_sec
        while time.monotonic() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_for_subscriber(self, topic_name, timeout_sec=3.0):
        end_time = time.monotonic() + timeout_sec
        while time.monotonic() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.node.count_subscribers(topic_name) > 0:
                return True
        return False

    def _count_events(self, predicate):
        return sum(1 for e in self.received_events if predicate(e))

    def _compute_inside_pois(self):
        """subscriber が観測した ENTER / EXIT から、現時点で inside と推定される
        POI 名集合を返す。

        mapoi_nav2_bridge 側の `poi_inside_state_` を直接覗けないため、subscriber 視点の
        近似として使う。STOPPED/RESUMED test は途中で `received_events.clear()` する
        ため、event assertion 用履歴とは独立した state tracker を使う。
        """
        return {name for name, inside in type(self).inside_state.items() if inside}

    def _reset_inside_state(self, exit_timeout_sec=3.0):
        """mapoi_nav2_bridge 側の `poi_inside_state_` を全 false に戻す helper。

        現在 inside と推定される POI を列挙し、pose を全 POI から十分遠い
        `(100, 100)` に動かしてから、各 POI の EVENT_EXIT を bounded timeout で
        待つ。fixed sleep ではなく完了条件付き待機にすることで、TF 反映や
        tolerance_check 遅延時にも tearDown 側で fail を検出できる (#165)。

        tearDown と test 内 (順序非依存な isolation 検証) で共有する。
        """
        inside_pois = self._compute_inside_pois()
        self._set_robot_pose(100.0, 100.0)
        for poi_name in inside_pois:
            ok = self._wait_for_event(
                lambda e, n=poi_name: (e.event_type == PoiEvent.EVENT_EXIT
                                       and e.poi.name == n),
                timeout_sec=exit_timeout_sec)
            self.assertTrue(
                ok,
                f"isolation reset: {poi_name} の EVENT_EXIT が"
                f" {exit_timeout_sec}s 以内に観測されなかった")
        self.received_events.clear()

    def _wait_for_event(self, predicate, timeout_sec=None):
        """述語を満たす PoiEvent を受信するまで spin する。受信時点で即 True、
        timeout したら False を返す。

        flaky 対策の背景 (#153):
        - 旧実装は `_spin_and_wait(2.0)` の固定時間 spin で、TF buffer の反映や
          mapoi_nav2_bridge の初期 POI fetch のタイミング次第で取りこぼしていた。
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

    # 旧 _enter_poi / _enter_and_stop_poi は #220 で撤去。v0.5.0+ spec では _activate_route
    # 経由で ROUTE mode に入ってから ENTER を待つ流れに変更 (各 test 内で構築)。

    # --- v0.5.0+ の PoiEvent 仕様 (#220) ---
    # EVENT_ENTER / EVENT_PAUSED / EVENT_EXIT の 3 種別 + route 走行中 + route 登録 POI のみ。
    #
    # NOTE: ROUTE-mode integration test (ENTER / PAUSED / EXIT 発火確認) は本 PR では
    # 含まない。bridge は `mapoi_route_cb` で Nav2 FollowWaypoints action server に
    # 接続できないと `reset_nav_state()` で `nav_mode_` を IDLE に戻すため、ROUTE mode
    # を維持して event 発火を観測するには Nav2 action server mock が必要。これは scope が
    # 中規模 (mock setup + test infra) のため別 issue で扱う。本 PR では下の
    # `test_no_event_in_non_route_mode` で「v0.5.0+ spec で route 走行外 (IDLE / GOAL mode) は
    # event を発火しない」ことを negative evidence として確認する。

    def test_no_event_in_non_route_mode(self):
        """route 走行外 (IDLE / GOAL mode、ROUTE 未開始) では POI に侵入しても event 発火しない (#220)。

        旧 spec では IDLE / GOAL でも EVENT_ENTER が発火していたため、本 test は
        v0.5.0+ spec への migration evidence として機能する (旧コードでは fail、新コードで pass)。
        """
        # ROUTE mode に入っていない = nav_mode_ は IDLE のまま
        self._set_robot_pose(1.0, 0.0)
        # 発火しないことを timeout で確認
        not_found = self._wait_for_event(
            lambda e: e.poi.name == 'poi_goal_only',
            timeout_sec=2.0)
        self.assertFalse(not_found,
                         "route 走行外 (IDLE / GOAL mode) では route POI でも event 発火しないべき")
