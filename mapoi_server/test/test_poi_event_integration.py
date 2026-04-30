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
        try:
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

    def setUp(self):
        self.received_events.clear()

    def tearDown(self):
        """各 test 後に nav_server の poi_inside_state_ をリセットする (#165)。

        unittest の実行順 (alphabet) と「各 test が別 POI を使う」前提への暗黙
        依存を切るため、明示的に隔離する。実体は :meth:`_reset_inside_state` 側。
        """
        self._reset_inside_state()

    def _set_robot_pose(self, x, y):
        """publish callback が次周期から拾う robot 位置を更新する。

        全アクセスは unittest main thread 上で行われ、timer callback も
        `rclpy.spin_once` 経由でしか実行されない (SingleThreadedExecutor 前提)
        ため lock 不要。MultiThreadedExecutor や background spin に戻す場合は
        改めて同期が必要。
        """
        type(self)._robot_xy = (x, y)

    def _compute_inside_pois(self):
        """`received_events` の履歴から、現時点で inside と推定される POI 名集合
        を返す。EVENT_ENTER で True、EVENT_EXIT で False に更新した最終状態。

        nav_server 側の `poi_inside_state_` を直接覗けないため、subscriber 視点の
        近似として使う。test 中に発生した event のみを見るので、setUp で必ず
        `received_events.clear()` した状態で開始する前提。
        """
        state = {}
        for ev in self.received_events:
            if ev.event_type == PoiEvent.EVENT_ENTER:
                state[ev.poi.name] = True
            elif ev.event_type == PoiEvent.EVENT_EXIT:
                state[ev.poi.name] = False
        return {name for name, inside in state.items() if inside}

    def _reset_inside_state(self, exit_timeout_sec=3.0):
        """nav_server 側の `poi_inside_state_` を全 false に戻す helper。

        現在 inside と推定される POI を列挙し、pose を全 POI から十分遠い
        `(100, 100)` に動かしてから、各 POI の EVENT_EXIT を bounded timeout で
        待つ。fixed sleep ではなく完了条件付き待機にすることで、TF 反映や
        radius_check 遅延時にも tearDown 側で fail を検出できる (#165)。

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

    def test_isolation_re_enter_same_poi(self):
        """isolation 検証 (#165): 同 POI に ENTER → reset → 再 ENTER できる
        ことを 1 test 内で完結確認する。実行順や個別実行に依存しない。

        reset helper で `poi_inside_state_` を false に戻していなければ
        ENTER の再発火条件 (was_inside=false && dist <= radius) を満たせず
        2 回目の `_wait_for_event` が timeout する。helper を tearDown とも
        共有しているので、本 test が PASS する限り tearDown isolation も
        実装上等価に検証できる。
        """
        self._set_robot_pose(1.0, 0.0)
        found1 = self._wait_for_event(
            lambda e: (e.event_type == PoiEvent.EVENT_ENTER
                       and e.poi.name == 'poi_goal_only'))
        self.assertTrue(found1,
                        "1 回目: poi_goal_only に ENTER できるべき")

        self._reset_inside_state()

        self._set_robot_pose(1.0, 0.0)
        found2 = self._wait_for_event(
            lambda e: (e.event_type == PoiEvent.EVENT_ENTER
                       and e.poi.name == 'poi_goal_only'))
        self.assertTrue(found2,
                        "reset 後: 同 POI への再 ENTER が観測されるべき")

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
