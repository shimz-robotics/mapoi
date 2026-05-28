"""mapoi モードでの単発 Go (GOAL モード) 到達判定の integration test (#261).

`waypoint_arrival_mode=mapoi` のとき、単発「POIへのGo」(`mapoi/nav/goal_pose_poi`) も
POI 個別の `tolerance.xy` + `tolerance.yaw` を尊重して到達判定する経路を NavigateToPose mock
で pin する。

検証する shape:
- OR トリガ a (radius + yaw): robot が goal POI の tolerance.xy 内かつ姿勢が tolerance.yaw 内に
  あれば、Nav2 SUCCEEDED を待たず即 "succeeded" になる (snappy)。
- yaw 不一致: tolerance.xy 内でも姿勢が tolerance.yaw を超えていれば radius では完了せず、
  Nav2 (mock) SUCCEEDED で初めて完了する (最終姿勢合わせを待つ)。
- xy 範囲外: robot が遠い場合は radius では完了せず、Nav2 SUCCEEDED で完了する (OR トリガ b)。

mock は `navigate_to_pose` action server。受理した goal を cancel まで EXECUTING で保持し、
`succeed_current_goal()` で SUCCEEDED finalize する (route mapoi arrival test と同方針)。
"""

import math
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
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String


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
            'cmd_vel_msg_type': 'twist',
            # 本 test の対象: mapoi モードでの単発 Go 到達判定 (#261)。
            'waypoint_arrival_mode': 'mapoi',
        }],
    )

    return launch.LaunchDescription([
        mapoi_server_node,
        mapoi_nav2_bridge_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'mapoi_server': mapoi_server_node, 'mapoi_nav2_bridge': mapoi_nav2_bridge_node}


class FakeNavigateToPoseServer:
    """`navigate_to_pose` の最小 mock (route mapoi arrival test と同方針)。

    受理 goal の (x, y) を順に記録し、cancel まで EXECUTING で保持する。
    `succeed_current_goal()` で実行中の goal を SUCCEEDED finalize する (OR トリガ b 用途)。
    """

    def __init__(self):
        self._node = rclpy.create_node('fake_navigate_to_pose_server_goal')
        callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self._node, NavigateToPose, 'navigate_to_pose',
            execute_callback=self._execute_callback,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            handle_accepted_callback=lambda gh: gh.execute(),
            callback_group=callback_group,
        )
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._node)
        self._stop_event = threading.Event()
        self._succeed_event = threading.Event()
        self._lock = threading.Lock()
        self._goals_xy = []
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        while rclpy.ok() and not self._stop_event.is_set():
            self._executor.spin_once(timeout_sec=0.05)

    def _execute_callback(self, goal_handle):
        p = goal_handle.request.pose.pose.position
        with self._lock:
            self._goals_xy.append((p.x, p.y))
        while (rclpy.ok() and not self._stop_event.is_set()
               and not goal_handle.is_cancel_requested
               and not self._succeed_event.is_set()):
            time.sleep(0.05)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif self._succeed_event.is_set():
            self._succeed_event.clear()
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return NavigateToPose.Result()

    def succeed_current_goal(self):
        self._succeed_event.set()

    def goals_xy(self):
        with self._lock:
            return list(self._goals_xy)

    def reset(self):
        self._succeed_event.clear()
        with self._lock:
            self._goals_xy.clear()

    def shutdown(self):
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        try:
            self._executor.shutdown()
        finally:
            try:
                self._action_server.destroy()
            finally:
                self._node.destroy_node()


class TestPoiEventGoalMapoiArrival(unittest.TestCase):
    """mapoi モードでの単発 Go 到達判定 (#261) を pin する。"""

    NAV_STATUS_WAIT_TIMEOUT = 5.0
    GOAL_WAIT_TIMEOUT = 5.0
    TF_PUBLISH_INTERVAL = 0.05
    FAR_AWAY_XY = (100.0, 100.0)

    # poi_goal_only: pose=(1,0,yaw=0), tolerance.xy=0.5, tolerance.yaw=0.785 (≒45°)。
    GOAL_POI = 'poi_goal_only'
    GOAL_XY = (1.0, 0.0)
    GOAL_YAW = 0.0

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_poi_event_goal_mapoi_node')
        cls.received_nav_status = []

        nav_status_qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.nav_status_sub = cls.node.create_subscription(
            String, 'mapoi/nav/status', cls._nav_status_callback, nav_status_qos)
        cls.tf_broadcaster = TransformBroadcaster(cls.node)
        cls.goal_pub = cls.node.create_publisher(String, 'mapoi/nav/goal_pose_poi', 1)
        cls.cancel_pub = cls.node.create_publisher(String, 'mapoi/nav/cancel', 1)

        cls._robot_xy = cls.FAR_AWAY_XY
        cls._robot_yaw = 0.0
        cls._tf_timer = cls.node.create_timer(
            cls.TF_PUBLISH_INTERVAL, cls._tf_publish_callback)

        cls.fake_server = FakeNavigateToPoseServer()

    @classmethod
    def tearDownClass(cls):
        try:
            cls.fake_server.shutdown()
        finally:
            try:
                cls.node.destroy_timer(cls._tf_timer)
                cls.node.destroy_node()
            finally:
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
        t.transform.rotation.z = math.sin(cls._robot_yaw / 2.0)
        t.transform.rotation.w = math.cos(cls._robot_yaw / 2.0)
        cls.tf_broadcaster.sendTransform(t)

    @classmethod
    def _nav_status_callback(cls, msg):
        cls.received_nav_status.append(msg.data)

    def setUp(self):
        self._set_robot_pose(*self.FAR_AWAY_XY, 0.0)
        self._publish_cancel()
        self._spin_for(0.3)
        self.received_nav_status.clear()
        self.fake_server.reset()
        self.assertTrue(self._wait_for_subscriber('mapoi/nav/goal_pose_poi'),
                        'mapoi_nav2_bridge が mapoi/nav/goal_pose_poi を subscribe していない')

    def tearDown(self):
        self._publish_cancel()
        self._spin_for(0.4)
        self._set_robot_pose(*self.FAR_AWAY_XY, 0.0)
        self.received_nav_status.clear()
        self.fake_server.reset()

    # --- helpers ---

    def _set_robot_pose(self, x, y, yaw):
        type(self)._robot_xy = (x, y)
        type(self)._robot_yaw = yaw

    def _spin_for(self, duration_sec):
        end = time.monotonic() + duration_sec
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_for_subscriber(self, topic_name, timeout_sec=5.0):
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.node.count_subscribers(topic_name) > 0:
                return True
        return False

    def _wait_for_nav_status(self, status, timeout_sec=None):
        if timeout_sec is None:
            timeout_sec = self.NAV_STATUS_WAIT_TIMEOUT
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if any(s == status or s.startswith(status + ':')
                   for s in self.received_nav_status):
                return True
        return False

    def _wait_for_goal_count(self, n, timeout_sec=None):
        if timeout_sec is None:
            timeout_sec = self.GOAL_WAIT_TIMEOUT
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if len(self.fake_server.goals_xy()) >= n:
                return True
        return False

    def _publish_goal(self, poi_name):
        msg = String()
        msg.data = poi_name
        self.goal_pub.publish(msg)

    def _publish_cancel(self):
        msg = String()
        msg.data = ''
        self.cancel_pub.publish(msg)

    def _activate_goal(self, poi_name):
        """Go を投入し、NavigateToPose goal が mock に届くまで待つ (discovery race を吸収)。"""
        for _ in range(5):
            self._publish_goal(poi_name)
            if self._wait_for_goal_count(1, timeout_sec=2.0):
                return
        self.fail(f"Go '{poi_name}' で NavigateToPose goal が mock に届かなかった")

    # --- tests ---

    def test_goal_completes_on_radius_and_yaw(self):
        """OR トリガ a: tolerance.xy 内 + 姿勢が tolerance.yaw 内なら Nav2 を待たず即 succeeded。"""
        # robot を goal POI 位置・同 yaw に置いてから Go を投入する。
        self._set_robot_pose(*self.GOAL_XY, self.GOAL_YAW)
        self._activate_goal(self.GOAL_POI)
        self.assertTrue(
            self._wait_for_nav_status('succeeded'),
            'radius + yaw 一致で Nav2 SUCCEEDED を待たず succeeded になるはず')

    def test_goal_not_completed_on_radius_with_wrong_yaw(self):
        """tolerance.xy 内でも姿勢が tolerance.yaw 超なら radius では完了せず Nav2 SUCCEEDED を待つ。"""
        # 位置は一致だが yaw を π (≒180°、tolerance.yaw=0.785 を大きく超える) にする。
        self._set_robot_pose(*self.GOAL_XY, math.pi)
        self._activate_goal(self.GOAL_POI)
        self.assertFalse(
            self._wait_for_nav_status('succeeded', timeout_sec=0.8),
            'yaw 不一致では radius だけで succeeded にならないはず (Nav2 完走待ち)')
        # mock が SUCCEEDED にすると完了 (OR トリガ b)。
        self.fake_server.succeed_current_goal()
        self.assertTrue(
            self._wait_for_nav_status('succeeded'),
            'Nav2 SUCCEEDED で Go が succeeded になるはず')

    def test_goal_completes_on_nav2_succeeded_when_far(self):
        """OR トリガ b: robot が tolerance.xy 外なら radius では完了せず Nav2 SUCCEEDED で完了する。"""
        self._set_robot_pose(*self.FAR_AWAY_XY, self.GOAL_YAW)
        self._activate_goal(self.GOAL_POI)
        self.assertFalse(
            self._wait_for_nav_status('succeeded', timeout_sec=0.8),
            'xy 範囲外では radius だけで succeeded にならないはず')
        self.fake_server.succeed_current_goal()
        self.assertTrue(
            self._wait_for_nav_status('succeeded'),
            'Nav2 SUCCEEDED で Go が succeeded になるはず')
