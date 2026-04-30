# #154: reload_map_info で `mapoi_initialpose_poi` topic に skip message
# (poi_name="") が publish され、transient_local (depth=1) の latched 値が
# 上書きされることを検証する launch_test。
#
# 「reload 後に起動した late subscriber が古い POI 名を latched 配信されない」
# というシナリオを直接検証する。test design は subscribe を 2 段階に分け、
# (1) reload 前 subscriber が起動時 latched (POI list 先頭) を受信、
# (2) reload 後 subscriber が skip message (poi_name="") を受信、
# を確認する形 (#149 round 7 follow-up)。
import os
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from std_srvs.srv import Trigger
from mapoi_interfaces.msg import InitialPoseRequest
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

    return launch.LaunchDescription([
        mapoi_server_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'mapoi_server': mapoi_server_node}


def _transient_local_qos():
    # publisher 側 (rclcpp::QoS(1).transient_local()) と互換: reliable + keep_last(1) + transient_local。
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


class TestReloadClearsInitialpose(unittest.TestCase):
    """reload_map_info 後に新規 subscribe する late subscriber が、起動時の
    POI 名ではなく skip message (poi_name="") を latched 受信することを確認する (#154)。"""

    SPIN_TIMEOUT = 5.0

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_reload_clears_initialpose_node')
        cls.reload_client = cls.node.create_client(Trigger, 'reload_map_info')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout=None):
        timeout = timeout if timeout is not None else self.SPIN_TIMEOUT
        end = self.node.get_clock().now().nanoseconds + int(timeout * 1e9)
        while self.node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _take_one_latched(self, timeout=None):
        """新規 subscription を作って transient_local の latched 値を 1 個受信する。"""
        received = []
        sub = self.node.create_subscription(
            InitialPoseRequest, 'mapoi_initialpose_poi',
            lambda msg: received.append(msg),
            _transient_local_qos())
        try:
            got = self._spin_until(lambda: len(received) >= 1, timeout=timeout)
            self.assertTrue(got, "Latched message not received within timeout")
            return received[0]
        finally:
            self.node.destroy_subscription(sub)

    def test_initial_latched_is_first_poi_name(self):
        # 前提条件: 起動直後の latched は POI list 先頭 ("poi_goal_only")。
        # この test が壊れた場合は publish_initial_poi_name 側の regression を疑う。
        self.assertTrue(
            self.reload_client.wait_for_service(timeout_sec=self.SPIN_TIMEOUT),
            "reload_map_info service did not become available")
        msg = self._take_one_latched()
        self.assertEqual(msg.poi_name, "poi_goal_only",
                         f"Initial latched poi_name should be POI list first, got '{msg.poi_name}'")

    def test_reload_clears_latched_for_late_subscriber(self):
        # 起動時 publish が完了するまで service の準備を待つ。
        self.assertTrue(
            self.reload_client.wait_for_service(timeout_sec=self.SPIN_TIMEOUT),
            "reload_map_info service did not become available")

        # reload を呼ぶ。reload_map_info_service 内で publish_initialpose_clear が呼ばれ、
        # transient_local の latched 値が skip message (poi_name="") に上書きされる。
        future = self.reload_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.SPIN_TIMEOUT)
        result = future.result()
        self.assertIsNotNone(result, "reload_map_info call did not complete")
        self.assertTrue(result.success, f"reload_map_info failed: {result.message}")

        # reload 後に subscribe する late subscriber は、上書きされた skip message を受信する。
        msg = self._take_one_latched()
        self.assertEqual(msg.poi_name, "",
                         f"Expected empty poi_name after reload (skip message), got '{msg.poi_name}'")
