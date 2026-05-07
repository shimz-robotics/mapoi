"""mapoi_amcl_localization_bridge の localization backend readiness publish テスト (#209)。

AMCL (= /initialpose subscriber) が不在の状態で `mapoi_amcl_localization_bridge` を起動し、
`mapoi/localization/backend_status` topic に `backend_ready: false` が 1Hz で publish される
ことを確認する。Navigation 側 (#198 / `test_backend_status_no_nav2.py`) と同じ pattern で
localization 軸の minimal contract を pin する。
"""
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from mapoi_interfaces.msg import LocalizationBackendStatus


@launch_testing.markers.keep_alive
def generate_test_description():
    bridge = launch_ros.actions.Node(
        package='mapoi_server',
        executable='mapoi_amcl_localization_bridge',
        name='mapoi_amcl_localization_bridge',
    )

    return launch.LaunchDescription([
        bridge,
        launch_testing.actions.ReadyToTest(),
    ]), {'mapoi_amcl_localization_bridge': bridge}


class TestLocalizationBackendStatusNoAmcl(unittest.TestCase):
    """AMCL 不在環境での readiness publish 動作を pin する。"""

    SPIN_TIMEOUT = 5.0

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_localization_backend_status_no_amcl_node')
        cls.received = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.sub = cls.node.create_subscription(
            LocalizationBackendStatus,
            'mapoi/localization/backend_status',
            cls._on_msg,
            qos,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_msg(cls, msg):
        cls.received.append(msg)

    def test_publishes_backend_unready_when_amcl_absent(self):
        # 1Hz publish なので 3 秒待てば複数件届く想定。複数件受信確認で
        # 「1Hz publish が継続している」回帰 (timer 停止・1 回しか出ない 等) を検知する。
        deadline = time.monotonic() + self.SPIN_TIMEOUT
        while len(self.received) < 2 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.assertGreaterEqual(
            len(self.received), 2,
            'expected at least 2 LocalizationBackendStatus messages within timeout '
            '(1 Hz publish should yield multiple samples)')
        msg = self.received[-1]
        # Minimal contract (#209): backend_type / backend_ready / reason のみが contract。
        self.assertEqual(msg.backend_type, 'amcl')
        self.assertFalse(msg.backend_ready)
        self.assertNotEqual(msg.reason, '',
                            'reason should be populated when backend_ready is false')
