"""mapoi_nav_server の navigation backend readiness publish テスト (#198)。

Nav2 が不在の状態で `mapoi_nav_server` を起動し、`mapoi/nav/backend_status` topic に
`backend_ready: false` が 1Hz で publish されることを確認する。Route Run の無限待ち回帰
や、WebUI の `Navigation connected` 誤判定が再発しないように pin する。
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
from mapoi_interfaces.msg import NavigationBackendStatus


@launch_testing.markers.keep_alive
def generate_test_description():
    nav_server = launch_ros.actions.Node(
        package='mapoi_server',
        executable='mapoi_nav_server',
        name='mapoi_nav_server',
    )

    return launch.LaunchDescription([
        nav_server,
        launch_testing.actions.ReadyToTest(),
    ]), {'mapoi_nav_server': nav_server}


class TestBackendStatusNoNav2(unittest.TestCase):
    """Nav2 不在環境での readiness publish 動作を pin する。"""

    SPIN_TIMEOUT = 5.0

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_backend_status_no_nav2_node')
        cls.received = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.sub = cls.node.create_subscription(
            NavigationBackendStatus,
            'mapoi/nav/backend_status',
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

    def test_publishes_backend_unready_when_nav2_absent(self):
        # 1Hz publish なので最大 ~3 秒で複数件届く想定。最初の 1 件で十分。
        deadline = time.monotonic() + self.SPIN_TIMEOUT
        while not self.received and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.assertTrue(self.received, 'no NavigationBackendStatus message received')
        msg = self.received[-1]
        self.assertEqual(msg.backend_type, 'nav2')
        self.assertTrue(msg.backend_present)
        self.assertFalse(msg.backend_ready)
        # action / service / subscriber が一切いない環境では各 capability も全て false。
        # 個別 readiness を assert しておくことで「backend_ready 算出ロジックの引数が
        # ずれた」回帰 (例: goal/route 以外を AND に追加した) を検出できる (#205 review low)。
        self.assertFalse(msg.goal_ready)
        self.assertFalse(msg.route_ready)
        self.assertFalse(msg.switch_map_ready)
        self.assertFalse(msg.initialpose_ready)
        self.assertNotEqual(msg.reason, '',
                            'reason should be populated when backend_ready is false')
