"""Liveliness QoS staleness 検出 contract test (#208).

`mapoi/nav/backend_status` / `mapoi/localization/backend_status` の msg
contract で MANUAL_BY_TOPIC liveliness + 3s lease を要求している
(NavigationBackendStatus.msg / LocalizationBackendStatus.msg の QoS contract
セクション)。本テストでは:

1. publisher が 1 件 publish した時点で subscriber 側 liveliness event が
   `alive_count >= 1` で発火する
2. その後 publish を停止すると lease (3s) 経過後に `alive_count == 0` が
   発火する

を pin し、PR #210 で入れた `count_publishers() == 0` polling 暫定実装
(WebUI Python / RViz panel C++) からの proper fix が機能していることを保証する。
test 用 topic を使うのでサーバープロセスを subprocess 起動する必要はなく
in-process で完結する (launch_testing 不要 → ament_cmake_pytest で登録)。
"""
import time
import unittest

import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, LivelinessPolicy, QoSProfile

try:  # Jazzy (rclpy 3.x+) and newer
    from rclpy.event_handler import SubscriptionEventCallbacks
except ImportError:  # Humble fallback
    from rclpy.qos_event import SubscriptionEventCallbacks

from mapoi_interfaces.msg import LocalizationBackendStatus, NavigationBackendStatus


def _make_qos():
    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        liveliness=LivelinessPolicy.MANUAL_BY_TOPIC,
        liveliness_lease_duration=Duration(seconds=3),
    )


class TestBackendStatusLiveliness(unittest.TestCase):
    """MANUAL_BY_TOPIC liveliness + 3s lease の contract を pin する (#208)."""

    LEASE_PLUS_MARGIN_SEC = 6.0

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def _spin_until(self, sub_node, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(sub_node, timeout_sec=0.1)

    def _run_liveliness_lease_test(self, msg_type, topic_name):
        qos = _make_qos()
        events = []

        sub_node = rclpy.create_node(f'liveliness_sub_{topic_name.replace("/", "_")}')
        sub_node.create_subscription(
            msg_type, topic_name, lambda msg: None, qos,
            event_callbacks=SubscriptionEventCallbacks(
                liveliness=lambda event: events.append(
                    (event.alive_count, event.not_alive_count))))

        pub_node = rclpy.create_node(f'liveliness_pub_{topic_name.replace("/", "_")}')
        pub = pub_node.create_publisher(msg_type, topic_name, qos)

        # publish 1 件 → publisher discover + lease 起動
        msg = msg_type()
        msg.backend_type = 'test'
        msg.backend_ready = True
        pub.publish(msg)

        # alive_count >= 1 が来るまで spin (publisher discover 確認)
        self._spin_until(
            sub_node, lambda: any(alive >= 1 for alive, _ in events), timeout_sec=3.0)
        self.assertTrue(
            any(alive >= 1 for alive, _ in events),
            f'{topic_name}: expected alive event after publisher creation + first publish; '
            f'events so far: {events}')

        events_after_alive = len(events)

        # publish を止めて lease (3s) 経過を待つ。pub_node は生かしたままなので
        # 「publisher process は生きているが 1Hz publish が止まった」状態を再現する
        # (= bridge で 1Hz timer が止まったケースの模倣、これが #208 の本質)。
        self._spin_until(
            sub_node,
            lambda: any(alive == 0 for alive, _ in events[events_after_alive:]),
            timeout_sec=self.LEASE_PLUS_MARGIN_SEC)

        pub_node.destroy_node()
        sub_node.destroy_node()

        self.assertTrue(
            any(alive == 0 for alive, _ in events[events_after_alive:]),
            f'{topic_name}: expected liveliness lost (alive_count == 0) within '
            f'lease + margin ({self.LEASE_PLUS_MARGIN_SEC}s); events so far: {events}')

    def test_navigation_backend_liveliness_lease(self):
        # 本番 topic 名と衝突しないよう test-specific suffix を付ける。
        self._run_liveliness_lease_test(NavigationBackendStatus, 'test/issue208/nav_backend_status')

    def test_localization_backend_liveliness_lease(self):
        self._run_liveliness_lease_test(LocalizationBackendStatus, 'test/issue208/loc_backend_status')
