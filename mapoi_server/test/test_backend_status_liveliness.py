"""Liveliness QoS staleness 検出 contract test (#208).

`mapoi/nav/backend_status` / `mapoi/localization/backend_status` の msg contract
(NavigationBackendStatus.msg / LocalizationBackendStatus.msg の QoS contract
セクション参照) を pin する。本テストの 2 つの軸:

1. **Lease lost contract** (`test_*_liveliness_lease`):
   pub=MANUAL_BY_TOPIC + sub=AUTOMATIC + lease 5s で、
   - publisher が 1 件 publish した時点で subscriber 側 liveliness event が
     `alive_count >= 1` で発火する
   - その後 publish を停止すると lease (5s) 経過後に `alive_count == 0` が発火する
   PR #210 で入れた `count_publishers() == 0` polling 暫定実装からの proper fix を保証。

2. **Contract violation rejection** (`test_*_legacy_publisher_incompatible`):
   liveliness QoS 未設定 (= infinite lease) の publisher は subscriber と QoS
   incompatible で接続不可。これは「contract に従わない publisher を意図的に拒否」
   する設計の pin (#212 codex review)。

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


def _make_pub_qos():
    """Publisher 側 QoS: MANUAL_BY_TOPIC + 5s lease (msg contract、#208)."""
    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        liveliness=LivelinessPolicy.MANUAL_BY_TOPIC,
        liveliness_lease_duration=Duration(seconds=5),
    )


def _make_sub_qos():
    """Subscriber 側 QoS: AUTOMATIC + 5s lease (旧 publisher 互換、#212 codex review)."""
    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        liveliness=LivelinessPolicy.AUTOMATIC,
        liveliness_lease_duration=Duration(seconds=5),
    )


class TestBackendStatusLiveliness(unittest.TestCase):
    """MANUAL_BY_TOPIC publisher + AUTOMATIC subscriber + 5s lease を pin する (#208)."""

    LEASE_PLUS_MARGIN_SEC = 8.0

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
        sub_qos = _make_sub_qos()
        pub_qos = _make_pub_qos()
        events = []

        sub_node = rclpy.create_node(f'liveliness_sub_{topic_name.replace("/", "_")}')
        sub_node.create_subscription(
            msg_type, topic_name, lambda msg: None, sub_qos,
            event_callbacks=SubscriptionEventCallbacks(
                liveliness=lambda event: events.append(
                    (event.alive_count, event.not_alive_count))))

        pub_node = rclpy.create_node(f'liveliness_pub_{topic_name.replace("/", "_")}')
        pub = pub_node.create_publisher(msg_type, topic_name, pub_qos)

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

        # publish を止めて lease (5s) 経過を待つ。pub_node は生かしたままなので
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

    def _run_legacy_publisher_incompatible_test(self, msg_type, topic_name):
        """msg contract に従わない publisher (liveliness QoS 未設定 = lease infinite) は
        contract subscriber (lease 5s) と QoS incompatible で接続不可となることを pin する。

        QoS rule: `pub.lease <= sub.lease` でないと incompatible。pub=infinite > sub=5s で違反。
        これは「contract 違反 publisher を意図的に拒否」する設計の確認 (#212 codex review)。
        """
        # Legacy publisher: durability/reliability は新 contract に揃えるが、liveliness QoS を
        # 設定しない (= default infinite lease)。これが `pub.lease > sub.lease` を引き起こす。
        legacy_pub_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sub_qos = _make_sub_qos()  # 新 contract: AUTOMATIC + lease 5s

        incompatible_events = []
        messages = []

        sub_node = rclpy.create_node(
            f'incompat_sub_{topic_name.replace("/", "_")}')
        sub_node.create_subscription(
            msg_type, topic_name, lambda msg: messages.append(msg.data), sub_qos,
            event_callbacks=SubscriptionEventCallbacks(
                incompatible_qos=lambda event: incompatible_events.append(
                    (event.total_count, event.last_policy_kind))))

        pub_node = rclpy.create_node(
            f'incompat_pub_{topic_name.replace("/", "_")}')
        pub = pub_node.create_publisher(msg_type, topic_name, legacy_pub_qos)
        msg = msg_type()
        msg.backend_type = 'legacy-test'
        msg.backend_ready = True
        pub.publish(msg)

        # incompatible_qos event 発火を待つ (もしくは 2 秒経っても message が来ないことを確認)。
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not incompatible_events:
            rclpy.spin_once(sub_node, timeout_sec=0.1)

        pub_node.destroy_node()
        sub_node.destroy_node()

        self.assertTrue(
            incompatible_events,
            f'{topic_name}: expected incompatible_qos event (LIVELINESS) when legacy '
            f'publisher (infinite lease) connects to contract subscriber (lease 5s); '
            f'incompatible_events: {incompatible_events}')
        self.assertEqual(
            messages, [],
            f'{topic_name}: contract violator must NOT deliver messages; got: {messages}')

    def test_navigation_backend_legacy_publisher_incompatible(self):
        self._run_legacy_publisher_incompatible_test(
            NavigationBackendStatus, 'test/issue208/nav_legacy_incompat')

    def test_localization_backend_legacy_publisher_incompatible(self):
        self._run_legacy_publisher_incompatible_test(
            LocalizationBackendStatus, 'test/issue208/loc_legacy_incompat')
