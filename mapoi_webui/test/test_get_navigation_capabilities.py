"""Unit test for `MapoiWebNode.get_navigation_capabilities` (#199, #198 / #205 / #212).

`_resolve_backend_status_for_ui` 単体 (`test_resolve_backend_status_for_ui.py`) は
状態遷移ロジックしか pin しない。本 test はその resolver と各 publisher の
`get_subscription_count()` を組み合わせた `get_navigation_capabilities` の **統合契約**
を固定する:

- backend_status 未受信 (None) → subscriber 数による legacy fallback
- command topic (goal/route/cancel/pause/resume) の subscriber が availability の根拠。
  `switch_map` / `initialpose` は command 扱いしない (#... initialpose は localization
  bridge が残すため除外)
- backend_status 受信済み → backend_ready が legacy fallback を上書き
  (#212 codex review high: command subscriber が残っても false-enable しない)
- backend_status 受信済み + lost (alive=False) → 明示 unready
- localization_status は navigation とは独立した payload

ROS 2 Node を本物で起動せず、`get_navigation_capabilities` を duck-typed node に
unbound 呼び出しする。`_resolve_backend_status_for_ui` は `@staticmethod` なので
実物をそのまま class 属性に貼り、本物の resolver で統合経路を検証する。
"""
import unittest

from mapoi_webui.mapoi_webui_node import MapoiWebNode


class _FakePub:
    """`get_subscription_count()` だけを持つ最小 publisher stub。"""

    def __init__(self, count=0):
        self._count = count

    def get_subscription_count(self):
        return self._count


class _NavCapNode:
    """`get_navigation_capabilities` が参照する attr / staticmethod だけを持つ duck-typed node.

    7 publisher の subscriber 数と 4 つの backend_status 系 attr を constructor で差し込む。
    `_resolve_backend_status_for_ui` は実物 (staticmethod) をそのまま使う。
    """

    _resolve_backend_status_for_ui = staticmethod(
        MapoiWebNode._resolve_backend_status_for_ui)

    def __init__(self, *, counts=None, backend_status=None, nav_alive=True,
                 localization_status=None, localization_alive=True):
        counts = counts or {}
        self.switch_map_pub_ = _FakePub(counts.get('switch_map', 0))
        self.goal_poi_pub_ = _FakePub(counts.get('goal', 0))
        self.route_pub_ = _FakePub(counts.get('route', 0))
        self.cancel_pub_ = _FakePub(counts.get('cancel', 0))
        self.pause_pub_ = _FakePub(counts.get('pause', 0))
        self.resume_pub_ = _FakePub(counts.get('resume', 0))
        self.initialpose_poi_pub_ = _FakePub(counts.get('initialpose', 0))
        self.backend_status_ = backend_status
        self.nav_backend_alive_ = nav_alive
        self.localization_status_ = localization_status
        self.localization_backend_alive_ = localization_alive


def _caps(**kwargs):
    node = _NavCapNode(**kwargs)
    return MapoiWebNode.get_navigation_capabilities(node)


class TestGetNavigationCapabilities(unittest.TestCase):

    # ---- legacy fallback (backend_status 未受信) -------------------------------

    def test_no_backend_status_no_subscribers_all_unavailable(self):
        """backend_status 未受信 + subscriber ゼロ → 全 unavailable, backend_status None."""
        caps = _caps()
        self.assertFalse(caps['navigation_available'])
        self.assertFalse(caps['switch_map_available'])
        self.assertFalse(caps['command_available'])
        self.assertIsNone(caps['backend_status'])
        # topics は 7 key 全て available False で揃う
        self.assertEqual(set(caps['topics']), {
            'switch_map', 'goal', 'route', 'cancel', 'pause', 'resume',
            'initialpose'})
        self.assertFalse(any(t['available'] for t in caps['topics'].values()))

    def test_legacy_command_subscriber_enables_navigation(self):
        """command topic に subscriber → legacy で navigation_available True。

        switch_map に subscriber がいなければ switch_map_available は False のまま
        (両者は独立、#205 round 3)。
        """
        caps = _caps(counts={'goal': 1})
        self.assertTrue(caps['command_available'])
        self.assertTrue(caps['navigation_available'])
        self.assertFalse(caps['switch_map_available'])
        self.assertTrue(caps['topics']['goal']['available'])
        self.assertEqual(caps['topics']['goal']['subscribers'], 1)

    def test_legacy_switch_map_subscriber_only(self):
        """switch_map のみ subscriber → switch_map 由来で legacy navigation_available True。"""
        caps = _caps(counts={'switch_map': 2})
        self.assertFalse(caps['command_available'])
        self.assertTrue(caps['switch_map_available'])
        self.assertTrue(caps['navigation_available'])  # legacy = switch_map or command
        self.assertEqual(caps['topics']['switch_map']['subscribers'], 2)

    def test_initialpose_subscriber_is_not_command_evidence(self):
        """initialpose の subscriber は command 扱いしない (localization bridge が残す等)。

        backend_status 未受信下で initialpose だけ subscriber がいても
        command_available / navigation_available は False のまま。
        """
        caps = _caps(counts={'initialpose': 3})
        self.assertTrue(caps['topics']['initialpose']['available'])
        self.assertFalse(caps['command_available'])
        self.assertFalse(caps['navigation_available'])
        self.assertFalse(caps['switch_map_available'])

    # ---- backend_status 受信済み (legacy を上書き) -----------------------------

    def test_backend_ready_overrides_legacy_true(self):
        """backend_status ready (alive) → subscriber ゼロでも navigation_available True。

        switch_map_available は backend_ready の alias になる (#205 round 3 high #2)。
        """
        status = {'backend_type': 'nav2', 'backend_ready': True, 'reason': ''}
        caps = _caps(backend_status=status, nav_alive=True)
        self.assertTrue(caps['navigation_available'])
        self.assertTrue(caps['switch_map_available'])
        self.assertEqual(caps['backend_status'], status)

    def test_backend_not_ready_overrides_legacy_command_subscriber(self):
        """backend_status not-ready (alive) は command subscriber が残っていても勝つ。

        #212 codex review high: backend_status path に乗ったら legacy false-enable を
        起こさず navigation_available False。
        """
        status = {'backend_type': 'nav2', 'backend_ready': False, 'reason': 'Nav2 missing'}
        caps = _caps(counts={'goal': 1, 'route': 1}, backend_status=status, nav_alive=True)
        self.assertTrue(caps['command_available'])  # subscriber は存在する
        self.assertFalse(caps['navigation_available'])  # が backend_ready=False が勝つ
        self.assertFalse(caps['switch_map_available'])

    def test_backend_seen_but_lost_is_explicit_unready(self):
        """backend_status 受信済みだが lost (alive=False) → 明示 unready。

        cache は ready=True を保持していても liveliness lost で False に倒れ、
        command subscriber が残っていても navigation_available False (#212)。
        """
        status = {'backend_type': 'nav2', 'backend_ready': True, 'reason': ''}
        caps = _caps(counts={'goal': 1}, backend_status=status, nav_alive=False)
        self.assertFalse(caps['navigation_available'])
        self.assertFalse(caps['switch_map_available'])
        # 返却 backend_status は backend_ready False に上書きされている
        self.assertFalse(caps['backend_status']['backend_ready'])
        self.assertIn('liveliness', caps['backend_status']['reason'].lower())

    # ---- localization は navigation と独立 -------------------------------------

    def test_localization_status_is_independent(self):
        """localization_status は navigation 判定に影響せず、独立 payload として出る。"""
        loc = {'backend_type': 'amcl', 'backend_ready': True, 'reason': ''}
        caps = _caps(counts={'goal': 1}, localization_status=loc, localization_alive=True)
        # navigation は backend_status 未受信なので legacy (goal subscriber) のまま
        self.assertTrue(caps['navigation_available'])
        # localization は別 key で resolver を通った値
        self.assertEqual(caps['localization_status'], loc)

    def test_localization_lost_does_not_disable_navigation(self):
        """localization が lost でも navigation 側は独立に評価される。"""
        loc = {'backend_type': 'amcl', 'backend_ready': True, 'reason': ''}
        caps = _caps(counts={'goal': 1}, localization_status=loc, localization_alive=False)
        self.assertTrue(caps['navigation_available'])  # navigation は legacy のまま生きている
        self.assertFalse(caps['localization_status']['backend_ready'])  # localization は unready


if __name__ == '__main__':
    unittest.main()
