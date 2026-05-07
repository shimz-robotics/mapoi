"""Unit test for `MapoiWebuiNode._resolve_backend_status_for_ui` (#208, #212 codex review).

`get_navigation_capabilities` の状態遷移ロジックを切り出した pure 関数。3 つの状態
(未受信 / 受信済み+alive / 受信済み+lost) を直接 assert する。Codex review #212 で
「受信済み+lost で legacy fallback に落ちると command subscriber が残った状態で
false-enable する」という回帰を pin する。
"""
import unittest

from mapoi_webui.mapoi_webui_node import MapoiWebuiNode


class TestResolveBackendStatusForUi(unittest.TestCase):

    def test_no_cache_returns_none(self):
        """未受信 (cached is None) は None を返す → caller は legacy fallback path に流す."""
        self.assertIsNone(
            MapoiWebuiNode._resolve_backend_status_for_ui(None, alive=True))
        self.assertIsNone(
            MapoiWebuiNode._resolve_backend_status_for_ui(None, alive=False))

    def test_alive_returns_cached_payload(self):
        """受信済み + alive は cache をそのまま返す (latched payload)."""
        cached = {'backend_type': 'nav2', 'backend_ready': True, 'reason': ''}
        self.assertEqual(
            MapoiWebuiNode._resolve_backend_status_for_ui(cached, alive=True),
            cached)

    def test_alive_returns_cached_even_if_payload_unready(self):
        """受信済み + alive は backend_ready=false の cache もそのまま返す
        (publisher は alive で「Nav2 not ready」を表明している正常状態)."""
        cached = {'backend_type': 'nav2', 'backend_ready': False, 'reason': 'Nav2 missing'}
        self.assertEqual(
            MapoiWebuiNode._resolve_backend_status_for_ui(cached, alive=True),
            cached)

    def test_seen_but_lost_returns_explicit_unready(self):
        """受信済み + lost は backend_ready=False に上書きした dict を返す.

        legacy fallback path に落とさず明示 unready にすることで、command subscriber が
        残った状態でも UI が enable に戻らないことを保証する (#212 codex review high)。
        """
        cached = {'backend_type': 'nav2', 'backend_ready': True, 'reason': ''}
        result = MapoiWebuiNode._resolve_backend_status_for_ui(cached, alive=False)

        # 戻り値は新 dict (cache を上書きしないこと)
        self.assertIsNot(result, cached)
        self.assertTrue(cached['backend_ready'],
                        'original cache MUST not be mutated')

        # backend_ready は必ず False
        self.assertFalse(result['backend_ready'])
        # backend_type は維持される (UI tooltip 表示等)
        self.assertEqual(result['backend_type'], 'nav2')
        # reason は staleness 由来であることがわかる文言に上書きされる
        self.assertNotEqual(result['reason'], '')
        self.assertIn('liveliness', result['reason'].lower())

    def test_seen_but_lost_localization_payload(self):
        """Localization payload も同じ semantics で扱われる."""
        cached = {'backend_type': 'amcl', 'backend_ready': True, 'reason': ''}
        result = MapoiWebuiNode._resolve_backend_status_for_ui(cached, alive=False)
        self.assertFalse(result['backend_ready'])
        self.assertEqual(result['backend_type'], 'amcl')


if __name__ == '__main__':
    unittest.main()
