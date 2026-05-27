"""Flask endpoint level test for `/api/pois` POST 楽観的競合検出 (#241 / #246).

`test_yaml_handler_version.py` は `compute_config_version` 単体の決定性しか pin して
いないため、Flask endpoint レベルで以下の `api_save_pois` 仕様契約を独立に固定する:

- `expected_version` 不一致 → 409 + `code: 'version_mismatch'` + `current_version`
- `expected_version` 省略 → 200 OK (旧クライアント / curl 後方互換)
- 保存後 response の `config_version` が新値に更新されている

ROS 2 Node を本物で起動すると DDS / service / publisher が絡んで test 重量級になるため、
`create_flask_app` を duck-typed `FakeNode` に bind して呼び出す (= `MapoiWebNode.create_flask_app`
は `self` を `node = self` で local capture するだけで、`/api/pois` 経路は
`node.get_config_path()` / `node.call_reload_map_info()` / `node.get_logger()` の 3 メソッドだけに
依存する)。他 endpoint の closures は登録時に評価されないため `FakeNode` に該当 attr が無くても
本 test は影響を受けない。
"""
import os
import tempfile
import unittest

import yaml

from mapoi_webui.mapoi_webui_node import MapoiWebNode


class _NullLogger:
    """`api_save_pois` の except 経路で `node.get_logger().error()` を呼ぶための最小 stub。"""
    def error(self, *_args, **_kwargs):
        pass

    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


class _FakeNode:
    """`/api/pois` 経路で参照される最小限の attr / method だけを持つ duck-typed node."""

    def __init__(self, config_path, reload_succeeds=True):
        self._config_path = config_path
        self._reload_succeeds = reload_succeeds

    def get_config_path(self, _map_name=None):
        return self._config_path

    def call_reload_map_info(self):
        return self._reload_succeeds

    def get_logger(self):
        return _NullLogger()


def _seed_config(path, pois=None):
    """tmp yaml に最小 schema を書き出す。`pois` 未指定なら空 list。"""
    with open(path, 'w') as f:
        yaml.safe_dump({
            'map': {},
            'poi': list(pois or []),
            'route': [],
        }, f)


def _valid_pois_payload():
    """`_validate_pois_*` 全 check を通る最小 1 POI body."""
    return [{
        'name': 'p1',
        'pose': {'x': 0.1, 'y': 0.2, 'yaw': 0.0},
        'tolerance': {'xy': 0.5, 'yaw': 0.785},
        'tags': ['waypoint'],
    }]


class TestApiSavePoisVersionConflict(unittest.TestCase):

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.config_path = os.path.join(self._tmp.name, 'cfg.yaml')
        _seed_config(self.config_path)
        self.fake_node = _FakeNode(self.config_path)
        self.app = MapoiWebNode.create_flask_app(self.fake_node)
        self.client = self.app.test_client()

    def _current_version(self):
        from mapoi_webui.yaml_handler import compute_config_version
        return compute_config_version(self.config_path)

    def test_save_with_matching_expected_version_returns_200_with_new_version(self):
        v_before = self._current_version()
        self.assertIsNotNone(v_before)

        resp = self.client.post('/api/pois', json={
            'pois': _valid_pois_payload(),
            'expected_version': v_before,
        })
        self.assertEqual(resp.status_code, 200, resp.get_data(as_text=True))
        body = resp.get_json()
        self.assertTrue(body.get('success'))
        # 保存後 response の config_version は新値 (= save 後の sha256) に更新されている
        self.assertIn('config_version', body)
        self.assertNotEqual(body['config_version'], v_before)
        self.assertEqual(body['config_version'], self._current_version())

    def test_save_with_mismatched_expected_version_returns_409(self):
        # GET 時点と POST 時点の間で外部編集が入ったケースをシミュレート: client は古い
        # version を握ったまま POST するが、backend file は別 client が書き換えて新 version に
        # なっている → 409 で reject されること、code / current_version フィールドを返すこと。
        stale_version = 'a' * 64  # 64 桁の偽 sha256
        self.assertNotEqual(stale_version, self._current_version())

        resp = self.client.post('/api/pois', json={
            'pois': _valid_pois_payload(),
            'expected_version': stale_version,
        })
        self.assertEqual(resp.status_code, 409, resp.get_data(as_text=True))
        body = resp.get_json()
        self.assertEqual(body.get('code'), 'version_mismatch')
        # frontend が直後の reload で受け取るべき current_version を含むこと
        self.assertIn('current_version', body)
        self.assertEqual(body['current_version'], self._current_version())
        # error フィールドにも reload を促す説明文を含むこと (UI banner で表示する想定)
        self.assertIn('reload', str(body.get('error', '')).lower())

    def test_save_without_expected_version_returns_200(self):
        """`expected_version` 省略時は version check をスキップして従来通り保存する
        (旧クライアント / curl / 別 client からの POST 後方互換、PR #245)。
        """
        resp = self.client.post('/api/pois', json={
            'pois': _valid_pois_payload(),
            # expected_version を意図的に省略
        })
        self.assertEqual(resp.status_code, 200, resp.get_data(as_text=True))
        body = resp.get_json()
        self.assertTrue(body.get('success'))
        self.assertIn('config_version', body)


if __name__ == '__main__':
    unittest.main()
