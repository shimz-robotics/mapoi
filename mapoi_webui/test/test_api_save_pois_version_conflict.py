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
    """`/api/pois` 経路で参照される最小限の attr / method だけを持つ duck-typed node.

    `call_reload_map_info()` は常に True を返す: 本 test は競合検出 / 保存契約を pin する
    のが目的で、reload 失敗時の挙動 (200 + warning フィールド) は scope 外 (`/api/pois` の
    別 follow-up で扱う、PR #253 round 2 review medium #3 メモ)。
    """

    def __init__(self, config_path):
        self._config_path = config_path

    def get_config_path(self, _map_name=None):
        return self._config_path

    def call_reload_map_info(self):
        return True

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

        payload = _valid_pois_payload()
        resp = self.client.post('/api/pois', json={
            'pois': payload,
            'expected_version': v_before,
        })
        self.assertEqual(resp.status_code, 200, resp.get_data(as_text=True))
        body = resp.get_json()
        self.assertTrue(body.get('success'))
        # 保存後 response の config_version は新値 (= save 後の sha256) に更新されている
        self.assertIn('config_version', body)
        self.assertNotEqual(body['config_version'], v_before)
        self.assertEqual(body['config_version'], self._current_version())
        # 実際にディスクの yaml が新 payload で書き換わっていることを直接確認
        # (version 系 assert だけだと「config_version は更新したが pois は書かない」回帰を
        # 検知できない、PR #253 round 2 review medium #1 対応)。
        with open(self.config_path, 'r') as f:
            saved = yaml.safe_load(f)
        saved_pois = saved.get('poi') or []
        self.assertEqual(len(saved_pois), 1)
        self.assertEqual(saved_pois[0]['name'], payload[0]['name'])

    def test_save_with_mismatched_expected_version_returns_409_without_writing(self):
        # GET 時点と POST 時点の間で外部編集が入ったケースをシミュレート: client は古い
        # version を握ったまま POST するが、backend file は別 client が書き換えて新 version に
        # なっている → 409 で reject されること、code / current_version フィールドを返すこと、
        # かつ **ディスク上の yaml が一切変更されないこと** (= 競合時に誤保存しない契約) を pin。
        # status_code だけだと「409 返したのに裏で save_pois も呼んでた」回帰を検知できないため、
        # PR #253 round 1 review medium #1 対応で content / version の不変も併せて assert する。
        v_before = self._current_version()
        with open(self.config_path, 'rb') as f:
            content_before = f.read()
        stale_version = 'a' * 64  # 64 桁の偽 sha256 (任意の不一致値)
        self.assertNotEqual(stale_version, v_before)

        resp = self.client.post('/api/pois', json={
            'pois': _valid_pois_payload(),
            'expected_version': stale_version,
        })
        self.assertEqual(resp.status_code, 409, resp.get_data(as_text=True))
        body = resp.get_json()
        self.assertEqual(body.get('code'), 'version_mismatch')
        # frontend が直後の reload で受け取るべき current_version を含むこと
        self.assertIn('current_version', body)
        self.assertEqual(body['current_version'], v_before)
        # error フィールドにも reload を促す説明文を含むこと (UI banner で表示する想定)
        self.assertIn('reload', str(body.get('error', '')).lower())

        # 競合時に save_pois が呼ばれていないこと: version 不変 + byte 単位で content 不変
        self.assertEqual(self._current_version(), v_before)
        with open(self.config_path, 'rb') as f:
            self.assertEqual(f.read(), content_before)

    def test_save_without_expected_version_returns_200(self):
        """`expected_version` 省略時は version check をスキップして従来通り保存する
        (旧クライアント / curl / 別 client からの POST 後方互換、PR #245)。
        """
        v_before = self._current_version()
        resp = self.client.post('/api/pois', json={
            'pois': _valid_pois_payload(),
            # expected_version を意図的に省略
        })
        self.assertEqual(resp.status_code, 200, resp.get_data(as_text=True))
        body = resp.get_json()
        self.assertTrue(body.get('success'))
        self.assertIn('config_version', body)
        # 実際に保存された証拠として version が変わっていること (config_version 存在だけだと
        # 「response 形だけ整えて中身は書いていない」を検知できない、round 1 review low)。
        self.assertNotEqual(body['config_version'], v_before)
        self.assertEqual(body['config_version'], self._current_version())

    def test_save_with_explicit_null_expected_version_returns_200(self):
        """`expected_version: null` を明示送信した場合も省略時と同じ後方互換挙動 (200 OK + 保存)
        になることを pin。README には「省略 (null / 不在)」と書いており、frontend が
        `JSON.stringify({expected_version: null})` 経路でも壊れないことを保証する
        (round 1 review medium #2 対応)。
        """
        v_before = self._current_version()
        resp = self.client.post('/api/pois', json={
            'pois': _valid_pois_payload(),
            'expected_version': None,
        })
        self.assertEqual(resp.status_code, 200, resp.get_data(as_text=True))
        body = resp.get_json()
        self.assertTrue(body.get('success'))
        self.assertNotEqual(body.get('config_version'), v_before)
        self.assertEqual(body['config_version'], self._current_version())

    def test_save_with_explicit_empty_string_expected_version_returns_409(self):
        """`expected_version: ""` (空文字) は省略扱いではなく「明示送信された不一致値」として
        409 になることを pin (round 2 review medium #2 対応)。

        実装は `data.get('expected_version')` の戻り値を `is not None` でしか除外しないため、
        `""` は check に入り、必ず current sha256 (64 桁 hex) と不一致になる。空文字を
        「省略と同じ」と扱う実装にすると意図しない競合バイパス経路ができるため、現実装の
        「明示送信は常に check」契約を test で固定する。
        """
        resp = self.client.post('/api/pois', json={
            'pois': _valid_pois_payload(),
            'expected_version': '',
        })
        self.assertEqual(resp.status_code, 409, resp.get_data(as_text=True))
        body = resp.get_json()
        self.assertEqual(body.get('code'), 'version_mismatch')


if __name__ == '__main__':
    unittest.main()
