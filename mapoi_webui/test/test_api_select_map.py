"""Flask endpoint level test for `POST /api/maps/select` (#343).

SelectMap.srv の response フィールド rename (`initial_poi_name` →
`resolved_initial_poi_name`, #343) を REST JSON key レベルでも pin する。
srv 側の rename は C++ の再コンパイルで検知されるが、REST key は文字列なので
リファクタで旧名に戻っても CI で気づけない — この test がその regression を塞ぐ。

`test_api_nav_endpoints.py` と同じく `create_flask_app` を duck-typed
`_FakeNode` に bind し、`_call_service_sync` を差し替えて service 応答を偽装する。
"""
import unittest
from types import SimpleNamespace

from mapoi_webui.mapoi_webui_node import MapoiWebNode


class _NullLogger:
    def info(self, *_a, **_k):
        pass

    def warn(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass


class _FakeNode:
    """select_map endpoint 経路で参照される最小限の attr / method を持つ duck-typed node."""

    def __init__(self, *, service_response):
        self._service_response = service_response
        self.select_map_client_ = object()
        self.map_name_ = 'old_map'

    def _call_service_sync(self, _client, _req, _name, timeout_sec):
        return self._service_response

    def get_logger(self):
        return _NullLogger()


def _client(node):
    app = MapoiWebNode.create_flask_app(node)
    return app.test_client()


class TestApiSelectMap(unittest.TestCase):

    def test_success_returns_resolved_initial_poi_name_key(self):
        """成功時の JSON key は resolved_initial_poi_name (#343 rename の REST 側 pin)。"""
        node = _FakeNode(service_response=SimpleNamespace(
            success=True, error_message='', config_path='/maps/m/mapoi_config.yaml',
            resolved_initial_poi_name='poi_start'))
        resp = _client(node).post('/api/maps/select', json={'map_name': 'm'})
        self.assertEqual(resp.status_code, 200, resp.get_data(as_text=True))
        payload = resp.get_json()
        self.assertEqual(payload['resolved_initial_poi_name'], 'poi_start')
        self.assertNotIn('initial_poi_name', payload, '旧 key が復活している (#343 regression)')
        self.assertEqual(node.map_name_, 'm')

    def test_service_failure_returns_400_with_error_message(self):
        node = _FakeNode(service_response=SimpleNamespace(
            success=False, error_message='boom', config_path='',
            resolved_initial_poi_name=''))
        resp = _client(node).post('/api/maps/select', json={'map_name': 'm'})
        self.assertEqual(resp.status_code, 400)
        self.assertEqual(resp.get_json()['error'], 'boom')
        self.assertEqual(node.map_name_, 'old_map', '失敗時に map_name_ が更新されている')

    def test_service_unavailable_returns_503(self):
        node = _FakeNode(service_response=None)
        resp = _client(node).post('/api/maps/select', json={'map_name': 'm'})
        self.assertEqual(resp.status_code, 503)

    def test_missing_map_name_returns_400(self):
        node = _FakeNode(service_response=None)
        resp = _client(node).post('/api/maps/select', json={})
        self.assertEqual(resp.status_code, 400)


if __name__ == '__main__':
    unittest.main()
