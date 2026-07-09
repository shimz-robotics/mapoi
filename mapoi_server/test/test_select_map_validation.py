"""select_map の map_name 入力検証 launch_test (#328).

`select_map_service` は operator UI (WebUI / RViz panel) からの service 入力である
`request.map_name` を maps_path に path 連結する。separator や ".." を含む名前を
そのまま連結すると maps_path 外の任意 config YAML の load と、`nav2_map_urls` 経由で
Nav2 map_server の LoadMap 先も maps_path 外を指せてしまう (state file 側は PR #327 で
`is_plain_directory_name` により対応済みの同種 path traversal)。

検証する shape:
- traversal 名 (`../maps/test_map_b`): 連結結果は実在 config に解決される
  (= 修正前は success=true になっていた入力) が、名前検証で reject され
  current map も変わらない。
- plain 名 (`test_map_b`): 従来どおり成功する (検証追加の regression がない)。
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers

import rclpy
from ament_index_python.packages import get_package_share_directory
from mapoi_interfaces.srv import GetMapsInfo, SelectMap


@launch_testing.markers.keep_alive
def generate_test_description():
    pkg_share = get_package_share_directory('mapoi_server')
    maps_dir = os.path.join(pkg_share, 'test', 'maps')

    mapoi_server_node = launch_ros.actions.Node(
        package='mapoi_server',
        executable='mapoi_server',
        name='mapoi_server',
        parameters=[{
            'maps_path': maps_dir,
            'map_name': 'test_map_a',
            'config_file': 'mapoi_config.yaml',
        }],
    )

    return launch.LaunchDescription([
        mapoi_server_node,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'mapoi_server': mapoi_server_node,
    }


class TestSelectMapValidation(unittest.TestCase):
    """select_map の map_name 検証 (#328) を service 経路で pin する。"""

    SPIN_TIMEOUT = 10.0

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_select_map_validation_node')
        cls.select_map_client = cls.node.create_client(SelectMap, 'mapoi/select_map')
        cls.get_maps_info_client = cls.node.create_client(GetMapsInfo, 'mapoi/get_maps_info')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request):
        self.assertTrue(client.wait_for_service(timeout_sec=self.SPIN_TIMEOUT),
                        f'{client.srv_name} service did not become available')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.SPIN_TIMEOUT)
        result = future.result()
        self.assertIsNotNone(result, f'{client.srv_name} call did not complete')
        return result

    def _current_map(self):
        return self._call(self.get_maps_info_client, GetMapsInfo.Request()).map_name

    def test_a_traversal_map_name_is_rejected(self):
        """separator / '..' を含む map_name は、連結結果が実在 config に解決されても
        reject され、current map も変わらない。"""
        self.assertEqual(self._current_map(), 'test_map_a')

        # maps_path が .../test/maps なので '../maps/test_map_b' の連結結果は
        # 実在する test_map_b の config に解決される = 名前検証がなければ通ってしまう入力。
        result = self._call(
            self.select_map_client,
            SelectMap.Request(map_name='../maps/test_map_b'))
        self.assertFalse(
            result.success,
            'separator を含む map_name が reject されない (#328 regression)')
        self.assertNotEqual(result.error_message, '', 'reject 時の error_message が空')

        self.assertEqual(
            self._current_map(), 'test_map_a',
            'reject されたのに current map が変わっている')

    def test_b_plain_map_name_still_works(self):
        """plain な directory 名の select_map は従来どおり成功する。"""
        result = self._call(
            self.select_map_client, SelectMap.Request(map_name='test_map_b'))
        self.assertTrue(
            result.success,
            f'plain な map_name の select_map が失敗: {result.error_message}')
        self.assertEqual(self._current_map(), 'test_map_b')
