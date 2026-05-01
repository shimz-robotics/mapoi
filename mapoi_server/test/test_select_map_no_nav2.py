import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers

import rclpy
from ament_index_python.packages import get_package_share_directory
from mapoi_interfaces.srv import SelectMap, GetMapsInfo


@launch_testing.markers.keep_alive
def generate_test_description():
    pkg_share = get_package_share_directory('mapoi_server')
    maps_path = os.path.join(pkg_share, 'test', 'select_map_maps')

    mapoi_server_node = launch_ros.actions.Node(
        package='mapoi_server',
        executable='mapoi_server',
        name='mapoi_server',
        parameters=[{
            'maps_path': maps_path,
            'map_name': 'map_a',
            'config_file': 'mapoi_config.yaml',
        }],
    )

    return launch.LaunchDescription([
        mapoi_server_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'mapoi_server': mapoi_server_node}


class TestSelectMapNoNav2(unittest.TestCase):
    """select_map は mapoi_server の context 更新だけを行い、Nav2 service を待たない。"""

    SPIN_TIMEOUT = 5.0

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_select_map_no_nav2_node')
        cls.select_map_client = cls.node.create_client(SelectMap, 'select_map')
        cls.get_maps_client = cls.node.create_client(GetMapsInfo, 'get_maps_info')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call_select_map(self, map_name):
        req = SelectMap.Request()
        req.map_name = map_name
        start = time.monotonic()
        future = self.select_map_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.SPIN_TIMEOUT)
        elapsed = time.monotonic() - start
        return future.result(), elapsed

    def test_select_map_returns_without_nav2(self):
        self.assertTrue(
            self.select_map_client.wait_for_service(timeout_sec=self.SPIN_TIMEOUT),
            'select_map service did not become available')

        response, elapsed = self._call_select_map('map_b')
        self.assertIsNotNone(response, 'select_map call did not complete')
        self.assertTrue(response.success, response.error_message)
        self.assertLess(elapsed, 2.0, 'select_map should not wait for Nav2 load_map services')
        self.assertTrue(response.config_path.endswith('/map_b/mapoi_config.yaml'))
        self.assertEqual(response.initial_poi_name, 'b_start')
        self.assertEqual(list(response.nav2_node_names), ['/missing_map_server'])
        self.assertEqual(len(response.nav2_map_urls), 1)
        self.assertTrue(response.nav2_map_urls[0].endswith('/map_b/map_b.yaml'))

        self.assertTrue(
            self.get_maps_client.wait_for_service(timeout_sec=self.SPIN_TIMEOUT),
            'get_maps_info service did not become available')
        future = self.get_maps_client.call_async(GetMapsInfo.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.SPIN_TIMEOUT)
        maps_response = future.result()
        self.assertIsNotNone(maps_response)
        self.assertEqual(maps_response.map_name, 'map_b')
