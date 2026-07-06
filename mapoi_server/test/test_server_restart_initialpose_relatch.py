"""mapoi_server crash/restart による initial pose 再 latch (#297) の再現 launch_test.

現状の bug 挙動を characterization として pin する:
mapoi_server の再起動は新しい DDS publisher の出現なので、constructor の起動時
publish (#144) が「後起動 subscriber」だけでなく **再起動していない稼働中の
subscriber にも新規サンプルとして届く** (active push)。operator が map B へ切替済みでも、
再起動した mapoi_server は起動パラメータの map A の先頭 POI を再 publish し、稼働中の
mapoi_amcl_localization_bridge 相当の subscriber へ即座に配信される。

本 test の長寿命 subscription は「一度も再起動していない稼働中の amcl bridge」の
stand-in。server プロセスの kill / 再起動を test 本体が直接制御するため、server は
launch description ではなく subprocess で起動する。

**注意 (#297 修正 PR への申し送り)**: `test_restart_pushes_startup_poi_to_running_subscriber`
の最終 assert は現状の bug 挙動 (非空 POI の active push) を pin している。#297 修正後は
この assert を反転させること (期待挙動: 再起動後に稼働中 subscriber へ非空 poi_name の
sample が届かない。clear のみ、または publish なし)。
"""

import os
import signal
import subprocess
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from mapoi_interfaces.msg import InitialPoseRequest
from mapoi_interfaces.srv import RequestInitialPose, SelectMap


@launch_testing.markers.keep_alive
def generate_test_description():
    # server プロセスは test 本体が subprocess で管理する (kill / 再起動を制御するため)。
    return launch.LaunchDescription([
        launch_testing.actions.ReadyToTest(),
    ]), {}


def _transient_local_qos(depth):
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


class TestServerRestartInitialposeRelatch(unittest.TestCase):
    """mapoi_server 再起動 → 稼働中 subscriber への initial pose 再配信 (#297) を pin する。"""

    SPIN_TIMEOUT = 20.0

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_server_restart_node')
        cls.maps_dir = os.path.join(
            get_package_share_directory('mapoi_server'), 'test', 'maps')
        cls.server_exe = os.path.join(
            get_package_prefix('mapoi_server'), 'lib', 'mapoi_server', 'mapoi_server')
        cls.select_map_client = cls.node.create_client(SelectMap, 'select_map')
        cls.request_initialpose_client = cls.node.create_client(
            RequestInitialPose, 'request_initial_pose')
        cls.server_proc = None

    @classmethod
    def tearDownClass(cls):
        cls._stop_server()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _start_server(cls):
        # 起動パラメータは常に map A (= #297 の「crash/restart 後は起動パラメータの map に
        # 巻き戻る」を成立させる条件)。
        cls.server_proc = subprocess.Popen([
            cls.server_exe, '--ros-args',
            '-p', f'maps_path:={cls.maps_dir}',
            '-p', 'map_name:=test_map_a',
            '-p', 'config_file:=mapoi_config.yaml',
        ])

    @classmethod
    def _stop_server(cls):
        if cls.server_proc is not None and cls.server_proc.poll() is None:
            # crash 相当の即死 (SIGKILL)。graceful shutdown だと DDS の unregister が走るが、
            # #297 の対象は crash / 強制再起動なので kill で揃える。
            cls.server_proc.send_signal(signal.SIGKILL)
            cls.server_proc.wait(timeout=10.0)
        cls.server_proc = None

    def _spin_until(self, predicate, timeout=None):
        timeout = timeout if timeout is not None else self.SPIN_TIMEOUT
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _call(self, client, request):
        self.assertTrue(client.wait_for_service(timeout_sec=self.SPIN_TIMEOUT),
                        f'{client.srv_name} service が上がらない')
        future = client.call_async(request)
        self.assertTrue(self._spin_until(future.done),
                        f'{client.srv_name} が応答しない')
        return future.result()

    def test_restart_pushes_startup_poi_to_running_subscriber(self):
        # 稼働中 amcl bridge の stand-in: server の再起動をまたいで生き続ける subscription。
        samples = []
        sub = self.node.create_subscription(
            InitialPoseRequest, 'mapoi/initialpose_poi',
            lambda msg: samples.append((msg.map_name, msg.poi_name)),
            _transient_local_qos(10))
        try:
            # (1) 初回起動: 起動時 publish (#144 の意図された初回挙動) を受信する。
            self._start_server()
            self.assertTrue(
                self._spin_until(
                    lambda: ('test_map_a', 'map_a_start') in samples),
                '初回起動の initialpose publish が受信できない')

            # (2) operator の map B への切替を simulate: select_map (clear が publish される)
            #     → request_initial_pose (nav2_bridge が LoadMap 成功後に叩くのと同じ service)。
            select_result = self._call(
                self.select_map_client, SelectMap.Request(map_name='test_map_b'))
            self.assertTrue(select_result.success)
            self.assertEqual(select_result.initial_poi_name, 'map_b_start')
            request = RequestInitialPose.Request(
                map_name='test_map_b', poi_name='map_b_start')
            self.assertTrue(self._call(self.request_initialpose_client, request).success)
            self.assertTrue(
                self._spin_until(
                    lambda: ('test_map_b', 'map_b_start') in samples),
                'map B への切替後の initialpose publish が受信できない')

            # (3) crash 相当の kill → 同一起動パラメータ (map A) で再起動。
            n_before_restart = len(samples)
            self._stop_server()
            self._start_server()

            # (4) 【#297 bug 挙動の pin】稼働中 subscriber (本 subscription) に、再起動した
            #     server の起動時 publish {map A, 先頭 POI} が新規サンプルとして届いてしまう。
            #     現在 map は B のまま = 別 map の initial pose が active push される。
            #     #297 修正後はこの assert を反転させること (ファイル冒頭 docstring 参照)。
            self.assertTrue(
                self._spin_until(
                    lambda: ('test_map_a', 'map_a_start') in samples[n_before_restart:]),
                '再起動後の active push が観測されなかった: #297 の前提が変わった可能性があり、'
                '修正が入ったなら本 test を期待挙動 (非空 POI が届かない) に反転させること')
        finally:
            self.node.destroy_subscription(sub)
            self._stop_server()
