"""mapoi_server crash/restart の initial pose 再 latch (#297) の E2E launch_test.

#297 修正 (state_path による last-selected map 永続化) 後の 2 側面を pin する:

1. **state_path あり (= #297 修正の期待挙動)**: 再起動した mapoi_server は state file から
   「運用中再起動」を判別し、last-selected map へ context を復元した上で clear
   (poi_name 空) のみを publish する。稼働中 subscriber へ非空 POI の active push は
   届かない (PR #326 時点の characterization assert を反転したもの)。
2. **state_path なし (= opt-out、default)**: 従来挙動のまま。再起動は稼働中 subscriber へ
   起動パラメータ map の先頭 POI を active push する。永続化を有効にしない構成では
   #297 の挙動が残ることを明示的に文書化する pin。

本 test の長寿命 subscription は「一度も再起動していない稼働中の amcl bridge」の
stand-in。server プロセスの kill / 再起動を test 本体が直接制御するため、server は
launch description ではなく subprocess で起動する。
"""

import os
import signal
import subprocess
import tempfile
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
from mapoi_interfaces.srv import GetMapsInfo, RequestInitialPose, SelectMap


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
    """mapoi_server 再起動 → 稼働中 subscriber への配信挙動 (#297) を両構成で pin する。"""

    SPIN_TIMEOUT = 20.0
    # 「届かないこと」を観測する窓。clear 受信 (正の同期点) の後に追加で spin する時間。
    QUIET_WINDOW = 3.0

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
        cls.get_maps_info_client = cls.node.create_client(GetMapsInfo, 'get_maps_info')
        cls.server_proc = None

    @classmethod
    def tearDownClass(cls):
        cls._stop_server()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _start_server(cls, state_path=None):
        # 起動パラメータは常に map A (= #297 の「crash/restart 後は起動パラメータの map に
        # 巻き戻る」を成立させる条件)。state_path は #297 永続化の opt-in (None = 従来挙動)。
        args = [
            cls.server_exe, '--ros-args',
            '-p', f'maps_path:={cls.maps_dir}',
            '-p', 'map_name:=test_map_a',
            '-p', 'config_file:=mapoi_config.yaml',
        ]
        if state_path is not None:
            args += ['-p', f'state_path:={state_path}']
        cls.server_proc = subprocess.Popen(args)

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

    def _spin_for(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _call(self, client, request):
        self.assertTrue(client.wait_for_service(timeout_sec=self.SPIN_TIMEOUT),
                        f'{client.srv_name} service が上がらない')
        future = client.call_async(request)
        self.assertTrue(self._spin_until(future.done),
                        f'{client.srv_name} が応答しない')
        return future.result()

    def _run_operation_then_restart(self, samples, state_path=None):
        """初回起動 → map B へ切替 → crash 相当 kill → 同一パラメータで再起動、まで進める。

        戻り値: 再起動直前の samples 長 (以降の slice が「再起動後に届いた新規サンプル」)。
        """
        # (1) 初回起動: 起動時 publish (#144 の意図された初回挙動) を受信する。
        #     state_path ありでも state file が無い真の初回起動では従来どおり publish される。
        self._start_server(state_path=state_path)
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
        self._start_server(state_path=state_path)
        return n_before_restart

    def test_restart_with_state_path_restores_map_and_pushes_clear_only(self):
        # 稼働中 amcl bridge の stand-in: server の再起動をまたいで生き続ける subscription。
        samples = []
        sub = self.node.create_subscription(
            InitialPoseRequest, 'mapoi/initialpose_poi',
            lambda msg: samples.append((msg.map_name, msg.poi_name)),
            _transient_local_qos(10))
        state_path = tempfile.mkdtemp(prefix='mapoi_state_')
        try:
            n_before_restart = self._run_operation_then_restart(
                samples, state_path=state_path)

            # state file に最後に選択した map B が残っていること (select_map 時に永続化され、
            # 再起動後の startup 書き直しでも restore した B が維持される)。restore 判定の
            # 入力/出力を file 側からも直接確認する。
            with open(os.path.join(state_path, 'last_selected_map')) as f:
                self.assertEqual(f.read().strip(), 'test_map_b')

            # (4) 【#297 修正の期待挙動】再起動した server は state file から「運用中再起動」を
            #     判別し、restore した map B を map_name に載せた clear のみを publish する。
            self.assertTrue(
                self._spin_until(
                    lambda: ('test_map_b', '') in samples[n_before_restart:]),
                '再起動後の clear publish (restore した map B 由来) が受信できない')

            # clear 受信後も QUIET_WINDOW の間 spin し、非空 poi_name のサンプルが
            # 1 件も届かないこと (= 旧 characterization assert の反転) を確認する。
            self._spin_for(self.QUIET_WINDOW)
            non_empty = [s for s in samples[n_before_restart:] if s[1] != '']
            self.assertEqual(
                non_empty, [],
                f'再起動後に非空 POI が稼働中 subscriber へ届いた (#297 再発): {non_empty}')

            # (5) map context も B に復元されていること (initial pose は #297 の症状の一つで、
            #     restart による context 巻き戻りは goal/route 解決も壊すため独立に確認する)。
            maps_info = self._call(self.get_maps_info_client, GetMapsInfo.Request())
            self.assertEqual(maps_info.map_name, 'test_map_b')
        finally:
            self.node.destroy_subscription(sub)
            self._stop_server()

    def test_restart_without_state_path_still_pushes_startup_poi(self):
        # state_path なし (default) の opt-out 挙動を pin する: 再起動した server の起動時
        # publish {map A, 先頭 POI} が稼働中 subscriber に新規サンプルとして届く (従来挙動)。
        # 永続化を設定しない構成では #297 の active push が残ることの明示的な文書化。
        samples = []
        sub = self.node.create_subscription(
            InitialPoseRequest, 'mapoi/initialpose_poi',
            lambda msg: samples.append((msg.map_name, msg.poi_name)),
            _transient_local_qos(10))
        try:
            n_before_restart = self._run_operation_then_restart(samples, state_path=None)
            self.assertTrue(
                self._spin_until(
                    lambda: ('test_map_a', 'map_a_start') in samples[n_before_restart:]),
                'state_path なしの再起動で active push が観測されなかった: opt-out 時は従来挙動の'
                'はずで、default 挙動を変えたなら本 test と README の記述を併せて更新すること')
        finally:
            self.node.destroy_subscription(sub)
            self._stop_server()
