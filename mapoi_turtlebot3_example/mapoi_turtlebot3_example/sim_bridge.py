#!/usr/bin/env python3
"""mapoi_sim_bridge: Gazebo (Classic) entity replacement on map switch.

mapoi_config_path topic を購読し、map_name 変化を検知すると
- 旧マップの障害物 entity を /delete_entity で削除
- 新マップの障害物 entity を /spawn_entity で生成 (model://... URI で)
- ロボット (burger) も /delete_entity + /spawn_entity で initial_pose POI 座標に再生成
する。Gazebo 本体は無停止で /clock, /tf, /odom, /scan の継続性を保つ。

NOTE: 現状 Humble (Gazebo Classic) 専用。Jazzy (gz-sim) 対応は
ros_gz_interfaces 系の service 名・型に切り替える必要があり、#42 と統合した
別 PR で対応予定。

設計メモ: ロボット teleport には本来 /set_entity_state (gazebo_ros_state plugin)
を使うのが筋だが、Humble + Gazebo Classic では plugin の launch arg load が
upstream の既知問題で client から呼べない (gazebo_ros_pkgs issue #1526, #1157,
upstream archived 2025-07)。delete + spawn で代替する。

並列性の扱い: subscribe callback (`_on_config_path`) は受信した msg を queue に
push するだけで即 return し、別 thread の worker が queue から逐次取り出して
service call を含む処理を行う。これにより:
- 連続 switch_map での state 崩れを防ぐ (queue が直列化、lock 不要)
- callback 内で `spin_until_future_complete` を呼ぶことに伴う ROS 2 executor の
  ネスト spin 問題を回避 (worker は executor 外の thread で `Future.add_done_callback`
  + `threading.Event.wait` で結果を待つ)
"""

import math
import os
import queue
import threading

import rclpy
import yaml
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class MapoiSimBridge(Node):
    def __init__(self):
        super().__init__('mapoi_sim_bridge')

        self.declare_parameter('map_obstacle_table_yaml', '')
        self.declare_parameter('robot_entity_name', 'burger')
        self.declare_parameter('robot_sdf_path', '')

        self._table = self._load_table()
        self._robot_name = self.get_parameter(
            'robot_entity_name').get_parameter_value().string_value
        self._robot_sdf_path = self.get_parameter(
            'robot_sdf_path').get_parameter_value().string_value

        # 内部状態 (worker thread のみが触る、callback thread は queue.put のみ)
        self._current_map_name = None
        self._current_obstacle_name = None
        self._current_config_path = None

        # service client は ReentrantCallbackGroup (worker から call_async すれば
        # done callback は executor の別 thread で呼ばれる)
        self._cb_group = ReentrantCallbackGroup()
        self._spawn_cli = self.create_client(
            SpawnEntity, 'spawn_entity', callback_group=self._cb_group)
        self._delete_cli = self.create_client(
            DeleteEntity, 'delete_entity', callback_group=self._cb_group)

        # subscribe callback は queue に push するだけ
        self._work_queue = queue.Queue()
        self._config_path_sub = self.create_subscription(
            String, 'mapoi_config_path', self._on_config_path, 10,
            callback_group=self._cb_group)

        # worker thread (queue を直列消費)
        self._worker_stop = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f'mapoi_sim_bridge initialized: robot={self._robot_name}, '
            f'maps={list(self._table.keys())}')

    def _load_table(self):
        param = self.get_parameter(
            'map_obstacle_table_yaml').get_parameter_value().string_value
        if not param:
            self.get_logger().warn(
                'map_obstacle_table_yaml is empty; sim bridge will skip swaps')
            return {}
        try:
            return yaml.safe_load(param) or {}
        except Exception as e:
            self.get_logger().error(
                f'Failed to parse map_obstacle_table_yaml: {e}')
            return {}

    def _on_config_path(self, msg):
        # 軽量。queue に push して即 return。state 操作と service call は worker。
        self._work_queue.put(msg)

    def _worker_loop(self):
        while not self._worker_stop.is_set():
            try:
                msg = self._work_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            try:
                self._process_config_path(msg)
            except Exception as e:
                self.get_logger().error(
                    f'sim_bridge worker exception: {e}', exc_info=True)

    def _process_config_path(self, msg):
        path = msg.data
        if path == self._current_config_path:
            return
        self._current_config_path = path

        # path の構造: <maps_path>/<map_name>/<config_file>
        map_name = os.path.basename(os.path.dirname(path))
        self.get_logger().info(f'config_path changed → map_name={map_name}')

        if map_name == self._current_map_name:
            return

        prev_map_name = self._current_map_name
        self._current_map_name = map_name

        if prev_map_name is None:
            # 起動時の最初の通知: launch がすでに対応する world で起動済みと仮定し、
            # 内部状態のみ更新。entity 入れ替えはしない。
            entry = self._table.get(map_name)
            if entry:
                self._current_obstacle_name = entry.get('name')
            self.get_logger().info(
                f'Initial map={map_name}; assuming Gazebo already loaded matching world')
            return

        x, y, yaw = self._extract_initial_pose(path)
        self._switch_world(prev_map_name, map_name, x, y, yaw)

    def _extract_initial_pose(self, config_path):
        """mapoi_config.yaml から initial_pose タグ POI の座標を取得。

        見つからなければ (0, 0, 0) を返す。
        """
        try:
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f'Failed to read {config_path}: {e}')
            return 0.0, 0.0, 0.0
        for poi in cfg.get('poi', []) or []:
            if 'initial_pose' in (poi.get('tags') or []):
                pose = poi.get('pose', {})
                return (
                    float(pose.get('x', 0.0)),
                    float(pose.get('y', 0.0)),
                    float(pose.get('yaw', 0.0)),
                )
        self.get_logger().warn(
            f'No initial_pose POI in {config_path}; using (0, 0, 0)')
        return 0.0, 0.0, 0.0

    def _switch_world(self, prev_map, new_map, x, y, yaw):
        prev_entry = self._table.get(prev_map)
        new_entry = self._table.get(new_map)

        if prev_entry and self._current_obstacle_name:
            self._delete_entity(self._current_obstacle_name)
            self._current_obstacle_name = None

        if new_entry:
            self._spawn_entity_from_uri(new_entry['name'], new_entry['uri'])
            self._current_obstacle_name = new_entry['name']
        else:
            self.get_logger().warn(
                f"No map_obstacle_table entry for '{new_map}'; obstacle not spawned")

        self._respawn_robot(x, y, yaw)

    def _call_blocking(self, client, req, timeout_sec=10.0):
        """worker thread から service を blocking call。

        executor の他 thread が future を resolve する前提なので、Multi+Reentrant
        が必要。`spin_until_future_complete` を使わず done_callback + Event.wait
        で待つので、callback thread のネスト spin にならない。
        """
        future = client.call_async(req)
        done_event = threading.Event()
        future.add_done_callback(lambda _f: done_event.set())
        if done_event.wait(timeout=timeout_sec):
            try:
                return future.result()
            except Exception as e:
                self.get_logger().warn(f'service call raised: {e}')
                return None
        return None

    def _delete_entity(self, name):
        if not self._delete_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('delete_entity service not available')
            return
        req = DeleteEntity.Request()
        req.name = name
        res = self._call_blocking(self._delete_cli, req, timeout_sec=5.0)
        if res is None:
            self.get_logger().warn(f'delete_entity({name}) timed out')
        else:
            self.get_logger().info(
                f'delete_entity({name}): success={res.success}')

    def _spawn_entity_from_uri(self, name, uri):
        if not self._spawn_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('spawn_entity service not available')
            return
        sdf = (
            '<?xml version="1.0"?>'
            '<sdf version="1.6"><world name="default">'
            f'<include><uri>{uri}</uri></include>'
            '</world></sdf>'
        )
        req = SpawnEntity.Request()
        req.name = name
        req.xml = sdf
        res = self._call_blocking(self._spawn_cli, req, timeout_sec=10.0)
        if res is None:
            self.get_logger().warn(f'spawn_entity({name}) timed out')
        else:
            self.get_logger().info(
                f'spawn_entity({name}, {uri}): success={res.success}')

    def _respawn_robot(self, x, y, yaw):
        """ロボットを delete + spawn で新位置に再生成。

        set_entity_state の代替 (上記 NOTE 参照)。/odom の連続性は失われ、AMCL は
        再 collapse が必要だが、本 PoC のスコープでは許容 (PR #32 の auto initial_pose
        publish が補正する想定)。

        preflight (SDF 存在 + spawn service ready + SDF 読込) を delete より先に
        やる。これにより spawn 不能なときに robot を消したまま復旧不能、を避ける。
        """
        # 1. preflight: 失敗するなら delete もしない
        if not self._robot_sdf_path:
            self.get_logger().warn(
                'robot_sdf_path parameter is empty; skipping respawn (robot stays at old pose)')
            return
        if not os.path.exists(self._robot_sdf_path):
            self.get_logger().error(
                f'robot_sdf_path not found: {self._robot_sdf_path}; skipping respawn')
            return
        if not self._spawn_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                'spawn_entity service not available; skipping respawn (robot stays at old pose)')
            return
        try:
            with open(self._robot_sdf_path, 'r') as f:
                sdf = f.read()
        except Exception as e:
            self.get_logger().error(
                f'Failed to read robot SDF ({self._robot_sdf_path}): {e}; skipping respawn')
            return

        # 2. delete (preflight 通過後にのみ実行)
        self._delete_entity(self._robot_name)

        # 3. spawn
        req = SpawnEntity.Request()
        req.name = self._robot_name
        req.xml = sdf
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = 0.01
        req.initial_pose.orientation.z = math.sin(yaw / 2.0)
        req.initial_pose.orientation.w = math.cos(yaw / 2.0)
        res = self._call_blocking(self._spawn_cli, req, timeout_sec=10.0)
        if res is None:
            self.get_logger().warn(
                f'respawn_robot({self._robot_name}) timed out (robot may be deleted)')
        else:
            self.get_logger().info(
                f'respawn_robot({self._robot_name}, {x:.2f}, {y:.2f}, yaw={yaw:.2f}): '
                f'success={res.success}')


def main(args=None):
    rclpy.init(args=args)
    node = MapoiSimBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._worker_stop.set()
        node._worker.join(timeout=2.0)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
