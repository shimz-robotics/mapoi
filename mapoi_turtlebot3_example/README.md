# mapoi_turtlebot3_example

TurtleBot3 シミュレーション環境で mapoi の動作を確認するためのサンプルパッケージです。
SLAM による地図作成から Navigation2 を用いた自律走行までの一連の手順を試すことができます。

## 依存パッケージのインストール

rosdep を使用する場合:

```sh
# cd path/to/your_ws/
rosdep install --from-paths src --ignore-src -r -y
```

手動でインストールする場合:

```sh
sudo apt install -y \
  ros-${ROS_DISTRO}-turtlebot3-simulations \
  ros-${ROS_DISTRO}-turtlebot3-cartographer \
  ros-${ROS_DISTRO}-turtlebot3-navigation2 \
  ros-${ROS_DISTRO}-mouse-teleop
```

## クイックスタート

```sh
export TURTLEBOT3_MODEL=burger
ros2 launch mapoi_turtlebot3_example turtlebot3_navigation.launch.yaml
```

別ターミナルから POI への自律走行をテスト:

```sh
ros2 topic pub -1 /mapoi_goal_pose_poi std_msgs/msg/String "{data: conference_room}"
```

### Headless 起動（GUI なし）

CI / SSH 接続環境 / 軽量検証用に RViz2・Gazebo GUI を起動しない形で立ち上げられます。WebUI (http://localhost:8765) は従来どおり使用可能です。

```sh
ros2 launch mapoi_turtlebot3_example turtlebot3_navigation.launch.yaml \
  rviz:=false gazebo_gui:=false
```

個別制御も可能:

| arg | default | 効果 |
|---|---|---|
| `rviz` | `true` | RViz2 の起動有無 |
| `gazebo_gui` | `true` | シミュレータ GUI の起動有無。`false` でも server (Humble: `gzserver` / Jazzy: `gz sim -s`) は起動し物理シミュレーションは動作 |

`gazebo_gui` は Humble (Gazebo Classic) と Jazzy (gz-sim) のどちらでも有効です。distro 別の wrapper (`gazebo_headless_aware.launch.yaml` / `gz_sim_headless_aware.launch.yaml`) を `turtlebot3_navigation.launch.yaml` から `ROS_DISTRO` で使い分けています。`rviz:=false` は distro によらず動作。

## 地図の作成（SLAM）

### 1. シミュレーターの起動

```sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 2. Cartographer の起動

```sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### 3. ロボットの操作

```sh
ros2 run mouse_teleop mouse_teleop --ros-args -r /mouse_vel:=/cmd_vel
```

マウスでロボットを操作して環境全体を走行させ、地図を完成させます。

### 4. 地図の保存

```sh
ros2 run nav2_map_server map_saver_cli -f maps/<地図名>/<地図名>
```

### 5. 設定ファイルの編集

保存した地図ディレクトリに `mapoi_config.yaml` を作成し、POI とルートを設定します。
フォーマットの詳細は [mapoi_server の README](../mapoi_server/README.md) を参照してください。

## サンプルクライアントノード

mapoi のサービス・アクションの使用例として、以下のクライアントノードが含まれています。

| ノード名 | 説明 |
| --- | --- |
| `get_maps_info_client` | 地図一覧の取得 |
| `get_pois_info_client` | POI 一覧の取得 |
| `mapoi_switch_map_client` | `/mapoi_switch_map` による operator 地図切り替え |
| `navigate_to_pose_client` | 指定位置へのナビゲーション |
| `follow_waypoints_client` | ウェイポイント追従 |
| `print_initialpose` | 初期位置の表示 |

実行例:

```sh
ros2 run mapoi_turtlebot3_example get_maps_info_client
ros2 run mapoi_turtlebot3_example get_pois_info_client
```

## サンプル地図

シナリオに即した POI 配置 + tag 組合せの全パターンをカバーする 2 サンプルを同梱しています (#146)。

| 地図名 | シナリオ | 検証できる主な機能 |
| --- | --- | --- |
| `turtlebot3_world` | オフィス見学ツアー (見学客を案内、要所で撮影 / 音声ガイド / 自動停止) | system tag 単独 / 複合、`tolerance.yaw` 厳密 ↔ 緩い、`route.waypoints + landmarks` 両方持ち、連続 `pause`、撮影 / 音声 / 観察対象の custom tag |
| `turtlebot3_dqn_stage1` | 障害物 sandbox での回避走行 (ハザード / 観察 / 通過点) | 複合 tag (`event + landmark + custom`)、`tolerance.yaw=π` で yaw 不問の通過点 (= pass_through 代替)、`pause` 中継、`route.landmarks` の利用 |

### POI / tag 構成

#### turtlebot3_world (route: `tour_full` / `tour_short`)

| POI | tags | 役割 |
| --- | --- | --- |
| `elevator_hall` | `waypoint` | POI list 先頭 = default initial pose (#149) |
| `meeting_room_a` | `waypoint` | 普通の経由点 |
| `conference_room` | `waypoint`, `capture_trigger` | 撮影地点、`tolerance.yaw=0.10` で厳密 |
| `corridor_a` | `waypoint`, `pause` | 連続 `pause` #1、`tolerance.yaw=π` で通り過ぎ |
| `corridor_b` | `waypoint`, `pause` | 連続 `pause` #2 (`tour_full` で連続 pause 発火を検証) |
| `model_exhibit` | `waypoint`, `audio_info` | 音声ガイド再生 (waypoint + custom tag) |
| `tour_landmark` | `landmark` | 純粋 landmark (tag 単独) |
| `exhibit_display` | `landmark`, `audio_info` | 観察対象でガイド再生 |
| `capture_target_painting` | `landmark`, `capture_target` | 撮影アクションの対象として参照 |

#### turtlebot3_dqn_stage1 (route: `avoidance_a` / `avoidance_b`)

| POI | tags | 役割 |
| --- | --- | --- |
| `start_zone` | `waypoint` | POI list 先頭 = default initial pose (#149) |
| `checkpoint_west` | `waypoint`, `checkpoint` | yaw 不問の通過点 (`tolerance.yaw=π`) |
| `checkpoint_east` | `waypoint`, `checkpoint` | yaw 不問の通過点 (`tolerance.yaw=π`) |
| `pause_intersection` | `waypoint`, `pause` | 交差点で自動停止 |
| `north_goal` | `waypoint` | 北側目標、`tolerance.yaw=0.10` で厳密 |
| `hazard_south` | `event`, `landmark`, `hazard` | 複合 tag のハザード (Nav2 goal 不可) |
| `observation_point` | `landmark`, `observation` | 観察対象 (route 走行中の参照点) |

> **NOTE**: `landmark × pause` は #143 validation で reject されるため組み合わせていません。

参考: [TurtleBot3 E-Manual (Simulation)](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
