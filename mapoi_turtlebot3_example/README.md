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
ros2 launch mapoi_turtlebot3_example turtlebot3_navigation_launch.yaml
```

別ターミナルから POI への自律走行をテスト:

```sh
ros2 topic pub -1 /mapoi_goal_pose_poi std_msgs/msg/String "{data: conference_room}"
```

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
| `get_map_info_client` | 地図情報の取得 |
| `get_tagged_pois_client` | タグ指定による POI の取得 |
| `switch_map_client` | 地図の切り替え |
| `navigate_to_pose_client` | 指定位置へのナビゲーション |
| `follow_waypoints_client` | ウェイポイント追従 |
| `print_initialpose` | 初期位置の表示 |

実行例:

```sh
ros2 run mapoi_turtlebot3_example get_map_info_client
ros2 run mapoi_turtlebot3_example get_tagged_pois_client
```

## サンプル地図

| 地図名 | 説明 |
| --- | --- |
| `turtlebot3_world` | TurtleBot3 標準ワールドの地図（POI・ルート設定済み） |
| `turtlebot3_dqn_stage1` | DQN ステージ1 の地図 |

参考: [TurtleBot3 E-Manual (Simulation)](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
