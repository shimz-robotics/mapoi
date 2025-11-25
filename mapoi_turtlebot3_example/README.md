# mapoi_turtlebot3_example

Turtlebot3の[Eマニュアル](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)を参考に、SLMAとNavigationを行う。

必要パッケージのインストール

rosdepを使用できる場合

```sh
# cd path/to/your_ws/
rosdep install --from-paths src --ignore-src -r -y
```

rosdepを使用できない場合

```sh
sudo apt install -y \
ros-${ROS_DISTRO}-turtlebot3-simulations \
ros-${ROS_DISTRO}-turtlebot3-cartographer \
ros-${ROS_DISTRO}-turtlebot3-navigation2 \
ros-${ROS_DISTRO}-mouse-teleop
```

## 地図を取得する

シミュレーターの起動

```sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Cartographerの起動

```sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

地図を保存

```sh
# ./scripts/save_map_2d_sh mapoi_turtlebot3/maps/test_map
```

## 地図の編集

作成したmapディレクトリにある。`map_info.yaml`を編集する。

## 自律走行テスト

Navigationとmapoi_poi_serverを立ち上げる。

```sh
ros2 run mapoi_server mapoi_poi_server
```

走行テスト用のノードを実行する。

```sh
ros2 run mapoi_server mapoi_nav_server --ros-args -p route_name:=route_1
```
