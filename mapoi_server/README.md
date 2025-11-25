# mapoi_server

## ROS API

| Type | Name | Example or msg_type | Description |
| ---- | ---- | ---- | ---- |
| Parameter | maps_path | path/to/maps | Path to the directory where your maps are stored |
| Parameter | map_name | turtlebot3_world | Name of the map to load at startup |
| Service | warp_pose  |  request robot position on Gazebo  |  geometry_msgs::msg::Pose  |
| Publisher | warp_pose  |  request robot position on Gazebo  |  geometry_msgs::msg::Pose  |

## 実行確認

```sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

```sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```

## 既存.pcdファイルから.pgmへ変換する場合の操作方法
- .pcdファイルから直接.pgmに変換する場合はmap~lidatrodom間のstatic_transform_publisherが必要

1. $MAPNAMEを指定して以下実行
```
ros2 run tf2_ros static_transform_publisher 0.0 0.0 -0.74 -1.57.0 0.0 0.0 lidar_odom map & ros2 launch pc2map pcd_publisher.launch.py pcd_file_path:="$WS_ROOT/src/shimz_pkgs/mapoi_server/map/$MAPNAME/GlobalMap.pcd"
```

1.  以下実行
- 以下の例では`map`フレームで0.2-2[m]の点群を.pgm化
```
ros2 launch pc2map pc2map.launch.xml map_min_z:=0.2 map_max_z:=2.0
```

1.  $MAPNAMEを指定して以下実行
```
export MAPNAME=test #例：test
ros2 service call /cmd pc2map_msgs/srv/SaveMap "{filepath: $WS_ROOT/src/shimz_pkgs/mapoi_server/map/$MAPNAME/GlobalMap.pcd}"
```

1.  $MAPNAMEを指定して以下実行
```
export MAPNAME=test #例：test
ros2 run nav2_map_server map_saver_cli -f $WS_ROOT/src/shimz_pkgs/mapoi_server/map/$MAPNAME/map --ros-args -p save_map_timeout:=5000.0
```

1. CloudCompareでGlobalMap.pcdをz軸方向に-90度回転させて上書き保存する


### mapoi_map_saver

- 引数として、保存先のmapsディレクトリとmap名の2を受け取る。もし引数を忘れている場合にはエラーを出して使用方法を表示する。
- nav2_map_serverのmap_saver_serverを呼び出して、保存先のmapsディレクトリにmap名.yamlとmap名.pgmを作成する。
- map名.yamlとmap名.pgmをコピーして、保存先のmapsディレクトリにmap名_keepout.yamlとmap名_keepout.pgmを作成する。
- map名_keepout.yamlの`image: map名.pgm`を`image: map名_keepout.pgm`に変更する。
- 保存先のmapsディレクトリにmap名_poi.yamlを以下の内容で作成する。
```yaml
gazebo_model: giken2F
gazebo_pose: {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}
poi:
  - id: 400
    name: initial_position
    description: 初期位置
    pose: {x: 0.0, y: 0.0, yaw: 0.0}
    radius: 0.5
    tags: [destination, origin, initial]
  - id: 401
    name: destinationA
    description: 目的地A
    pose: {x: -2.476456, y: -1.170249, yaw: 3.137366}
    radius: 0.5
    tags: [destination, ELV]
  - id: 401
    name: ELVHall
    description: エレベーター
    pose: {x: -2.476456, y: -1.170249, yaw: 3.137366}
    radius: 0.5
    tags: [destination, ELV]
```


### mapoi_nav_server

mapoi_interfacesパッケージのNavigateToPoi.actionを利用して指定されたstring nameへの自律走行を行う。

