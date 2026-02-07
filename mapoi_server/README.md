# mapoi_server

**mapoi のメインパッケージです。**
地図と POI の情報を管理し、他のパッケージにサービスとして提供します。

## ノード

### mapoi_server

地図・POI の設定ファイルを読み込み、情報提供サービスを公開するノードです。

#### パラメータ

| パラメータ名 | 型 | デフォルト | 説明 |
| --- | --- | --- | --- |
| `maps_path` | `string` | - | 地図ディレクトリのパス |
| `map_name` | `string` | - | 起動時に読み込む地図名 |
| `config_file` | `string` | - | 設定ファイル名（例: `mapoi_config.yaml`） |
| `pub_interval_ms` | `int` | `5000` | 設定パスの配信間隔（ミリ秒） |

#### サービス

| サービス名 | 型 | 説明 |
| --- | --- | --- |
| `get_maps_info` | `mapoi_interfaces/srv/GetMapsInfo` | 利用可能な地図一覧と現在の地図名を取得 |
| `get_pois_info` | `mapoi_interfaces/srv/GetPoisInfo` | 現在の地図の全 POI を取得 |
| `get_route_pois` | `mapoi_interfaces/srv/GetRoutePois` | ルート上の POI を取得 |
| `switch_map` | `mapoi_interfaces/srv/SwitchMap` | 地図を切り替え |
| `reload_map_info` | `std_srvs/srv/Trigger` | 設定ファイルを再読み込み |

#### パブリッシャー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi_config_path` | `std_msgs/msg/String` | 現在の設定ファイルのパス |
| `initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 地図切替時の初期位置の配信 |

### mapoi_nav_server

POI 名を指定した自律走行を行うノードです。トピック経由で初期位置の設定やゴール指定、ルート走行を受け付けます。

#### サブスクライバー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi_initialpose_poi` | `std_msgs/msg/String` | 指定した POI 名の位置に初期位置を設定 |
| `mapoi_goal_pose_poi` | `std_msgs/msg/String` | 指定した POI 名の位置に自律走行 |
| `mapoi_route` | `std_msgs/msg/String` | 指定したルート名のウェイポイントを順に走行 |
| `mapoi_cancel` | `std_msgs/msg/String` | ルート走行のキャンセル |

#### パブリッシャー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 初期位置の配信 |
| `goal_pose` | `geometry_msgs/msg/PoseStamped` | ゴール位置の配信 |

#### アクションクライアント

| アクション名 | 型 | 説明 |
| --- | --- | --- |
| `follow_waypoints` | `nav2_msgs/action/FollowWaypoints` | Navigation2 のウェイポイント追従 |

### mapoi_rviz2_publisher

RViz2 上に POI のマーカーを表示するためのノードです。

#### パブリッシャー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi_goal_marks` | `visualization_msgs/msg/MarkerArray` | goal・waypoint タグを持つ POI のマーカー |
| `mapoi_event_marks` | `visualization_msgs/msg/MarkerArray` | event・origin タグを持つ POI のマーカー |

## 設定ファイル (mapoi_config.yaml)

各地図ディレクトリに `mapoi_config.yaml` を配置して、地図と POI を設定します。

```yaml
map:
  - node_name: /map_server
    map_file: turtlebot3.yaml

poi:
  - name: elevator_hall
    pose: {x: -2.0, y: -0.5, yaw: 0.0}
    radius: 0.5
    tags: [goal, initial_pose]
    description: エレベーターホール
  - name: meeting_room_a
    pose: {x: -0.5, y: -0.5, yaw: 0.0}
    radius: 0.5
    tags: [goal]
    description: 会議室A

route:
  - name: route_1
    waypoints: [meeting_room_a, elevator_hall, meeting_room_a]
```

### フィールド説明

- **map**: 使用する地図ファイルの設定
  - `node_name`: map_server のノード名
  - `map_file`: 地図の YAML ファイル名
- **poi**: POI の定義
  - `name`: POI の名前（トピックで指定する際に使用）
  - `pose`: 位置（`x`, `y`, `yaw`）
  - `radius`: 到達判定の半径
  - `tags`: タグのリスト（フィルタ用）
  - `description`: 説明文
- **route**: ルートの定義
  - `name`: ルート名
  - `waypoints`: 巡回する POI 名のリスト

## ディレクトリ構成

```
maps/
└── <地図名>/
    ├── mapoi_config.yaml    # POI・ルート設定
    ├── <地図名>.yaml        # 地図メタデータ
    └── <地図名>.pgm         # 地図画像
```
