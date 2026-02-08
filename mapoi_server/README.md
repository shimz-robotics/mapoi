# mapoi_server

mapoi のメインパッケージです。
地図と POI の情報を管理し、他のパッケージにサービスとして提供します。また、POI 名を指定した自律走行やPOI半径イベントの検知を行います。

## ノード

### mapoi_server

地図・POI の設定ファイルを読み込み、情報提供サービスを公開するノードです。

#### パラメータ

| パラメータ名 | 型 | デフォルト | 説明 |
| --- | --- | --- | --- |
| `maps_path` | `string` | `{pkg_share}/maps` | 地図ディレクトリのパス |
| `map_name` | `string` | `turtlebot3_world` | 起動時に読み込む地図名 |
| `config_file` | `string` | `mapoi_config.yaml` | 設定ファイル名 |
| `pub_interval_ms` | `int` | `5000` | 設定パスの配信間隔（ミリ秒） |

#### サービス

| サービス名 | 型 | 説明 |
| --- | --- | --- |
| `get_maps_info` | `GetMapsInfo` | 利用可能な地図一覧と現在の地図名を取得 |
| `get_pois_info` | `GetPoisInfo` | 現在の地図の全 POI を取得 |
| `get_route_pois` | `GetRoutePois` | ルート上の POI を取得 |
| `get_routes_info` | `GetRoutesInfo` | 利用可能なルート一覧を取得 |
| `get_tag_definitions` | `GetTagDefinitions` | タグ定義（システム/ユーザー）を取得 |
| `switch_map` | `SwitchMap` | 地図を切り替え（Nav2 の LoadMap を呼出） |
| `reload_map_info` | `std_srvs/Trigger` | 設定ファイルを再読み込み |

#### パブリッシャー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi_config_path` | `std_msgs/String` | 現在の設定ファイルのパス（定期配信） |

### mapoi_nav_server

POI 名を指定した自律走行と、POI 半径イベント検知を行うノードです。

#### パラメータ

| パラメータ名 | 型 | デフォルト | 説明 |
| --- | --- | --- | --- |
| `radius_check_hz` | `double` | `5.0` | POI 半径チェックの頻度 (Hz) |
| `hysteresis_exit_multiplier` | `double` | `1.15` | EXIT 判定の閾値倍率（チャタリング防止） |
| `map_frame` | `string` | `map` | TF の親フレーム |
| `base_frame` | `string` | `base_link` | TF の子フレーム |

#### サブスクライバー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi_initialpose_poi` | `std_msgs/String` | 指定した POI 名の位置に初期位置を設定 |
| `mapoi_goal_pose_poi` | `std_msgs/String` | 指定した POI 名の位置に自律走行 |
| `mapoi_route` | `std_msgs/String` | 指定したルート名のウェイポイントを順に走行 |
| `mapoi_cancel` | `std_msgs/String` | ナビゲーションのキャンセル |
| `mapoi_config_path` | `std_msgs/String` | マップ切替検知（transient_local QoS） |

#### パブリッシャー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 初期位置の配信 |
| `goal_pose` | `geometry_msgs/PoseStamped` | ゴール位置の配信 |
| `mapoi_nav_status` | `std_msgs/String` | ナビゲーション状態（`navigating`, `succeeded`, `aborted`, `canceled`） |
| `mapoi_poi_events` | `mapoi_interfaces/PoiEvent` | POI 侵入・退出イベント |

#### アクションクライアント

| アクション名 | 型 | 説明 |
| --- | --- | --- |
| `follow_waypoints` | `nav2_msgs/FollowWaypoints` | ウェイポイント追従 |
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` | 単一ゴールナビゲーション |

#### POI 半径イベント検知

ユーザータグ（`audio_info`, `event` 等）を持つ POI の半径内にロボットが侵入/退出した際に `mapoi_poi_events` トピックにイベントを発行します。

- TF lookup (`map` -> `base_link`) でロボット位置を取得（デフォルト 5Hz）
- **侵入判定**: 距離 <= `radius`
- **退出判定**: 距離 > `radius * hysteresis_exit_multiplier`（デフォルト 1.15）
- システムタグ（`goal`, `waypoint`, `origin`）のみの POI はイベント対象外
- マップ切替時は内部状態をリセットし、新しい POI リストで監視を再開

### mapoi_rviz2_publisher

RViz2 上に POI のマーカーを表示するためのノードです。

#### サブスクライバー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi_highlight_goal` | `std_msgs/String` | ハイライトするゴール POI 名 |
| `mapoi_highlight_route` | `std_msgs/String` | ハイライトするルート POI 名（カンマ区切り） |

#### パブリッシャー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi_goal_marks` | `visualization_msgs/MarkerArray` | goal・waypoint タグの POI マーカー |
| `mapoi_event_marks` | `visualization_msgs/MarkerArray` | event・origin タグの POI マーカー（半径表示含む） |

## タグシステム

POI にはタグを付与して用途を分類できます。タグは **システムタグ** と **ユーザータグ** の2種類があります。

### システムタグ

`maps/tag_definitions.yaml` で定義されるグローバルなタグです。

| タグ名 | 説明 |
| --- | --- |
| `goal` | ナビゲーション目的地 |
| `waypoint` | 中間ウェイポイント |
| `origin` | 地図原点の基準点 |

### ユーザータグ

各地図の `mapoi_config.yaml` 内の `custom_tags` セクションで定義します。ユーザータグを持つ POI は半径イベント検知の対象になります。

```yaml
custom_tags:
  - name: audio_info
    description: "音声ガイドのトリガー"
```

## 設定ファイル (mapoi_config.yaml)

各地図ディレクトリに `mapoi_config.yaml` を配置して、地図と POI を設定します。

```yaml
custom_tags:
  - name: audio_info
    description: "音声ガイドのトリガー"

map:
  path_planning: map_file_name
  localization: map_file_name

poi:
  - name: elevator_hall
    pose: {x: -2.0, y: -0.5, yaw: 0.0}
    radius: 0.5
    tags: [goal, initial_pose]
    description: エレベーターホール
  - name: info_point_a
    pose: {x: 1.0, y: 2.0, yaw: 1.57}
    radius: 1.0
    tags: [audio_info]
    description: 音声案内ポイントA

route:
  - name: route_1
    waypoints: [elevator_hall, info_point_a]
```

### フィールド説明

- **custom_tags**: ユーザー定義タグ
  - `name`: タグ名
  - `description`: タグの説明
- **map**: 使用する地図ファイルの設定
  - `path_planning`: 経路計画用の地図ファイル名
  - `localization`: 自己位置推定用の地図ファイル名
- **poi**: POI の定義
  - `name`: POI の名前（トピックで指定する際に使用）
  - `pose`: 位置（`x`, `y`, `yaw`）
  - `radius`: 半径（到達判定・イベント検知に使用）
  - `tags`: タグのリスト
  - `description`: 説明文
- **route**: ルートの定義
  - `name`: ルート名
  - `waypoints`: 巡回する POI 名のリスト

## ディレクトリ構成

```
maps/
├── tag_definitions.yaml     # システムタグ定義（グローバル）
└── <地図名>/
    ├── mapoi_config.yaml    # POI・ルート・ユーザータグ設定
    ├── <地図名>.yaml        # 地図メタデータ
    └── <地図名>.pgm         # 地図画像
```
