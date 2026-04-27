# mapoi_server

mapoi のメインパッケージです。
地図と POI の情報を管理し、他のパッケージにサービスとして提供します。また、POI 名を指定した自律走行やPOI半径イベントの検知を行います。

## 自分のロボットへの組み込み方

3 つの core node (`mapoi_server` / `mapoi_nav_server` / `mapoi_rviz2_publisher`) と、必要に応じてシミュレータ連動 bridge をまとめて起動する bringup launch を提供しています。自前の launch から以下のように include してください:

```yaml
- include:
    file: "$(find-pkg-share mapoi_server)/launch/mapoi_bringup.launch.yaml"
    arg:
      - {name: maps_path, value: "/path/to/your/maps"}    # REQUIRED
      - {name: map_name, value: "initial_map_name"}        # REQUIRED
      - {name: config_file, value: "mapoi_config.yaml"}    # optional
      - {name: pub_interval_ms, value: "500"}              # optional
      - {name: simulator, value: "none"}                   # gazebo|gz|none (default: none)
      - {name: robot_entity_name, value: "burger"}         # simulator=gazebo|gz のとき必要
      - {name: robot_sdf_path, value: "/path/.../model.sdf"} # simulator=gazebo のとき必要
      - {name: init_world_name, value: "default"}          # simulator=gz のとき必要 (gz_sim 起動時の world 名)
```

`maps_path` と `map_name` は **必須** です (未指定だと `mapoi_server` が config ファイルを解決できず起動失敗します)。

`simulator` arg はシミュレータ連動 bridge の起動を制御します:
- `gazebo` (Gazebo Classic / Humble): `mapoi_gazebo_bridge` を起動。SwitchMap 時に Gazebo 内の `world_model` entity を入れ替え + ロボットを delete + spawn で `initial_pose` POI 座標に再生成
- `gz` (gz-sim / Jazzy): `mapoi_gz_bridge` + `parameter_bridge` (`ros_gz_bridge`) を起動。SwitchMap 時に gz-sim 内の `world_model` entity を入れ替え + ロボットを `SetEntityPose` で `initial_pose` POI 座標に teleport (gz-sim では Classic と異なり set_pose service が使えるため delete + spawn は不要)
- `none` (default): bridge 起動なし (実機運用)

Web UI も使う場合は `mapoi_webui.launch.yaml` も併せて include:

```yaml
- include:
    file: "$(find-pkg-share mapoi_webui)/launch/mapoi_webui.launch.yaml"
    arg:
      - {name: maps_path, value: "/path/to/your/maps"}
      - {name: map_name, value: "initial_map_name"}
```

> **NOTE**: `mapoi_webui` はシステムタグを `mapoi_server` の `get_tag_definitions` service 経由で取得します。`mapoi_webui` 単体を起動しても system tags の表示は service 通信になるため、`mapoi_server` の package install 自体は不要 (ただし service responder として `mapoi_server` ノードが起動している必要あり)。

TurtleBot3 を使った動作例は `mapoi_turtlebot3_example` パッケージを参照してください。

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
| `mapoi_config_path` | `std_msgs/String` | 現在の設定ファイルのパス（定期配信、transient_local QoS） |

### mapoi_nav_server

POI 名を指定した自律走行と、POI 半径イベント検知を行うノードです。

#### パラメータ

| パラメータ名 | 型 | デフォルト | 説明 |
| --- | --- | --- | --- |
| `radius_check_hz` | `double` | `5.0` | POI 半径チェックの頻度 (Hz) |
| `hysteresis_exit_multiplier` | `double` | `1.15` | EXIT 判定の閾値倍率（チャタリング防止） |
| `map_frame` | `string` | `map` | TF の親フレーム |
| `base_frame` | `string` | `base_link` | TF の子フレーム |
| `initial_pose_topic` | `string` | `/initialpose` | initial_pose の配信先 topic 名 (非 AMCL の localization に対応する場合に変更) |
| `initial_pose_subscriber_wait_timeout_sec` | `double` | `10.0` | 自動 publish 前に subscriber readiness を待つ秒数 (起動の遅い localization は延長) |

#### サブスクライバー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi_initialpose_poi` | `std_msgs/String` | 指定した POI 名の位置に初期位置を設定 |
| `mapoi_goal_pose_poi` | `std_msgs/String` | 指定した POI 名の位置に自律走行 |
| `mapoi_route` | `std_msgs/String` | 指定したルート名のウェイポイントを順に走行 |
| `mapoi_pause` | `std_msgs/String` | ナビゲーションの一時停止 |
| `mapoi_resume` | `std_msgs/String` | ナビゲーションの再開 |
| `mapoi_cancel` | `std_msgs/String` | ナビゲーションのキャンセル |
| `mapoi_config_path` | `std_msgs/String` | マップ切替検知（transient_local QoS） |

#### パブリッシャー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 初期位置の配信 (topic 名は `initial_pose_topic` parameter で変更可)。`mapoi_initialpose_poi` で明示指定された場合に加え、地図ロード/切替時に `initial_pose` タグ付き POI から自動配信。自動 publish 前は subscriber readiness を最大 `initial_pose_subscriber_wait_timeout_sec` 秒待つ |
| `goal_pose` | `geometry_msgs/PoseStamped` | ゴール位置の配信 |
| `mapoi_nav_status` | `std_msgs/String` | ナビゲーション状態（`navigating`, `succeeded`, `aborted`, `canceled`, `paused`）。transient_local QoS で配信され、後起動 subscriber も最後の状態を取得可能 |
| `mapoi_poi_events` | `mapoi_interfaces/PoiEvent` | POI 侵入・退出イベント |

#### アクションクライアント

| アクション名 | 型 | 説明 |
| --- | --- | --- |
| `follow_waypoints` | `nav2_msgs/FollowWaypoints` | ウェイポイント追従 |
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` | 単一ゴールナビゲーション |

#### 対応 localization パッケージの要件

mapoi_nav_server の自動 initial_pose 配信は、以下を満たす localization パッケージで動作します:

- `geometry_msgs/PoseWithCovarianceStamped` を topic で受信できる (default `/initialpose`)
- 動的 (起動後の SwitchMap) でも上記 topic 経由で初期位置を受け付ける

**動作確認済み**: Nav2 AMCL (Humble / Jazzy)。

**別 topic を使うパッケージ** (例: 自社実装) は、`initial_pose_topic` parameter で配信先を変更できます。受信 message 型が `PoseWithCovarianceStamped` でない場合は、launch 構成で adapter ノードを介する必要があります (将来的な adapter pattern は別 issue で検討予定)。

**注意**: SwitchMap 経由の map 入替は現状 Nav2 `LoadMap` action 経由のため、Nav2 lifecycle に乗らない localization では map 入替動作の整合は別途検証・対応が必要です。

#### POI 半径イベント検知

POI の半径内にロボットが侵入/退出した際に `mapoi_poi_events` トピックにイベントを発行します。全 POI がタグに関わらず検知対象です。

- TF lookup (`map` -> `base_link`) でロボット位置を取得（デフォルト 5Hz）
- **侵入判定**: 距離 <= `radius`
- **退出判定**: 距離 > `radius * hysteresis_exit_multiplier`（デフォルト 1.15）
- `pause` タグ付き POI では侵入時に走行を自動一時停止（併せてイベントも発行）
- マップ切替時は内部状態をリセットし、新しい POI リストで監視を再開

### mapoi_rviz2_publisher

RViz2 上に POI のマーカーを表示するためのノードです。

#### パラメータ

| パラメータ名 | 型 | デフォルト | 説明 |
| --- | --- | --- | --- |
| `arrow_size_ratio` | `double` | `1.0` | POI 矢印の scale を radius に対する比率で指定 (`arrow_length = radius × ratio`)。Route 矢印 (waypoint 間) は radius 概念を持たないので対象外。`ros2 param set` で runtime 変更も次周期に反映 |
| `poi_label_format` | `string` | `"index"` | POI label の表示形式: `"index"` = POI Editor 行番号 (1-based 通し、tag フィルタ非依存) / `"name"` = POI 名 / `"both"` = `"<index>: <name>"` / `"none"` = 非表示 |

#### サブスクライバー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi_highlight_goal` | `std_msgs/String` | ハイライトするゴール POI 名 |
| `mapoi_highlight_route` | `std_msgs/String` | ハイライトするルート POI 名（カンマ区切り） |

#### パブリッシャー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi_goal_marks` | `visualization_msgs/MarkerArray` | goal タグの POI マーカー |
| `mapoi_event_marks` | `visualization_msgs/MarkerArray` | event・origin タグの POI マーカー（半径表示含む） |

## タグシステム

POI にはタグを付与して用途を分類できます。タグは **システムタグ** と **ユーザータグ** の2種類があります。

### システムタグ

`maps/tag_definitions.yaml` で定義されるグローバルなタグです。

| タグ名 | 説明 |
| --- | --- |
| `goal` | ナビゲーション目的地 |
| `origin` | 地図原点の基準点 |
| `pause` | ロボットが POI 半径内に入ったとき、走行中であれば自動的に一時停止 |
| `initial_pose` | 地図ロード/切替時に AMCL 等の自己位置推定へ初期位置として自動配信（地図あたり 1 つを推奨。複数ある場合は先頭を採用し警告ログ） |

### ユーザータグ

各地図の `mapoi_config.yaml` 内の `custom_tags` セクションで定義します。

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
  - name: checkpoint_a
    pose: {x: 0.5, y: 0.5, yaw: 0.0}
    radius: 0.5
    tags: [goal, pause]       # 半径内に入ると自動一時停止
    description: 中間チェックポイント
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
- **gazebo** (任意、`mapoi_gazebo_bridge` のみが参照): SwitchMap 時に入れ替える Gazebo Classic 側のモデル定義
  - `world_model.uri`: モデル URI (例: `model://turtlebot3_world`)
  - `world_model.name`: Gazebo 内での entity 名 (delete/spawn のキー)

## ディレクトリ構成

```
maps/
├── tag_definitions.yaml     # システムタグ定義（グローバル）
└── <地図名>/
    ├── mapoi_config.yaml    # POI・ルート・ユーザータグ設定
    ├── <地図名>.yaml        # 地図メタデータ
    └── <地図名>.pgm         # 地図画像
```
