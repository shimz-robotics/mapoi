# mapoi_server

mapoi のメインパッケージです。
地図と POI の情報を管理し、他のパッケージにサービスとして提供します。また、POI 名を指定した自律走行やPOI半径イベントの検知を行います。

## 自分のロボットへの組み込み方

4 つの core node (`mapoi_server` / `mapoi_nav2_bridge` / `mapoi_amcl_localization_bridge` / `mapoi_rviz2_publisher`) と、必要に応じてシミュレータ連動 bridge をまとめて起動する bringup launch を提供しています。自前の launch から以下のように include してください:

```yaml
- include:
    file: "$(find-pkg-share mapoi_server)/launch/mapoi_bringup.launch.yaml"
    arg:
      - {name: maps_path, value: "/path/to/your/maps"}    # REQUIRED
      - {name: map_name, value: "initial_map_name"}        # REQUIRED
      - {name: config_file, value: "mapoi_config.yaml"}    # optional
      - {name: simulator, value: "none"}                   # gazebo|gz|none (default: none)
      - {name: robot_entity_name, value: "burger"}         # simulator=gazebo|gz のとき必要
      - {name: robot_sdf_path, value: "/path/.../model.sdf"} # simulator=gazebo のとき必要
      - {name: init_world_name, value: "default"}          # simulator=gz のとき必要 (gz_sim 起動時の world 名)
```

`maps_path` と `map_name` は **必須** です。`maps_path` 未指定 / 存在しないパス / ディレクトリでない場合は `RCLCPP_FATAL` ログ + 終了コード 1 で起動失敗します (#163)。`mapoi_turtlebot3_example` の sample を流用するなら `$(find-pkg-share mapoi_turtlebot3_example)/maps` を指定してください。

### CLI から直接起動する例

include せず `ros2 launch` を直接叩く場合の最小例:

```bash
# mapoi_turtlebot3_example の sample maps を流用 (apt / 配布 install 後にも動く)
ros2 launch mapoi_server mapoi_bringup.launch.yaml \
  maps_path:=$(ros2 pkg prefix --share mapoi_turtlebot3_example)/maps \
  map_name:=turtlebot3_world

# ソースツリーから直接 (ros2_ws 内で source 後)
ros2 launch mapoi_server mapoi_bringup.launch.yaml \
  maps_path:=$(pwd)/src/mapoi/mapoi_turtlebot3_example/maps \
  map_name:=turtlebot3_dqn_stage1
```

#### 引数の取り違え注意

各 launch arg のセマンティクスは以下のとおり (`{maps_path}/{map_name}/{config_file}` でアクセスする 3 レベルの分割):

| arg | 渡すもの | 例 |
|---|---|---|
| `maps_path` | **地図群を束ねる親ディレクトリ** (各地図は配下のサブディレクトリ) | `/path/to/maps` |
| `map_name` | `maps_path` 直下の地図サブディレクトリ名 | `turtlebot3_world` |
| `config_file` | 地図サブディレクトリ内の設定ファイル名 (基本デフォルトのまま) | `mapoi_config.yaml` |

実際にロードされるパスは `{maps_path}/{map_name}/{config_file}` で、上記例なら `/path/to/maps/turtlebot3_world/mapoi_config.yaml` となります。

`maps_path` と `map_name` はどちらも `mapoi_bringup.launch.yaml` の **必須 launch arg** で、未指定なら `ros2 launch` 自体が `Required launch argument '<name>' was not provided` で fail します (node 起動前)。

**❌ よくある間違い**: `maps_path` に **config.yaml ファイルパスを直接指定** してしまう:

```bash
# WRONG: maps_path はファイルではなくディレクトリ
ros2 launch mapoi_server mapoi_bringup.launch.yaml \
  maps_path:=./maps/turtlebot3_dqn_stage1/mapoi_config.yaml \
  map_name:=turtlebot3_dqn_stage1
```

→ 起動時に `mapoi_server` ノードが以下を 1 行で出力して FATAL 終了します (`mapoi_server.cpp` 内の `RCLCPP_FATAL` ログ全文):

```
[FATAL] maps_path '...mapoi_config.yaml' does not exist or is not a directory. 正しい maps ディレクトリ path を指定してください (例: $(find-pkg-share mapoi_turtlebot3_example)/maps)。
```

`maps_path` は **地図群を束ねる親ディレクトリ** で、特定の地図ディレクトリやファイルではありません。ディレクトリ構成は本 README 末尾の [ディレクトリ構成](#ディレクトリ構成) 節を参照してください。

`simulator` arg はシミュレータ連動 bridge の起動を制御します:
- `gazebo` (Gazebo Classic / Humble): `mapoi_gazebo_bridge` を起動。operator map switch 時に Gazebo 内の `world_model` entity を入れ替え + ロボットを delete + spawn で **POI list 先頭** の座標に再生成 (#144 で旧 `initial_pose` system tag を廃止し、yaml 順序で表現する semantics に変更)
- `gz` (gz-sim / Jazzy): `mapoi_gz_bridge` + `parameter_bridge` (`ros_gz_bridge`) を起動。operator map switch 時に gz-sim 内の `world_model` entity を入れ替え + ロボットを `SetEntityPose` で **POI list 先頭** の座標に teleport (gz-sim では Classic と異なり set_pose service が使えるため delete + spawn は不要)
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
| `maps_path` | `string` | **REQUIRED** | 地図ディレクトリのパス (#163 で sample maps 廃止により必須化) |
| `map_name` | `string` | `turtlebot3_world` | 起動時に読み込む地図名 |
| `config_file` | `string` | `mapoi_config.yaml` | 設定ファイル名 |

#### サービス

| サービス名 | 型 | 説明 |
| --- | --- | --- |
| `get_maps_info` | `GetMapsInfo` | 利用可能な地図一覧と現在の地図名を取得 |
| `get_pois_info` | `GetPoisInfo` | 現在の地図の全 POI を取得 |
| `get_route_pois` | `GetRoutePois` | ルート上の POI を取得 |
| `get_routes_info` | `GetRoutesInfo` | 利用可能なルート一覧を取得 |
| `get_tag_definitions` | `GetTagDefinitions` | タグ定義（システム/ユーザー）を取得 |
| `select_map` | `SelectMap` | 現在 map context を切り替え（Nav2 は呼ばない） |
| `reload_map_info` | `std_srvs/Trigger` | 設定ファイルを再読み込み |

#### パブリッシャー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi/config_path` | `std_msgs/String` | 現在の設定ファイルのパス（定期配信、transient_local QoS） |

### mapoi_nav2_bridge

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
| `mapoi/nav/switch_map` | `std_msgs/String` | operator map switch 指示。受信後に `select_map` → Nav2 `LoadMap` → `mapoi/initialpose_poi` publish (= localization bridge へ初期位置 trigger) を実行 |
| `mapoi/nav/goal_pose_poi` | `std_msgs/String` | 指定した POI 名の位置に自律走行 |
| `mapoi/nav/route` | `std_msgs/String` | 指定したルート名のウェイポイントを順に走行 |
| `mapoi/nav/pause` | `std_msgs/String` | ナビゲーションの一時停止 |
| `mapoi/nav/resume` | `std_msgs/String` | ナビゲーションの再開 |
| `mapoi/nav/cancel` | `std_msgs/String` | ナビゲーションのキャンセル |
| `mapoi/config_path` | `std_msgs/String` | マップ切替検知（transient_local QoS） |

#### パブリッシャー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi/initialpose_poi` | `mapoi_interfaces/InitialPoseRequest` | Nav2 LoadMap 完了後、新 map の `initial_poi_name` を localization bridge 向けに publish する trigger 用 (#209)。`/initialpose` 自体への配信は `mapoi_amcl_localization_bridge` が担当 |
| `goal_pose` | `geometry_msgs/PoseStamped` | ゴール位置の配信 |
| `mapoi/nav/status` | `std_msgs/String` | ナビゲーション状態を `"status"` または `"status:target"` 形式で配信（例: `"navigating:kitchen"`、`"succeeded:patrol_route"`、`"paused:patrol_route"`、`"map_switching:turtlebot3_world"`）。`status` は `navigating` / `succeeded` / `aborted` / `canceled` / `paused` / `map_switching` / `map_switch_succeeded` / `map_switch_failed` / `backend_unavailable`。`backend_unavailable` は Nav2 action / service 不在で goal / route / resume を実行できなかった場合（#198）。`target` は POI 名（goal mode）、route 名（route mode）、または map 名で、subscriber 側は最初の `:` で split して復元する（target 内に `:` が含まれても残り全体を target として扱える）。`transient_local` QoS の **現在状態 snapshot**（depth=1）で、後起動 subscriber が最後の状態を受信できるが状態遷移履歴は復元できない。現在走行中かは `navigating` / `paused` / `map_switching` で判定し、終端状態（`succeeded` / `aborted` / `canceled` / `map_switch_succeeded` / `map_switch_failed` / `backend_unavailable`）は直近結果として扱う |
| `mapoi/nav/backend_status` | `mapoi_interfaces/NavigationBackendStatus` | navigation bridge の readiness summary を 1Hz で配信（#198）。`transient_local` QoS。Minimal 3 フィールド（`backend_type` / `backend_ready` / `reason`）。WebUI / panel は `backend_ready` で navigation 操作 UI を一括 gate する。詳細は ルート README "Navigation backend 仕様" 節を参照 |
| `mapoi/events` | `mapoi_interfaces/PoiEvent` | POI 侵入・退出イベント |

#### アクションクライアント

| アクション名 | 型 | 説明 |
| --- | --- | --- |
| `follow_waypoints` | `nav2_msgs/FollowWaypoints` | ウェイポイント追従 |
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` | 単一ゴールナビゲーション |

#### POI 半径イベント検知

POI の半径内にロボットが侵入/退出した際に `mapoi/events` トピックにイベントを発行します。全 POI がタグに関わらず検知対象です。

- TF lookup (`map` -> `base_link`) でロボット位置を取得（デフォルト 5Hz）
- **侵入判定**: 距離 <= `tolerance.xy`
- **退出判定**: 距離 > `tolerance.xy * hysteresis_exit_multiplier`（デフォルト 1.15）
- `pause` タグ付き POI では侵入時に走行を自動一時停止（併せてイベントも発行）
- マップ切替時は内部状態をリセットし、新しい POI リストで監視を再開

### mapoi_amcl_localization_bridge

AMCL 互換 localization (`/initialpose` を `geometry_msgs/PoseWithCovarianceStamped` で受ける構成) 向けの localization bridge ノードです (#209)。`mapoi_nav2_bridge` から AMCL adapter を分離して、Navigation backend と Localization backend を独立した仕様で扱えるようにしています。slam_toolbox / NDT / 自前 localization に切替える場合は、本 bridge の代替として同じ topic 仕様を満たす自作 bridge を用意してください（ルート README の「Localization backend 仕様」節参照）。

#### パラメータ

| パラメータ名 | 型 | デフォルト | 説明 |
| --- | --- | --- | --- |
| `initial_pose_topic` | `string` | `/initialpose` | initial_pose の配信先 topic 名 (非 AMCL の localization に対応する場合に変更) |
| `initialpose_retry_interval_sec` | `double` | `0.1` | subscriber 後起動 / 検知後 republish の timer 間隔 (秒)。`0.01` 未満は default に clamp |
| `initialpose_retry_max_attempts` | `int` | `50` | subscriber 検知前の最大 wait 試行回数 (= `interval_sec × max_attempts` 秒で諦め)。default = 約 5 秒 |
| `initialpose_post_subscribe_republish_count` | `int` | `3` | subscriber 検知後に追加 republish する回数。AMCL の「visible だが処理 ready 直前」取りこぼしを防ぐ |
| `map_frame` | `string` | `map` | `PoseWithCovarianceStamped.header.frame_id` |

#### サブスクライバー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi/initialpose_poi` | `mapoi_interfaces/InitialPoseRequest` | 初期位置に採用する POI 名 (`{map_name, poi_name}`)。`get_pois_info` で resolve した pose を `/initialpose` に流す。空 `poi_name` は無視 |

#### パブリッシャー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 初期位置の配信 (topic 名は `initial_pose_topic` parameter で変更可)。`mapoi/initialpose_poi` で受信した POI の pose を流す。subscriber 後起動 / 検知後 ready 直前ケースに備えて wall_timer ベースで非同期 retry + post-subscribe republish する (`initialpose_retry_*` parameter 群、#152) |
| `mapoi/localization/backend_status` | `mapoi_interfaces/LocalizationBackendStatus` | localization bridge の readiness summary を 1Hz で配信 (#209)。`transient_local` QoS。Minimal 3 フィールド (`backend_type` / `backend_ready` / `reason`)。`backend_ready` は `/initialpose` の subscriber 数 > 0 を ready proxy として採用。WebUI / panel は `backend_ready` で Initial Pose UI を gate する |

#### サービスクライアント

| サービス名 | 型 | 説明 |
| --- | --- | --- |
| `get_pois_info` | `GetPoisInfo` | POI 名から pose を resolve するために `mapoi_server` を呼び出す |

#### 対応 localization パッケージの要件

`mapoi_amcl_localization_bridge` は以下を満たす localization パッケージで動作します:

- `geometry_msgs/PoseWithCovarianceStamped` を topic で受信できる (default `/initialpose`)
- 動的 (起動後の operator map switch) でも上記 topic 経由で初期位置を受け付ける

**動作確認済み**: Nav2 AMCL (Humble / Jazzy)。

**別 topic を使うパッケージ** (例: 自社実装) は、`initial_pose_topic` parameter で配信先を変更できます。受信 message 型が `PoseWithCovarianceStamped` でない場合は、本 bridge を停止して同じ仕様を満たす自作 bridge を用意してください（ルート README の「Localization backend 仕様」節を参照）。

**注意**: `/mapoi/nav/switch_map` 経由の map 入替は Nav2 `LoadMap` service 経由のため、Nav2 lifecycle に乗らない localization では map 入替動作の整合は別途検証・対応が必要です。

### mapoi_rviz2_publisher

RViz2 上に POI のマーカーを表示するためのノードです。

#### パラメータ

| パラメータ名 | 型 | デフォルト | 説明 |
| --- | --- | --- | --- |
| `show_tolerance_sector` | `bool` | `true` | POI tolerance visualization (#136 / #179) を表示するか。対象 layer = xy 判定円 outline + yaw 制約扇形 (`0 < tolerance.yaw < π` の時のみ重ね描き) + pause overlay (xy 円沿いの dot pattern)。`false` で全 POI のこれら 3 layer を抑制 (Editor 中心の使い方や RViz が情報過多な時の用途)。WebUI 側にも同等の描画仕様 (`mapoi_webui/web/js/map-viewer.js`) があるため、仕様変更時はペアで更新する |
| `poi_label_format` | `string` | `"index"` | POI label の表示形式: `"index"` = POI Editor 行番号 (1-based 通し、tag フィルタ非依存) / `"name"` = POI 名 / `"both"` = `"<index>: <name>"` / `"none"` = 非表示 |
| `route_display_mode` | `string` | `"selected"` | Route marker の表示形式: `"all"` = 全 route 表示 (active route は太線 + 不透明で強調) / `"selected"` = active route のみ表示 / `"none"` = 表示しない |

#### サブスクライバー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi/highlight/goal` | `std_msgs/String` | ハイライトするゴール POI 名 |
| `mapoi/highlight/route` | `std_msgs/String` | ハイライトするルート POI 名（カンマ区切り） |

#### パブリッシャー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi/markers/pois` | `visualization_msgs/MarkerArray` | POI マーカー。`waypoint` は緑、`landmark` は灰、custom tag のみの POI は青で描画 |
| `mapoi/markers/routes` | `visualization_msgs/MarkerArray` | Route マーカー。route line と選択中 route の方向矢印を描画 |

`mapoi/markers/pois` は Marker namespace で layer を分けています。RViz の `Namespaces` toggle で layer 単位に表示を切り替えられます。

| Marker namespace | 説明 |
| --- | --- |
| `arrow/<poi_name>` | POI の向き矢印と label |
| `tolerance_xy/<poi_name>` | `tolerance.xy` の判定円 |
| `tolerance_yaw/<poi_name>` | `tolerance.yaw` の扇形 |
| `status_paused/<poi_name>` | `pause` tag POI の dot overlay |

## タグシステム

POI にはタグを付与して用途を分類できます。タグは **システムタグ** と **ユーザータグ** の2種類があります。

### システムタグ

`mapoi_server.cpp` 内に const 定義されるグローバルなタグです (`kSystemTags`)。`get_tag_definitions` service で配信されます。system tag はコア機能と一体のため変更非推奨で、増減はコア改修扱いです。

| タグ名 | 説明 |
| --- | --- |
| `waypoint` | Nav2 navigation の到達対象（単発 navigation goal、route の中間点いずれも対応） |
| `landmark` | Nav2 navigation 対象にしない参照専用 POI（可視化のみ、waypoint 候補と initial pose 候補から除外） |
| `pause` | ロボットが POI 半径内に入ったとき、**ROUTE 走行中かつ active route の `waypoints` / `landmarks` に含まれる POI** であれば自動的に一時停止（#143）。GOAL 走行中・IDLE では発火しない。`landmark` との併用は不可 |

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
    tolerance: {xy: 0.5, yaw: 0.785}
    tags: [waypoint]                # POI list 先頭が default initial pose に採用される (#144)
    description: エレベーターホール
  - name: checkpoint_a
    pose: {x: 0.5, y: 0.5, yaw: 0.0}
    tolerance: {xy: 0.5, yaw: 0.785}
    tags: [waypoint, pause]   # tolerance.xy 半径内に入ると自動一時停止
    description: 中間チェックポイント
  - name: info_point_a
    pose: {x: 1.0, y: 2.0, yaw: 1.57}
    tolerance: {xy: 1.0, yaw: 0.785}
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
- **poi**: POI の定義（**順序が semantics を持ちます**: 各 map で `poi:` 配下の **先頭 POI** が地図ロード/切替時の default 初期位置として採用される。先頭は `landmark` タグなし & `pose.x/y/yaw` が完備された POI である必要あり。明示指定したい場合は `SelectMap.srv` の `initial_poi_name` で POI 名を渡す、#144）
  - `name`: POI の名前（トピックで指定する際に使用）
  - `pose`: 位置（`x`, `y`, `yaw` すべて必須。default 初期位置候補として使われる場合は欠落不可）
  - `tolerance`: Nav2 align tolerance struct
    - `xy`: Euclidean tolerance (m)。POI 進入判定半径としても使用される
    - `yaw`: Angular tolerance (rad)。`0` = 未指定として Nav2 default にフォールバック
  - `tags`: タグのリスト
  - `description`: 説明文
- **route**: ルートの定義
  - `name`: ルート名
  - `waypoints`: 巡回する POI 名のリスト（`waypoint` タグ付き POI を Nav2 `FollowWaypoints` に送る）
  - `landmarks` (任意): この route 走行中に意識する補助 POI 名のリスト（#143）。Nav2 へは送らず radius event 監視と pause 発火スコープにのみ使う。`landmark` タグ付き POI を指定する想定
- **gazebo** (任意、`mapoi_gazebo_bridge` のみが参照): operator map switch 時に入れ替える Gazebo Classic 側のモデル定義
  - `world_model.uri`: モデル URI (例: `model://turtlebot3_world`)
  - `world_model.name`: Gazebo 内での entity 名 (delete/spawn のキー)

## ディレクトリ構成

`maps_path` 配下は地図ごとのサブディレクトリで構成されます (システムタグ定義は `mapoi_server.cpp` 内に const として保持されるため、この dir には含まれません)。

```
maps/
└── <地図名>/
    ├── mapoi_config.yaml    # POI・ルート・ユーザータグ設定
    ├── <地図名>.yaml        # 地図メタデータ
    └── <地図名>.pgm         # 地図画像
```
