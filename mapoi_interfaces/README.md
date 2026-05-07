# mapoi_interfaces

mapoi パッケージ群で使用するメッセージとサービスの定義パッケージです。

## メッセージ (msg)

### Tolerance.msg

POI の到達判定 tolerance（Nav2 `SimpleGoalChecker` の `xy_goal_tolerance` / `yaw_goal_tolerance` と align）。

| フィールド | 型 | 説明 |
| --- | --- | --- |
| `xy` | `float64` | Euclidean tolerance (m)。POI 進入判定半径としても使われる |
| `yaw` | `float64` | Angular tolerance (rad)。`0` = 未指定として Nav2 default にフォールバック |

### PointOfInterest.msg

POI（Point of Interest）を表すメッセージです。

| フィールド | 型 | 説明 |
| --- | --- | --- |
| `id` | `int32` | POI の識別子 |
| `name` | `string` | POI の名前 |
| `pose` | `geometry_msgs/Pose` | POI の位置・姿勢 |
| `tolerance` | `mapoi_interfaces/Tolerance` | xy / yaw tolerance（v0.3.0 で旧 `radius` から置換） |
| `tags` | `string[]` | POI に紐づくタグ（例: `goal`, `audio_info`） |
| `description` | `string` | POI の説明 |

### PoiEvent.msg

POI tolerance.xy 半径への侵入・退出 / 停止 / 再開イベントを表すメッセージです。

| フィールド | 型 | 説明 |
| --- | --- | --- |
| `EVENT_ENTER` | `uint8` (定数=1) | 侵入イベント |
| `EVENT_EXIT` | `uint8` (定数=2) | 退出イベント |
| `EVENT_STOPPED` | `uint8` (定数=3) | tolerance.xy 半径内で停止 (publish 判定 logic は別 issue) |
| `EVENT_RESUMED` | `uint8` (定数=4) | 停止後に動き出した (publish 判定 logic は別 issue) |
| `event_type` | `uint8` | イベント種別 |
| `poi` | `mapoi_interfaces/PointOfInterest` | 対象 POI の情報 |
| `stamp` | `builtin_interfaces/Time` | イベント発生時刻 |

### NavigationBackendStatus.msg

Navigation bridge (Nav2 bridge / 自前 bridge) が 1 Hz で `mapoi/nav/backend_status` に publish する readiness メッセージです。UI 側 (`mapoi_webui`, `mapoi_rviz_plugins`) は `backend_ready` を見て navigation 操作を gate します。

| フィールド | 型 | 説明 |
| --- | --- | --- |
| `backend_type` | `string` | bridge 識別子 (例: `nav2`, `custom_lidar_planner`)。tooltip 表示用 |
| `backend_ready` | `bool` | bridge が navigation コマンドを受け付け実行できる状態か |
| `reason` | `string` | `backend_ready=false` 時の human-readable 理由 (任意) |

#### QoS contract (issue #208)

publisher / subscriber 双方で以下を必ず指定:

- `durability`: `TRANSIENT_LOCAL` (late-joiner UI が最新 status を受信できる)
- `reliability`: `RELIABLE`
- `liveliness` (publisher): `MANUAL_BY_TOPIC` (publish() ごとに liveliness assert)
- `liveliness` (subscriber): `AUTOMATIC` (pub `MANUAL_BY_TOPIC` × sub `AUTOMATIC` のみ互換)
- `liveliness_lease_duration`: 5 s (両側、`pub.lease <= sub.lease` を満たす)

詳細・違反パターンは `msg/NavigationBackendStatus.msg` の冒頭コメント参照。

#### Custom bridge 実装者向けガイダンス (issue #207)

`backend_ready` の算出は bridge ごとに「実際に expose する capability の AND」とすること。`mapoi_nav_server` (Nav2 bridge) の `goal_ready && route_ready && switch_map_ready` は **3 capability 全部を expose する bridge にのみ正しい**。片機能 bridge (例: NavigateToPose のみ) でこれを真似ると、expose していない capability が常に false → `backend_ready` も常に false → UI が常に "Navigation unavailable" になる。

具体例とフィールド populate 方針 (`reason` の慣習表記含む) は `msg/NavigationBackendStatus.msg` の冒頭コメントに記載。

## サービス (srv)

### SelectMap.srv

現在の地図 context を切り替える Nav2-free サービスです。Operator mode の地図切替は `/mapoi/nav/switch_map` topic で指示し、`mapoi_nav_server` がこの service を呼んだ後に Nav2 `LoadMap` を実行します。

| 方向 | フィールド | 型 | 説明 |
| --- | --- | --- | --- |
| Request | `map_name` | `string` | 選択する地図名 |
| Request | `initial_poi_name` | `string` | 初期姿勢に使う POI 名。空なら先頭の有効 POI |
| Response | `success` | `bool` | context 選択成功の有無 |
| Response | `error_message` | `string` | 失敗時の理由 |
| Response | `config_path` | `string` | 選択後の設定ファイル path |
| Response | `initial_poi_name` | `string` | 解決済み初期姿勢 POI 名 |
| Response | `nav2_node_names` | `string[]` | Nav2 map server node 名 |
| Response | `nav2_map_urls` | `string[]` | 対応する map YAML path |

### GetMapsInfo.srv

利用可能な地図の一覧と現在の地図名を取得するサービスです。

| 方向 | フィールド | 型 | 説明 |
| --- | --- | --- | --- |
| Response | `maps_list` | `string[]` | 利用可能な地図名のリスト |
| Response | `map_name` | `string` | 現在の地図名 |

### GetPoisInfo.srv

現在の地図に登録されている全 POI を取得するサービスです。

| 方向 | フィールド | 型 | 説明 |
| --- | --- | --- | --- |
| Response | `pois_list` | `PointOfInterest[]` | POI のリスト |

### GetRoutePois.srv

指定されたルートに含まれる POI を取得するサービスです。

| 方向 | フィールド | 型 | 説明 |
| --- | --- | --- | --- |
| Request | `route_name` | `string` | ルート名 |
| Response | `pois_list` | `PointOfInterest[]` | ルート上の POI のリスト |

### GetRoutesInfo.srv

利用可能なルートの一覧を取得するサービスです。

| 方向 | フィールド | 型 | 説明 |
| --- | --- | --- | --- |
| Response | `routes_list` | `string[]` | ルート名のリスト |

### GetTagDefinitions.srv

タグ定義（システムタグ・ユーザータグ）を取得するサービスです。

| 方向 | フィールド | 型 | 説明 |
| --- | --- | --- | --- |
| Response | `tag_names` | `string[]` | タグ名のリスト |
| Response | `tag_descriptions` | `string[]` | タグ説明のリスト |
| Response | `is_system` | `bool[]` | システムタグかどうか（`true` = システムタグ） |
