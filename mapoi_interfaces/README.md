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
