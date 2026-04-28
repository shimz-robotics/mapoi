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

### SwitchMap.srv

地図を切り替えるサービスです。

| 方向 | フィールド | 型 | 説明 |
| --- | --- | --- | --- |
| Request | `map_name` | `string` | 切り替え先の地図名 |
| Response | `success` | `bool` | 切り替え成功の有無 |

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
