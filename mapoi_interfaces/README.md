# mapoi_interfaces

mapoi パッケージ群で使用するメッセージとサービスの定義パッケージです。

## メッセージ (msg)

### PointOfInterest.msg

POI（Point of Interest）を表すメッセージです。

| フィールド | 型 | 説明 |
| --- | --- | --- |
| `id` | `int32` | POI の識別子 |
| `name` | `string` | POI の名前 |
| `pose` | `geometry_msgs/Pose` | POI の位置・姿勢 |
| `radius` | `float64` | POI の半径 |
| `tags` | `string[]` | POI に紐づくタグ（例: `goal`, `waypoint`, `audio_info`） |
| `description` | `string` | POI の説明 |

### PoiEvent.msg

POI 半径への侵入・退出イベントを表すメッセージです。

| フィールド | 型 | 説明 |
| --- | --- | --- |
| `EVENT_ENTER` | `uint8` (定数=1) | 侵入イベント |
| `EVENT_EXIT` | `uint8` (定数=2) | 退出イベント |
| `event_type` | `uint8` | イベント種別（`EVENT_ENTER` or `EVENT_EXIT`） |
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
