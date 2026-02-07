# mapoi_interfaces

mapoi パッケージ群で使用するメッセージ、サービス、アクションの定義パッケージです。

## メッセージ (msg)

### PointOfInterest.msg

POI（Point of Interest）を表すメッセージです。

| フィールド | 型 | 説明 |
| --- | --- | --- |
| `id` | `int32` | POI の識別子 |
| `name` | `string` | POI の名前 |
| `pose` | `geometry_msgs/Pose` | POI の位置・姿勢 |
| `radius` | `float64` | POI の半径 |
| `tags` | `string[]` | POI に紐づくタグ（例: `goal`, `initial_pose`, `waypoint`） |
| `description` | `string` | POI の説明 |

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

