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

### GetTaggedPois.srv

指定されたタグに一致する POI を取得するサービスです。

| 方向 | フィールド | 型 | 説明 |
| --- | --- | --- | --- |
| Request | `tag` | `string` | フィルタするタグ（デフォルト: `"all"`） |
| Response | `pois_list` | `PointOfInterest[]` | 該当する POI のリスト |

### GetRoutePois.srv

指定されたルートに含まれる POI を取得するサービスです。

| 方向 | フィールド | 型 | 説明 |
| --- | --- | --- | --- |
| Request | `route_name` | `string` | ルート名 |
| Response | `pois_list` | `PointOfInterest[]` | ルート上の POI のリスト |

## アクション (action)

### FollowPoiList.action

POI 名のリストに従って順番にナビゲーションを行うアクションです。

| 区分 | フィールド | 型 | 説明 |
| --- | --- | --- | --- |
| Goal | `poi_list` | `string[]` | 巡回する POI 名のリスト |
| Goal | `behavior_tree` | `string` | 使用する Behavior Tree |
| Result | `result` | `std_msgs/Empty` | 完了結果 |
| Feedback | `poi_name` | `string` | 現在のナビゲーション先 POI 名 |
| Feedback | `current_pose` | `geometry_msgs/PoseStamped` | 現在のロボット位置 |
| Feedback | `navigation_time` | `builtin_interfaces/Duration` | ナビゲーション経過時間 |
| Feedback | `estimated_time_remaining` | `builtin_interfaces/Duration` | 推定残り時間 |
| Feedback | `number_of_recoveries` | `int16` | リカバリー回数 |
| Feedback | `distance_remaining` | `float32` | 残り距離 |
