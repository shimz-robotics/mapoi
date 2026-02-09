# mapoi_webui

mapoi の Web UI パッケージです。
ブラウザから地図の表示、POI の編集、ルートの確認、ナビゲーション操作、ロボット位置の監視ができます。
スマートフォンやタブレットからも利用可能です。

## 機能

- **地図表示**: 占有格子地図をブラウザ上に表示、地図の切り替え
- **POI 編集**: POI の追加・編集・削除・保存（mapoi_server に自動リロード通知）
- **ルート表示**: ルートのポリライン表示、矢印マーカー、表示/非表示の切り替え
- **ナビゲーション操作**: POI へのゴール走行、ルート走行、停止
- **自己位置推定リセット**: POI 選択による Initial Pose の設定
- **ロボット位置表示**: TF (`map` → `base_link`) によるリアルタイムのロボット位置マーカー表示
- **タグシステム**: システムタグ・ユーザータグの表示、タグによる POI 色分け

## ノード

### mapoi_webui_node

Flask ベースの HTTP サーバーを内蔵した ROS2 ノードです。

#### パラメータ

| パラメータ名 | 型 | デフォルト | 説明 |
| --- | --- | --- | --- |
| `maps_path` | `string` | `""` | 地図ディレクトリのパス |
| `map_name` | `string` | `turtlebot3_world` | 起動時に読み込む地図名 |
| `config_file` | `string` | `mapoi_config.yaml` | 設定ファイル名 |
| `web_port` | `int` | `8765` | HTTP サーバーのポート |
| `web_host` | `string` | `0.0.0.0` | HTTP サーバーのバインドアドレス |
| `map_frame` | `string` | `map` | TF の親フレーム |
| `base_frame` | `string` | `base_link` | TF の子フレーム |

#### パブリッシャー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi_goal_pose_poi` | `std_msgs/String` | ゴール POI 名の配信 |
| `mapoi_route` | `std_msgs/String` | ルート走行の開始 |
| `mapoi_cancel` | `std_msgs/String` | ナビゲーションのキャンセル |
| `mapoi_initialpose_poi` | `std_msgs/String` | 初期位置設定の POI 名 |

#### サブスクライバー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi_nav_status` | `std_msgs/String` | ナビゲーション状態の受信 |
| `mapoi_config_path` | `std_msgs/String` | 外部からの地図切り替え検知 |

#### TF

| 変換 | 説明 |
| --- | --- |
| `map` → `base_link` | ロボット位置の取得（5Hz） |

## REST API

| メソッド | エンドポイント | 説明 |
| --- | --- | --- |
| GET | `/api/maps` | 地図一覧と現在の地図名 |
| GET | `/api/maps/<name>/image` | 地図画像（PNG） |
| GET | `/api/maps/<name>/metadata` | 地図メタデータ（解像度・原点・サイズ） |
| GET | `/api/pois` | POI 一覧 |
| POST | `/api/pois` | POI の保存 |
| GET | `/api/routes` | ルート一覧 |
| GET | `/api/tag_definitions` | タグ定義一覧 |
| GET | `/api/nav/status` | ナビゲーション状態・ロボット位置 |
| POST | `/api/nav/goal` | POI へのゴール走行 |
| POST | `/api/nav/route` | ルート走行の開始 |
| POST | `/api/nav/cancel` | ナビゲーションの停止 |
| POST | `/api/nav/initialpose` | 自己位置推定のリセット |

## 起動方法

```bash
ros2 launch mapoi_webui mapoi_webui_launch.yaml maps_path:=/path/to/maps map_name:=your_map
```

ブラウザで `http://<ホスト>:8765` にアクセスしてください。

## 依存パッケージ

- `rclpy`
- `std_msgs`, `std_srvs`
- `mapoi_interfaces`
- `tf2_ros`
- `python3-flask`
- `python3-pil`
- `python3-yaml`
