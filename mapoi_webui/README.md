# mapoi_webui

mapoi の Web UI パッケージです。
ブラウザから地図の表示、POI の編集、ルートの確認、ナビゲーション操作、ロボット位置の監視ができます。
スマートフォンやタブレットからも利用可能です。

## 機能

- **地図表示**: 占有格子地図をブラウザ上に表示、地図の切り替え
- **POI 編集**: POI の追加・編集・削除・保存（mapoi_server に自動リロード通知）
- **ルート表示**: ルートのポリライン表示、矢印マーカー、表示/非表示の切り替え
- **ナビゲーション操作**: POI へのゴール走行、ルート走行、一時停止・再開、停止
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
| `mapoi_pause` | `std_msgs/String` | ナビゲーションの一時停止 |
| `mapoi_resume` | `std_msgs/String` | ナビゲーションの再開 |
| `mapoi_cancel` | `std_msgs/String` | ナビゲーションのキャンセル |
| `mapoi_initialpose_poi` | `std_msgs/String` | 初期位置設定の POI 名 |

#### サブスクライバー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi_nav_status` | `std_msgs/String` | ナビゲーション状態の受信（`"status"` / `"status:target"` 形式、transient_local QoS）。`:` split で target を抽出し REST `/api/nav/status` の `target` field として公開 |
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
| POST | `/api/nav/pause` | ナビゲーションの一時停止 |
| POST | `/api/nav/resume` | ナビゲーションの再開 |
| POST | `/api/nav/cancel` | ナビゲーションの停止 |
| POST | `/api/nav/initialpose` | 自己位置推定のリセット |

## 起動方法 (3 つのシナリオ)

利用目的に応じて 3 つの起動方法を使い分けてください。いずれもブラウザで `http://<ホスト>:8765` にアクセスします。

### A. webui のみ起動 (オペレーター用、`mapoi_server` を別プロセスで起動済み)

ロボットが既に動いている (`mapoi_server` / `mapoi_nav_server` 等が別プロセスで稼働) 状態で、RViz の代替 UI として webui を後付けする用途。

```bash
ros2 launch mapoi_webui mapoi_webui.launch.yaml maps_path:=/path/to/maps map_name:=your_map
```

### B. webui + mapoi_server を一緒に起動 (エディター用)

PC でデスクトップ上の POI/Route/CustomTags 編集だけ行いたい用途 (ロボット動作 / Nav2 / RViz は不要)。`mapoi_bringup` を minimal config (`with_nav_server=false`, `with_rviz_publisher=false`, `simulator=none`) で include する。

```bash
ros2 launch mapoi_webui mapoi_editor.launch.yaml maps_path:=/path/to/maps map_name:=your_map
```

### C. 統合運用 / デモ (Nav2 + sim + webui を全部)

`mapoi_turtlebot3_example` の `turtlebot3_navigation.launch.yaml` のような bringup + webui 統合 launch を使う。詳細はそのパッケージの README を参照。

## REST API と server 依存

各 endpoint が `mapoi_server` (or `mapoi_nav_server`) の起動を必要とするかの早見表:

| endpoint | server 必要 | 理由 |
|---|---|---|
| `GET /api/maps` `/maps/<name>/image\|metadata` | 不要 | `maps_path` を直 ls / YAML/PNG 直読み |
| `GET /api/pois` `/api/routes` | 不要 | YAML 直読み |
| `POST /api/pois` `/api/routes` `/api/custom_tags` | **必要** | YAML 書き込み後に `reload_map_info` service を呼ぶ |
| `GET /api/tag_definitions` | **必要** | `get_tag_definitions` service 経由 |
| `POST /api/nav/{goal,route,cancel,pause,resume,initialpose}` | **必要 (mapoi_nav_server)** | publisher → nav_server が listener、subscriber 0 件なら warning を返す |
| `GET /api/nav/status` | 不要 (subscriber 経由で受信値返却) | nav_server がいなければ default 値 |

server / nav_server 不在時の挙動 (endpoint カテゴリ別):
- `GET /api/tag_definitions`: `503 Service Unavailable` (service 必須、unavailable / timeout 時)
- `POST /api/pois` / `/api/routes` / `/api/custom_tags`: `200 OK` + `warning` フィールド (YAML 書き込み自体は成功するため、`reload_map_info` の unavailable / timeout / failure は warning として通知)
- `POST /api/nav/{goal,route,cancel,pause,resume,initialpose}`: `200 OK` + `warning` フィールド (publish 自体は成功扱い、subscriber 不在を best-effort で検出して通知)

## 開発者規約

### Flask thread から ROS 2 service を呼ぶ場合

**必ず `MapoiWebNode._call_service_sync()` 経由で呼ぶ**。直接 `client.call_async()` を fire-and-forget したり、`spin_until_future_complete()` を Flask thread から呼ぶのは禁止 (前者は silent failure、後者は main thread の `rclpy.spin` と executor 競合)。

```python
response = node._call_service_sync(
    node.some_client_, SomeReq(), 'some_service', timeout_sec=3.0)
if response is None:
    return jsonify({'error': 'service unavailable'}), 503
```

### Flask thread から ROS 2 publish する場合

**必ず `MapoiWebNode.publish_with_subscriber_check()` 経由で publish する**。subscriber 0 件 (例: `mapoi_nav_server` 未起動) の silent failure を避けるため。

```python
_, warning = node.publish_with_subscriber_check(
    node.some_pub_, msg, 'some_topic')
return jsonify({'success': True, 'warning': warning} if warning else {'success': True})
```

## 依存パッケージ

- `rclpy`
- `std_msgs`, `std_srvs`
- `mapoi_interfaces`
- `tf2_ros`
- `python3-flask`
- `python3-pil`
- `python3-yaml`
