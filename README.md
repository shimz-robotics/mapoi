# mapoi

Navigation2 向けの地図（Map）と関心地点（POI: Point of Interest）を管理するメタパッケージです。
地図の切り替え、POI の管理、RViz2 GUI からの自律走行操作、POI 半径イベントの検知を提供します。

## パッケージ構成

| パッケージ | 説明 |
| --- | --- |
| [mapoi_server](./mapoi_server/) | 地図・POI 情報の管理サーバー、ナビゲーションサーバー、RViz2 マーカー配信（メインパッケージ） |
| [mapoi_interfaces](./mapoi_interfaces/) | メッセージ・サービスの定義 |
| [mapoi_rviz_plugins](./mapoi_rviz_plugins/) | RViz2 プラグイン（地図切替・POI 選択・自律走行の GUI、POI エディタ） |
| [mapoi_webui](./mapoi_webui/) | Web UI（ブラウザからの地図表示・POI 編集・ナビゲーション操作・ロボット位置表示） |
| [mapoi_turtlebot3_example](./mapoi_turtlebot3_example/) | TurtleBot3 シミュレーション環境でのサンプル |

## ビルドとサンプルの実行

```sh
# cd path/to/your_ws
git clone git@github.com:shimz-robotics/mapoi.git src/mapoi_packages
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch mapoi_turtlebot3_example turtlebot3_navigation_launch.yaml
```

ブラウザから Web UI にアクセス:

http://localhost:8765

スマートフォンからも同一ネットワーク内であればアクセスできます。その場合、localhostの部分を実行しているPCのIPアドレスに変更してください。
地図表示・POI 編集・ナビゲーション操作・ロボット位置表示が可能です。

コマンドで目的地をしたい場合には、別ターミナルから自律走行をテストできます。

```sh
ros2 topic pub -1 /mapoi_goal_pose_poi std_msgs/msg/String "{data: conference_room}"
```

## Docker で試す（demo）

Docker イメージは **demo / 動作確認用**です。自作ロボットに組み込む場合は
次節「自分のロボットへの導入方法」を参照してください（将来 apt/rosdep での
提供を予定しています）。

Linux ホストが前提で、Turtlebot3 サンプルを Gazebo / RViz2 / Web UI で体験できます。

### 前提

- Docker Engine + Docker Compose v2（ソースからビルドする場合）
- Linux ホスト（X11 経由で GUI 表示）
- GPU は不要（CPU レンダリングで動作）

### ビルド済みイメージから試す（最速）

ghcr.io に公開されたイメージを pull するだけで起動できます。

```sh
xhost +local:docker
docker run --rm -it --network host --ipc host \
  -e DISPLAY=$DISPLAY \
  -e GAZEBO_MODEL_DATABASE_URI= \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/home/ros/.Xauthority:ro \
  ghcr.io/shimz-robotics/mapoi:latest
```

起動後、ブラウザで http://localhost:8765 にアクセスすると Web UI が表示されます。

利用可能なタグ:

| タグ | 説明 |
| --- | --- |
| `latest` | 推奨版（現状 ROS 2 Humble ベース、随時更新） |
| `humble` | ROS 2 Humble ベースで pin したい場合 |
| `vX.Y.Z` | リリースタグ版（primary distro） |
| `vX.Y.Z-humble` | リリースタグ版の humble ビルド |

中身の ROS 2 バージョンは `docker inspect ghcr.io/shimz-robotics/mapoi:latest | grep ROS_DISTRO` で確認できます。

### ソースから docker compose build（開発者向け）

ローカル変更を含めて試したい場合は、リポジトリを clone して compose でビルドします。

```sh
git clone git@github.com:shimz-robotics/mapoi.git
cd mapoi
xhost +local:docker
docker compose up demo
```

起動後、ブラウザで http://localhost:8765 にアクセスすると Web UI が表示されます。

### 開発用（ホストのソースを bind mount、コンテナ内でビルド）

```sh
# 初回ビルド
docker compose build dev

# シェルに入る（ホストの ./ が /ros2_ws/src/mapoi にマウント済み）
docker compose run --rm dev

# コンテナ内
cd /ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch mapoi_turtlebot3_example turtlebot3_navigation_launch.yaml
```

ホスト側の `git` / VSCode でソース編集し、コンテナ内で再ビルドして反復できます。

### 権限（UID/GID）の調整

bind mount 時の所有者ずれを避けるため、ホストの UID/GID に合わせてビルドできます:

```sh
USER_ID=$(id -u) GROUP_ID=$(id -g) docker compose build dev
```

### 将来の Jazzy 対応

Dockerfile は `ARG ROS_DISTRO` でディストリを切替可能にしてあります。

```sh
ROS_DISTRO=jazzy docker compose build
```

ただし Jazzy では Gazebo Classic → Gazebo Harmonic への移行など、別途対応が必要です（別 Issue で対応予定）。

## 主な機能

- **地図管理**: 複数地図の切り替え、Nav2 との連携
- **POI 管理**: YAML ベースの POI 定義、サービス経由での取得
- **自律走行**: POI 名指定でのゴール走行、ルート走行、一時停止・再開
- **POI 半径イベント**: POI の半径にロボットが侵入/退出した際のイベント発行
- **タグシステム**: システムタグ（`goal`, `origin`, `pause`）とユーザー定義タグによる POI 分類
- **RViz2 GUI**: 地図切替・ゴール指定・ルート走行の操作パネル、POI エディタ、ポーズ指定ツール
- **Web UI**: ブラウザからの地図表示・POI 編集・ナビゲーション操作・ロボット位置表示（スマートフォン対応）
- **マーカー表示**: RViz2 上での POI 可視化、ハイライト表示、半径表示

## 自分のロボットへの導入方法

現状は本リポジトリを clone して colcon build で組み込んでください。
**将来的に apt/rosdep での提供を予定しています**（別 Issue で対応）。

### 1. 地図ディレクトリの作成

パッケージ内に `maps/<地図名>/` ディレクトリを作成し、地図ファイルと設定ファイルを配置します。

```
maps/
├── site_a/
│   ├── mapoi_config.yaml    # POI・ルート・ユーザータグ設定
│   ├── map.pgm              # 地図画像
│   └── map.yaml             # 地図メタデータ
└── site_b/
    ├── mapoi_config.yaml
    ├── map.pgm
    └── map.yaml
```

`mapoi_config.yaml` のフォーマットについては [mapoi_server の README](./mapoi_server/README.md) を参照してください。

### 2. launch ファイルへの追加

`your_robot_launch.yaml` に mapoi のノードを追加します。

```yaml
# Mapoi Server
- node:
    pkg: mapoi_server
    exec: mapoi_server
    param:
      - {name: maps_path, value: "$(find-pkg-share your_package)/maps"}
      - {name: map_name, value: "site_a"}
      - {name: config_file, value: "mapoi_config.yaml"}
      - {name: pub_interval_ms, value: 500}

- node:
    pkg: mapoi_server
    exec: mapoi_nav_server

- node:
    pkg: mapoi_server
    exec: mapoi_rviz2_publisher

# Mapoi Web UI（オプション）
- node:
    pkg: mapoi_webui
    exec: mapoi_webui_node.py
    param:
      - {name: maps_path, value: "$(find-pkg-share your_package)/maps"}
      - {name: map_name, value: "site_a"}
      - {name: web_port, value: 8765}
```

### 3. AMCL パラメータの設定

初期位置の自動設定や地図切り替え機能を利用する場合は、AMCL のパラメータに以下を追加します。

```yaml
amcl:
  ros__parameters:
    # 初期位置の自動設定
    set_initial_pose: True
    initial_pose: {x: -2.0, y: -0.5, yaw: 0.0}
    # 地図切り替えの有効化
    first_map_only_: False
```

### 4. POI 半径イベントの利用（オプション）

ユーザータグを持つ POI の半径内にロボットが入ると `mapoi_poi_events` トピックにイベントが配信されます。

```sh
# イベントの確認
ros2 topic echo /mapoi_poi_events
```

詳細は [mapoi_server の README](./mapoi_server/README.md) を参照してください。

## ライセンス

MIT
