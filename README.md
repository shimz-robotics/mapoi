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

## バージョン方針 (SemVer)

本プロジェクトは現在 **v0.x の開発フェーズ** にあります。

- **v0.x 系**: API は安定していません。設計の見直しによる **破壊的変更が任意のリリースで発生する可能性** があります。各リリースの破壊的変更は [`CHANGELOG.rst`](./CHANGELOG.rst) と [GitHub Releases](https://github.com/shimz-robotics/mapoi/releases) で明示します
- **v1.0.0 以降**: 公開 API (msg / topic / service / launch param / YAML schema 等) の後方互換性を保証します。破壊的変更は major バージョン bump (v2.0.0 等) で明示します

## v0.3.0 topic migration

v0.3.0 では mapoi runtime topic を `/mapoi/...` namespace に再編しました。旧 flat `/mapoi_*` topic は互換 alias なしで削除したため、launch / script / RViz 設定 / 外部 node は下表の新 topic に更新してください。

| 旧 topic | 新 topic |
| --- | --- |
| `/mapoi_goal_marks` | `/mapoi/markers/pois` |
| `/mapoi_event_marks` | `/mapoi/markers/pois` |
| `/mapoi_route_marks` | `/mapoi/markers/routes` |
| `/mapoi_highlight_goal` | `/mapoi/highlight/goal` |
| `/mapoi_highlight_route` | `/mapoi/highlight/route` |
| `/mapoi_goal_pose_poi` | `/mapoi/nav/goal_pose_poi` |
| `/mapoi_route` | `/mapoi/nav/route` |
| `/mapoi_cancel` | `/mapoi/nav/cancel` |
| `/mapoi_pause` | `/mapoi/nav/pause` |
| `/mapoi_resume` | `/mapoi/nav/resume` |
| `/mapoi_switch_map` | `/mapoi/nav/switch_map` |
| `/mapoi_nav_status` | `/mapoi/nav/status` |
| `/mapoi_poi_events` | `/mapoi/events` |
| `/mapoi_config_path` | `/mapoi/config_path` |
| `/mapoi_initialpose_poi` | `/mapoi/initialpose_poi` |

Nav2 native topic (`/goal_pose`, `/initialpose`) と RViz PoseTool 由来の `mapoi_rviz_pose` は変更ありません。

## 計画中の破壊的変更

以下は v1.0.0 までに段階的に着手予定の破壊的変更です。具体的な実施タイミングは [GitHub Milestones](https://github.com/shimz-robotics/mapoi/milestones) を参照してください。

- system tag spec 整理 (段階 1 `goal` → `waypoint` リネーム / 段階 2 `route.landmarks` + `pause` 発火条件厳格化 / 段階 3 `initial_pose` 廃止) — [#89](https://github.com/shimz-robotics/mapoi/issues/89) 派生
- WebUI 地図切替時の backend map context 同期 — [#130](https://github.com/shimz-robotics/mapoi/issues/130) (Editor / Operator mode 分離 [#61](https://github.com/shimz-robotics/mapoi/issues/61) と合わせて検討)

## ビルドとサンプルの実行

```sh
# cd path/to/your_ws
git clone git@github.com:shimz-robotics/mapoi.git src/mapoi_packages
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch mapoi_turtlebot3_example turtlebot3_navigation.launch.yaml
```

ブラウザから Web UI にアクセス:

http://localhost:8765

スマートフォンからも同一ネットワーク内であればアクセスできます。その場合、localhostの部分を実行しているPCのIPアドレスに変更してください。
地図表示・POI 編集・ナビゲーション操作・ロボット位置表示が可能です。

コマンドで目的地をしたい場合には、別ターミナルから自律走行をテストできます。

```sh
ros2 topic pub -1 /mapoi/nav/goal_pose_poi std_msgs/msg/String "{data: conference_room}"
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

#### Humble (Gazebo Classic)

```sh
xhost +local:docker
docker run --rm -it --network host --ipc host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e GAZEBO_MODEL_DATABASE_URI= \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ghcr.io/shimz-robotics/mapoi:humble
```

#### Jazzy (Gazebo Harmonic)

```sh
xhost +local:docker
docker run --rm -it --network host --ipc host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ghcr.io/shimz-robotics/mapoi:jazzy
```

Nav2 lifecycle 立ち上げに 30〜60 秒ほどかかるため、起動直後の信号受信で kill せず、少し待ってから Web UI にアクセスしてください。

起動後、ブラウザで http://localhost:8765 にアクセスすると Web UI が表示されます。

X11 接続が拒否される場合は `xhost +local:` を試してください（ローカル unix socket 経由の全接続を許可）。配布イメージ内のユーザーは UID 1000 固定のため、`$HOME/.Xauthority` の bind mount は行わず、X11 アクセス許可は `xhost` に委ねています。

利用可能なタグ:

| タグ | 説明 |
| --- | --- |
| `latest` | 推奨版（現状 `humble` と同一、Jazzy 成熟後に段階的に `jazzy` へ切替予定） |
| `humble` | ROS 2 Humble + Gazebo Classic |
| `jazzy` | ROS 2 Jazzy + Gazebo Harmonic |
| `vX.Y.Z` | リリースタグ版（primary distro: humble） |
| `vX.Y.Z-humble` | リリースタグ版の humble ビルド |
| `vX.Y.Z-jazzy` | リリースタグ版の jazzy ビルド |

中身の ROS 2 バージョンは `docker inspect ghcr.io/shimz-robotics/mapoi:<タグ> | grep ROS_DISTRO` で確認できます。

### ソースから docker compose build（開発者向け）

ローカル変更を含めて試したい場合は、リポジトリを clone して compose でビルドします。

> **`ROS_DISTRO` 環境変数の挙動**: `docker compose` の image / build は `${ROS_DISTRO:-humble}` で distro を切り替えます。
> - shell / `.env` 由来の `ROS_DISTRO` が **非空** ならその値を採用 (例: host で ROS 2 Humble を source 済みなら `humble`、Jazzy なら `jazzy`)
> - **空または未設定** なら fallback default は `humble`
> - **別 distro を一時的に試す** には `ROS_DISTRO=jazzy docker compose ...` のように前置で上書き (親 shell の `ROS_DISTRO` には影響しない)
> - local image は `mapoi:<service>-${ROS_DISTRO}` の形 (例: `mapoi:dev-jazzy` / `mapoi:demo-humble`) で distro 別保存、並行運用可
> - 詳細: [Docker Compose interpolation](https://docs.docker.com/reference/compose-file/interpolation/)

> **DDS 環境の扱い**: container 内 DDS 環境変数は以下の方針で設定されます:
> - **`RMW_IMPLEMENTATION`**: shell の `${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}` で host 値を **継承** (未設定なら `rmw_fastrtps_cpp` fallback)。host と一致しないと `network_mode: host` + `ipc: host` 越しに container <-> host の DDS discovery が成立しないため、host が cyclonedds なら container も cyclonedds に揃える
> - **`ROS_DOMAIN_ID`**: 同様に `${ROS_DOMAIN_ID:-0}` で host 継承、未設定なら 0。multi-robot / parallel simulation で domain 分離している運用も尊重
> - **`CYCLONEDDS_URI`**: 常に **空で固定** (host の custom URI を継承させない)。host で別 workspace の `CYCLONEDDS_URI=/path/to/custom.xml` を export していても、container 内では cyclonedds default config で動作 (これが過去 SwitchMap / discovery 障害の原因だった事例 → blocking)
> - Override したい場合 (例: 別 RMW で試す、別 domain 指定):
>   - `ROS_DISTRO=humble docker compose run --rm --env ROS_DOMAIN_ID=42 dev` で個別 override
>   - host の cyclonedds custom config を container でも使いたい (rare):
>     `ROS_DISTRO=humble docker compose run --rm --env CYCLONEDDS_URI=file:///path/to/cyclonedds.xml dev`
> - **注意**: `docker compose up` では `--env` flag は使えないため、`up` 経由で override が必要なら `docker-compose.yml` 編集または compose override file を使う

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
ros2 launch mapoi_turtlebot3_example turtlebot3_navigation.launch.yaml
```

ホスト側の `git` / VSCode でソース編集し、コンテナ内で再ビルドして反復できます。

### 権限（UID/GID）の調整

container 内 `ros` user (default UID 1000) と host user の UID/GID が一致しないと、bind mount (`./:/ros2_ws/src/mapoi`) 越しの **書き込み操作で permission denied** が発生します。代表ケース:

- POI Editor Panel の **Save** クリック (mapoi_config.yaml に書き込み)
- container 内 `colcon build` の install/build/log 出力 (host にも残る場合)
- container 内で source ファイルを編集

**host UID が 1000 と異なる場合は必須** の手順 (UID 1000 の host では default のまま動作):

```sh
# 推奨: .env を生成して docker compose に自動 inject (.gitignore + .dockerignore 済)
echo "USER_ID=$(id -u)"   > .env
echo "GROUP_ID=$(id -g)" >> .env
docker compose build dev    # .env が自動で読み込まれる

# あるいは都度 prefix:
USER_ID=$(id -u) GROUP_ID=$(id -g) docker compose build dev
```

`.env` 作成後は build / run / up すべての compose コマンドで自動的に host UID が反映されます。各 key の意味とコメントは [.env.example](.env.example) を参照。distro 別 image (mapoi:dev-${ROS_DISTRO}) ごとに UID は build 時に焼き込まれるため、UID 変更時は対象 distro の image を rebuild してください。

### GPU (NVIDIA) で加速する

デフォルトは CPU レンダリング (Mesa llvmpipe) で GPU 非依存ですが、NVIDIA GPU で RViz2 / Gazebo の描画を加速できます。

ホスト要件:
- NVIDIA ドライバがインストールされている Linux ホスト
- [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) が設定済み

pull 済みイメージから起動する場合（`--gpus all` を追加）:

```sh
xhost +local:docker
docker run --rm -it --network host --ipc host --gpus all \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e QT_X11_NO_MITSHM=1 \
  -e GAZEBO_MODEL_DATABASE_URI= \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ghcr.io/shimz-robotics/mapoi:humble
```

compose から起動する場合（`docker-compose.gpu.yml` を override で重ねる）:

```sh
docker compose -f docker-compose.yml -f docker-compose.gpu.yml up demo
```

GPU を使わない通常起動は `docker-compose.yml` 単体、`--gpus all` 無しの `docker run` を使ってください（既存手順そのまま）。

### Jazzy で compose から試す

Dockerfile / docker-compose は `ARG ROS_DISTRO` でディストリを切替可能です。Jazzy (Gazebo Harmonic) でソースビルドする場合:

```sh
ROS_DISTRO=jazzy docker compose build
ROS_DISTRO=jazzy docker compose up demo
```

ローカル image は `mapoi:dev-${ROS_DISTRO}` / `mapoi:demo-${ROS_DISTRO}` の形 (例: `mapoi:dev-jazzy` / `mapoi:demo-humble`) で distro 別に保存されます。Humble と Jazzy を同時に保持・並行実行可能で、distro 切替時の rebuild も不要です。

pull 済みイメージを使う場合は `ghcr.io/shimz-robotics/mapoi:jazzy` を直接 `docker run` する方が手軽です（上の「ビルド済みイメージから試す」節を参照）。Jazzy は Nav2 lifecycle 立ち上げに 30〜60 秒かかるので起動直後に kill しないでください。

## 主な機能

- **地図管理**: 複数地図の切り替え、Nav2 との連携
- **POI 管理**: YAML ベースの POI 定義、サービス経由での取得
- **自律走行**: POI 名指定でのゴール走行、ルート走行、一時停止・再開
- **POI 半径イベント**: POI の半径にロボットが侵入/退出した際のイベント発行
- **タグシステム**: システムタグ（`waypoint`, `landmark`, `pause`）とユーザー定義タグによる POI 分類
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

初期位置は `mapoi_config.yaml` の `initial_pose` タグ付き POI を **single source of truth** として `mapoi_nav_server` が `/initialpose` topic に自動配信します。AMCL の `set_initial_pose` (Nav2 native の self-init) と二重管理にならないよう、AMCL 側はそれを無効化し、地図切り替えは `first_map_only_` を `False` にしてください。

```yaml
amcl:
  ros__parameters:
    # 初期位置は mapoi_nav_server から /initialpose topic 経由で配信される
    # POI single source of truth 化のため AMCL native の self-init は使わない
    set_initial_pose: False
    # 地図切り替えの有効化
    first_map_only_: False
```

initial_pose は `mapoi_config.yaml` 側で:

```yaml
poi:
  - name: elevator_hall
    pose: {x: -2.0, y: -0.5, yaw: 0.0}
    tags: [goal, initial_pose]
```

別 topic 名・別 message 型を使う localization パッケージへの対応については [`mapoi_server/README.md`](./mapoi_server/README.md) の「対応 localization パッケージの要件」節を参照してください。

### 4. POI 半径イベントの利用（オプション）

ユーザータグを持つ POI の半径内にロボットが入ると `mapoi/events` トピックにイベントが配信されます。

```sh
# イベントの確認
ros2 topic echo /mapoi/events
```

詳細は [mapoi_server の README](./mapoi_server/README.md) を参照してください。

## ライセンス

MIT
