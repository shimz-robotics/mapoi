# mapoi_turtlebot3_example

TurtleBot3 シミュレーション環境で mapoi の動作を確認するためのサンプルパッケージです。
SLAM による地図作成から Navigation2 を用いた自律走行までの一連の手順を試すことができます。

## 依存パッケージのインストール

rosdep を使用する場合:

```sh
# cd path/to/your_ws/
rosdep install --from-paths src --ignore-src -r -y
```

手動でインストールする場合:

```sh
sudo apt install -y \
  ros-${ROS_DISTRO}-turtlebot3-simulations \
  ros-${ROS_DISTRO}-turtlebot3-cartographer \
  ros-${ROS_DISTRO}-turtlebot3-navigation2 \
  ros-${ROS_DISTRO}-mouse-teleop
```

## クイックスタート

```sh
export TURTLEBOT3_MODEL=burger
ros2 launch mapoi_turtlebot3_example turtlebot3_navigation.launch.yaml
```

別ターミナルから POI への自律走行をテスト:

```sh
ros2 topic pub -1 /mapoi/nav/goal_pose_poi std_msgs/msg/String "{data: goal}"
```

### 機能ごとの demo route (#230)

`turtlebot3_world` map には mapoi の主な機能を 1 route ずつ確認できる tutorial 系と、全機能を 1 route で踏む `tour_full` の合計 5 route が定義されています。7 POI で全機能をカバーする抽象命名構成 (自前 robot への流用を想定)。

推奨確認順序:

1. **`tutorial_01_basic`** — 基本 navigation health check (waypoint 3 点を順に走破)
2. WebUI / RViz の **map switch / POI 編集** をブラウザで操作 (route 不要)
3. **`tutorial_02_landmark`** — `route.landmarks` の概念 (`landmark_listed` のみ pub、`landmark_unlisted` は同 map 上に居ても pub されないコントラスト)
4. **`tutorial_03_pause`** — `pause` タグの動作 (手動 resume / `auto_resume_timeout_sec` opt-in による自動 resume)
5. **`tutorial_04_event_tag`** — custom tag + sample subscriber 連携 (`audio_info` → `audio_guide_node`)
6. **`tour_full`** — 全部入り総合 demo

各 route の起動例:

```sh
# 基本: 3 点 waypoint
ros2 topic pub -1 /mapoi/nav/route std_msgs/msg/String "{data: tutorial_01_basic}"

# landmark コントラスト: 別 terminal で `ros2 topic echo /mapoi/events`
# を流し、landmark_listed のみ EVENT_ENTER が出ることを確認
ros2 topic pub -1 /mapoi/nav/route std_msgs/msg/String "{data: tutorial_02_landmark}"

# pause + 手動 resume: 停止後に別 shell から
ros2 topic pub -1 /mapoi/nav/route std_msgs/msg/String "{data: tutorial_03_pause}"
ros2 topic pub -1 /mapoi/nav/resume std_msgs/msg/String "{data: ''}"

# pause + auto-resume (mapoi_nav2_bridge の auto_resume_timeout_sec を正値で起動した場合)
ros2 topic pub -1 /mapoi/nav/route std_msgs/msg/String "{data: tutorial_03_pause}"

# custom tag + EVENT_ENTER: audio_guide_node が反応
ros2 topic pub -1 /mapoi/nav/route std_msgs/msg/String "{data: tutorial_04_event_tag}"

# 総合 demo
ros2 topic pub -1 /mapoi/nav/route std_msgs/msg/String "{data: tour_full}"
```

### Headless 起動（GUI なし）

CI / SSH 接続環境 / 軽量検証用に RViz2・Gazebo GUI を起動しない形で立ち上げられます。WebUI (http://localhost:8765) は従来どおり使用可能です。

```sh
ros2 launch mapoi_turtlebot3_example turtlebot3_navigation.launch.yaml \
  rviz:=false gazebo_gui:=false
```

個別制御も可能:

| arg | default | 効果 |
|---|---|---|
| `rviz` | `true` | RViz2 の起動有無 |
| `gazebo_gui` | `true` | シミュレータ GUI の起動有無。`false` でも server (Humble: `gzserver` / Jazzy: `gz sim -s`) は起動し物理シミュレーションは動作 |

`gazebo_gui` は Humble (Gazebo Classic) と Jazzy (gz-sim) のどちらでも有効です。distro 別の wrapper (`gazebo_headless_aware.launch.yaml` / `gz_sim_headless_aware.launch.yaml`) を `turtlebot3_navigation.launch.yaml` から `ROS_DISTRO` で使い分けています。`rviz:=false` は distro によらず動作。

## 地図の作成（SLAM）

### 1. シミュレーターの起動

```sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 2. Cartographer の起動

```sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### 3. ロボットの操作

```sh
ros2 run mouse_teleop mouse_teleop --ros-args -r /mouse_vel:=/cmd_vel
```

マウスでロボットを操作して環境全体を走行させ、地図を完成させます。

### 4. 地図の保存

```sh
ros2 run nav2_map_server map_saver_cli -f maps/<地図名>/<地図名>
```

### 5. 設定ファイルの編集

保存した地図ディレクトリに `mapoi_config.yaml` を作成し、POI とルートを設定します。
フォーマットの詳細は [mapoi_server の README](../mapoi_server/README.md) を参照してください。

## サンプルクライアントノード

mapoi のサービス・アクションの使用例として、以下のクライアントノードが含まれています。

| ノード名 | 説明 |
| --- | --- |
| `get_maps_info_client` | 地図一覧の取得 |
| `get_pois_info_client` | POI 一覧の取得 |
| `mapoi_switch_map_client` | `/mapoi/nav/switch_map` による operator 地図切り替え |
| `navigate_to_pose_client` | 指定位置へのナビゲーション |
| `follow_waypoints_client` | ウェイポイント追従 |
| `print_initialpose` | 初期位置の表示 |
| `audio_guide_node` | `mapoi/events` の `EVENT_ENTER` を受けて `audio_info` POI で音声ガイド再生 (mock) (#88) |

実行例:

```sh
ros2 run mapoi_turtlebot3_example get_maps_info_client
ros2 run mapoi_turtlebot3_example get_pois_info_client
```

## PoiEvent 駆動 sample subscriber (#88)

`mapoi/events` の PoiEvent (`EVENT_ENTER` / `EVENT_PAUSED` / `EVENT_EXIT`) を受けて custom tag に応じたアクションを実行する参照実装です。route 走行中の特定 POI で「音声ガイド再生」等のシナリオを組む際の雛形になります。

本 PR では `EVENT_ENTER` を扱う `audio_guide_node` のみ同梱しています。`EVENT_PAUSED` / `EVENT_EXIT` 用 subscriber も同型 (`event_type` の比較値を `EVENT_PAUSED` / `EVENT_EXIT` に差し替えるだけ) で書けるため、別 sample は同梱せず必要になれば follow-up で再導入する方針です。pause / resume のしくみ自体の動作確認は `tutorial_03_pause` route で行えます。

### 起動方法

`turtlebot3_navigation.launch.yaml` は demo 用集約 launch のため、**audio_guide_node を常時起動** します:

```sh
ros2 launch mapoi_turtlebot3_example turtlebot3_navigation.launch.yaml
```

自前 robot に sample subscriber **だけ** を流用したい場合は単独 launch を使用:

```sh
ros2 launch mapoi_turtlebot3_example mapoi_event_samples.launch.yaml
```

### 動作確認シナリオ

`turtlebot3_world` map の `tutorial_04_event_tag` route を走行 (`audio_info` 付き `mid_audio` を含む):

```sh
ros2 topic pub -1 /mapoi/nav/route std_msgs/msg/String "{data: tutorial_04_event_tag}"
```

期待 log:
- `mid_audio` 通過時 → `[AUDIO_GUIDE] play: poi='mid_audio' ...` (audio_guide_node)
- `tour_full` で `mid_pause` 到達時 → 自動 pause → cmd_vel dwell (sample subscriber 無しでも mapoi_nav2_bridge が pause を発火)。手動 resume または `auto_resume_timeout_sec` 正値起動で再開

### custom tag 命名 guideline

mapoi の **custom tag (system tag 以外のユーザー定義タグ)** は以下の命名規約を推奨します。本 sample の `audio_info` は既存 yaml 命名を尊重して残していますが、新規追加するなら下表のパターンを推奨します:

| 命名パターン | 意味 | 例 |
|---|---|---|
| `<action>_trigger` | この POI で起動するアクション | `audio_trigger` (音声ガイド再生)、`capture_trigger` (撮影起動) — 本 sample の `audio_info` は規約に厳密に従うなら `audio_trigger` 相当 |
| `<action>_target` | アクション対象として参照される POI | `capture_target` (撮影対象、landmark で参照) |
| `<state>` | POI が表す状態カテゴリ | `hazard` / `atrium` / `inspection` |

### 実装ポイント (subscriber 側)

- `EVENT_ENTER` は **route 走行中 + route 登録 POI** のみ発火する (#220 仕様、route 走行外 = IDLE / GOAL mode では発火しない)
- `EVENT_PAUSED` は **`pause` タグ付き POI 内で navigation が停止 (cmd_vel dwell)** した瞬間に発火、1 visit につき 1 回のみ
- `EVENT_EXIT` で各 POI の per-visit 状態が reset される (再 visit で再発火)

詳細仕様は [mapoi_interfaces の README](../mapoi_interfaces/README.md#poieventmsg) と [PoiEvent.msg](../mapoi_interfaces/msg/PoiEvent.msg) のヘッダコメント参照。

## サンプル地図

シナリオに即した POI 配置 + tag 組合せの全パターンをカバーする 2 サンプルを同梱しています (#146)。

| 地図名 | シナリオ | 検証できる主な機能 |
| --- | --- | --- |
| `turtlebot3_world` | 機能カタログ demo (機能ごとに 1 route + 総合 `tour_full`) | system tag 単独 / 複合、`tolerance.yaw` 厳密 ↔ 緩い、`route.waypoints + landmarks` 両方持ち、`pause` + 手動/自動 resume、`route.landmarks` 列挙/列挙外のコントラスト、custom tag (`audio_info`) |
| `turtlebot3_dqn_stage1` | 障害物 sandbox での回避走行 (ハザード / 観察 / 通過点) | 複合 tag (`event + landmark + custom`)、`tolerance.yaw=π` で yaw 不問の通過点 (= pass_through 代替)、`pause` 中継、`route.landmarks` の利用 |

### POI / tag 構成

#### turtlebot3_world (route: `tutorial_01_basic` / `tutorial_02_landmark` / `tutorial_03_pause` / `tutorial_04_event_tag` / `tour_full`)

| POI | tags | 役割 |
| --- | --- | --- |
| `start` | `waypoint` | POI list 先頭 = default initial pose (#149)、全 route 起点、Gazebo spawn 位置 (-2.0, -0.5) と一致 |
| `mid_basic` | `waypoint` | `tutorial_01` / `tour_full` の南側中継点 (pure waypoint) |
| `mid_pause` | `waypoint`, `pause` | `tutorial_03` / `tour_full` の `pause` POI (両 landmark の半径外) |
| `mid_audio` | `waypoint`, `audio_info` | `tutorial_04` / `tour_full` の `audio_info` POI (両 landmark の半径外) |
| `goal` | `waypoint` | `tutorial_01` / `02` / `tour_full` の終点 (両 landmark の北側) |
| `landmark_listed` | `landmark` | `tutorial_02` / `tour_full` の `route.landmarks` 列挙対象 |
| `landmark_unlisted` | `landmark` | `tutorial_02` の列挙外コントラスト (同 map 上に居ても pub されない) |

> **NOTE**: 連続 `pause` 発火 (旧 `corridor_a` / `corridor_b` の demo 役) は本 demo route から外しました。連続 pause の機能検証は launch_test で pin する follow-up を予定 (現状の `test_poi_event_route_integration.py` は単発のみ pin)。

#### turtlebot3_dqn_stage1 (route: `avoidance_a` / `avoidance_b`)

| POI | tags | 役割 |
| --- | --- | --- |
| `start_zone` | `waypoint` | POI list 先頭 = default initial pose (#149) |
| `checkpoint_west` | `waypoint`, `checkpoint` | yaw 不問の通過点 (`tolerance.yaw=π`) |
| `checkpoint_east` | `waypoint`, `checkpoint` | yaw 不問の通過点 (`tolerance.yaw=π`) |
| `pause_intersection` | `waypoint`, `pause` | 交差点で自動停止 |
| `north_goal` | `waypoint` | 北側目標、`tolerance.yaw=0.10` で厳密 |
| `hazard_south` | `event`, `landmark`, `hazard` | 複合 tag のハザード (Nav2 goal 不可) |
| `observation_point` | `landmark`, `observation` | 観察対象 (route 走行中の参照点) |

> **NOTE**: `landmark × pause` は #143 validation で reject されるため組み合わせていません。

参考: [TurtleBot3 E-Manual (Simulation)](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
