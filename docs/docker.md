# Docker で試す（demo）

Docker イメージは **demo / 動作確認用**です。自作ロボットに組み込む場合は [自分のロボットへの導入方法](./integration.md) を参照してください（将来 apt/rosdep での提供を予定しています）。

Linux ホストが前提で、Turtlebot3 サンプルを Gazebo / RViz2 / Web UI で体験できます。

## 前提

- Docker Engine + Docker Compose v2（ソースからビルドする場合）
- Linux ホスト（X11 経由で GUI 表示）
- GPU は不要（CPU レンダリングで動作）

## ビルド済みイメージから試す（最速）

ghcr.io に公開されたイメージを pull するだけで起動できます。

### Jazzy (gz-sim、推奨)

```sh
xhost +local:docker
docker run --rm -it --network host --ipc host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ghcr.io/shimz-robotics/mapoi:jazzy
```

### Humble (Gazebo Classic、Classic は EoL 2025-01)

```sh
xhost +local:docker
docker run --rm -it --network host --ipc host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e GAZEBO_MODEL_DATABASE_URI= \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ghcr.io/shimz-robotics/mapoi:humble
```

Nav2 lifecycle 立ち上げに 30〜60 秒ほどかかるため、起動直後の信号受信で kill せず、少し待ってから Web UI にアクセスしてください。

起動後、ブラウザで http://localhost:8765 にアクセスすると Web UI が表示されます。

X11 接続が拒否される場合は `xhost +local:` を試してください（ローカル unix socket 経由の全接続を許可）。配布イメージ内のユーザーは UID 1000 固定のため、`$HOME/.Xauthority` の bind mount は行わず、X11 アクセス許可は `xhost` に委ねています。

利用可能なタグ:

| タグ | 説明 |
| --- | --- |
| `latest` | 推奨版 (`jazzy` と同一、primary distro) |
| `jazzy` | ROS 2 Jazzy + gz-sim (Gazebo Harmonic)、primary distro |
| `humble` | ROS 2 Humble + Gazebo Classic、Humble 環境向け継続提供 (Gazebo Classic は 2025-01 に EoL 済、ROS 2 Humble 自体は 2027-05 まで。新規プロジェクトでは Jazzy + gz-sim 推奨) |
| `vX.Y.Z` | リリースタグ版 (primary distro: jazzy) |
| `vX.Y.Z-jazzy` | リリースタグ版の jazzy ビルド |
| `vX.Y.Z-humble` | リリースタグ版の humble ビルド |

中身の ROS 2 バージョンは `docker inspect ghcr.io/shimz-robotics/mapoi:<タグ> | grep ROS_DISTRO` で確認できます。

## ソースから docker compose build（開発者向け）

ローカル変更を含めて試したい場合は、リポジトリを clone して compose でビルドします。

> **`ROS_DISTRO` 環境変数の挙動**: `docker compose` の image / build は `${ROS_DISTRO:-jazzy}` で distro を切り替えます。
> - shell / `.env` 由来の `ROS_DISTRO` が **非空** ならその値を採用 (例: host で ROS 2 Humble を source 済みなら `humble`、Jazzy なら `jazzy`)
> - **空または未設定** なら fallback default は `jazzy` (primary distro)
> - **別 distro を一時的に試す** には `ROS_DISTRO=humble docker compose ...` のように前置で上書き (親 shell の `ROS_DISTRO` には影響しない)
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

## 開発用（ホストのソースを bind mount、コンテナ内でビルド）

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

## 権限（UID/GID）の調整

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

`.env` 作成後は build / run / up すべての compose コマンドで自動的に host UID が反映されます。各 key の意味とコメントは [.env.example](../.env.example) を参照。distro 別 image (mapoi:dev-${ROS_DISTRO}) ごとに UID は build 時に焼き込まれるため、UID 変更時は対象 distro の image を rebuild してください。

## GPU (NVIDIA) で加速する

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
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ghcr.io/shimz-robotics/mapoi:jazzy
```

compose から起動する場合（`docker-compose.gpu.yml` を override で重ねる）:

```sh
docker compose -f docker-compose.yml -f docker-compose.gpu.yml up demo
```

GPU を使わない通常起動は `docker-compose.yml` 単体、`--gpus all` 無しの `docker run` を使ってください（既存手順そのまま）。

## Humble で compose から試す

`docker compose` の default は `jazzy` (primary distro) ですが、`ARG ROS_DISTRO` で Humble (Gazebo Classic) に切り替え可能です:

```sh
ROS_DISTRO=humble docker compose build
ROS_DISTRO=humble docker compose up demo
```

ローカル image は `mapoi:dev-${ROS_DISTRO}` / `mapoi:demo-${ROS_DISTRO}` の形 (例: `mapoi:dev-jazzy` / `mapoi:demo-humble`) で distro 別に保存されます。Humble と Jazzy を同時に保持・並行実行可能で、distro 切替時の rebuild も不要です。

pull 済みイメージを使う場合は `ghcr.io/shimz-robotics/mapoi:humble` を直接 `docker run` する方が手軽です（上の「ビルド済みイメージから試す」節を参照）。Humble は Nav2 lifecycle 立ち上げに 30〜60 秒かかるので起動直後に kill しないでください。

> **Gazebo Classic の状態**: `humble` image は Gazebo Classic を含みますが、**Gazebo Classic 自体は 2025-01 に EoL** (osrf による最終リリース 11 系で archived)、新規バグ修正や apt 配布が止まっています。新規プロジェクトでは Jazzy + gz-sim (Gazebo Harmonic、active maintained) を推奨します。ROS 2 Humble 自体の EoL は 2027-05 ですが、シミュレータ側の状況がより制約的です。
