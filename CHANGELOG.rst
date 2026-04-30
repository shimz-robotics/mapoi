^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This project follows `Semantic Versioning <https://semver.org/>`_. The 0.x
series is in active development; breaking changes may occur in any 0.x
release (per the SemVer 2.0.0 spec for 0.y.z initial-development phase).
From v1.0.0 onward, the public API (msgs, topics, services, launch
parameters, YAML schemas) will be backward-compatible across minor and
patch releases. See ``README.md`` for the full version policy.

For releases prior to v0.2.0, see the
`GitHub Releases page <https://github.com/shimz-robotics/mapoi/releases>`_.


Unreleased
==========

Breaking changes
----------------

* ``mapoi_server`` から sample maps を廃止し、``maps_path`` parameter を必須化 (#163 段階 1)。

  - 旧: ``mapoi_server`` package に ``maps/turtlebot3_world/`` /
    ``maps/turtlebot3_dqn_stage1/`` を同梱、``maps_path`` の default は
    ``{pkg_share}/maps``
  - 新: sample maps は ``mapoi_turtlebot3_example`` に集約 (single source of truth)。
    ``mapoi_server`` の ``maps_path`` は default なしで起動時必須、未指定で fatal
  - 残置: ``mapoi_server/maps/tag_definitions.yaml`` のみ system tag 定義として
    package-internal で残す
  - 移行: launch / param で ``maps_path`` を明示指定する。``mapoi_turtlebot3_example``
    の sample を流用するなら ``$(find-pkg-share mapoi_turtlebot3_example)/maps``

* ``mapoi_turtlebot3_example`` の sample map_file 名をワールド別差別化 (#163 段階 2)。

  - 旧: ``turtlebot3_world/turtlebot3.{pgm,yaml}`` /
    ``turtlebot3_dqn_stage1/turtlebot3.{pgm,yaml}`` (両 world で同名)
  - 新: ``turtlebot3_world/turtlebot3_world.{pgm,yaml}`` /
    ``turtlebot3_dqn_stage1/turtlebot3_dqn_stage1.{pgm,yaml}`` (ディレクトリ名と一致)
  - 各 ``mapoi_config.yaml`` の ``map_file`` 値、``image:`` 行も連動更新
  - 移行: 自前 launch で ``map_file`` を hardcode していなければ影響なし
    (``mapoi_config.yaml`` 経由で参照される)
  - 関連: ``scripts/check_sample_yaml_consistency.py`` の server / example pair
    sync 検査を廃止し、example 側 individual validation のみに簡素化

* ``mapoi_server`` の ``pub_interval_ms`` parameter を廃止 (#135)。

  - 旧: ``mapoi_config_path`` topic を ``pub_interval_ms`` (default 5000ms、
    sample launch では 500ms) 間隔で定期 publish
  - 新: 起動時 / ``SwitchMap`` / ``reload_map_info`` で明示 publish。
    publisher は ``transient_local`` QoS のまま、subscriber 側も
    ``transient_local`` に統一 (``poi_editor`` / ``mapoi_panel`` /
    ``mapoi_webui_node`` を default QoS から変更)
  - 移行: launch ファイル / param yaml で ``pub_interval_ms`` を指定して
    いる場合は削除する (引き続き渡しても unused parameter として ROS が
    warn するだけで動作には影響しない)
  - 動機: 編集中の ``poi_editor`` table / ``mapoi_panel`` ComboBox が定期
    publish trigger で再構築されて選択 / テキスト入力が中断される問題
    (PR #168 動作確認で発覚)。``transient_local`` QoS で起動順序問題は解消
    済みなので heartbeat 不要

* ``initial_pose`` system tag を廃止 (#89 段階 3, #144)。

  - 旧: ``initial_pose`` タグ付き POI の pose を地図ロード/切替時に自動配信
  - 新: 新 map の **POI list 先頭** (landmark タグ POI は除外) を default として
    採用。``SwitchMap.srv`` に追加した ``initial_poi_name`` で明示指定も可
  - 配信経路: ``mapoi_server`` が新 map ロード時に POI 名を
    ``mapoi_initialpose_poi`` (transient_local) に publish、
    ``mapoi_nav_server`` がそれを受けて ``/initialpose`` に流す
  - YAML migration: ``tags: [..., initial_pose]`` 行から ``initial_pose`` を
    削除し、開始 POI を ``poi:`` 配下の先頭に並べる
  - ``mapoi_gazebo_bridge`` / ``mapoi_gz_bridge`` の robot spawn 位置も同
    semantics (POI list 先頭) に移行
  - ``reload_map_info`` service の挙動変更: POI 編集後の reload で
    ``mapoi_initialpose_poi`` を再 publish しなくなった (運用中の自己位置を
    巻き戻すリスクを排除)。再設定は ``SwitchMap`` (= 地図切替) または
    手動経路 (RViz / WebUI / ``mapoi_initialpose_poi`` 直接 publish) で
  - ``mapoi_nav_server`` parameter 変更:
    ``initial_pose_subscriber_wait_timeout_sec`` を削除し、async retry
    timer ベースの新 parameter に置換 (``initialpose_retry_interval_sec``,
    ``initialpose_retry_max_attempts``,
    ``initialpose_post_subscribe_republish_count``)。blocking wait による
    他 callback 停止の回帰を防止
  - ``mapoi_initialpose_poi`` topic の型変更: ``std_msgs/String`` →
    ``mapoi_interfaces/InitialPoseRequest`` (``{map_name, poi_name}``)。
    SwitchMap 中の topic 同期 race (bridge / nav_server が古い世代の POI 名を
    採用してしまう) を防ぐため、subscriber 側は ``map_name`` で世代を検証する。
    publisher: ``mapoi_server``, ``mapoi_webui`` (両方更新済)。
    subscriber: ``mapoi_nav_server``, ``mapoi_gazebo_bridge``,
    ``mapoi_gz_bridge`` (全て更新済)

* ``pause`` system tag の発火条件を厳格化 (#89 段階 2, #143):

  - 旧: ``nav_mode_ != IDLE`` (= GOAL or ROUTE 走行中) なら全 POI で発火
  - 新: ``nav_mode_ == ROUTE`` かつ active route の ``waypoints`` / ``landmarks``
    に含まれる POI でのみ発火。GOAL 走行や IDLE では発火しない

  既存挙動からの変更点として、単発 ``mapoi_goal_pose_poi`` で偶然 ``pause`` POI に
  進入しても自動 pause しなくなる。route ベースのシナリオでのみ pause が走る形に
  整理した。

* ``GetRoutePois.srv`` の response に ``landmark_pois`` フィールドを追加 (#143)。
  yaml の ``route.landmarks`` で参照された POI が response に入る。Nav2 へは
  送られず、radius 監視と route スコープの pause 発火に使う。

* ``landmark × pause`` 排他追加。landmark POI は到達不可な reference のため
  pause 動作が成立しない。WebUI / rviz Panel / API の validation で reject。

* System tag ``goal`` renamed to ``waypoint``. Both single navigation
  targets and route waypoints share this tag (Nav2 navigation 到達対象).
  YAML ``tags: [goal, ...]`` must be migrated to ``tags: [waypoint, ...]``.
  Topic / service / action names referring to "goal" (e.g.
  ``mapoi_goal_pose_poi``, ``goal_pose``) remain unchanged because they
  follow Nav2 native terminology. (#89 段階 1, #142)
* System tag ``origin`` removed. Map origin reference points should be
  expressed with ``landmark`` system tag plus a custom tag (e.g.
  ``map_origin``) for color/semantic differentiation. Visualization
  defaults to gray; per-tag color is tracked in #70. (#89 段階 0)
* ``PointOfInterest.msg``: ``float64 radius`` field removed in favor of the
  new ``mapoi_interfaces/Tolerance tolerance`` struct (``xy`` + ``yaw``,
  Nav2 ``SimpleGoalChecker`` align). YAML ``poi.radius`` keys must be
  migrated to ``poi.tolerance: {xy, yaw}``. (#87)
* ``Tolerance.msg`` semantics: ``xy`` / ``yaw`` ともに ``>= 0.001`` を要求
  (`xy`: 1 mm / ``yaw``: 約 0.057°)。0 / 負値は禁止 (実用上「無反応 POI」を
  作れてしまうため)。"未指定で Nav2 default fallback" の semantics は
  削除。yaml load で違反値を検出した場合は ``0.001`` に補正 + WARN ログ。
  (#138)

* ``mapoi_nav_server`` の ROS parameter ``radius_check_hz`` を
  ``tolerance_check_hz`` にリネーム (#140)。``PointOfInterest.radius``
  field 廃止 (#87) → ``tolerance.xy`` 化に伴う命名整合。default 値 / 単位
  / 動作は変更なし。launch / yaml で ``radius_check_hz`` を指定している場合
  は ``tolerance_check_hz`` への置換が必要。

* ``mapoi_rviz2_publisher`` の ``arrow_size_ratio`` parameter を廃止 (#136)。

  - 旧: ``arrow_size_ratio`` (double, default 1.0) で POI 矢印サイズを
    ``radius × ratio`` に動的調整
  - 新: 矢印は固定 scale (length=0.15, shaft/head=0.04) で扇形 visualization の
    方向補助に位置付け (radius 連動の倍率は不要)
  - 移行: 既存 launch / yaml で ``arrow_size_ratio`` を declare / set している場合は
    削除する (parameter 自体が無くなる)

Interfaces
----------

* New ``mapoi_interfaces/msg/Tolerance.msg`` (``xy`` / ``yaw``). (#87)
* ``PoiEvent.msg``: new constants ``EVENT_STOPPED=3`` and
  ``EVENT_RESUMED=4``. Publish logic for these events is deferred to a
  follow-up issue. (#87)

WebUI / rviz_plugins
--------------------

* POI Editor (WebUI form / rviz_plugins POI Editor table) replaces the
  ``Radius`` input / column with ``tolerance.xy`` + ``tolerance.yaw``. UI
  入力単位は ``yaw`` のみ度 (deg) で表示・入力、内部 / yaml は rad で
  保存 (#138)。HTML ``min`` 制約と Panel ``ValidatePois`` で
  ``xy >= 0.001`` (m) / ``yaw >= 0.06`` (deg ≒ 0.001 rad) を強制。
* ``mapoi_webui_node`` の ``POST /api/pois`` で ``tolerance.{xy, yaw}``
  の min 制約を backend 側でも検証 (frontend bypass 防御)。(#138)
* RViz POI Editor の column 構造を 6 → 5 に再編 (#158):

  - 旧: ``name / description / pose / tolerance.xy / tolerance.yaw (deg) / tags``
  - 新: ``name / pose (x, y, yaw rad) / tolerance (xy m, yaw rad) / tags / description``
  - tolerance は 1 column 統合 (例: ``0.5, 0.7854``)、yaw は **rad 表示** に統一
    (pose.yaw が rad なので一貫性確保、#138 の暫定 deg 表示を撤回)。
  - description column を末尾に移動 (長文で横幅を圧迫しないように)。
  - validation の min 制約は ``xy >= 0.001 m`` / ``yaw >= 0.001 rad`` に簡素化
    (旧 deg 換算メッセージを撤廃)。
  - validation に **新 max 制約 ``yaw <= 2π rad``** を追加。本 PR で UI 入力単位を
    deg → rad に変更したため、旧 deg 入力 (例: ``45``) を rad として誤って入れた
    ケース (= 7 周分の異常値) を弾くガード。**既存 YAML で ``2π`` 超を使っていた
    場合は保存不可になる**ため、誤値修正が必要 (yaw は ``[0, 2π]`` または
    ``[-π, π]`` で表現するのが慣習で、それを超えるユースケースは事実上ない想定)。
    soft warning にせず hard reject を選んだ理由は typo 防止を優先するため。

* WebUI で rviz / 外部 save 由来の config 変更を SSE で受信し即時反映 (#135 (B))。

  - 旧: rviz の PoiEditor で save しても WebUI 側はブラウザ更新するまで反映されない
  - 新: ``mapoi_webui_node`` に ``/api/events`` SSE endpoint を追加。
    ``mapoi_config_path`` topic 受信時に ``{type: "config_changed"}`` を全 client に
    broadcast。frontend (``app.js``) は ``EventSource`` で受信して
    ``loadTagDefinitions / loadPois / loadRoutes`` を再 fetch
  - 関連: ``Flask app.run(threaded=True)`` を明示 (SSE long-lived connection と他
    request の並行処理のため)
  - PR #168 で対応した #135 (A) (PoiEditor / MapoiPanel の callback 修正) と
    合わせて #135 全体を close

* ``mapoi_rviz2_publisher`` / ``mapoi_webui`` で POI tolerance を扇形 (sector) で描画 (#136)。

  - 半径 = ``tolerance.xy``、扇角 = ``2 * tolerance.yaw``、中心線 = ``pose.yaw``
  - ``waypoint`` = 塗り扇形、``landmark`` = 中抜き扇形、``pause`` = 点線 outline
    の重ね描き (主 glyph は細い実線で対比)
  - ``0 < tolerance.yaw < π`` の時のみ扇形描画、それ以外 (yaw 不問) は完全円扱い
    (``pass_through`` 表現可能)
  - ``mapoi_rviz2_publisher`` に ``show_tolerance_sector`` parameter (bool, default true)
    を追加 — tolerance visualization を runtime トグル
  - 続く #179 で 円 (xy 判定領域) + 扇形 (yaw 制約) の重ね描きに分解 (下記参照)

* ``mapoi_rviz2_publisher`` / ``mapoi_webui`` で POI tolerance を 円 + 扇形 の重ね描きに
  分解 + pause overlay を dot 形式に変更 (#179、#178 user feedback 対応)。

  - 円 outline (細実線・薄め): ロボットの ``tolerance.xy`` 進入判定領域を常時表示。
    判定 logic (Euclidean < ``tolerance.xy``) の視覚的根拠として yaw 不問で描画
  - 扇形 (塗り or 中抜き): yaw 制約を強調。``0 < tolerance.yaw < π`` の時のみ
    重ね描き (完全円との情報冗長を回避)
  - **見た目の変化点**: 旧 (#178) では yaw 不問 POI (``tolerance.yaw == 0`` または ``>= π``)
    も扇形 = 完全円として描画されていた (waypoint = 緑塗り完全円、landmark = 灰
    中抜き完全円)。本 PR では yaw 不問なら **円アウトライン (細実線・薄め) のみ**表示で、
    塗り / 中抜き strokeは描画しない。判定 semantics (xy のみ) と一致するが、
    旧表示に慣れた利用者には印象変化あり (デモ・スクリーンショット用途は要確認)
  - pause overlay: 旧 dash (``dashArray: '6, 4'`` / RViz segment 0.05m, 1:1 比率) は
    「点と感じない、潰れて見える」user feedback (#178 PR コメント) を受け、
    sparse dot 形式に変更。WebUI ``dashArray: '2, 6'`` + ``lineCap: round`` (cycle 比
    25% on)、RViz dot 長 0.02m + 間隔 0.10m + line 0.04m (cycle 比 20% on)。
    pause 発火条件 (xy 円内) と境界が一致する xy 円沿いに重畳
  - ``show_tolerance_sector`` parameter は名称据え置きで制御対象を 円 + 扇形 + pause overlay
    全体に拡張 (backward compat 維持)
  - **保守注**: 描画仕様は ``mapoi_rviz2_publisher.cpp`` (RViz) / ``map-viewer.js`` (WebUI)
    で二重実装。仕様変更時はペアで更新する (#136 から継続、共通化は #70 と合わせて検討)

Samples
-------

* Sample yaml (``turtlebot3_world`` / ``turtlebot3_dqn_stage1``) を全パターン
  網羅シナリオに刷新 (#146)。新機能 (``route.landmarks`` #143、扇形描画 #136、
  ``tolerance.yaw`` #138) のリグレッション検証カバレッジを底上げする。

  - ``turtlebot3_world`` (オフィス見学ツアー): POI 5 → 9 に拡張。連続 ``pause``
    (``corridor_a`` + ``corridor_b``)、撮影地点 (``conference_room`` の
    ``tolerance.yaw=0.10``)、yaw 不問の通り過ぎ (``corridor_*`` の
    ``tolerance.yaw=π``)、純粋 ``landmark`` / ``landmark + audio_info`` /
    ``landmark + capture_target`` の 3 パターン、route ``tour_full``
    (waypoints + landmarks 両方持ち) と ``tour_short`` (landmarks 省略) の対比。
    custom tag ``capture_trigger`` / ``capture_target`` を追加。
  - ``turtlebot3_dqn_stage1`` (障害物 sandbox での回避): POI 5 → 7 に拡張。
    複合 tag (``event + landmark + hazard``) のハザード、``tolerance.yaw=π`` で
    yaw 不問の通過点 (= pass_through 代替) として ``checkpoint_west/east``、
    pause 中継 (``pause_intersection``)、route ``avoidance_a``
    (waypoints + landmarks) と ``avoidance_b`` (waypoints のみ)。custom tag
    ``observation`` を追加。
  - ``mapoi_turtlebot3_example/README.md`` にサンプル一覧表 (各シナリオで
    検証できる機能 / POI tag 構成) を追記。

  **Route 名のリネーム** (旧 → 新):

  - ``turtlebot3_world``: ``route_1`` → ``tour_full``、``route_2`` → ``tour_short``
  - ``turtlebot3_dqn_stage1``: 変更なし (``avoidance_a`` / ``avoidance_b``)

  ``tour_short`` は旧 ``route_2`` 互換 (``[elevator_hall, corridor_a,
  conference_room]``) で経路は同じ。launch / クライアント / 社内スクリプトで
  ``route_1`` / ``route_2`` を直接参照している場合は新名への移行が必要。
  サンプル YAML の固定スナップショットを前提にした外部チュートリアル等も同様
  (POI 数 / 名前 / tag 組合せが大幅に拡張されているため)。

* ``scripts/check_sample_yaml_consistency.py`` 新設 + CI 組み込み (#146)。
  サンプル yaml の POI 名 / route waypoint / landmark 参照の整合性、tag 排他
  (``waypoint × landmark``、``landmark × pause``)、``tolerance.{xy,yaw}``
  の min 制約 (>= 0.001) を pull request / push で自動検証。

Fixes
-----

* ``mapoi_server`` の ``reload_map_info`` service で ``mapoi_initialpose_poi``
  topic に skip message (``poi_name`` 空) を明示 publish するように修正
  (#154)。``transient_local`` (depth=1) の latched 値が reload 直前の古い
  POI 名のまま残り、reload 後に起動する nav_server / 再 connection した
  RViz が編集前の POI 名を受信してしまう stale message 問題を排除する。
  subscriber 側 (``mapoi_nav_server`` / ``mapoi_gazebo_bridge`` /
  ``mapoi_gz_bridge``) は元々 ``poi_name`` 空を無視する規約のため、運用中の
  自己位置巻き戻しは発生しない (#149 round 4 で確立した invariant を維持)。

Navigation server
-----------------

* ``mapoi_nav_server`` で ``EVENT_STOPPED`` / ``EVENT_RESUMED`` の publish 判定
  logic を実装 (#140)。``PoiEvent.msg`` に #87 で追加済の定数を実際に発火
  させる。判定 source は OR で 2 系統:

  - **Nav2 action SUCCEEDED**: ``FollowWaypoints`` / ``NavigateToPose`` の
    result_callback で SUCCEEDED を検知したら、現在 inside 状態の全 POI に
    ``EVENT_STOPPED`` を即時 publish (cmd_vel dwell 待ちを経由しない)
  - **cmd_vel ベース**: 線速・角速の両方が ``stopped_speed_threshold``
    (default ``0.01``) 未満の状態が ``stopped_dwell_time_sec`` (default
    ``1.0`` 秒) 続いたら ``EVENT_STOPPED`` を publish。tolerance_check_callback
    の各 tick で判定
  - **EVENT_RESUMED**: STOPPED 状態の POI で速度が閾値超に戻ったら即時
    publish (dwell 不要)。新規 ``mapoi_goal_pose_poi`` / ``mapoi_route``
    受信時にも全 STOPPED POI に対して RESUMED を publish (subscriber 側で
    「停止解除」を即時検知できるようにする)
  - **EXIT 時**: poi_inside_state_ → false と同時に poi_stopped_state_ も
    reset。EXIT イベントが「停止解除」を兼ねる扱い (RESUMED は明示 publish
    しない、ライフサイクルを ENTER → STOPPED → EXIT に整理)

  新規 ROS parameter:

  - ``stopped_speed_threshold`` (default ``0.01`` m/s 相当)
  - ``stopped_dwell_time_sec`` (default ``1.0``)
  - ``cmd_vel_topic`` (default ``cmd_vel``)

  state machine 純関数 ``compute_stopped_transition`` を切り出し、
  ``test_nav_server_unit`` で unit test カバレッジ確保 (5 件追加)。

Internal
--------

* initial pose 選定ロジックを共通ヘッダ
  ``mapoi_server/include/mapoi_server/initial_pose_resolver.hpp`` に切り出し、
  ``mapoi_server`` / ``mapoi_gazebo_bridge`` / ``mapoi_gz_bridge`` の 3 箇所の
  重複実装を統合 (#150)。今後 ``initial_poi_name`` 仕様 (除外条件 / fallback)
  が拡張された場合の simulator spawn と ``/initialpose`` の挙動乖離を防ぐ。
  挙動は同一 (純関数の rename + 物理配置移動のみ、unit test 互換)。


0.2.0 (2026-04-29)
==================

WebUI
-----

* New ``robot_radius`` ROS parameter on ``mapoi_webui_node`` (default
  ``0.15``). Determines the robot marker size and the active-route
  connector arrival threshold; previously hard-coded at 0.15. Value is
  also propagated to the frontend through ``/api/nav/status``. (#116,
  #117, #118, #126)
* Active-route highlight now propagates to direction arrows and order
  labels, and non-active routes are dimmed for focus. (#69, #105)
* Parallel-offset rendering for overlapping route polylines.
  Direction-independent canonical normal places reverse-direction routes
  on opposite sides of the shared segment. (#69, #128)
* Directional connector arrow from the current robot position to the
  first waypoint of the active route. Visual endpoint follows the offset
  polyline; physical reach detection uses the original POI coordinates.
  (#106, #107)
* POI / route name uniqueness validation, enforced on both the client
  side and the backend ``POST /api/pois`` / ``POST /api/routes``
  endpoints. (#109, #120)
* Sidebar UX: drag-resize handle with width persisted in
  ``localStorage``, sticky form-actions footer for narrow viewports, and
  ellipsis with hover tooltip for long names. (#122, #123, #124, #125)
* Navigation dropdowns auto-select the currently focused POI or route.
  (#111, #114)
* Robot marker color matches the active route's color. (#110, #113)
* POI editor: visible "SAVED" feedback after save, and table rebuilt
  post-save to keep row numbering aligned with visual order. (#74, #76,
  #77, #95)
* ``tag_definitions`` is fetched through the ``get_tag_definitions``
  service rather than read from yaml directly. (#39, #59)
* New ``mapoi_editor.launch.yaml`` for headless POI editing without
  ``mapoi_nav_server``; service / publisher calls degrade gracefully.
  (#60, #62)

Navigation server
-----------------

* ``mapoi_nav_status`` now uses ``transient_local`` QoS so late-starting
  rviz panels and the WebUI immediately receive the latest state. (#96,
  #103)
* ``mapoi_nav_status`` payload extended to the ``"status:target"`` form,
  preserving the target name through ``succeeded`` / ``aborted`` /
  ``canceled`` transitions. (#104, #119)
* Localization-agnostic Tier 1 support: ``initial_pose`` topic and
  timeout are configurable; AMCL ``set_initial_pose`` calls are
  de-duplicated. (#57, #58)
* ``/initialpose`` auto-publish now waits for subscriber readiness
  before sending. (#33, #56)
* Map switch detection on ``mapoi_config_path`` uses both path and
  mtime as the change guard. (#80, #81)

RViz plugins / publisher
------------------------

* New "Display Settings" group in the POI Editor panel. (#99, #100)
* POI radius rendered as a floor-plane circle (LINE_STRIP) for all
  POIs. (#67, #71)
* POI arrow size scales with radius via the new ``arrow_size_ratio``
  parameter. (#65, #72)
* POI labels are configurable and include line numbers. (#66, #73)
* All routes published as LINE_STRIP via ``mapoi_route_marks``; rviz
  display names made descriptive. (#68, #83, #101)
* ``mapoi_config_path`` subscription dynamically refreshes the
  displayed POI list. (#75, #78)
* Removed ``nav2_rviz_plugins/Selector`` and ``Docking`` panels from
  the default rviz config (not present on Humble). (#37, #51)

Simulation bridges
------------------

* ``mapoi_gazebo_bridge`` (Humble / Gazebo Classic): SwitchMap now
  performs Gazebo entity swap and robot delete + respawn. (#30, #46)
* ``mapoi_gz_bridge`` (Jazzy / gz-sim): SwitchMap performs entity swap
  with ``SetEntityPose`` for atomic robot teleport. (#48, #52)
* Symmetric ``NoGazeboSection`` cleanup and queue coalescing across
  both bridges. (#53, #54)

Docker / development
--------------------

* Distro-specific dev / demo image tags (``mapoi:dev-${ROS_DISTRO}``,
  ``mapoi:demo-${ROS_DISTRO}``) so parallel humble / jazzy runs do not
  invalidate each other's build cache. ``ROS_DISTRO`` inherit / override
  behaviour documented in ``README.md``. (#90, #92, #93)
* ``CYCLONEDDS_URI`` is forced to empty inside containers to prevent
  host configuration leak; ``RMW_IMPLEMENTATION`` and ``ROS_DOMAIN_ID``
  inherit host values for host-container discovery. (#84, #94)
* ``.env.example`` auto-sets ``USER_ID`` / ``GROUP_ID`` for bind-mount
  ownership. (#97, #98)

Examples
--------

* New ``gz_sim_headless_aware.launch.yaml`` wrapper for Jazzy
  headless launches. (#42, #49)
* TurtleBot3 ``turtlebot3_world`` and ``turtlebot3_dqn_stage1``
  example configs reworked with distinct themes and Nav2-aligned
  radii. (#79, #82)

CI / tests
----------

* New ``robot_radius`` drift check that compares the WebUI launch arg
  against the Nav2 ``burger.yaml`` ``robot_radius`` for both humble
  and jazzy parameter sets. (#127, #131)
* Vitest unit-test infrastructure and pure ``geometry.js`` helper
  extracted from ``MapViewer._offsetLatLngs``. (#129, #132)

Fixes
-----

* Fixed route editing lifecycle: ``MapViewer`` selection state could
  drift from the active route. (#112)
* Fixed ``mapoi_route_cb`` ``std::bind`` compile error on Humble by
  switching to a lambda capture. (#121)
* Removed redefinition warnings in ``test_nav_server_unit.cpp``.
  (#47, #50)

Internal
--------

* Stripped historical PR references and stale comments from
  production code. (#63, #64)

Known issues
------------

* AMCL drift after SwitchMap on Gazebo Classic (Humble) only;
  reproduces on Classic but not on gz-sim (Jazzy). Workaround: use
  Jazzy / gz-sim, or manually re-set the initial pose in rviz.
  (#91)
* Switching maps from the WebUI dropdown does not update the
  backend's current map context, so POI / route lists may show
  stale data. Workaround: set ``map_name`` via launch parameter or
  call the ``/switch_map`` service directly. (#130)
* The project is not yet registered with rosdep / rosdistro;
  ``apt`` / ``rosdep`` installation is not yet available. (#20)

Contributors: Shunsuke Kimura
