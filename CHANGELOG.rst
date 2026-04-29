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
