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

Added
-----

* All six package ``README.md`` files (root, ``mapoi_server``, ``mapoi_webui``,
  ``mapoi_turtlebot3_example``, ``mapoi_interfaces``, ``mapoi_rviz_plugins``)
  are now English-primary, ahead of ROS Index registration (#20); the prior
  Japanese content is preserved as a same-directory ``README.ja.md``
  snapshot, with cross-links between the two at the top of each file (#162).

* A metapackage ``mapoi`` (``mapoi/package.xml`` with ``exec_depend`` on
  the four core packages), so the core suite can be installed/depended on
  as one unit — groundwork for apt/rosdep distribution (#20) (#348).
  Three deliberate deviations from the issue text: it lives in a
  ``mapoi/`` subdirectory (navigation2-style) rather than at the
  repository root, because a package manifest at the root would stop
  colcon's package discovery from descending into the sibling packages;
  it omits the REP 140 ``<metapackage/>`` export, whose catkin_pkg
  validation is ROS 1 specific and would warn ("must buildtool_depend on
  catkin") on every colcon invocation; and it excludes
  ``mapoi_turtlebot3_example``, so a robot/production install does not
  pull in the TurtleBot3 + Gazebo simulator stack (same policy as
  navigation2, whose metapackage excludes the demo ``nav2_bringup``).
  Installing ``mapoi_turtlebot3_example`` directly still brings in the
  whole demo, since the example itself ``exec_depend``-s the mapoi core
  packages.

* New service ``request_initial_pose``
  (``mapoi_interfaces/srv/RequestInitialPose``,
  ``{map_name, poi_name} -> {success, error_message}``) on ``mapoi_server``
  (#211). Requesters ask ``mapoi_server`` — the sole writer of
  ``mapoi/initialpose_poi`` — to publish an initial-pose POI request; an
  empty ``poi_name`` publishes a clear (skip) sample. Since #299 a
  non-empty ``map_name`` must match the server's current map: a mismatch
  is rejected with ``success=false`` and nothing is published, so
  requesters must check ``response.success`` instead of treating a
  completed call as success (an empty ``map_name`` is passed through
  unvalidated for requesters that do not know the current map).

* ``mapoi_webui`` REST API consistency improvements (#343, second half; first
  half in #363): every JSON error response (4xx/5xx) now carries a
  machine-readable ``code`` field alongside the existing human-readable
  ``error`` message — ``invalid_request`` (400), ``not_found`` (404),
  ``version_mismatch`` (409, unchanged from #241), ``service_unavailable``
  (503), ``internal_error`` (500). This is purely additive; existing
  ``error`` consumers are unaffected. Also, the optimistic-concurrency
  ``expected_version`` check introduced for ``POST /api/pois`` (#241) is now
  available on ``POST /api/routes`` and ``POST /api/custom_tags`` as well,
  since all three write the same ``mapoi_config.yaml``: ``GET /api/routes``
  and ``GET /api/tag_definitions`` now return ``config_version``, and the
  corresponding ``POST`` endpoints accept ``expected_version`` and return
  ``409`` + ``code: version_mismatch`` on a stale write (``expected_version``
  omission still skips the check, matching the pre-existing ``/api/pois``
  contract). The WebUI frontend (route editor, tag editor) handles the
  conflict the same way the POI editor already does.

* New topic ``mapoi/nav/command_rejected`` (``std_msgs/String``, volatile QoS,
  payload = target string) on ``mapoi_nav2_bridge`` (#354). ``mapoi/nav/status``
  is a latched state snapshot, so a rejected command while navigation is
  already in progress (``nav_mode_ != IDLE``) intentionally does not publish
  ``"rejected"`` there (#339) to avoid clobbering the in-progress status.
  That left operators with no way to notice a rejected command (e.g. a typo
  goal) sent mid-navigation, short of reading the ROS logs. The new topic is
  an independent event notification, published unconditionally every time a
  command is rejected regardless of ``nav_mode_``. ``mapoi_webui`` forwards it
  as an SSE ``command_rejected`` event, and the WebUI frontend shows a small
  auto-dismissing toast (``Command rejected: <target>``). Purely additive;
  ``mapoi/nav/status`` semantics are unchanged.

Breaking changes
----------------

* All eight ``mapoi_server`` services now live under the ``mapoi/`` namespace,
  matching the topics (already namespaced since v0.3.0). No compatibility
  alias is provided. See ``docs/migration/v0.5.0.md`` for the full migration
  guide. Rename table:

  - ``get_pois_info`` -> ``mapoi/get_pois_info``
  - ``get_route_pois`` -> ``mapoi/get_route_pois``
  - ``get_maps_info`` -> ``mapoi/get_maps_info``
  - ``get_routes_info`` -> ``mapoi/get_routes_info``
  - ``select_map`` -> ``mapoi/select_map``
  - ``request_initial_pose`` -> ``mapoi/request_initial_pose``
  - ``reload_map_info`` -> ``mapoi/reload_map_info``
  - ``get_tag_definitions`` -> ``mapoi/get_tag_definitions``

  Update any external client (``ros2 service call``, custom bridges,
  monitoring scripts) that pins one of the old bare names. Service types,
  request/response fields, topics, and launch parameters are unchanged
  (#341).

* ``mapoi_interfaces/srv/GetRoutePois`` response gains ``bool success`` /
  ``string error_message`` (same pattern as ``SelectMap``), added ahead of
  the existing ``pois_list`` / ``landmark_pois`` fields. When
  ``route_name`` does not match any route on the current map,
  ``mapoi_server`` now returns ``success=false`` with a descriptive
  ``error_message`` and logs a ``WARN``, instead of silently returning
  empty lists. A route that exists but has zero waypoints still returns
  ``success=true`` (data-distinguishable from a typo'd route name).
  Recompile all clients against the updated ``.srv``; ``mapoi_nav2_bridge``,
  ``mapoi_rviz2_publisher``, and ``mapoi_rviz_plugins`` (``MapoiPanel``)
  are updated in this change to check ``response.success`` (#342).

* ``mapoi_interfaces/msg/PointOfInterest`` drops the ``int32 id`` field. Its
  sole producer, ``MapoiServer::yaml_to_poi_msg()``, never assigned it, so
  every ``id`` surfaced via ``get_pois_info`` / ``get_route_pois`` /
  ``PoiEvent.poi`` was always ``0``; ``name`` is the de facto unique key
  already used by the WebUI and REST API. Recompile all clients against the
  updated ``.msg``. Any external code reading ``PointOfInterest.id`` must
  switch to keying on ``name`` instead. See ``docs/migration/v0.5.0.md``
  (#338).

* ``mapoi_interfaces/srv/SelectMap`` response field ``initial_poi_name`` is
  renamed to ``resolved_initial_poi_name``. Request and response previously
  reused the same field name (``initial_poi_name``) for the *requested* POI
  and the *resolved* POI, which read ambiguously at a glance; the request
  field is unchanged. The ``mapoi_webui`` REST endpoint ``POST
  /api/maps/select`` mirrors the rename in its JSON response body.
  Recompile all clients against the updated ``.srv``; ``mapoi_server`` and
  ``mapoi_nav2_bridge`` are updated in this change. See
  ``docs/migration/v0.5.0.md`` (#343).

* ``mapoi_webui`` REST API URL hierarchy: editor-only endpoints now live
  under an ``/api/editor/`` prefix, separate from ``/api/nav/`` endpoints
  that act on the running robot. ``POST /api/maps/select`` only switches
  ``mapoi_server``'s edit-context map and does not touch Nav2 or the
  running robot, whereas ``POST /api/nav/switch-map`` actually switches
  the running robot's Nav2 map; the similar naming made this distinction
  easy to miss and risked unintended map switches on a live robot (#340).
  The remaining renames unify the URL separator convention on kebab-case
  (#343, fifth checklist item). No compatibility alias is provided; old
  paths now return ``404``. Rename table:

  - ``POST /api/maps/select`` -> ``POST /api/editor/select-map``
  - ``GET /api/tag_definitions`` -> ``GET /api/tag-definitions``
  - ``POST /api/custom_tags`` -> ``POST /api/custom-tags``
  - ``POST /api/nav/initialpose`` -> ``POST /api/nav/initial-pose``

  Update any external client (curl scripts, dashboards, custom
  frontends) pinning one of the old paths. See
  ``docs/migration/v0.5.0.md`` for the full migration guide.

Changed
-------

* **mapoi_nav2_bridge.cpp split into three translation units, step 1 of 2 (#345).**
  ``mapoi_server/src/mapoi_nav2_bridge.cpp`` had grown to 1,712 lines / ~40 methods
  covering seven-plus responsibilities. Route navigation (``FollowWaypoints`` + the
  mapoi-driven waypoint-arrival mode) and single-goal navigation (``NavigateToPose``)
  are now defined in new files ``mapoi_nav2_bridge_route.cpp`` and
  ``mapoi_nav2_bridge_goal.cpp``; all three compile into the same ``mapoi_nav2_bridge``
  executable and ``MapoiNav2Bridge`` remains a single class. This is an internal
  refactor with no behavior change: topic/service names, QoS, parameters, log
  messages, callback groups, mutex/lock granularity, and publish timing are all
  unchanged. Map switching (``LoadMap`` / ``select_map`` sync) and backend status
  publishing are deliberately left untouched for a follow-up PR (#345 step 2), as is
  the shared ``tolerance_check_callback`` POI-event judgment engine and the
  pause/resume/cancel/reset dispatchers, which read and write state from both route
  and goal navigation and are not cleanly separable without touching those.

* **mapoi_nav2_bridge.cpp split into five translation units, step 2 of 2 (final,
  #345).** Map switching (``mapoi_switch_map_cb`` / ``on_select_map_received`` /
  ``send_load_map_request`` / ``request_initial_pose``) and backend status
  publishing (``publish_backend_status``) are now defined in new files
  ``mapoi_nav2_bridge_map_switch.cpp`` and ``mapoi_nav2_bridge_backend_status.cpp``;
  all five compile into the same ``mapoi_nav2_bridge`` executable and
  ``MapoiNav2Bridge`` remains a single class. Internal refactor only, no behavior
  change (same guarantees as step 1). ``mapoi_nav2_bridge.cpp`` now holds node
  setup/constructor, the pause/resume/cancel/reset dispatchers, the shared
  ``tolerance_check_callback`` POI-event judgment engine, POI list / system tag
  fetch infrastructure, and cmd_vel monitoring — all of which are shared across
  route, goal, and map-switch navigation and are not cleanly separable per domain.
  Also removes ``clear_current_route_poi_names_``, a pre-existing dead private
  method with no remaining callers.

* **mapoi_webui frontend responsibility split (#346), internal refactor with no
  behavior change.** ``mapoi_webui/web/js/map-viewer.js`` (1,579 lines) had marker
  icon generation, POI tolerance sector/yaw-handle rendering, and route drawing all
  in one ``MapViewer`` class. Marker/route icon and color helpers (``getPoiColor``,
  ``getRouteColor``, ``routeDirectionDeg``, ``createRouteDirectionSvg``,
  ``createArrowIcon``, ``createRobotIcon``, ``createRouteLandmarkIcon``) move to the
  new ``map-icons.js`` (``MapoiMapIcons``, dual browser/Node export, continuing the
  ``geometry.js``/``poi-filter.js``/``poi-interactions.js`` pattern); POI tolerance
  sector drawing and the yaw-rotation drag handle (``_drawSectorForPoi``,
  ``_wedgePoints``, ``_circlePoints``, the yaw-handle trio, and the state they
  owned — ``sectorLayers`` / ``_poiWedgeByIndex`` / ``_yawHandle``) move to the new
  ``MapViewerSector`` class in ``map-viewer-sector.js``, which ``MapViewer`` now
  composes (``this._sector = new MapViewerSector(this)``). ``MapViewer`` keeps its
  public API and remains a single class; both new files are added to
  ``index.html``'s script load order ahead of ``map-viewer.js``.

* **app.js tag editor split into a TagEditor class (#346), internal refactor
  with no behavior change.** ``mapoi_webui/web/js/app.js`` had the custom tag
  list UI (rendering, add/delete, description disclosure, dirty tracking,
  save/discard against ``MapoiApi.saveCustomTags``) inline in its single IIFE.
  It now lives in the new ``TagEditor`` class (``tag-editor.js``), mirroring
  the ``PoiEditor``/``RouteEditor`` shape: DOM wiring in the constructor, a
  ``dirty`` field read by app.js's unsaved-changes guards, and
  ``onReload``/``onConflictReload`` callbacks so app.js still owns the actual
  ``loadTagDefinitions``/``loadPois``/``loadRoutes`` refetch sequencing after
  a save, discard, or 409 version-mismatch. ``tag-editor.js`` is added to
  ``index.html`` ahead of ``app.js``.

* **mapoi_rviz_plugins: PoiEditorPanel::ValidatePois split into its own
  translation unit, with its pose/tolerance/tag-exclusivity checks extracted
  as pure functions (#346), same staged-split approach as the
  mapoi_nav2_bridge split above (#345) and internal refactor with no behavior
  change.** ``ValidatePois()`` (152 lines) now lives in the new
  ``poi_editor_validation.cpp``, compiled into the same
  ``mapoi_rviz_plugins`` library; ``PoiEditorPanel`` remains a single class
  and its header is unchanged. Its per-cell decision logic — pose format
  ("x, y, yaw"), tolerance format/minimum/2π-overflow ("xy, yaw_rad"), and
  the waypoint/pause × landmark tag exclusivity (#85/#143) — moves to new
  Qt/ROS-independent pure functions (``validate_pose_cell``,
  ``validate_tolerance_cell``, ``check_tag_exclusivity``) in the existing
  ``poi_editor_helpers.hpp`` (alongside ``try_parse_finite_double`` /
  ``split_and_trim`` from #158). ``ValidatePois()`` itself still builds the
  exact same ``QMessageBox`` warning text from the returned status/value
  structs, so displayed messages are byte-for-byte unchanged. The three new
  functions get gtest coverage in ``test_poi_editor_validation.cpp``
  (``mapoi_rviz_plugins``), following the existing
  ``test_poi_editor_helpers.cpp`` / ``test_config_path_update_policy.cpp``
  pattern (docs/testing-policy.md §1(b): cheap, ROS/Qt-independent pure
  functions).

Fixed
-----

* **mapoi/initialpose_poi multi-writer race (#211).** The topic had four
  direct publishers (``mapoi_server``, ``mapoi_nav2_bridge``, WebUI, RViz
  panel). Because a ``transient_local`` cache is held *per writer*, a
  late-joining subscriber received each writer's last latched sample in
  undefined order, so ``mapoi_server``'s clear could not erase a stale POI
  still latched in another writer's cache. Publishing is now consolidated
  into ``mapoi_server`` as the **sole writer**: ``mapoi_nav2_bridge`` /
  WebUI / RViz panel request a publish via the new ``request_initial_pose``
  service instead of publishing directly, so the latched cache is unified
  and a clear is decisively last-write-wins. The timing gate that defers the
  operator-switch POI until after Nav2 ``LoadMap`` succeeds stays in
  ``mapoi_nav2_bridge``. The topic ``mapoi/initialpose_poi`` and its
  subscriber contract (``InitialPoseRequest``, ``transient_local``, empty
  ``poi_name`` = ignore) are unchanged, so custom localization bridges and
  direct external publishers keep working.

  Scope: this removes the cross-writer latched-cache race specifically. It
  does NOT make POI-name-to-pose resolution map-consistent — the localization
  bridge resolves a latched POI *name* against ``mapoi_server``'s *current*
  map (``map_name`` is intentionally unverified on the bridge side, #149
  r10), so a stale or concurrently-interleaved name could still resolve to
  a wrong pose right after a map switch. That residual is now closed on the
  server side by the ``request_initial_pose`` current-map validation (#299,
  see below); the generation/pending design considered in #155 was dropped
  as unnecessary. The crash/restart startup re-latch was fixed separately
  via ``state_path`` persistence (#297).

* **Stale initial pose during a map-switch transition window (#299).**
  When operator map switches overlap (A→B), ``mapoi_server`` finishes both
  ``select_map`` calls (context = B) before ``mapoi_nav2_bridge`` runs A's
  response callback, so A's ``request_initial_pose(A, poi_A)`` used to be
  latched-published even though it no longer matches the current map. The
  service now rejects a non-empty ``map_name`` that does not match the
  server's current map (``success=false``, nothing published). The
  requesters (``mapoi_nav2_bridge`` / WebUI / RViz panel) all check
  ``response.success`` and surface the rejection as an error instead of
  a false success.

Samples
-------

* The ``turtlebot3_world`` demo was redesigned into a **feature-catalog**
  layout (#230, PR #235): POI count 9 -> 5, with routes restructured
  into one-feature-per-route tutorials plus an all-in-one ``tour_full``.
  This renames/removes the sample POI and route names, so launch files,
  clients, tutorials, or scripts that pin the old ``turtlebot3_world``
  names must migrate.

  - **POIs** (new): ``start`` / ``basic_waypoint`` / ``pause_waypoint``
    / ``goal`` / ``audio_landmark``. The old office-tour POIs
    (``elevator_hall``, ``corridor_a`` / ``corridor_b``,
    ``conference_room``, ``model_exhibit`` and the landmark variants)
    were removed. This is **not a 1:1 rename**: the demo was rebuilt
    around the feature-catalog layout, so old POIs have no direct
    new-name counterpart.
  - **Routes** (new): ``tutorial_01_basic`` / ``tutorial_02_landmark``
    / ``tutorial_03_pause`` / ``tour_full``. ``tour_full`` keeps its
    name but its waypoints / landmarks were rebuilt for the new POI set;
    the old ``tour_short`` was removed.
  - Consecutive-``pause`` coverage (old ``corridor_a`` + ``corridor_b``)
    moved into the ``test_poi_event_route_integration`` launch_test
    (#236) instead of the demo config.

Internal / tests
----------------

* ``ros-test.yml``'s silent-test-loss guard no longer hard-codes
  ``EXPECTED_TARGETS``: it is now derived by grepping ``src/mapoi``'s
  ``CMakeLists.txt`` files for ``ament_(auto_)add_gtest`` /
  ``ament_add_pytest_test`` registrations, so the check can't silently
  drift out of sync again the way it repeatedly did in #297 / #344
  (#351).


0.4.0 (2026-05-08)
==================

**Notable for downstream users**: ``latest`` and ``vX.Y.Z`` (no distro
suffix) Docker tags now point at the **jazzy** build (was humble in
v0.3.0). Humble images remain available as ``humble`` /
``vX.Y.Z-humble``. See the "Default distro switch" entry below.

Breaking changes
----------------

* The Nav2 bridge node has been renamed from ``mapoi_nav_server`` to
  ``mapoi_nav2_bridge`` (#204, PR #218). Executable name, ROS 2 node name,
  C++ class (``MapoiNavServer`` → ``MapoiNav2Bridge``), header path
  (``mapoi_server/mapoi_nav2_bridge.hpp``), and the launch arg
  ``with_nav_server`` (now ``with_nav2_bridge``) all changed. No
  compatibility alias is provided. See ``docs/migration/v0.4.0.md`` for
  the full migration guide. Quick highlights:

  - Update launch files / scripts / docs that reference
    ``mapoi_nav_server`` (executable / node / arg name) to
    ``mapoi_nav2_bridge`` / ``with_nav2_bridge``.
  - ROS 2 launch silently ignores unknown args and applies defaults, so
    passing the old ``with_nav_server:=false`` to an updated
    ``mapoi_bringup.launch.yaml`` will be ignored and the bridge starts
    with its default. Audit your downstream launch with
    ``grep -rn 'with_nav_server:=\|mapoi_nav_server' --include='*.launch.*'``.
  - Auto-derived ROS 2 services (``/<node>/get_parameters``,
    ``/list_parameters``, ``/describe_parameters``, ``/set_parameters``)
    change with the node name. External monitoring tools like
    ``ros2 param get /mapoi_nav_server ...`` must be updated to
    ``/mapoi_nav2_bridge``.
  - ROS 2 topics, services (e.g. ``/mapoi/nav/...``,
    ``select_map``, ``get_pois_info``), explicitly declared parameters
    (``maps_path``, ``map_name``, ``config_file``, etc.), and Nav2 native
    actions (``navigate_to_pose`` etc.) are unchanged.

* AMCL adapter has been split out of the Nav2 bridge node into a new
  ``mapoi_amcl_localization_bridge`` executable in ``mapoi_server``
  (#209, PR #210). The bridge node (now called ``mapoi_nav2_bridge`` —
  see breaking entry above) is a Nav2-only navigation bridge and no
  longer publishes ``/initialpose``. Users who launch nodes individually
  must add ``mapoi_amcl_localization_bridge`` next to the bridge (the
  ``mapoi_bringup.launch.yaml`` already does this; new launch arg
  ``with_amcl_localization_bridge``, default ``true``). The following
  parameters moved from the old ``mapoi_nav_server`` to the new bridge:
  ``initial_pose_topic``, ``initialpose_retry_interval_sec``,
  ``initialpose_retry_max_attempts``,
  ``initialpose_post_subscribe_republish_count``. A new minimal contract
  message ``mapoi_interfaces/LocalizationBackendStatus`` and topic
  ``mapoi/localization/backend_status`` (``transient_local`` +
  ``MANUAL_BY_TOPIC`` liveliness, 5 s lease) are published by the bridge;
  WebUI / RViz panel now show a separate ``Localization`` indicator and
  gate the Set Initial Pose UI on it, independent from the Navigation
  indicator. See ``docs/backend-status.md`` for the full Navigation /
  Localization contract.

* System tag definitions (``waypoint`` / ``landmark`` / ``pause``) are now
  hardcoded in ``mapoi_server/include/mapoi_server/system_tags.hpp``
  (``mapoi::kSystemTags``); the
  ``mapoi_server/maps/tag_definitions.yaml`` file and the
  ``mapoi_server/maps/`` directory have been removed (#191, PR #192).
  The ``get_tag_definitions`` service contract is unchanged (3 system
  tags + user tags). Users who edited ``tag_definitions.yaml`` to
  override system tag names or descriptions must edit ``kSystemTags``
  and rebuild instead. Standard usage (no edits to system tags) is
  unaffected.

  Additional behavior changes worth noting:

  - Previously, if ``tag_definitions.yaml`` was missing or malformed,
    ``mapoi_server`` logged a warning and continued with an empty system
    tag list (a degenerate state in which ``get_tag_definitions`` returned
    0 system tags). The new implementation always exposes the 3 compiled-in
    system tags, removing the degenerate path.
  - The package install layout under ``share/mapoi_server/`` no longer
    contains a ``maps/`` directory. External scripts that listed
    ``share/mapoi_server/maps/`` must be updated.

* **Default distro switched from Humble to Jazzy** (PR #222). The
  Dockerfile ``ARG ROS_DISTRO`` default, ``docker-compose.yml``
  ``${ROS_DISTRO:-...}`` fallbacks, and the GHA tag-emit conditions for
  ``latest`` / ``vX.Y.Z`` (no distro suffix) all moved from ``humble`` to
  ``jazzy``. The motivation is that Gazebo Classic (shipped with the
  Humble image) reached EoL in 2025-01 and is no longer receiving
  upstream maintenance, while Jazzy bundles the actively maintained
  gz-sim (Gazebo Harmonic). Humble continues to be supported as a
  first-class option:

  - ``ghcr.io/shimz-robotics/mapoi:humble`` keeps shipping per-commit and
    per-release. ``vX.Y.Z-humble`` tags remain on every release.
  - ``ROS_DISTRO=humble docker compose build / up / run`` continues to
    produce ``mapoi:dev-humble`` / ``mapoi:demo-humble`` images.
  - The Dockerfile and compose file are unchanged for users who already
    pass ``ROS_DISTRO=humble`` explicitly.

  The break-on-upgrade case is users who pull ``ghcr.io/...:latest`` or
  ``ghcr.io/...:vX.Y.Z`` (no suffix) and expect a Humble build. From v0.4.0
  onward those tags resolve to a Jazzy image; pin ``:humble`` /
  ``:vX.Y.Z-humble`` to retain the old behavior.

Features
--------

* Navigation backend readiness contract (#198, PR #205). Introduced a
  minimal 3-field message ``mapoi_interfaces/NavigationBackendStatus``
  (``backend_type`` / ``backend_ready`` / ``reason``) published at 1 Hz on
  ``mapoi/nav/backend_status`` (``transient_local`` QoS). WebUI and RViz
  panel gate navigation operations on ``backend_ready`` alone, decoupling
  the UI from per-capability internals. Custom navigation bridges can
  integrate by populating the 3 fields, no other plumbing required.
  Localization readiness is exposed as a parallel
  ``LocalizationBackendStatus`` (#209 above).

* Liveliness QoS for ``backend_status`` topics (#208, PR #212). Both
  ``mapoi/nav/backend_status`` and ``mapoi/localization/backend_status``
  publishers now use ``MANUAL_BY_TOPIC`` liveliness with a 5 s lease;
  subscribers use ``AUTOMATIC`` liveliness. UI consumers detect bridge
  death via ``LivelinessChangedEvent`` (alive_count → 0) and treat the
  cached status as ``backend_ready=false`` regardless of the latched
  payload. Publishers without a finite liveliness QoS (legacy / no-config
  bridges) are rejected via QoS incompatibility (``pub.lease (∞) >
  sub.lease (5 s)``) — this is intentional, see the
  ``NavigationBackendStatus.msg`` header comment for the full rationale.

* WebUI Navigation detection and mobile-friendly focus UI (PR #197).
  WebUI now subscribes to ``mapoi/nav/backend_status`` and shows a
  ``Navigation connected / disconnected`` indicator; navigation control
  UI is enabled only when the bridge reports ``backend_ready=true``.
  Mobile (smartphone) focus interaction was reworked to match the panel
  layout for narrower screens.

* ``mapoi_gazebo_bridge`` AMCL drift fix on operator map switch (#91, PR
  #200). After a ``select_map`` flow the bridge now teleports the robot
  via Gazebo Classic ``delete_entity`` + ``spawn_entity`` re-creation to
  the first non-landmark POI, avoiding stale TF / costmap state in AMCL.

Fixes
-----

* ``backend_status`` 1 Hz publish keeps running during blocking calls
  (#213, PR #214). The Nav2 bridge node was previously a
  ``SingleThreadedExecutor`` with all callbacks in the default mutually
  exclusive group; ``send_load_map_request`` could block the executor
  for up to ~11 s per Nav2 map node and stall the ``backend_status``
  timer beyond the 5 s liveliness lease, causing false-positive
  ``LivelinessChanged`` events. The timer now runs on a separate
  ``MutuallyExclusive`` callback group within a
  ``MultiThreadedExecutor`` (thread count fixed at 2 to avoid
  ``hardware_concurrency()`` returning 1 in CPU-limited containers).

Documentation
-------------

* README split into a slim entry point + ``docs/`` directory (PR #223).
  Root ``README.md`` shrank from 430 → 95 lines (78% reduction); long
  sections moved to ``docs/migration/v0.4.0.md`` (rename + AMCL split +
  system tag hardcode), ``docs/migration/v0.3.0.md``,
  ``docs/docker.md`` (Docker demo / dev / GPU 詳細),
  ``docs/integration.md`` (自分のロボットへの導入手順),
  ``docs/backend-status.md`` (Navigation + Localization 仕様統合). Root
  README keeps package list, version policy, planned breaking changes,
  quickstart, Docker quickstart (Jazzy 1-liner), main features, and a
  table of links to the docs/.

* Custom navigation bridge ``backend_ready`` computation guidance
  (#207, PR #215). Added comments in
  ``mapoi_interfaces/msg/NavigationBackendStatus.msg``,
  ``mapoi_interfaces/README.md``, and the bridge source explaining how
  to compute ``backend_ready`` for partial-capability bridges
  (goal-only, route-only, etc.) and the privacy rules for the
  ``reason`` string (no credentials / absolute paths / internal hostnames
  / IPs / stack traces).

* CLI launch examples and ``maps_path`` / ``map_name`` argument
  semantics (#172, PR #219). Added a new section in
  ``mapoi_server/README.md`` with both ``$(ros2 pkg prefix --share ...)``
  and source-tree forms, an arguments table, and a worked example of the
  common ``maps_path``-points-to-config-file misuse with the actual
  FATAL log output.

* Docker + Intel/AMD 内蔵 GPU で RViz / Gazebo の **片方だけ** GUI を
  出す組合せ (例 ``gazebo_gui:=false``) のとき RViz が GL 初期化失敗で
  crash する問題 (#229, PR #296) の回避手段を整備。``docker-compose.dri.yml``
  (Intel/AMD iGPU 向け ``/dev/dri`` 共有 + ``group_add: [video, render]``
  の override) を新設し、``docs/docker.md`` に DRI override の使い方・
  GID 不一致時の対処・GPU 共有できない環境向けの software 経路
  (``gazebo_gui:=false`` で gz-sim を headless にして RViz だけ
  ``LIBGL_ALWAYS_SOFTWARE=1`` で起動) を追記。launch / アプリコードの
  変更はなし。

Internal / tests
----------------

* ``mapoi_interfaces`` docs ↔ implementation consistency lint (#216, PR
  #217). New CI step (``scripts/check_docs_consistency.py``) runs in the
  existing ``Consistency Check`` workflow and verifies (a) every
  ``rosidl_generate_interfaces``-listed msg / srv has a matching
  ``###`` heading in ``mapoi_interfaces/README.md``; (b)
  cross-references like ``mapoi_interfaces/msg/X.msg`` resolve to real
  files; (c) ``reason`` string literals in ``publish_backend_status``
  functions don't contain absolute paths / IPs / credentials /
  hostname-like literals. The first run of the lint also surfaced 3
  missing README sections (``TagDefinition`` / ``InitialPoseRequest`` /
  ``LocalizationBackendStatus``) which were filled in the same PR.

* Test suite reorganization and contract tests (PR #194, #195, #196).
  Test layout cleaned up (#193 in-progress); added
  ``test_system_tags_contract.cpp`` to pin ``kSystemTags`` against the
  WebUI tag list, ``test_resolve_backend_status_for_ui.py`` for the
  WebUI legacy-fallback / lost-liveliness logic, and three integration
  tests for ``EVENT_STOPPED`` / ``EVENT_RESUMED`` lifecycle (#177).


0.3.0 (2026-05-02)
==================

Breaking changes
----------------

* Runtime topic naming reorganized into the ``/mapoi/...`` namespace
  (#185, #70, #102). Legacy flat ``/mapoi_*`` runtime topics were removed
  without compatibility aliases; launch files, scripts, RViz configs, and
  external nodes must migrate to the new topic names listed below.

  - Marker topics consolidated to ``/mapoi/markers/pois`` and
    ``/mapoi/markers/routes`` in PR #186.
  - Non-marker mapoi topics moved to ``/mapoi/highlight/*`` /
    ``/mapoi/nav/*`` / ``/mapoi/events`` / ``/mapoi/config_path`` /
    ``/mapoi/initialpose_poi`` in PR #187.
  - The ``mapoi_rviz_pose`` topic from the RViz PoseTool is out of scope
    for this namespace reorganization and is unchanged.

  .. list-table::
     :header-rows: 1

     * - Old topic
       - New topic
     * - ``/mapoi_goal_marks``
       - ``/mapoi/markers/pois``
     * - ``/mapoi_event_marks``
       - ``/mapoi/markers/pois``
     * - ``/mapoi_route_marks``
       - ``/mapoi/markers/routes``
     * - ``/mapoi_highlight_goal``
       - ``/mapoi/highlight/goal``
     * - ``/mapoi_highlight_route``
       - ``/mapoi/highlight/route``
     * - ``/mapoi_goal_pose_poi``
       - ``/mapoi/nav/goal_pose_poi``
     * - ``/mapoi_route``
       - ``/mapoi/nav/route``
     * - ``/mapoi_cancel``
       - ``/mapoi/nav/cancel``
     * - ``/mapoi_pause``
       - ``/mapoi/nav/pause``
     * - ``/mapoi_resume``
       - ``/mapoi/nav/resume``
     * - ``/mapoi_switch_map``
       - ``/mapoi/nav/switch_map``
     * - ``/mapoi_nav_status``
       - ``/mapoi/nav/status``
     * - ``/mapoi_poi_events``
       - ``/mapoi/events``
     * - ``/mapoi_config_path``
       - ``/mapoi/config_path``
     * - ``/mapoi_initialpose_poi``
       - ``/mapoi/initialpose_poi``

* Sample maps removed from ``mapoi_server``; ``maps_path`` parameter
  is now required (#163 stage 1).

  - Before: ``mapoi_server`` shipped ``maps/turtlebot3_world/`` /
    ``maps/turtlebot3_dqn_stage1/``, with ``maps_path`` defaulting to
    ``{pkg_share}/maps``.
  - After: Sample maps live only in ``mapoi_turtlebot3_example``
    (single source of truth). ``mapoi_server`` no longer provides a
    default for ``maps_path``; omitting it is fatal at startup.
  - Retained: ``mapoi_server/maps/tag_definitions.yaml`` is kept as the
    package-internal system tag definition.
  - Migration: Set ``maps_path`` explicitly via launch or parameter. To
    reuse the sample, use
    ``$(find-pkg-share mapoi_turtlebot3_example)/maps``.

* ``mapoi_turtlebot3_example`` sample ``map_file`` names differentiated
  per world (#163 stage 2).

  - Before: ``turtlebot3_world/turtlebot3.{pgm,yaml}`` /
    ``turtlebot3_dqn_stage1/turtlebot3.{pgm,yaml}`` (same name in both
    worlds).
  - After: ``turtlebot3_world/turtlebot3_world.{pgm,yaml}`` /
    ``turtlebot3_dqn_stage1/turtlebot3_dqn_stage1.{pgm,yaml}`` (matches
    the directory name).
  - The ``map_file`` value and ``image:`` line in each
    ``mapoi_config.yaml`` were updated accordingly.
  - Migration: No effect unless launch files hard-code ``map_file``
    (it is normally referenced via ``mapoi_config.yaml``).
  - Related: ``scripts/check_sample_yaml_consistency.py`` no longer
    performs server/example pair-sync checks; it now only validates the
    example side individually.

* ``pub_interval_ms`` parameter removed from ``mapoi_server`` (#135).

  - Before: ``/mapoi_config_path`` was periodically published at
    ``pub_interval_ms`` (default ``5000`` ms, ``500`` ms in sample
    launches).
  - After: Published explicitly at startup, on ``SwitchMap``, and on
    ``reload_map_info``. The publisher keeps ``transient_local`` QoS
    and subscribers (``poi_editor`` / ``mapoi_panel`` /
    ``mapoi_webui_node``) were aligned to ``transient_local`` from the
    default QoS.
  - Migration: Remove ``pub_interval_ms`` from launch files and
    parameter YAMLs. (Passing it is harmless—ROS will only warn about
    an unused parameter—but it has no effect.)
  - Motivation: The periodic publish trigger rebuilt the
    ``poi_editor`` table / ``mapoi_panel`` ComboBox while the user was
    editing, interrupting selection and text input (surfaced during
    PR #168 verification). The ``transient_local`` QoS already solves
    the startup-order problem, so the heartbeat is no longer needed.

* ``initial_pose`` system tag removed (#89 stage 3, #144).

  - Before: POIs tagged ``initial_pose`` were auto-published on map
    load / switch.
  - After: The **first POI** in the new map's POI list (excluding
    ``landmark``-tagged POIs) is used as the default. The new
    ``initial_poi_name`` field on ``SwitchMap.srv`` allows explicit
    specification.
  - Publish path: When ``mapoi_server`` loads a new map, it publishes
    the POI name to ``/mapoi/initialpose_poi`` (transient_local);
    ``mapoi_nav_server`` receives it and forwards to ``/initialpose``.
  - YAML migration: Remove ``initial_pose`` from
    ``tags: [..., initial_pose]`` and place the starting POI first
    under ``poi:``.
  - ``mapoi_gazebo_bridge`` / ``mapoi_gz_bridge`` robot spawn positions
    follow the same semantics (POI list head).
  - ``reload_map_info`` service behavior change: After POI edits, it
    no longer republishes ``/mapoi/initialpose_poi`` (avoids the risk
    of rewinding the live robot pose). Re-initialization should go
    through ``SwitchMap`` (i.e., a map switch) or a manual path
    (RViz / WebUI / direct publish to ``/mapoi/initialpose_poi``).
  - ``mapoi_nav_server`` parameter changes: Removed
    ``initial_pose_subscriber_wait_timeout_sec`` and replaced it with
    an async retry-timer based design
    (``initialpose_retry_interval_sec``,
    ``initialpose_retry_max_attempts``,
    ``initialpose_post_subscribe_republish_count``). Prevents the
    regression where blocking waits stopped other callbacks.
  - The legacy ``/mapoi_initialpose_poi`` was migrated to
    ``/mapoi/initialpose_poi``, and the type changed from
    ``std_msgs/String`` to
    ``mapoi_interfaces/InitialPoseRequest`` (``{map_name, poi_name}``).
    To prevent SwitchMap-time topic synchronization races (bridges /
    ``mapoi_nav_server`` picking up a stale-generation POI name),
    subscribers now validate the generation via ``map_name``.
    Publishers: ``mapoi_server``, ``mapoi_webui`` (both updated).
    Subscribers: ``mapoi_nav_server``, ``mapoi_gazebo_bridge``,
    ``mapoi_gz_bridge`` (all updated).

* ``pause`` system tag firing conditions tightened (#89 stage 2, #143):

  - Before: Fired at any POI when ``nav_mode_ != IDLE`` (i.e., during
    GOAL or ROUTE navigation).
  - After: Fires only when ``nav_mode_ == ROUTE`` *and* the POI is in
    the active route's ``waypoints`` / ``landmarks``. Does not fire
    during GOAL navigation or IDLE.

  Behavioral change: A single ``/mapoi/nav/goal_pose_poi`` no longer
  triggers an auto-pause if the robot incidentally enters a ``pause``
  POI. Pause now runs only in route-based scenarios.

* ``GetRoutePois.srv`` response gained a ``landmark_pois`` field (#143).
  POIs referenced by ``route.landmarks`` in YAML appear here. They are
  not sent to Nav2 and are used only for radius monitoring and
  route-scoped pause firing.

* ``landmark × pause`` mutual exclusion added. ``landmark`` POIs are
  unreachable references, so pause cannot trigger on them. Rejected by
  WebUI / RViz Panel / API validation.

* System tag ``goal`` renamed to ``waypoint``. Both single navigation
  targets and route waypoints share this tag (the Nav2 navigation
  destination tag). YAML ``tags: [goal, ...]`` must be migrated to
  ``tags: [waypoint, ...]``. Topic / service / action names referring
  to "goal" (e.g. ``/mapoi/nav/goal_pose_poi``, ``goal_pose``) remain
  unchanged because they follow Nav2 native terminology.
  (#89 stage 1, #142)
* System tag ``origin`` removed. Map origin reference points should be
  expressed with the ``landmark`` system tag plus a custom tag (e.g.
  ``map_origin``) for color / semantic differentiation. Visualization
  defaults to gray; per-tag color is tracked in #70. (#89 stage 0)
* ``PointOfInterest.msg``: the ``float64 radius`` field was removed in
  favor of the new ``mapoi_interfaces/Tolerance tolerance`` struct
  (``xy`` + ``yaw``, aligned with Nav2 ``SimpleGoalChecker``). YAML
  ``poi.radius`` keys must be migrated to
  ``poi.tolerance: {xy, yaw}``. (#87)
* ``Tolerance.msg`` semantics: both ``xy`` and ``yaw`` are required to
  be ``>= 0.001`` (``xy``: 1 mm / ``yaw``: about 0.057°). Zero and
  negative values are disallowed (they would otherwise allow
  constructing a practically unresponsive POI). The "unspecified ->
  Nav2 default fallback" semantics is removed. Out-of-range values
  found at YAML load time are clamped to ``0.001`` with a WARN log.
  (#138)

* ``mapoi_nav_server`` ROS parameter ``radius_check_hz`` renamed to
  ``tolerance_check_hz`` (#140). Naming alignment for the
  ``PointOfInterest.radius`` -> ``tolerance.xy`` migration (#87).
  Default value, unit, and behavior are unchanged. Launch files and
  YAMLs that set ``radius_check_hz`` must rename it to
  ``tolerance_check_hz``.

* ``arrow_size_ratio`` parameter removed from ``mapoi_rviz2_publisher``
  (#136).

  - Before: ``arrow_size_ratio`` (double, default ``1.0``) dynamically
    sized POI arrows to ``radius × ratio``.
  - After: Arrows use a fixed scale (length ``0.15``, shaft / head
    ``0.04``) and serve as a directional cue for the sector
    visualization (a radius-coupled multiplier is no longer needed).
  - Migration: Remove any ``arrow_size_ratio`` declaration / set in
    existing launch files or YAMLs (the parameter itself is gone).

Interfaces
----------

* New ``mapoi_interfaces/msg/Tolerance.msg`` (``xy`` / ``yaw``). (#87)
* ``PoiEvent.msg``: new constants ``EVENT_STOPPED=3`` and
  ``EVENT_RESUMED=4``. Publish logic for these events is deferred to a
  follow-up issue. (#87)

WebUI / rviz_plugins
--------------------

* POI Editor (WebUI form / rviz_plugins POI Editor table) replaces the
  ``Radius`` input / column with ``tolerance.xy`` + ``tolerance.yaw``.
  Only the ``yaw`` UI input was displayed and entered in degrees;
  internal storage and YAML remained in radians (#138). The HTML
  ``min`` constraint and Panel ``ValidatePois`` enforce
  ``xy >= 0.001`` (m) / ``yaw >= 0.06`` (deg ≈ 0.001 rad).
* ``mapoi_webui_node``'s ``POST /api/pois`` validates the
  ``tolerance.{xy, yaw}`` min constraints on the backend as well, so
  frontend bypasses are caught (#138).
* RViz POI Editor column layout reorganized from 6 columns to 5 (#158):

  - Before: ``name / description / pose / tolerance.xy / tolerance.yaw (deg) / tags``.
  - After: ``name / pose (x, y, yaw rad) / tolerance (xy m, yaw rad) / tags / description``.
  - Tolerance is collapsed into one column (e.g., ``0.5, 0.7854``);
    ``yaw`` is now displayed in **radians** to stay consistent with
    ``pose.yaw`` (this reverts the temporary degree display introduced
    in #138).
  - The ``description`` column moved to the end (so long text doesn't
    crowd the table width).
  - Validation min constraints simplified to ``xy >= 0.001 m`` /
    ``yaw >= 0.001 rad`` (the old degree-conversion message is gone).
  - Validation gained a **new max constraint ``yaw <= 2π rad``**.
    Because this PR switched the UI input unit from degrees to
    radians, this guards against entering an old-style degree value
    (e.g., ``45``) that would otherwise be interpreted as ~7 turns of
    rotation. **Existing YAML files with ``yaw > 2π`` will fail to
    save**, so any out-of-range values must be corrected (yaw is
    conventionally expressed in ``[0, 2π]`` or ``[-π, π]``; values
    beyond that are practically never legitimate). A hard reject was
    chosen over a soft warning to prioritize typo prevention.

* WebUI now picks up config changes saved from RViz or external tools
  via SSE and reloads immediately (#135 (B)).

  - Before: Saving in the rviz PoiEditor did not propagate to the
    WebUI until the browser was reloaded.
  - After: ``mapoi_webui_node`` exposes a new ``/api/events`` SSE
    endpoint. When ``/mapoi/config_path`` is received, it broadcasts
    ``{type: "config_changed"}`` to all connected clients. The
    frontend (``app.js``) consumes the event via ``EventSource`` and
    re-fetches via ``loadTagDefinitions / loadPois / loadRoutes``.
  - Related: ``Flask app.run(threaded=True)`` is now set explicitly
    (so long-lived SSE connections and other requests run
    concurrently).
  - Closes #135 in combination with #135 (A) addressed in PR #168
    (PoiEditor / MapoiPanel callback fixes).

* POI tolerance is rendered as a sector marker in
  ``mapoi_rviz2_publisher`` and ``mapoi_webui`` (#136).

  - Radius = ``tolerance.xy``; sector angle = ``2 * tolerance.yaw``;
    centerline = ``pose.yaw``.
  - ``waypoint`` = filled sector, ``landmark`` = hollow sector,
    ``pause`` = dashed outline overlay (a thin solid line is used for
    the primary glyph for contrast).
  - The sector is drawn only when ``0 < tolerance.yaw < π``; otherwise
    (yaw-agnostic) the POI is treated as a full circle (allowing
    ``pass_through`` to be expressed).
  - New ``show_tolerance_sector`` parameter on
    ``mapoi_rviz2_publisher`` (bool, default ``true``) toggles the
    tolerance visualization at runtime.
  - Subsequently #179 split this into a circle (xy detection region)
    + sector (yaw constraint) overlay (see below).

* POI tolerance split into circle + sector overlays, and the pause
  overlay reworked into a dot pattern in
  ``mapoi_rviz2_publisher`` / ``mapoi_webui`` (#179, addressing user
  feedback from #178).

  - Circle outline (thin solid line, faint): always shown to indicate
    the robot's ``tolerance.xy`` entry-detection region. Drawn
    yaw-agnostically as a visual cue for the detection logic
    (Euclidean ``< tolerance.xy``).
  - Sector (filled or hollow): emphasizes the yaw constraint. Drawn
    only when ``0 < tolerance.yaw < π`` to avoid information
    redundancy with the full circle.
  - **Visual change**: Previously (#178), yaw-agnostic POIs
    (``tolerance.yaw == 0`` or ``>= π``) were also drawn as a sector
    that happened to cover the full circle (filled green for
    ``waypoint``, hollow gray for ``landmark``). With this change,
    yaw-agnostic POIs show **only the thin faint circle outline**
    without the filled or hollow stroke. This matches the detection
    semantics (xy only), but viewers used to the old display will
    notice a shift (worth checking before reusing demos / screenshots).
  - Pause overlay: The old dashed style (``dashArray: '6, 4'`` /
    RViz segment 0.05 m, 1:1 ratio) was reported as "doesn't read as
    dots, just looks crushed" (#178 PR comments), so it was changed
    to a sparse dot pattern. WebUI uses ``dashArray: '2, 6'`` with
    ``lineCap: round`` (25 % cycle on); RViz uses dots 0.02 m long
    with 0.10 m gaps and a 0.04 m line (20 % cycle on). The dots are
    overlaid along the xy circle so the boundary aligns with the
    pause-firing condition (inside the xy circle).
  - The ``show_tolerance_sector`` parameter keeps its old name but now
    controls the entire circle + sector + pause overlay (preserves
    backward compatibility).
  - **Maintenance note**: The drawing spec is duplicated in
    ``mapoi_rviz2_publisher.cpp`` (RViz) and ``map-viewer.js``
    (WebUI). Spec changes must be applied to both (continuing from
    #136; unifying the two is being considered together with #70).
  - **POI Editor Panel Display Settings UI fix**: When PR #178
    removed ``arrow_size_ratio`` from the publisher, the Panel's
    DoubleSpinBox UI was left in place, causing
    ``get_parameters`` / ``set_parameters`` calls to fail (a
    regression). This PR replaces that UI with a checkbox for
    ``show_tolerance_sector`` so the runtime display-layer toggle is
    available from the Panel.

* Legacy ``event`` tag ``elif`` branch removed from
  ``mapoi_rviz2_publisher`` (#180).

  - ``event`` was originally a system tag but had been demoted to a
    custom tag in #89 stage 1. The RViz publisher still had a
    hard-coded branch drawing a blue arrow + label for it, which
    PR #178's cursor review flagged as high-priority "legacy
    leftover" follow-up.
  - This PR removes the ``elif`` branch. POIs with the ``event`` tag
    are now drawn (sector + arrow) only if they also carry another
    system tag (``waypoint`` / ``landmark``); otherwise they are
    omitted from the visualization, matching the behavior of other
    custom tags.
  - Composite-tag POIs such as ``hazard_south`` are reorganized to
    ``[landmark, hazard]`` (sample-side change, see Samples).
  - Color / glyph reorganization is tracked in #70.

Samples
-------

* Sample YAMLs (``turtlebot3_world`` / ``turtlebot3_dqn_stage1``)
  reworked to cover the full feature matrix (#146). Strengthens
  regression coverage for new features (``route.landmarks`` #143,
  sector rendering #136, ``tolerance.yaw`` #138).

  - ``turtlebot3_world`` (office tour scenario): POI count 5 -> 9.
    Adds consecutive ``pause`` (``corridor_a`` + ``corridor_b``), a
    photography stop (``conference_room`` with
    ``tolerance.yaw=0.10``), yaw-agnostic passes (``corridor_*`` with
    ``tolerance.yaw=π``), three landmark variations (pure
    ``landmark`` / ``landmark + audio_info`` /
    ``landmark + capture_target``), and a contrast between
    ``tour_full`` (with both waypoints and landmarks) and
    ``tour_short`` (no landmarks). New custom tags
    ``capture_trigger`` / ``capture_target``.
  - ``turtlebot3_dqn_stage1`` (obstacle-avoidance sandbox): POI count
    5 -> 7. Adds composite-tag hazards (``landmark + hazard`` -
    originally ``[event, landmark, hazard]`` but reorganized to
    ``[landmark, hazard]`` after the ``event`` tag removal in #180),
    yaw-agnostic pass-through points (``checkpoint_west`` /
    ``checkpoint_east`` with ``tolerance.yaw=π``, used in lieu of a
    ``pass_through`` tag), a pause checkpoint
    (``pause_intersection``), and a contrast between ``avoidance_a``
    (waypoints + landmarks) and ``avoidance_b`` (waypoints only). New
    custom tag ``observation``.
  - ``mapoi_turtlebot3_example/README.md`` gained a sample summary
    table listing the features / POI-tag composition each scenario
    exercises.

  **Route renames** (old -> new):

  - ``turtlebot3_world``: ``route_1`` -> ``tour_full``, ``route_2`` ->
    ``tour_short``.
  - ``turtlebot3_dqn_stage1``: unchanged (``avoidance_a`` /
    ``avoidance_b``).

  ``tour_short`` is functionally compatible with the old ``route_2``
  (``[elevator_hall, corridor_a, conference_room]``); the path is the
  same. Launch files, clients, and internal scripts that reference
  ``route_1`` / ``route_2`` directly must migrate to the new names.
  External tutorials that pin the sample YAML snapshot also need
  updates because the POI count, names, and tag combinations have
  expanded significantly.

* New ``scripts/check_sample_yaml_consistency.py`` plus CI hookup
  (#146). On pull-request and push, it automatically verifies POI
  name / route waypoint / landmark reference consistency, tag
  exclusion rules (``waypoint × landmark``, ``landmark × pause``),
  and the ``tolerance.{xy, yaw} >= 0.001`` minimum constraint in
  sample YAMLs.

Fixes
-----

* ``mapoi_server`` ``reload_map_info`` service now explicitly publishes
  a skip message (empty ``poi_name``) to ``/mapoi/initialpose_poi``
  (#154). This eliminates the stale-message problem where the
  ``transient_local`` (depth=1) latched value retained the pre-reload
  POI name, causing late-starting ``mapoi_nav_server`` instances or
  reconnecting RViz panels to receive a pre-edit POI name. Subscribers
  (``mapoi_nav_server`` / ``mapoi_gazebo_bridge`` / ``mapoi_gz_bridge``)
  ignore empty ``poi_name`` by convention, so the live pose is not
  rewound (the invariant established in #149 round 4 is preserved).

Navigation server
-----------------

* ``mapoi_nav_server`` now publishes ``EVENT_STOPPED`` /
  ``EVENT_RESUMED`` (#140). Activates the constants added to
  ``PoiEvent.msg`` in #87. Two OR-combined trigger sources:

  - **Nav2 action SUCCEEDED**: When the ``FollowWaypoints`` /
    ``NavigateToPose`` result callback detects ``SUCCEEDED``,
    ``EVENT_STOPPED`` is published immediately for every POI in the
    inside state (without waiting for the ``cmd_vel`` dwell).
  - **cmd_vel based**: When both linear and angular velocity stay
    below ``stopped_speed_threshold`` (default ``0.01``) for at
    least ``stopped_dwell_time_sec`` (default ``1.0`` s),
    ``EVENT_STOPPED`` is published. Evaluated on every
    ``tolerance_check_callback`` tick.
  - **EVENT_RESUMED**: When a STOPPED POI's velocity goes back above
    the threshold, ``EVENT_RESUMED`` is published immediately (no
    dwell required). It is also published for all STOPPED POIs upon
    receiving a new ``/mapoi/nav/goal_pose_poi`` / ``/mapoi/nav/route``
    so subscribers can detect "pause released" without delay.
  - **On EXIT**: ``poi_inside_state_`` -> false also resets
    ``poi_stopped_state_``. The EXIT event implicitly covers "pause
    released", so RESUMED is not published here. The lifecycle is
    organized as ENTER -> STOPPED -> EXIT.

  New ROS parameters:

  - ``stopped_speed_threshold`` (default ``0.01`` m/s equivalent).
  - ``stopped_dwell_time_sec`` (default ``1.0``).
  - ``cmd_vel_topic`` (default ``cmd_vel``).

  The state-machine pure function ``compute_stopped_transition`` was
  extracted, with five new unit tests added in
  ``test_nav_server_unit``.

Internal
--------

* Initial-pose selection logic extracted into a shared header
  ``mapoi_server/include/mapoi_server/initial_pose_resolver.hpp``,
  unifying the three duplicate implementations across
  ``mapoi_server`` / ``mapoi_gazebo_bridge`` / ``mapoi_gz_bridge``
  (#150). Prevents future divergence between simulator spawn behavior
  and ``/initialpose`` if the ``initial_poi_name`` specification
  (exclusion rules / fallback) evolves. Behavior is preserved (pure
  function rename and file relocation only; existing unit tests still
  pass).


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
