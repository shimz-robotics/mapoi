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
