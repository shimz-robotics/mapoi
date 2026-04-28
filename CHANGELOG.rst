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

* ``PointOfInterest.msg``: ``float64 radius`` field removed in favor of the
  new ``mapoi_interfaces/Tolerance tolerance`` struct (``xy`` + ``yaw``,
  Nav2 ``SimpleGoalChecker`` align). YAML ``poi.radius`` keys must be
  migrated to ``poi.tolerance: {xy, yaw}``. Old configs without
  ``tolerance`` will WARN and fall back to default ``xy=0.5, yaw=0.0``.
  (#87)

Interfaces
----------

* New ``mapoi_interfaces/msg/Tolerance.msg`` (``xy`` / ``yaw``). (#87)
* ``PoiEvent.msg``: new constants ``EVENT_STOPPED=3`` and
  ``EVENT_RESUMED=4``. Publish logic for these events is deferred to a
  follow-up issue. (#87)

WebUI / rviz_plugins
--------------------

* POI Editor (WebUI form / rviz_plugins POI Editor table) replaces the
  ``Radius`` input / column with ``tolerance.xy``. ``tolerance.yaw`` input
  UI is intentionally deferred to a follow-up issue; existing
  ``tolerance.yaw`` values in YAML are preserved through the WebUI form
  but are reset to ``0.0`` when saving from the rviz Panel. (#87)


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
