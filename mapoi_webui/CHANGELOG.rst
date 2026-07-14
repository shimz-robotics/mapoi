^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mapoi_webui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This per-package file lists only the changes affecting ``mapoi_webui``
(the ``mapoi_webui_node`` REST/SSE backend and the WebUI frontend), in the
flat format expected by bloom / buildfarm. Full project-level narrative:
root ``CHANGELOG.rst`` / the GitHub Releases page.

Forthcoming
-----------

0.5.0 (2026-07-10)
------------------

* Add a machine-readable ``code`` field to every JSON error response, and extend the optimistic-concurrency ``expected_version`` check to ``POST /api/routes`` and ``POST /api/custom_tags`` (409 + ``version_mismatch``) (#343, #363).
* Forward ``mapoi/nav/command_rejected`` as an SSE ``command_rejected`` event and show an auto-dismissing toast in the frontend (#354).
* Allow the selected POI's marker to be dragged on the map to update ``pose.x`` / ``pose.y`` in the working copy (saved on Save) (#239, #274).
* Add a rotation handle on the tolerance sector tip to adjust ``pose.yaw`` by dragging (#275, #276).
* Separate select from edit: a single click selects a POI, a double-click opens the edit form (#240, #272).
* Render a yaw-agnostic POI tolerance (``tolerance.yaw >= pi``) as a filled disc and clean up the degree display (#267, #268, #271).
* Highlight a POI/route on the map when picked from the Navigation dropdowns (#262, #278).
* Add Undo/Redo to the POI editor (per-mutation snapshot stack capped at 50), with dirty-state derived from saved content (#300, #301).
* ``GET /api/pois`` returns a ``config_version`` (sha256) and ``POST /api/pois`` rejects a stale write with ``409``; frontend surfaces a reload-confirmation dialog (#241, #245).
* Add a floating button to toggle the header/panel UI and a Display section with a floating-overlay mode plus a UI-opacity slider (persisted in ``localStorage``) (#323, #324, #325).
* Add a floating Undo icon on narrow screens and centralize the route-editor reload guard at the ``loadRoutes()`` call sites (#332, #334).
* Reorganize the REST URL hierarchy under ``/api/editor/`` (editor) vs ``/api/nav/`` (running robot) and unify separators on kebab-case; old paths return ``404`` (#340, #343).
* Split ``map-viewer.js`` into ``map-icons.js`` + ``MapViewerSector`` and extract the custom-tag UI into a ``TagEditor`` class; internal refactor, no behavior change (#346).
* Remove the map zoom +/- buttons and lock POI position-drag editing behind a toggle by default (#335).
* Further POI/Route editor UX refinements (click-to-add-waypoint, selection sync, Escape-to-cancel, English warning strings) (#306, #312, #314, #316, #318, #320, #322).
* Fix a CSS bug where the map collapsed to zero height on mobile layouts and enlarge under-sized touch targets (#324, #331).
* ``POST /api/nav/switch-map`` now rejects a non-string ``map_name`` with ``400`` instead of coercing it (#199, #281).
* Round POI floats on Save to the YAML precision (``xy`` 3 decimals, ``yaw`` 4 decimals) on both frontend and backend (#242, #244).
* Fix the pose tool committing a POI position on the first click; position is now a preview until the second click confirms (#269, #270).

0.4.0 (2026-05-08)
------------------

* Subscribe to ``mapoi/nav/backend_status`` and show a Navigation connected/disconnected indicator, enabling navigation control only when ``backend_ready=true``; reworked mobile focus interaction (#197, #198).
* Detect bridge death via ``LivelinessChangedEvent`` and treat the cached status as ``backend_ready=false`` (#208).
* Show a separate Localization indicator and gate the Set Initial Pose UI on ``LocalizationBackendStatus`` (#209).

0.3.0 (2026-05-02)
------------------

* Replace the POI Editor ``Radius`` input with ``tolerance.xy`` + ``tolerance.yaw`` (yaw entered in degrees, stored in radians); backend ``POST /api/pois`` also validates the ``>= 0.001`` minimums (#138).
* Pick up config changes from RViz/external tools via a new ``/api/events`` SSE endpoint and reload immediately (#135).
* Render POI tolerance as circle + sector overlays with a dot-pattern pause overlay (spec mirrored with ``mapoi_rviz2_publisher``) (#136, #179).

0.2.0 (2026-04-29)
------------------

* Add the ``robot_radius`` parameter (default ``0.15``) driving the robot marker size and active-route arrival threshold, propagated to the frontend (#116, #117, #118, #126).
* Active-route highlight now propagates to direction arrows and order labels, dimming non-active routes; parallel-offset rendering for overlapping route polylines; directional connector arrow to the first waypoint (#69, #105, #128, #106, #107).
* POI/route name uniqueness validation on both client and backend ``POST /api/pois`` / ``POST /api/routes`` (#109, #120).
* Sidebar UX: drag-resize handle with persisted width, sticky form-actions footer, and ellipsis-with-tooltip for long names (#122, #123, #124, #125).
* Navigation dropdowns auto-select the focused POI/route; robot marker color matches the active route's color (#111, #114, #110, #113).
* POI editor: visible "SAVED" feedback and table rebuilt post-save to keep row numbering aligned (#74, #76, #77, #95).
* Fetch ``tag_definitions`` through the ``get_tag_definitions`` service instead of reading YAML directly (#39, #59).
* Add ``mapoi_editor.launch.yaml`` for headless POI editing without the nav bridge; service/publisher calls degrade gracefully (#60, #62).
