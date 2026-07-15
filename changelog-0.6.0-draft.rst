Added
-----

* WebUI: pressing ``Delete`` with a POI selected removes it (working-copy only,
  revertible with Ctrl+Z); the input-focus guard already used by the Ctrl+Z /
  Escape keydown funnel keeps it from firing while typing (#374).

* WebUI: Ctrl/Cmd+S saves all dirty editors — an open edit form is first
  confirmed into the working copy — and always calls ``preventDefault()`` to
  suppress the browser's native "save page" shortcut (#375).

* WebUI: a ``beforeunload`` handler warns before closing the tab or reloading
  with unsaved edits, reusing the same dirty-editor / open-form "blocker"
  collection the SSE reload guard below is built on (#380).

* WebUI: the map now shows the cursor's world (x, y) coordinates in a corner
  readout while the mouse moves over it, rounded to the same precision as the
  POI YAML and hidden once the cursor leaves the map or the coordinates are
  non-finite (#381).

* WebUI: selecting a POI from the list, or from the Navigation section's
  dropdowns, now pans the map to it if it is off-screen; a marker click does
  not pan, since a clicked POI is already visible and an unrequested viewport
  jump would be disorienting (#382).

* WebUI: a search box above the POI list filters it by name (case-insensitive
  substring); it only affects the list rendering, not marker visibility on
  the map, which stays governed by the existing per-POI checkboxes (#383).

* WebUI: single-key shortcuts ``L`` (toggle POI position-lock) and ``U``
  (toggle UI panel visibility) mirror the existing lock / hide-UI buttons —
  each key is its target's initial letter, for a consistent mnemonic — and
  drive the same ``.click()`` code path as the buttons so there is no
  separate state-sync implementation; both are ignored while an input /
  textarea has focus, while a modifier key is held, and on key-repeat. The
  UI-toggle icon itself was also fixed to a static ☰ glyph in the same
  change, replacing an earlier eye/eye-off swap that read ambiguously over a
  map (#390).

* WebUI: a ``?`` button/key opens a Help modal (closed via the ``×`` button,
  Escape, or a background click) covering keyboard shortcuts, map
  operations, editing/saving, the POI list, and navigation controls, plus a
  note that a Vim-style browser extension can intercept the shortcuts (#391).

* Documentation received a new-user-onboarding pass: both READMEs (en/ja)
  were reordered around a newcomer's reading path (overview -> screenshots ->
  features -> Docker quickstart -> build) and gained desktop/mobile WebUI
  screenshots, captured headless against the mock e2e server's
  ``turtlebot3_world`` data, plus CI/release/license/distro badges. A new
  ``docs/architecture.md`` (English, with a ``.ja.md`` snapshot) adds a node
  diagram and two mermaid data-flow walkthroughs (go-to-POI, map switch)
  plus a full topic/service/action table cross-checked against the
  ``create_*`` call sites. The remaining ``docs/`` pages (integration,
  docker, backend-status, migration) were translated to English-primary to
  match the READMEs' v0.5.0 switch, and GitHub issue templates for bug
  reports and feature requests were added (#396).

* RViz's ``MapoiPanel`` subscribes to ``mapoi/nav/command_rejected`` and
  shows the rejected target as a transient one-line notice, auto-cleared
  after 5s, closing a gap where a command rejected mid-navigation (e.g. a
  typo goal sent while already driving) was invisible in RViz even though
  the WebUI already toasted it (#398).

* RViz's ``MapoiPanel`` now routes operation failures and initial-pose
  success through the same transient-notice mechanism introduced by #398
  above (a shared ``ShowTransientNotice()`` helper on ``CommandRejectedLabel``,
  green for success / red for failure): a failed ``get_route_pois`` or
  ``request_initial_pose`` call, a service that never came up, and a
  successful initial-pose request. Previously these paths only logged to
  ``RCLCPP_ERROR``, so an operator not watching the console could believe an
  initial pose had been set when the server had actually rejected it (#401).

* RViz's ``MapoiPanel`` gained a small connection-status badge each for Nav2
  and localization, built from a pure ``build_backend_badge_text()`` helper:
  "Connected", "Not ready (<reason>)" while the backend status message's
  ``reason`` field is set, and "Disconnected (bridge stopped)" once DDS
  liveliness is lost. Previously a backend outage only disabled six widgets
  silently, giving no way to tell why buttons were grayed out (#400).

* RViz's ``MapoiPanel`` subscribes to ``mapoi/events`` and shows "Entered:
  <poi> (n/total)" / "Paused at: <poi> (n/total)" / "Passed: <poi> (n/total)"
  on a new ``RouteProgressLabel`` while a route is driving; ``total`` comes
  from the panel's own route selection, and the display is suppressed once
  navigation ends so a late-arriving event cannot resurrect stale progress.
  ``mapoi/events`` only fires during ``ROUTE`` navigation, so the label stays
  empty for single-goal driving (#406).

* RViz's ``PoiEditor`` panel now guards against an external
  ``mapoi/config_path`` republish — another client's save, or a map switch —
  silently rebuilding the table over unsaved cell edits: a ``table_dirty_``
  flag, set on every mutating action, gates a "discard and reload / keep
  editing" confirmation before ``ConfigPathCallback`` proceeds, closing an
  asymmetry with the WebUI's existing dirty-guard / 409-conflict machinery
  (#399).

* RViz's ``PoiEditor`` table gained a name filter box (``QLineEdit`` +
  ``setRowHidden``, case-insensitive substring match) alongside the existing
  tag filter. Rows are hidden rather than removed, so ``rowCount()`` — and
  therefore Save's row loop — stays unaffected, and the two filters compose:
  the tag filter narrows the row set first, and the name filter further
  hides within it (#405).

* RViz's ``PoiEditor`` panel gained Undo/Redo via a ``QUndoStack`` covering
  row add / copy / delete, drag reorder, and cell edits (Ctrl+Z /
  Ctrl+Shift+Z), matching the Undo/Redo the WebUI got in v0.5.0 (#407).
  Because the shortcuts are ``QShortcut``-only with no on-screen affordance,
  real-hardware testing subsequently found it was mistaken for "not
  implemented": the panel needs keyboard focus first (RViz's 3D view
  usually holds it), Linux Redo is Ctrl+Shift+Z rather than Ctrl+Y, and the
  history clears on save / tag-filter change / a full table rebuild in ways
  that make "edit -> save -> Ctrl+Z" a no-op. Two toolbar buttons (disabled
  by default, enabled via ``QUndoStack::canUndoChanged``/``canRedoChanged``)
  now give Undo/Redo an explicit, always-visible affordance with tooltips
  spelling out the shortcuts (#435).

* ``MapoiPanel``'s ``NavStatusLabel`` / ``RouteProgressLabel`` /
  ``CommandRejectedLabel`` now show a gray idle placeholder ("Nav status: —"
  / "Route progress: —" / "Notice: —") instead of sitting empty, so the
  layout space reserved to avoid jumping reads as "a value goes here" rather
  than unexplained blank lines (#451).

* RViz plugin UI strings are now English throughout
  (``mapoi_panel_nav_status.cpp``, ``mapoi_panel_nav_control.cpp``, the
  ``.ui`` files, and the two remaining ``tr()``-wrapped-but-Japanese
  validation messages) — the last holdout after the WebUI (already
  English-only) and most of ``PoiEditor`` (#431).

Changed
-------

* **``mapoi_rviz_plugins`` translation-unit split continues (#397, 8 PRs),
  follow-up to the two-step ``mapoi_nav2_bridge.cpp`` split in v0.5.0.**
  ``MapoiPanel``'s and ``PoiEditorPanel``'s remaining Qt-heavy
  responsibilities move into new files —
  ``mapoi_panel_nav_status.cpp``, ``mapoi_panel_nav_control.cpp``,
  ``mapoi_panel_config.cpp``, ``mapoi_panel_backend_status.cpp``,
  ``poi_editor_save.cpp``, ``poi_editor_tags.cpp``, and
  ``poi_editor_display_settings.cpp`` — all compiled into the same
  ``mapoi_rviz_plugins`` library, with ``MapoiPanel``/``PoiEditorPanel``
  staying single classes. ``calcYaw``/``join``/``SplitSentence`` were also
  converted to Qt-independent free functions in ``poi_editor_helpers.hpp``
  (same pattern as the #158/#346 helpers) and gained gtest coverage. Net
  effect: ``mapoi_panel.cpp`` goes from 587 to 202 lines and
  ``poi_editor.cpp`` from 873 to 456 lines (the latter also grew
  concurrently from the #405/#407/#399 features above). Internal refactor,
  no behavior change.

* ``mapoi_rviz2_publisher``'s 1Hz marker timer now rebuilds POI/route
  markers only on a dirty tick (a POI/route/highlight update or a
  drawing-parameter change); a tick with no subscribers on either topic
  skips construction and publish entirely, and a subscribed-but-clean tick
  re-``publish``\ es the cached ``MarkerArray`` with just ``header.stamp``
  refreshed, since the volatile QoS still needs a steady stream for a
  late-joining subscriber. The dirty flag is never cleared while there are
  no subscribers, so a data change that happens while RViz is disconnected
  still forces one full rebuild on the first tick after a subscriber
  reappears. Removes the previously-unconditional per-second cos/sin-heavy
  sector/wedge marker construction (#402).

* Both ``MapoiPanel::ConfigPathCallback`` and
  ``PoiEditorPanel::ConfigPathCallback`` now dedup a same-map
  ``config_path`` republish by also comparing the YAML file's mtime, on top
  of the existing map-name comparison, mirroring the dedup already done in
  ``mapoi_rviz2_publisher``. A same-path event with an unchanged mtime is a
  no-op; a changed mtime still triggers the existing blocking service
  re-fetch and full table/combo-box rebuild, since that is how a same-path
  Save (#135) is detected (#403).

Fixed
-----

* **WebUI: an SSE-triggered full reload no longer silently discards a dirty
  tab's unsaved edits (#373).** ``config_changed`` — fired whenever another
  tab or an external client such as RViz saves — used to run ``loadMaps()``
  unconditionally, wiping any in-progress POI/route/tag edit, its undo
  history, and its dirty flag with no warning. The reload is now deferred
  behind a "reload (discard edits) / keep editing" confirmation whenever a
  dirty editor or an open edit form exists; choosing to keep editing leaves
  the stale ``config_version`` in place so the eventual Save still falls
  into the existing #343 409-conflict dialog instead of silently
  clobbering.

* **WebUI: a tab no longer reloads, and loses its own undo history and map
  viewport, after saving its own changes (#384).** The same
  ``config_changed`` broadcast that triggers the #373 guard above used to
  also fire on the saving tab itself, since ``mapoi_server`` republishes
  ``config_path`` after every save. The backend now includes the YAML's
  ``config_version`` (sha256) in the SSE payload, and a tab whose own
  editors already hold that version treats the event as self-originated and
  skips the reload — while still redistributing the new version to its
  editors, so a subsequent save in another section does not spuriously 409.

* **WebUI: Leaflet is now vendored instead of loaded from a CDN, restoring
  offline operation (#394, release blocker for #20).** ``index.html`` pulled
  ``leaflet.css``/``leaflet.js`` from ``unpkg.com`` at runtime with no local
  fallback, so the map — the WebUI's core function — did not render on an
  offline robot LAN, the primary target of the upcoming apt/rosdep
  distribution. Leaflet 1.9.4's dist files are now vendored under
  ``mapoi_webui/web/vendor/leaflet/`` (BSD-2-Clause, with its own
  ``LICENSE``/``README.md`` recording provenance); ``CMakeLists.txt`` needed
  no change since it already installs ``web/`` wholesale.

* WebUI: the UI-visibility toggle button's ``aria-pressed`` (and its blue
  "on" styling) is inverted to mean "UI panels are showing," matching the
  lock button's existing "pressed = default/active state" convention; the
  previous "pressed while hidden" reading was ARIA-correct but read
  backwards against the ☰ icon's "show a menu" connotation, per real-device
  feedback (#449).

* ``PoiEditorPanel::PoiPoseCallback`` (the ``PoiEditor`` side of RViz's Set
  Mapoi Pose tool) wrote into the pose column via the hardcoded literal
  ``2`` instead of ``kColPose``, a leftover from the #158 column reorder
  that moved every other call site to the ``kCol*`` constants. In practice
  the pose tool ended up overwriting the *tolerance* cell with a 3-value
  string, which then failed ``validate_tolerance_cell``'s 2-value check on
  Save (#427).

* ``PoiEditorPanel::TagFilterChanged`` discarded all unsaved table edits
  without confirmation whenever the tag-filter combo box changed —
  including re-selecting the same tag, since Qt's ``activated`` fires on
  that too — and cleared the ``QUndoStack``, making the loss unrecoverable
  even with Ctrl+Z. A ``decide_tag_filter_change()`` pure-function guard
  (same shape as the #399 config-reload guard above) now asks for
  confirmation when the table is dirty, and restores the combo box's prior
  selection if the user chooses to keep editing (#428).

* ``PoiEditorPanel::SaveButton`` no longer crashes RViz when the target YAML
  cannot be read — an unreadable placeholder file reached via a degraded
  ``onInitialize``/``InitConfigs`` path, or external deletion/corruption of
  the file. The previously bare ``YAML::LoadFile`` call is now wrapped in a
  try/catch that reports the failure through a ``QMessageBox`` instead of
  letting ``YAML::BadFile``/``ParserException`` propagate out of a Qt slot
  (#429).

* Set Mapoi Pose (the RViz pose tool feeding ``PoiEditor``) now checks that
  RViz's Fixed Frame is ``map`` (tolerating a leading ``/`` from older
  nodes) before reflecting a pose into the table, and warns and refuses
  instead of writing it when the frame does not match. Previously the
  tool's raw Fixed-Frame x/y/theta was recorded unchanged as if already in
  the ``map`` frame — plausible-looking numbers that, once saved, would
  route Nav2 to the wrong place on a robot whose Fixed Frame was not
  ``map`` (#430).

* ``PoiEditorPanel::is_table_color_`` is now initialized (``= false``)
  instead of left indeterminate, and ``TableChanged`` bounds-checks
  ``shadow_`` before writing to it. Both defects were only reachable with a
  dead panel (a service timeout during ``onInitialize``/``UpdatePoiTable``)
  whose bundled placeholder rows stay editable: an unlucky
  uninitialized-bool read could mark the panel dirty and push undo
  commands, and the unguarded ``shadow_[row][column]`` write against an
  unbuilt shadow model was an out-of-bounds vector write (#432).

* Removed ``PoiTable``'s permanently-shown ascending sort indicator
  (``setSortIndicatorShown``/``setSortIndicator``), present since the
  initial implementation despite the table never actually supporting
  sorting — row order is YAML order, optionally reordered by drag — so the
  arrow was lying about the table's actual order. Enabling real sorting was
  ruled out because Undo, the shadow model, and Save all assume logical row
  indices (#433).

* ``PoiEditor``'s New/Copy insert position is now computed from the
  selected row's *visual* index (via a new ``insert_move_target_visual()``
  helper) instead of its logical index, and the inserted row is moved to
  match. After a drag-reorder, New/Copy used to insert at an unrelated
  visual position (Qt inserts new sections at their logical index), which
  read as "the button didn't work" and encouraged repeated clicks that
  piled up duplicate rows — rows Save then persists wherever they visually
  landed (#434).

* ``PoiEditor``'s green "edited" cell highlight now clears once Undo
  returns a cell's text to its post-load baseline, instead of staying lit
  regardless of the actual value. A ``clean_texts_`` baseline snapshot,
  refreshed on every full table rebuild and kept in sync across row
  insert/delete, backs the new ``RefreshCellEditMark()`` /
  ``ClearAllEditMarks()`` helpers, so highlighting now reflects "differs
  from the saved baseline" rather than "has ever been touched" (#445).

* ``MapoiPanel``'s docked layout no longer stretches its status labels
  apart with excess vertical whitespace when there is spare height: a
  trailing expanding ``QSpacer`` now absorbs the slack at the bottom
  instead (#446).

* Seven blocking ``rclcpp::spin_until_future_complete`` service calls in
  ``mapoi_rviz_plugins`` (``get_maps_info``, two combo-box population
  calls, ``MapComboBox``, ``SaveButton``'s ``reload_map_info``,
  ``LoadTagDefinitions``, and ``UpdatePoiTable``) previously had no timeout
  of their own beyond the initial 3s ``wait_for_service``, so a service
  that accepted the request and then hung could freeze the whole panel
  indefinitely. All seven now pass a 5s timeout and log-and-return on
  expiry, matching the pattern already used elsewhere in the same files;
  ``LoadTagDefinitions`` also gained the ``SUCCESS`` result check it was
  previously missing (#404).

Internal / tests
----------------

* First release using per-package ``CHANGELOG.rst`` files: the project-wide
  history was split into ``mapoi/``, ``mapoi_interfaces/``,
  ``mapoi_server/``, ``mapoi_webui/``, ``mapoi_rviz_plugins/``, and
  ``mapoi_turtlebot3_example/`` — the ``catkin_pkg``-parseable, flat format
  bloom/buildfarm expect, with this root file remaining the project-wide
  narrative going forward (see the note at the top of this document). The
  ``mapoi`` metapackage also gained the ``LICENSE`` it had been missing
  since its v0.5.0 introduction, needed for bloom's per-package deb
  assembly, and ``mapoi_rviz_plugins/package.xml`` gained the
  ``<buildtool_depend>ament_cmake</buildtool_depend>`` declaration the other
  five packages already had — both ahead of the #20 ROS Index registration
  (#395).

* A CI step now verifies that every ``CHANGELOG.rst`` parses via
  ``catkin_pkg.changelog.get_changelog_from_path`` (catching a broken
  version-heading underline or a missing ``Forthcoming`` section) and that
  every ``package.xml``-bearing directory has a co-located
  ``CHANGELOG.rst`` and ``LICENSE``, so a future regression in the
  packaging groundwork laid by #395 is caught at PR time instead of at the
  next bloom-release. Because the script parses RST from PRs, including
  forks, it explicitly disables docutils' ``file``/``raw`` insertion
  directives to keep an untrusted CHANGELOG from being able to read
  arbitrary files (#454).
