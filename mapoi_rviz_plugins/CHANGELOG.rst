^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mapoi_rviz_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This per-package file lists only the changes affecting ``mapoi_rviz_plugins``
(the ``MapoiPanel`` / ``PoiEditorPanel`` panels and the RViz PoseTool). The
``mapoi_rviz2_publisher`` node lives in ``mapoi_server``, so its rendering
changes are recorded there. Full project-level narrative: root
``CHANGELOG.rst`` / the GitHub Releases page.

Forthcoming
-----------

0.5.0 (2026-07-10)
------------------

* Render a yaw-agnostic POI tolerance (``tolerance.yaw >= pi``) as a filled disc in the RViz view instead of omitting the sector (#267, #268, #271).
* Update ``MapoiPanel`` to check ``response.success`` on ``GetRoutePois`` (now returning ``success`` / ``error_message``) instead of assuming a completed call succeeded (#342).
* Route the RViz panel's initial-pose publishing through the new ``request_initial_pose`` service, so ``mapoi_server`` is the sole writer of ``mapoi/initialpose_poi`` (#211).
* Split ``PoiEditorPanel::ValidatePois`` into its own translation unit and extract the pose/tolerance/tag-exclusivity checks as Qt/ROS-independent pure functions with gtest coverage; displayed messages byte-for-byte unchanged (#346).

0.4.0 (2026-05-08)
------------------

* Gate navigation operations in the panel on ``NavigationBackendStatus.backend_ready`` and detect bridge death via ``LivelinessChangedEvent`` (#198, #208).
* Show a separate Localization indicator and gate the Set Initial Pose UI on ``LocalizationBackendStatus`` (#209).

0.3.0 (2026-05-02)
------------------

* Replace the POI Editor table's ``Radius`` column with ``tolerance.xy`` + ``tolerance.yaw``; ``ValidatePois`` enforces the ``>= 0.001`` minimums (#138).
* Reorganize the POI Editor table from 6 columns to 5 (``name / pose / tolerance / tags / description``), display ``yaw`` in radians, and add a new ``yaw <= 2*pi`` max validation to catch old degree values (#158).
* Fix the POI Editor Panel Display Settings UI: replace the removed ``arrow_size_ratio`` control with a ``show_tolerance_sector`` checkbox, fixing a ``get_parameters`` / ``set_parameters`` regression (#179).

0.2.0 (2026-04-29)
------------------

* Add a "Display Settings" group to the POI Editor panel (#99, #100).
* The POI Editor panel dynamically refreshes its displayed POI list on ``mapoi_config_path`` (#75, #78).
* Remove the ``nav2_rviz_plugins`` ``Selector`` and ``Docking`` panels from the default RViz config (not present on Humble) (#37, #51).
