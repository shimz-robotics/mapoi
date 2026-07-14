^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mapoi
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This per-package file lists only the changes affecting the ``mapoi``
metapackage, in the flat format expected by bloom / buildfarm. Full
project-level narrative: root ``CHANGELOG.rst`` / the GitHub Releases page.

Forthcoming
-----------

0.5.0 (2026-07-10)
------------------

* Introduce the ``mapoi`` metapackage (``exec_depend`` on ``mapoi_server`` / ``mapoi_interfaces`` / ``mapoi_rviz_plugins`` / ``mapoi_webui``), so the core suite can be installed or depended on as one unit; ``mapoi_turtlebot3_example`` is intentionally excluded so a robot/production install does not pull in the TurtleBot3 + Gazebo stack (#348).
