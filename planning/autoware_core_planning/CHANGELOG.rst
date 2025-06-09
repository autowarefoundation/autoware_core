^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_core_planning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.2.0 (2025-06-09)
------------------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* feat(autoware_motion_velocity_planner): point-cloud clustering optimization (`#409 <https://github.com/autowarefoundation/autoware_core/issues/409>`_)
  * Core changes for point-cloud maksing and clustering
  * fix
  * style(pre-commit): autofix
  * Update planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/planner_data.hpp
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  * fix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: autoware_path_generator, missing parameters in config file (`#362 <https://github.com/autowarefoundation/autoware_core/issues/362>`_)
  fix::autoware_path_generator::missing parameters in config file
* Contributors: Arjun Jagdish Ram, Yutaka Kondo, 心刚

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* fix(autoware_core_planner): fix wrong package name dependency (`#339 <https://github.com/autowarefoundation/autoware_core/issues/339>`_)
  fix dependency to autoware_behavior_velocity_stop_line_module
* feat(autoware_core): add autoware_core package with launch files (`#304 <https://github.com/autowarefoundation/autoware_core/issues/304>`_)
* Contributors: Maxime CLEMENT, Ryohsuke Mitsudome
