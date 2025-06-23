^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_core_planning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

Forthcoming
-----------
* fix: to be consistent version in all package.xml(s)
* feat: use component_interface_specs for mission_planner (`#546 <https://github.com/autowarefoundation/autoware_core/issues/546>`_)
* chore(behavior_velocity_planner): remove unecessary stop_line_extend_length (`#515 <https://github.com/autowarefoundation/autoware_core/issues/515>`_)
  * chore(behavior_velocity_planner): remove unecessary stop_line_extend_length from param files
  * Update behavior_velocity_planner.param.yaml
  ---------
* feat(planning_factor): add console output option (`#513 <https://github.com/autowarefoundation/autoware_core/issues/513>`_)
  fix param json
  fix param json
  snake_case
  set default
* feat(obstacle_stop_module): maintain larger stop distance for opposing traffic (`#451 <https://github.com/autowarefoundation/autoware_core/issues/451>`_)
  * Opposing traffic handling
  * Changes to core params
  * fix
  * fixes
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
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
* Contributors: Arjun Jagdish Ram, Kosuke Takeuchi, Ryohsuke Mitsudome, Yutaka Kondo, github-actions, 心刚

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* fix(autoware_core_planner): fix wrong package name dependency (`#339 <https://github.com/autowarefoundation/autoware_core/issues/339>`_)
  fix dependency to autoware_behavior_velocity_stop_line_module
* feat(autoware_core): add autoware_core package with launch files (`#304 <https://github.com/autowarefoundation/autoware_core/issues/304>`_)
* Contributors: Maxime CLEMENT, Ryohsuke Mitsudome
