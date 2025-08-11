^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_core_planning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: change planning output topic name to /planning/trajectory (`#602 <https://github.com/autowarefoundation/autoware_core/issues/602>`_)
  * change planning output topic name to /planning/trajectory
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(motion_velocity_planner, motion_velocity_planner_common): update pointcloud preprocess design (`#591 <https://github.com/autowarefoundation/autoware_core/issues/591>`_)
  * updare pcl preprocess
  ---------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* fix(obstacle_stop): fix for failing scenario (`#566 <https://github.com/autowarefoundation/autoware_core/issues/566>`_)
  fix for failing scenario
* feat(obstacle_stop_module)!: add leading vehicle following by rss stop position determination (`#537 <https://github.com/autowarefoundation/autoware_core/issues/537>`_)
  * add new feature
  ---------
* refactor: implement varying lateral acceleration and steering rate threshold in velocity smoother (`#531 <https://github.com/autowarefoundation/autoware_core/issues/531>`_)
  * refactor: implement varying steering rate threshold in velocity smoother
  * feat: implement varying lateral acceleration limit
  * fix  typo in readme
  * fix bugs in unit conversion
  * clean up the obsolete parameter in core planning launch
  ---------
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
* feat(obstacle_stop_module): add cut in stop feature (`#517 <https://github.com/autowarefoundation/autoware_core/issues/517>`_)
  * restore the old function to pass universe CI
  * add new feature
  * add todo comment
  ---------
* Contributors: Arjun Jagdish Ram, Ryohsuke Mitsudome, Yuki TAKAGI, Yukihiro Saito, Yuxuan Liu

1.3.0 (2025-06-23)
------------------
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
