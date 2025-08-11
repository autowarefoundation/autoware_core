^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_velocity_planner_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* refactor(mvp_common): add docstrings and comments (`#600 <https://github.com/autowarefoundation/autoware_core/issues/600>`_)
* feat(motion_velocity_planner, motion_velocity_planner_common): update pointcloud preprocess design (`#591 <https://github.com/autowarefoundation/autoware_core/issues/591>`_)
  * updare pcl preprocess
  ---------
* refactor(motion_velocity_planner): restrict copy of planner_data  (`#587 <https://github.com/autowarefoundation/autoware_core/issues/587>`_)
  * restrict plannar data copy
  ---------
* fix(obstacle_stop): fix the brief stop decision logic (`#583 <https://github.com/autowarefoundation/autoware_core/issues/583>`_)
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* chore: apply pre-commit fix (`#574 <https://github.com/autowarefoundation/autoware_core/issues/574>`_)
* feat(motion_utils): update insert orientation function to use tangent direction of a spline cruve (`#556 <https://github.com/autowarefoundation/autoware_core/issues/556>`_)
  * update insertOrientationSpline()
  * update test
  * use insertOrientationSpline() in motion_velcity_planner_common
  ---------
* feat(obstacle_stop_module): add cut in stop feature (`#517 <https://github.com/autowarefoundation/autoware_core/issues/517>`_)
  * restore the old function to pass universe CI
  * add new feature
  * add todo comment
  ---------
* refactor(motion_velocity_planner_common): splt get_predicted_pose() (`#558 <https://github.com/autowarefoundation/autoware_core/issues/558>`_)
  * rename
  * restore the old function to pass universe CI
* Contributors: Mamoru Sobue, Ryohsuke Mitsudome, Takayuki Murooka, Yuki TAKAGI

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(autoware_motion_velocity_planner)!: only wait for the required subscriptions (`#505 <https://github.com/autowarefoundation/autoware_core/issues/505>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_motion_velocity_obstacle_stop_module): point-cloud points ahead of the terminal stop-point (`#502 <https://github.com/autowarefoundation/autoware_core/issues/502>`_)
  * fix
  * moved resample_trajectory_points to non-anonymous status
  * style(pre-commit): autofix
  * fix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_motion_velocity_obstacle_stop_module): fix for mishandling lateral-distance (`#452 <https://github.com/autowarefoundation/autoware_core/issues/452>`_)
  * Fix for mishandling lateral-distance
  * fix
  * style(pre-commit): autofix
  * fix
  * fix
  * fix
  * style(pre-commit): autofix
  * fix
  * style(pre-commit): autofix
  * Update planning/motion_velocity_planner/autoware_motion_velocity_planner_common/src/utils.cpp
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix(autoware_motion_velocity_planner_common): fix deprecated autoware_utils header (`#445 <https://github.com/autowarefoundation/autoware_core/issues/445>`_)
  * fix autoware_utils header
  * style(pre-commit): autofix
  * add missing header
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
* Contributors: Arjun Jagdish Ram, Masaki Baba, Ryohsuke Mitsudome, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat: autoware_motion_velocity_planner and autoware_motion_velocity_planner_common to core (`#242 <https://github.com/autowarefoundation/autoware_core/issues/242>`_)
  * feat: modify autoware_universe_utils to autoware_utils
  * feat: remove useless dependecy
  * style(pre-commit): autofix
  * feat: modify autoware_universe_utils to autoware_utils
  * feat: autoware_motion_velocity_planner_node to core
  * style(pre-commit): autofix
  * feat: autoware_motion_velocity_planner_node to core
  * feat: modify autoware_universe_utils to autoware_utils
  * feat: remove useless dependecy
  * style(pre-commit): autofix
  * feat: modify autoware_universe_utils to autoware_utils
  * style(pre-commit): autofix
  * feat: port autoware_behavior_velocity_planner to core
  * fix: apply latest changes from autoware.universe
  * fix: deadlinks in README
  * fix(autoware_motion_velocity_planner): porting autoware_motion_velocity_planner, autoware_motion_velocity_planner, sync with latest universe: v0.2
  * style(pre-commit): autofix
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/node.hpp
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/include/autoware/behavior_velocity_planner_common/utilization/util.hpp
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/src/utilization/util.cpp
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/test/src/test_util.cpp
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/debug.cpp
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/manager.cpp
  * style(pre-commit): autofix
  * Update planning/motion_velocity_planner/autoware_motion_velocity_planner/CMakeLists.txt
  * style(pre-commit): autofix
  * feat(autoware_motion_velocity_planner_node): porting autoware_motion_velocity_planner_node, autoware_motion_velocity_planner_node, remove metrics msgs publish according to pr-10342 under universe repo: v0.5
  * feat(autoware_motion_velocity_planner_common): porting autoware_motion_velocity_planner_common, autoware_motion_velocity_planner_common, port to core repo: v0.0
  * style(pre-commit): autofix
  * move
  * rename
  * fix exec
  * add maintainer
  * style(pre-commit): autofix
  * fix test_depend
  * feat(autoware_motion_velocity_planner_node): porting autoware_motion_velocity_planner_node, autoware_motion_velocity_planner_node, resolve build issue: v0.6
  ---------
  Co-authored-by: suchang <chang.su@autocore.ai>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <ryohsuke.mitsudome@tier4.jp>
  Co-authored-by: liuXinGangChina <lxg19892021@gmail.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: 心刚 <90366790+liuXinGangChina@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, storrrrrrrrm
