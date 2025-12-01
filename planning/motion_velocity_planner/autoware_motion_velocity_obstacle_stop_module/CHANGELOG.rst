^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_velocity_obstacle_stop_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(obstacle_stop_module): add maintainer (`#674 <https://github.com/autowarefoundation/autoware_core/issues/674>`_)
* feat(obstacle_stop): hold behavior stop margin (`#673 <https://github.com/autowarefoundation/autoware_core/issues/673>`_)
* feat(obstacle_stop): add filter for outside obstacle (`#667 <https://github.com/autowarefoundation/autoware_core/issues/667>`_)
* feat(obstacle_stop_module, motion_velocity_planner_common): add safety_factor to obstacle_stop module (`#572 <https://github.com/autowarefoundation/autoware_core/issues/572>`_)
  add safety factor, add planning_factor test
* chore(mvp_planner_common): add docstrings for PlannerData:::object (`#615 <https://github.com/autowarefoundation/autoware_core/issues/615>`_)
* fix(obstacle_stp): fix several bugs (`#596 <https://github.com/autowarefoundation/autoware_core/issues/596>`_)
  * fix(obstacle_stp): fix several bugs
  * fix
  * fix
  * fix
  ---------
* feat(motion_velocity_planner_common):  lateral margin adjustment for the ego's curvature and target obstacle motion (`#619 <https://github.com/autowarefoundation/autoware_core/issues/619>`_)
  * add additonal off-track featureuki.takagi@tier4.jp>
  ---------
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* feat(obstacle_stop): enable object specified obstacle_filtering parameter and refactor obstacle type handling (`#613 <https://github.com/autowarefoundation/autoware_core/issues/613>`_)
  * refactor obstacle_filtering structure and type handling
  ---------
* fix(obstacle_stop): fix bug for the backwoard motions (`#617 <https://github.com/autowarefoundation/autoware_core/issues/617>`_)
  fix back stop
* fix(autoware_motion_velocity_obstacle_stop_module): rm wrong dependency (`#597 <https://github.com/autowarefoundation/autoware_core/issues/597>`_)
* feat(obstacle_stop): add velocity estimation feature for point cloud (`#590 <https://github.com/autowarefoundation/autoware_core/issues/590>`_)
  add velocity estimation for pcl
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mamoru Sobue, Maxime CLEMENT, Mete Fatih Cırıt, Takayuki Murooka, Yuki TAKAGI, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(motion_velocity_planner, motion_velocity_planner_common): update pointcloud preprocess design (`#591 <https://github.com/autowarefoundation/autoware_core/issues/591>`_)
  * updare pcl preprocess
  ---------
* fix(obstacle_stop): fix the brief stop decision logic (`#583 <https://github.com/autowarefoundation/autoware_core/issues/583>`_)
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* fix(obstacle_stop_module): fix outside stop feature (`#576 <https://github.com/autowarefoundation/autoware_core/issues/576>`_)
* fix(obstacle_stop): fix for failing scenario (`#566 <https://github.com/autowarefoundation/autoware_core/issues/566>`_)
  fix for failing scenario
* feat(obstacle_stop_module)!: add leading vehicle following by rss stop position determination (`#537 <https://github.com/autowarefoundation/autoware_core/issues/537>`_)
  * add new feature
  ---------
* feat(obstacle_stop_module): add cut in stop feature (`#517 <https://github.com/autowarefoundation/autoware_core/issues/517>`_)
  * restore the old function to pass universe CI
  * add new feature
  * add todo comment
  ---------
* refactor(motion_velocity_planner_common): splt get_predicted_pose() (`#558 <https://github.com/autowarefoundation/autoware_core/issues/558>`_)
  * rename
  * restore the old function to pass universe CI
* Contributors: Arjun Jagdish Ram, Ryohsuke Mitsudome, Takayuki Murooka, Yuki TAKAGI

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(obstacle_stop): update parameter explanation (`#501 <https://github.com/autowarefoundation/autoware_core/issues/501>`_)
  * feat(obstacle_stop): update parameter explanation
  * update
  * update
  * fix
  ---------
* feat(autoware_motion_velocity_planner)!: only wait for the required subscriptions (`#505 <https://github.com/autowarefoundation/autoware_core/issues/505>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_motion_velocity_obstacle_stop_module): point-cloud points ahead of the terminal stop-point (`#502 <https://github.com/autowarefoundation/autoware_core/issues/502>`_)
  * fix
  * moved resample_trajectory_points to non-anonymous status
  * style(pre-commit): autofix
  * fix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat!: remove obstacle_stop_planner and obstacle_cruise_planner (`#495 <https://github.com/autowarefoundation/autoware_core/issues/495>`_)
  * feat: remove obstacle_stop_planner and obstacle_cruise_planner
  * update
  * fix
  ---------
* feat(obstacle_stop_module): maintain larger stop distance for opposing traffic (`#451 <https://github.com/autowarefoundation/autoware_core/issues/451>`_)
  * Opposing traffic handling
  * Changes to core params
  * fix
  * fixes
  * style(pre-commit): autofix
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
* fix(autoware_motion_velocity_obstacle_stop_module): fix deprecated autoware_utils header (`#443 <https://github.com/autowarefoundation/autoware_core/issues/443>`_)
  * fix autoware_utils header
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
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
* Contributors: Arjun Jagdish Ram, Masaki Baba, Ryohsuke Mitsudome, Takayuki Murooka, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* fix(motion_velocity_obstacle_stop_module): fix debug topic name (`#341 <https://github.com/autowarefoundation/autoware_core/issues/341>`_)
  fix(motion_velocity_obstacle_xxx_module): fix debug topic name
* fix(autoware_motion_velocity_obstacle_stop_module): fix plugin export (`#333 <https://github.com/autowarefoundation/autoware_core/issues/333>`_)
* feat(autoware_motion_velocity_obstacle_stop_module): port to core repo (`#310 <https://github.com/autowarefoundation/autoware_core/issues/310>`_)
* Contributors: Ryohsuke Mitsudome, Takayuki Murooka, 心刚
