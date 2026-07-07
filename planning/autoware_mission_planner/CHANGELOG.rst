^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_mission_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_mission_planner): extract pure check_reroute_safety free function (`#1113 <https://github.com/autowarefoundation/autoware_core/issues/1113>`_)
  * refactor(autoware_mission_planner): extract pure check_reroute_safety free function
  Extract the reroute-safety algorithm out of the MissionPlanner node method into a
  pure, dependency-injected free function declared in reroute_safety.hpp (route + lanelet
  map + scalars in, bool out). The node method now forwards to it after validating its own
  odometry / map members, so there is no public-API change (the method stays private).
  The two byte-for-byte identical start-segment distance branches (start_idx_target != 0 &&
  start_idx_original > 1 vs else) are collapsed into a single arc_length_to_lanelet_end
  helper that is parameterized only by which original-route segment supplies the primitives.
  Add table-driven unit tests over synthetic routes / lanelets covering every early-return
  branch (empty routes, null map, stopped-vehicle short-circuit, no common segment, ego not
  on first target section) and the final velocity-scaled safety-length comparison (safe /
  unsafe / velocity dependence). This is a behavior-preserving, ABI-neutral testability win
  on the package's most complex previously untested algorithm.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * test(autoware_mission_planner): pin start-segment -1 branch and empty-segment break (`#69 <https://github.com/autowarefoundation/autoware_core/issues/69>`_)
  Add a fixture case where the target route starts mid-original-route so start_idx_target != 0 && start_idx_original > 1, exercising the previously-unhit start_idx_original - 1 selector branch of check_reroute_safety; verified RED against a regression that drops the -1. Also pin the empty-primitives accumulation break.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  ---------
* feat(autoware_vehicle_info_utils): add base_pose to createFootprint (`#1072 <https://github.com/autowarefoundation/autoware_core/issues/1072>`_)
* Contributors: Sarun MUKDAPITAK, Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(mission_planner): unused using-declaration with GCC 15 (`#1000 <https://github.com/mitsudome-r/autoware_core/issues/1000>`_)
* chore(planning, bvp): remove unused lanelet2_extension header (`#902 <https://github.com/mitsudome-r/autoware_core/issues/902>`_)
  * remove unused lanelet2_extension in bvp modules
  * remove unused lanelet2_extension in planning components
  ---------
* feat(autoware_mission_planner): remove glog component (`#879 <https://github.com/mitsudome-r/autoware_core/issues/879>`_)
  feat: remove glog component
* fix(lanelet2_utils): change is_in_lanelet argument order (`#890 <https://github.com/mitsudome-r/autoware_core/issues/890>`_)
* feat(lanelet2_extension): port lanelet2_extension utilities functions (final)  (`#838 <https://github.com/mitsudome-r/autoware_core/issues/838>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* chore: organize maintainer (`#860 <https://github.com/mitsudome-r/autoware_core/issues/860>`_)
  * chore: organize maintainer
  * chore: organize maintainer
  ---------
* Contributors: Guilhem Saurel, Sarun MUKDAPITAK, Satoshi OTA, Tetsuhiro Kawaguchi, github-actions

1.7.0 (2026-02-14)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* refactor(planning): deprecate lanelet_extension geometry conversion function (`#834 <https://github.com/autowarefoundation/autoware_core/issues/834>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* refactor(planning, common): replace lanelet2_extension function (`#796 <https://github.com/autowarefoundation/autoware_core/issues/796>`_)
* Contributors: Mamoru Sobue, Ryohsuke Mitsudome

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_lanelet2_utils): replace from/toBinMsg (`#737 <https://github.com/autowarefoundation/autoware_core/issues/737>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* feat(autoware_lanelet2_utils): define remove_const in header (`#741 <https://github.com/autowarefoundation/autoware_core/issues/741>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* refactor(vehicle_info_utils): reduce autoware_utils deps (`#754 <https://github.com/autowarefoundation/autoware_core/issues/754>`_)
* chore: tf2_ros to hpp headers (`#616 <https://github.com/autowarefoundation/autoware_core/issues/616>`_)
* Contributors: Mete Fatih Cırıt, Sarun MUKDAPITAK, Tim Clephas, github-actions

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_lanelet2_utils): replace ported functions from autoware_lanelet2_extension (`#695 <https://github.com/autowarefoundation/autoware_core/issues/695>`_)
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: update maintainer (`#701 <https://github.com/autowarefoundation/autoware_core/issues/701>`_)
* feat(autoware_lanelet2_utils): porting functions from lanelet2_extension to autoware_lanelet2_utils package (`#621 <https://github.com/autowarefoundation/autoware_core/issues/621>`_)
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Sarun MUKDAPITAK, Takagi, Isamu, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(autoware_mission_planner, velocity_smoother): use transient_local for operation_mode_state (`#598 <https://github.com/autowarefoundation/autoware_core/issues/598>`_)
  subscribe transient_local with transient_local
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Kem (TiankuiXian), Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat: use component_interface_specs for mission_planner (`#546 <https://github.com/autowarefoundation/autoware_core/issues/546>`_)
* fix(mission_planner): fix check if goal footprint is inside route (`#534 <https://github.com/autowarefoundation/autoware_core/issues/534>`_)
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* fix(autoware_mission_planner): fix deprecated autoware_utils header (`#421 <https://github.com/autowarefoundation/autoware_core/issues/421>`_)
  * fix autoware_utils header
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Kosuke Takeuchi, Masaki Baba, Ryohsuke Mitsudome, Tim Clephas, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat: port simplified version of autoware_mission_planner from Autoware Universe  (`#329 <https://github.com/autowarefoundation/autoware_core/issues/329>`_)
  * feat: port autoware_mission_planner from Autoware Universe
  * chore: reset package version and remove CHANGELOG
  * chore: remove the _universe suffix from autoware_mission_planner
  * feat: repalce tier4_planning_msgs with autoware_internal_planning_msgs
  * feat: remove route_selector module
  * feat: remove reroute_availability and modified_goal subscription
  * remove unnecessary image
  * style(pre-commit): autofix
  * fix: remove unnecessary include file
  * fix: resolve useInitializationList error from cppcheck
  * Apply suggestions from code review
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Ryohsuke Mitsudome
