^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_mission_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

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
