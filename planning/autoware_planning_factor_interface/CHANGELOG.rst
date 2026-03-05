^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_factor_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(obstacle_stop_module, motion_velocity_planner_common): add safety_factor to obstacle_stop module (`#572 <https://github.com/autowarefoundation/autoware_core/issues/572>`_)
  add safety factor, add planning_factor test
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Yuki TAKAGI, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(planning_factor): add console output option (`#513 <https://github.com/autowarefoundation/autoware_core/issues/513>`_)
  fix param json
  fix param json
  snake_case
  set default
* feat!: remove obstacle_stop_planner and obstacle_cruise_planner (`#495 <https://github.com/autowarefoundation/autoware_core/issues/495>`_)
  * feat: remove obstacle_stop_planner and obstacle_cruise_planner
  * update
  * fix
  ---------
* fix(autoware_planning_factor_interface): removed unused autoware_utils (`#440 <https://github.com/autowarefoundation/autoware_core/issues/440>`_)
  removed unused autoware_utils
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* Contributors: Kosuke Takeuchi, Masaki Baba, Takayuki Murooka, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* fix(planning_factor_interface): set control point data independently (`#291 <https://github.com/autowarefoundation/autoware_core/issues/291>`_)
  * fix(planning_factor_interface): set shift length properly
  * chore: add comment
  ---------
* Contributors: Satoshi OTA

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* chore: rename from `autoware.core` to `autoware_core` (`#290 <https://github.com/autowarefoundation/autoware.core/issues/290>`_)
* feat(autoware_planning_factor_interface): move to core from universe (`#241 <https://github.com/autowarefoundation/autoware.core/issues/241>`_)
* Contributors: Yutaka Kondo, mitsudome-r, 心刚
