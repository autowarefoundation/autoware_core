^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* fix(autoware_path_optimizer): incorrect application of input velocity due to badly mapping output trajectory to input trajectory (`#355 <https://github.com/autowarefoundation/autoware_core/issues/355>`_)
  * changes to avoid improper mapping
  * Update common/autoware_motion_utils/include/autoware/motion_utils/trajectory/trajectory.hpp
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* refactor(autoware_motion_utils): rewrite using modern C++ without API breakage (`#348 <https://github.com/autowarefoundation/autoware_core/issues/348>`_)
* Contributors: Arjun Jagdish Ram, Yutaka Kondo

1.7.0 (2026-02-14)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(autoware_motion_utils): stop point detection while resampling trajectory (`#808 <https://github.com/autowarefoundation/autoware_core/issues/808>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
* chore(autoware_motion_utils, autoware_trajectory, autoware_interpolation): add maintainers for packages (`#821 <https://github.com/autowarefoundation/autoware_core/issues/821>`_)
  * add maintainers for autoware_interpolation
  * add maintainers for autoware_motion_utils
  * add maintainers for autoware_trajectory
  ---------
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* chore(autoware_motion_utils): add a maintainer (`#820 <https://github.com/autowarefoundation/autoware_core/issues/820>`_)
* chore(autoware_motion_utils): remove boost (`#646 <https://github.com/autowarefoundation/autoware_core/issues/646>`_)
  remove boost from cmake and package.xml
* Contributors: Felix F Xu, Junya Sasaki, Mehmet Emin BAŞOĞLU, Ryohsuke Mitsudome, mkquda

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* ci(pre-commit): autoupdate (`#723 <https://github.com/autowarefoundation/autoware_core/issues/723>`_)
  * pre-commit formatting changes
* Contributors: Mete Fatih Cırıt, github-actions

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: change planning output topic name to /planning/trajectory (`#602 <https://github.com/autowarefoundation/autoware_core/issues/602>`_)
  * change planning output topic name to /planning/trajectory
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* fix(motion_utils): update motion utils trajectory (`#569 <https://github.com/autowarefoundation/autoware_core/issues/569>`_)
* feat(motion_utils): update insert orientation function to use tangent direction of a spline cruve (`#556 <https://github.com/autowarefoundation/autoware_core/issues/556>`_)
  * update insertOrientationSpline()
  * update test
  * use insertOrientationSpline() in motion_velcity_planner_common
  ---------
* Contributors: Ryohsuke Mitsudome, Yuki TAKAGI, Yukihiro Saito

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(obstacle_stop_module): maintain larger stop distance for opposing traffic (`#451 <https://github.com/autowarefoundation/autoware_core/issues/451>`_)
  * Opposing traffic handling
  * Changes to core params
  * fix
  * fixes
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* refactor(motion_utils): use `autoware_utils\_*` instead of `autoware_utils` (`#383 <https://github.com/autowarefoundation/autoware_core/issues/383>`_)
  * feat(interpolation): use split autoware utils
  * feat(motion_utils): use split autoware utils
  ---------
* fix(autoware_path_optimizer): incorrect application of input velocity due to badly mapping output trajectory to input trajectory (`#355 <https://github.com/autowarefoundation/autoware_core/issues/355>`_)
  * changes to avoid improper mapping
  * Update common/autoware_motion_utils/include/autoware/motion_utils/trajectory/trajectory.hpp
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* refactor(autoware_motion_utils): rewrite using modern C++ without API breakage (`#348 <https://github.com/autowarefoundation/autoware_core/issues/348>`_)
* Contributors: Arjun Jagdish Ram, Takagi, Isamu, Tim Clephas, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* test(autoware_motion_utils): add tests for missed lines (`#275 <https://github.com/autowarefoundation/autoware.core/issues/275>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* feat: porting `autoware_motion_utils` from universe to core (`#184 <https://github.com/autowarefoundation/autoware.core/issues/184>`_)
  * add(autoware_motion_utils): ported as follows (see below):
  * From `autoware.universe/common` to `autoware.core/common`
  * The history can be traced via:
  https://github.com/autowarefoundation/autoware.universe/tree/3274695847dfc76153bdc847e28b66821e16df60/common/autoware_motion_utils
  * fix(package.xml): set the version to `0.0.0` as the initial port
  ---------
* Contributors: Junya Sasaki, NorahXiong, mitsudome-r
