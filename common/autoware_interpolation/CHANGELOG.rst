^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_interpolation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* refactor(interpolation): use `autoware_utils\_*` instead of `autoware_utils` (`#382 <https://github.com/autowarefoundation/autoware_core/issues/382>`_)
  feat(interpolation): use split autoware utils
* fix(autoware_path_optimizer): incorrect application of input velocity due to badly mapping output trajectory to input trajectory (`#355 <https://github.com/autowarefoundation/autoware_core/issues/355>`_)
  * changes to avoid improper mapping
  * Update common/autoware_motion_utils/include/autoware/motion_utils/trajectory/trajectory.hpp
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Arjun Jagdish Ram, Takagi, Isamu

1.7.0 (2026-02-14)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore(autoware_motion_utils, autoware_trajectory, autoware_interpolation): add maintainers for packages (`#821 <https://github.com/autowarefoundation/autoware_core/issues/821>`_)
  * add maintainers for autoware_interpolation
  * add maintainers for autoware_motion_utils
  * add maintainers for autoware_trajectory
  ---------
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* chore(autoware_interpolation): add a maintainer (`#818 <https://github.com/autowarefoundation/autoware_core/issues/818>`_)
* feat(autoware_interpolation): exposing access to coefficients (`#671 <https://github.com/autowarefoundation/autoware_core/issues/671>`_)
  * Changes to spline_interpolation code to expose access to spline coefficients
  * fixes
  * style(pre-commit): autofix
  * removed unnecessary comments
  * Added includes
  * add includes
  * added comment
  * style(pre-commit): autofix
  * unit-testing for new public functions
  * style(pre-commit): autofix
  * extra unit tests
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taiki Yamada <129915538+TaikiYamada4@users.noreply.github.com>
* perf(interpolation): optimize calc_closest_segment_indices (`#797 <https://github.com/autowarefoundation/autoware_core/issues/797>`_)
* Contributors: Arjun Jagdish Ram, Junya Sasaki, Maxime CLEMENT, Ryohsuke Mitsudome, mkquda

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
* fix(interpolation): ensure consistent output size in splineYawFromPoints (`#670 <https://github.com/autowarefoundation/autoware_core/issues/670>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Maxime CLEMENT, Mete Fatih Cırıt, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* fix(motion_utils): update motion utils trajectory (`#569 <https://github.com/autowarefoundation/autoware_core/issues/569>`_)
* Contributors: Ryohsuke Mitsudome, Yuki TAKAGI

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* chore: no longer support ROS 2 Galactic (`#492 <https://github.com/autowarefoundation/autoware_core/issues/492>`_)
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* refactor(interpolation): use `autoware_utils\_*` instead of `autoware_utils` (`#382 <https://github.com/autowarefoundation/autoware_core/issues/382>`_)
  feat(interpolation): use split autoware utils
* fix(autoware_path_optimizer): incorrect application of input velocity due to badly mapping output trajectory to input trajectory (`#355 <https://github.com/autowarefoundation/autoware_core/issues/355>`_)
  * changes to avoid improper mapping
  * Update common/autoware_motion_utils/include/autoware/motion_utils/trajectory/trajectory.hpp
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Arjun Jagdish Ram, Takagi, Isamu, Tim Clephas, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* fix(autoware_interpolation): add missing dependencies (`#235 <https://github.com/autowarefoundation/autoware.core/issues/235>`_)
* feat: port autoware_interpolation from autoware.universe (`#149 <https://github.com/autowarefoundation/autoware.core/issues/149>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* Contributors: mitsudome-r, ralwing, shulanbushangshu
