^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_core_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(`localization`): remove duplicated config files, which wrongly remained due to my mistake (I'm sorry) (`#1237 <https://github.com/autowarefoundation/autoware_core/issues/1237>`_)
  remove(`localization`): duplicated config files
* fix(`localization`): remove duplicated config files (`#1054 <https://github.com/autowarefoundation/autoware_core/issues/1054>`_)
  * chore(`localization`): remove duplicated config files
  * chore(`localization`): remove duplicated config files
  * bug(`pose_initializer`): restore the used default values (see below)
  * We can trace that the change is derived from here:
  - https://github.com/autowarefoundation/autoware_core/pull/1054/changes#diff-74292a33fd9e2f1bb16981050ebc0f61cff9abd1f50dde3be6424c3f989f854bL4-L5
  * bug(`localization`): reuse pose_initializer launch (see below)
  Until this commit, some parameters such as `ekf_enabled`, `gnss_enabled`, ... etc are not passed.
  It seems we were using the hard-coded values in the previous `pose_initializer.param.yaml`.
  So applied fixes to:
  * Pass pose initializer flags via launch include
  * Add defaults and bool params in pose initializer launch
  * bug(`localization`): fix to pass `pose_initializer` variables (see below)
  * Followed a way that of `autowarefoundation/autoware_launch`
  - https://github.com/autowarefoundation/autoware_launch/blob/6b71b90f3a1fd07c08defe974c7caff917759108/tier4_universe_launch/tier4_localization_launch/launch/pose_twist_estimator/pose_twist_estimator.launch.xml#L26-L29
  * This commit reverts the hard-coded values in the following commit
  - https://github.com/autowarefoundation/autoware_core/pull/1054/changes/da904d6555e600ca93cc32653be3292843ddc527
  * style(pre-commit): autofix
  * bug: fix my stupid mistake when conflict resolve
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Junya Sasaki, github-actions

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_core_localization): sync ekf_localizer param (`#1093 <https://github.com/autowarefoundation/autoware_core/issues/1093>`_)
* feat(autoware_ndt_scan_matcher): publish all map points if publisher has subscribers (`#995 <https://github.com/autowarefoundation/autoware_core/issues/995>`_)
  * publish all map points if publisher has subscribers
  * style(pre-commit): autofix
  * fix error
  * add parameter
  * fix param name and add json
  * add loaded map clear on ndt ptr reset
  * reserve before adding points
  * add new param to autoware_core_lozalization
  * Update localization/autoware_ndt_scan_matcher/include/autoware/ndt_scan_matcher/map_update_module.hpp
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
* Contributors: Kazusa Hashimoto, Takagi, Isamu, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_core): add USE_SCOPED_HEADER_INSTALL_DIR to localization packages (`#984 <https://github.com/mitsudome-r/autoware_core/issues/984>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* fix: launch params (`#975 <https://github.com/mitsudome-r/autoware_core/issues/975>`_)
* fix(autoware_core_localization): add missing dependency (`#872 <https://github.com/mitsudome-r/autoware_core/issues/872>`_)
* Contributors: Mete Fatih Cırıt, Takagi, Isamu, Vishal Chauhan, github-actions

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
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat: update autoware component launch files (`#496 <https://github.com/autowarefoundation/autoware_core/issues/496>`_)
  * feat(autoware_core_localization): add pointcloud based localization packages to launch file
  * feat(autoware_core_map): add pointcloud map loader to launch file
  * feat(autoware_core_perception): add euclidean clustering and ground filter to launch
  * feat: update rviz config
  * style(pre-commit): autofix
  * fix: typo in package name
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* Contributors: Ryohsuke Mitsudome, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat(autoware_core): add autoware_core package with launch files (`#304 <https://github.com/autowarefoundation/autoware_core/issues/304>`_)
* Contributors: Ryohsuke Mitsudome
