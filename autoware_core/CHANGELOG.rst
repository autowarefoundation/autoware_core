^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* feat: add autoware_core_api package (`#551 <https://github.com/autowarefoundation/autoware_core/issues/551>`_)
  * feat: add autoware_core_api package
  * remove unnecessary change
  * update README
  ---------
* chore: sync files (`#354 <https://github.com/autowarefoundation/autoware_core/issues/354>`_)
  Co-authored-by: github-actions <github-actions@github.com>
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* Contributors: Ryohsuke Mitsudome, Yutaka Kondo, awf-autoware-bot[bot], github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat(autoware_core): add autoware_core package with launch files (`#304 <https://github.com/autowarefoundation/autoware_core/issues/304>`_)
* Contributors: Ryohsuke Mitsudome
