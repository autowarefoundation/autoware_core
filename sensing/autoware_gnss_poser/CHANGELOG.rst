^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_gnss_poser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore: tf2_ros to hpp headers (`#616 <https://github.com/autowarefoundation/autoware_core/issues/616>`_)
* Contributors: Tim Clephas, github-actions

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: update maintainer (`#637 <https://github.com/autowarefoundation/autoware_core/issues/637>`_)
  * chore: update maintainer
  remove Takeshi Ishita
  * chore: update maintainer
  remove Kento Yabuuchi
  * chore: update maintainer
  remove Shintaro Sakoda
  * chore: update maintainer
  remove Ryu Yamamoto
  ---------
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Motz, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* chore: no longer support ROS 2 Galactic (`#492 <https://github.com/autowarefoundation/autoware_core/issues/492>`_)
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* test(autoware_gnss_poser): add unit tests (`#414 <https://github.com/autowarefoundation/autoware_core/issues/414>`_)
  * test(autoware_gnss_poser): add unit tests
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: NorahXiong, Tim Clephas, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* fix(autoware_gnss_poser): depend on geographiclib through its Find module (`#313 <https://github.com/autowarefoundation/autoware_core/issues/313>`_)
  fix: depend on geographiclib through it's provided Find module
* Contributors: Shane Loretz

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* feat(autoware_gnss_poser): porting from universe to core (`#166 <https://github.com/autowarefoundation/autoware.core/issues/166>`_)
* Contributors: mitsudome-r, 心刚
