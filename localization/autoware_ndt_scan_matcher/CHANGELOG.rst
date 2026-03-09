^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_ndt_scan_matcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(ndt_scan_matcher): fix segmentation fault in MultiVoxelGridCovariance::createKdtree() (`#733 <https://github.com/autowarefoundation/autoware_core/issues/733>`_)
  * fix(ndt_scan_matcher): handle nullptr
  * fix: avoid wasting memory
  ---------
  Co-authored-by: Anh Nguyen <vietanhng17@gmail.com>
* perf(localization, sensing): reduce subscription queue size from 100 to 10 (`#751 <https://github.com/autowarefoundation/autoware_core/issues/751>`_)
* chore: jazzy-porting: fix test depend launch-test missing (`#738 <https://github.com/autowarefoundation/autoware_core/issues/738>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: tf2_ros to hpp headers (`#616 <https://github.com/autowarefoundation/autoware_core/issues/616>`_)
* Contributors: Shumpei Wakabayashi, Tim Clephas, Yutaka Kondo, github-actions, 心刚

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: jazzy-porting:fix qos profile issue (`#634 <https://github.com/autowarefoundation/autoware_core/issues/634>`_)
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
* chore: jazzy-porting:fix missing header file (`#630 <https://github.com/autowarefoundation/autoware_core/issues/630>`_)
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Motz, Yutaka Kondo, mitsudome-r, 心刚

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_ndt_scan_matcher): extend timeout for test case (`#601 <https://github.com/autowarefoundation/autoware_core/issues/601>`_)
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(autoware_ndt_scan_matcher): port the package from Autoware Universe   (`#490 <https://github.com/autowarefoundation/autoware_core/issues/490>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Ryohsuke Mitsudome, github-actions

* fix: to be consistent version in all package.xml(s)
* feat(autoware_ndt_scan_matcher): port the package from Autoware Universe   (`#490 <https://github.com/autowarefoundation/autoware_core/issues/490>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Ryohsuke Mitsudome, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-22)
------------------

0.2.0 (2025-02-07)
------------------

0.0.0 (2024-12-02)
------------------
