^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_euclidean_cluster_object_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.7.0 (2026-02-14)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore: use local default config files instead of autoware_launch (for core) (`#832 <https://github.com/autowarefoundation/autoware_core/issues/832>`_)
  use local default config files instead of autoware_launch
* Contributors: Ryohsuke Mitsudome, Taeseung Sohn

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_euclidean_cluster_object_detector): prevent zero division in getCentroid (`#745 <https://github.com/autowarefoundation/autoware_core/issues/745>`_)
  - Add zero division check in getCentroid() function
  - Return origin point (0, 0, 0) when empty point cloud is passed
  - Add unit test for empty point cloud case
* fix(autoware_euclidean_cluster_object_detector): add early return for empty point cloud in voxel grid node (`#747 <https://github.com/autowarefoundation/autoware_core/issues/747>`_)
* fix(autoware_euclidean_cluster_object_detector): add empty point cloud check (`#743 <https://github.com/autowarefoundation/autoware_core/issues/743>`_)
* Contributors: Yutaka Kondo, github-actions

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
* test(autoware_euclidean_cluster_object_detector): add test cases (`#488 <https://github.com/autowarefoundation/autoware_core/issues/488>`_)
  * test(autoware_euclidean_cluster_object_detector): add test cases
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat: support ROS 2 Jazzy (`#487 <https://github.com/autowarefoundation/autoware_core/issues/487>`_)
  * fix ekf_localizer
  * fix lanelet2_map_loader_node
  * MUST REVERT
  * fix pybind
  * fix depend
  * add buildtool
  * remove
  * revert
  * find_package
  * wip
  * remove embed
  * find python_cmake_module
  * public
  * remove ament_cmake_python
  * fix autoware_trajectory
  * add .lcovrc
  * fix egm
  * use char*
  * use global
  * namespace
  * string view
  * clock
  * version
  * wait
  * fix egm2008-1
  * typo
  * fixing
  * fix egm2008-1
  * MUST REVERT
  * fix egm2008-1
  * fix twist_with_covariance
  * Revert "MUST REVERT"
  This reverts commit 93b7a57f99dccf571a01120132348460dbfa336e.
  * namespace
  * fix qos
  * revert some
  * comment
  * Revert "MUST REVERT"
  This reverts commit 7a680a796a875ba1dabc7e714eaea663d1e5c676.
  * fix dungling pointer
  * fix memory alignment
  * ignored
  * spellcheck
  ---------
* feat(autoware_euclidean_cluster_object_detector): port autoware_euclidean_cluster to autoware.core (`#460 <https://github.com/autowarefoundation/autoware_core/issues/460>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: NorahXiong, Yutaka Kondo, github-actions, 心刚

* fix: to be consistent version in all package.xml(s)
* test(autoware_euclidean_cluster_object_detector): add test cases (`#488 <https://github.com/autowarefoundation/autoware_core/issues/488>`_)
  * test(autoware_euclidean_cluster_object_detector): add test cases
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat: support ROS 2 Jazzy (`#487 <https://github.com/autowarefoundation/autoware_core/issues/487>`_)
  * fix ekf_localizer
  * fix lanelet2_map_loader_node
  * MUST REVERT
  * fix pybind
  * fix depend
  * add buildtool
  * remove
  * revert
  * find_package
  * wip
  * remove embed
  * find python_cmake_module
  * public
  * remove ament_cmake_python
  * fix autoware_trajectory
  * add .lcovrc
  * fix egm
  * use char*
  * use global
  * namespace
  * string view
  * clock
  * version
  * wait
  * fix egm2008-1
  * typo
  * fixing
  * fix egm2008-1
  * MUST REVERT
  * fix egm2008-1
  * fix twist_with_covariance
  * Revert "MUST REVERT"
  This reverts commit 93b7a57f99dccf571a01120132348460dbfa336e.
  * namespace
  * fix qos
  * revert some
  * comment
  * Revert "MUST REVERT"
  This reverts commit 7a680a796a875ba1dabc7e714eaea663d1e5c676.
  * fix dungling pointer
  * fix memory alignment
  * ignored
  * spellcheck
  ---------
* feat(autoware_euclidean_cluster_object_detector): port autoware_euclidean_cluster to autoware.core (`#460 <https://github.com/autowarefoundation/autoware_core/issues/460>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: NorahXiong, Yutaka Kondo, github-actions, 心刚

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-22)
------------------

0.2.0 (2025-02-07)
------------------

0.0.0 (2024-12-02)
------------------
