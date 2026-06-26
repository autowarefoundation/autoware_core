^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_euclidean_cluster_object_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* perf(autoware_euclidean_cluster_object_detector): eliminate hot-loop allocations in clustering (`#1124 <https://github.com/autowarefoundation/autoware_core/issues/1124>`_)
  * perf(autoware_euclidean_cluster_object_detector): eliminate hot-loop allocations in clustering
  Build the output clusters in place with emplace_back instead of heap-allocating a
  pcl::PointCloud via new and deep-copying it into the output vector per cluster, and
  reserve the per-cluster point buffers up front. Cache the single unordered_map hash
  lookup per input point in the voxel path (previously map[index] was re-probed several
  times), reserve the map, and reserve the 2D-flatten loops in both implementations.
  Internal-only and behavior-preserving; the public cluster() API is unchanged.
  Characterization tests pinning cluster membership, per-point coordinates, the empty
  -input path, and the objects/clusters lockstep relationship are added first to prove
  equivalence.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * fix(autoware_euclidean_cluster_object_detector): address review feedback
  - Replace pcl::PointCloud::emplace_back with push_back for PCL portability
  - Correct centroid comment to match the asserted x/y bounds
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * test(autoware_euclidean_cluster_object_detector): pin empty-cloud input on implemented voxel cluster overload (`#71 <https://github.com/autowarefoundation/autoware_core/issues/71>`_)
  Add a characterization test calling the implemented 3-arg VoxelGridBasedEuclideanCluster::cluster(msg, objects, clusters) overload with an empty point cloud, pinning that it returns true with empty objects and clusters (no crash, no guard needed).
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * test(autoware_euclidean_cluster_object_detector): pin clustering output via set comparison
  Rewrite the clustering test so the named input point sets are the single source of truth: sort the points within each cluster and the clusters themselves, then assert a single EXPECT_EQ against {near_points\_, far_points\_}. Drops the per-point if-reclassification and the tautological near/far counters that re-derived the production decision on the test side, per review.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  ---------
* Contributors: Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_euclidean_cluster_object_detector): fix bugprone-narrowing-conversions warnings (`#938 <https://github.com/mitsudome-r/autoware_core/issues/938>`_)
  * fix(autoware_euclidean_cluster_object_detector): fix bugprone-narrowing-conversions warnings
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* refactor(autoware_crop_box_filter): extract CropBoxFilter logic class (`#941 <https://github.com/mitsudome-r/autoware_core/issues/941>`_)
  * refactor(autoware_crop_box_filter): rename CropBoxFilter node class to CropBoxFilterNode
  * refactor(autoware_crop_box_filter): extract CropBoxFilter logic class
  * refactor(autoware_crop_box_filter): remove output_frame from CropBoxFilterConfig
  * refactor(autoware_crop_box_filter): remove unnecessary member variables
  Convert tf_input_orig_frame\_ and max_queue_size\_ to local variables
  as they are only used in the constructor.
  * test(autoware_crop_box_filter): add unit tests for preprocess and postprocess transform
  * refactor(crop_box_filter): rename variable for clarity in transform handling
  * refactor(crop_box_filter): simplify transform handling by removing has_value checks
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* chore(euclidean_cluster): move header files from include to src (`#869 <https://github.com/mitsudome-r/autoware_core/issues/869>`_)
  * chore(autoware_euclidean_cluster_object_detector): move header files from include/ to src/
  * chore(euclidean_cluster): remove pragma once
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* Contributors: NorahXiong, Takahisa Ishikawa, github-actions

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
