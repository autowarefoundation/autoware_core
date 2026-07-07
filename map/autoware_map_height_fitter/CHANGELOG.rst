^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_map_height_fitter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* perf(autoware_map_height_fitter): extract testable ground-height kernel and use a cached KdTree (`#1107 <https://github.com/autowarefoundation/autoware_core/issues/1107>`_)
  * perf(autoware_map_height_fitter): extract testable ground-height kernel and use a cached KdTree
  Extract the pure ground-height search out of the Node/TF-bound pimpl Impl into
  free functions in a new internal header so the core algorithm becomes unit-testable
  without ROS:
  - get_ground_height_from_pointcloud(cloud[, kdtree], x, y, fallback_z)
  - get_ground_height_from_vector_map(map, x, y, fallback_z)
  - build_pointcloud_xy_kdtree(cloud)
  Impl::get_ground_height now delegates to these. The two full O(N) linear passes
  over the point cloud per fit() request are replaced with a pcl::KdTreeFLANN built
  once when the cloud is set (on_pcd_map / get_partial_point_cloud_map) and reused:
  nearestKSearch finds the closest point and radiusSearch restricts the lowest-z scan
  to a local candidate set, dropping per-call cost from O(N) to ~O(log N + k).
  Behavior is preserved: the KdTree indexes the horizontal (z = 0) projection so the
  2D distance metric is unchanged, and the membership test is recomputed in double
  precision with the original strict '< (sqrt(min_dist2) + 1.0)^2' comparison over the
  candidate set, so the selected point set (and thus the returned height) is identical
  to the original two-pass scan. The vector-map nearest-point lookup and all
  non-finite / empty / missing-map fallbacks are unchanged.
  Also fold in two cheap, behavior-preserving micro-optimizations flagged in the audit:
  radius2 is computed as r*r instead of std::pow(..., 2.0), and the partial-map merge
  loop reserves the total data size before concatenating grids.
  Add a gtest suite (test/test_map_height_fitter_kernel.cpp) covering the empty-cloud
  and single-point fallbacks, lowest-z-within-radius selection, the horizontal-only
  distance metric, the prebuilt-vs-convenience overload equivalence, the vector-map
  nearest height and its non-finite/empty fallbacks, and a 300-trial randomized
  characterization test asserting the new kernel matches the original two-pass scan
  exactly. The package previously had zero tests.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * perf(autoware_map_height_fitter): preserve double-precision nearest search
  The KdTree path searched entirely in float: both the query and the indexed
  coordinates were rounded to float, and min_dist2 was taken from the single
  index returned by nearestKSearch. At large map coordinates (MGRS/UTM-derived)
  or for near-tied points the float nearest can differ from the true
  double-precision nearest, changing the radius and the selected height, so the
  documented bit-for-bit behavior preservation did not hold (demonstrably ~1 in
  2000 at coordinates up to 2e6).
  Recompute min_dist2 in double precision over a margin-enlarged candidate set
  that is a guaranteed superset of the double-nearest point, and size both the
  nearest and the height radiusSearch margins from the coordinate magnitude
  (2 * coord_max * 2^-23 + 1e-3) instead of a fixed 1e-3, so every point the
  strict double-precision test accepts is in the candidate set regardless of
  coordinate scale. The double-precision filter then reproduces the original
  membership exactly.
  Add a large-coordinate / near-tie characterization test (5000 trials, centers
  up to 2e6) that asserts bit-equality against the original two-pass scan; it
  fails on the previous float-only logic and passes now.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * test(autoware_map_height_fitter): pin non-finite kernel inputs and guard NaN query (`#67 <https://github.com/autowarefoundation/autoware_core/issues/67>`_)
  Add edge-case kernel tests for non-finite cloud coords, query, and z against the in-test double-precision oracle; guard a non-finite query so the KdTree path returns fallback_z instead of tripping a FLANN assert, preserving the pre-PR scan behavior.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * ci(autoware_map_height_fitter): add cspell:ignore for distance var names
  * refactor(autoware_map_height_fitter): move internal kernel header to src/
  Address review: map_height_fitter_kernel.hpp is only used within this package
  (nothing outside includes it), so move it out of the public include/ tree into
  src/ as a non-installed internal header. Switch its include guard to the
  bare-filename form required for src/ headers, point the three includers at the
  relative path, and let the gtest reach it via target_include_directories(PRIVATE src).
  ---------
* Contributors: Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_core): add USE_SCOPED_HEADER_INSTALL_DIR to map packages (`#976 <https://github.com/mitsudome-r/autoware_core/issues/976>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* chore(common, map): remove unused lanelet2_extension header (`#903 <https://github.com/mitsudome-r/autoware_core/issues/903>`_)
  * remove unused lanelet2_extension in map component
  * remove unused lanelet2_extension in common component
  ---------
* Contributors: Sarun MUKDAPITAK, Vishal Chauhan, github-actions

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_lanelet2_utils): replace from/toBinMsg (`#737 <https://github.com/autowarefoundation/autoware_core/issues/737>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* chore: tf2_ros to hpp headers (`#616 <https://github.com/autowarefoundation/autoware_core/issues/616>`_)
* Contributors: Sarun MUKDAPITAK, Tim Clephas, github-actions

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: update maintainer (`#701 <https://github.com/autowarefoundation/autoware_core/issues/701>`_)
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
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Motz, Takagi, Isamu, Yutaka Kondo, mitsudome-r, 心刚

1.4.0 (2025-08-11)
------------------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(localization): add autoware_pose_initializer and autoware_map_height_fitter to autoware core (`#493 <https://github.com/autowarefoundation/autoware_core/issues/493>`_)
* Contributors: github-actions, 心刚

* fix: to be consistent version in all package.xml(s)
* feat(localization): add autoware_pose_initializer and autoware_map_height_fitter to autoware core (`#493 <https://github.com/autowarefoundation/autoware_core/issues/493>`_)
* Contributors: github-actions, 心刚

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-22)
------------------

0.2.0 (2025-02-07)
------------------

0.0.0 (2024-12-02)
------------------
