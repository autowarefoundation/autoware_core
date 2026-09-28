^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_point_types
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_point_types): add PointXYZIRCT point type (`#1422 <https://github.com/autowarefoundation/autoware_core/issues/1422>`_)
  * feat(autoware_point_types): add PointXYZIRCT point type
  Add PointXYZIRC extended by a per-point time_stamp (uint32 nanoseconds
  relative to the point cloud's header stamp), together with its field
  generator, layout check, field factory and PCL registration.
  This is the output point type for point clouds that no longer share a
  single sensor origin -- notably the concatenation of several LiDARs,
  where the azimuth/elevation/distance fields of PointXYZIRCAEDT lose
  their meaning but the per-point acquisition time is still needed by
  time-aware ML models.
  The layout is a strict superset of PointXYZIRC, so consumers that check
  is_data_layout_compatible_with_point_xyzirc() and read fields at their
  PointXYZIRC offsets remain compatible.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * test(autoware_point_types): address review on the PointXYZIRCT tests
  - Drop TEST(PointLayout, PointXYZIRCT); the prefix property it pinned is
  already covered by the cross-type layout tests.
  - Cover xyzirct in every direction of MismatchedTypesReturnFalse, grouped by
  source layout like the surrounding cases.
  - Widen the superset test to all layouts extending xyzirc and shorten its
  comment to one line.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * docs(autoware_point_types): define time_stamp for PointXYZIRCT and PointXYZIRCAEDT
  Both are a non-negative offset in nanoseconds from the containing point cloud's
  header.stamp. This was only stated loosely for PointXYZIRCT and not at all for
  PointXYZIRCAEDT, though producers and consumers already rely on it.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* feat(point-types): add function that converts class name to PointCloudClassification (`#1382 <https://github.com/autowarefoundation/autoware_core/issues/1382>`_)
* fix(common): declare the dependencies these packages use (`#1367 <https://github.com/autowarefoundation/autoware_core/issues/1367>`_)
  Each of these packages uses a package it never declares. Either it includes a
  header of that package, or it names a symbol of it while the header arrives
  through another dependency. Both build today only because some declared
  dependency re-exports the owner, so a change in an unrelated repository can
  break them without anything here changing.
  The tag follows where the dependency is used: a use in an installed header or
  in code compiled into the library takes <depend>, one reached only from test/
  takes <test_depend>. System libraries are named by the rosdep key this
  workspace already prefers. Boost.Serialization is declared separately from
  libboost-dev because it needs its own library at link time.
* feat(point_types, object_recognition_utils): segmentation pointcloud (`#1288 <https://github.com/autowarefoundation/autoware_core/issues/1288>`_)
  * feat: add definition of point type for segmentation points
  * feat: add helper function for segmented pointcloud label
  * feat: replace default entropy value by Nan
  * feat: add PointCloudClassification::INVALID
  * refactor: move PointCloudClassification to autoware_point_types
  * docs: update README
  ---------
* Contributors: Kotaro Uetake, Max Schmeller, Mete Fatih Cırıt, github-actions

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* test(autoware_point_types): cover memory.hpp layout helpers and mark them inline (`#1131 <https://github.com/autowarefoundation/autoware_core/issues/1131>`_)
  memory.hpp had zero test coverage for its eight is_data_layout_compatible_with_point\_* overloads and four create_fields_point\_* factories, and the free functions were defined non-inline in a header (an ODR hazard if the header is included in more than one translation unit).
  - Mark all memory.hpp free functions inline (additive, ODR-safe; existing signatures unchanged).
  - Add test/test_memory.cpp with round-trip (create -> is_compatible) checks for all four point types, exact create_fields\_* content assertions, PointCloud2-overload forwarding, cross-type mismatch, and negative cases (wrong name/offset/datatype/count).
  - Pin the existing field-count guard contract: xyzi/xyzirc/xyziradrt use a 'size() < N' guard (extra trailing fields ignored, still accepted) while xyzircaedt uses a strict 'size() != 10' guard (extra fields rejected). Behavior is preserved; the test characterizes the current divergence.
  - Add sensor_msgs as a direct dependency (memory.hpp includes sensor_msgs headers directly) and register the new gtest in CMakeLists.txt.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
* Contributors: Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
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
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* Contributors: Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-21)
------------------
* chore: rename from `autoware.core` to `autoware_core` (`#290 <https://github.com/autowarefoundation/autoware.core/issues/290>`_)
* test(autoware_point_types): add tests for missed lines (`#260 <https://github.com/autowarefoundation/autoware.core/issues/260>`_)
* feat(point_types): reimplemented the pointcloud preprocesor's memory layout checks (`#197 <https://github.com/autowarefoundation/autoware.core/issues/197>`_)
  feat: reimplemented the pointcloud preprocesor's memory layout checks in the point types package to avoid depending on the pointcloud preprocessor
* Contributors: Kenzo Lobos Tsunekawa, NorahXiong, Yutaka Kondo

0.2.0 (2025-02-07)
------------------
* unify version to 0.1.0
* update changelog
* feat: port autoware_point_types from autoware_universe (`#151 <https://github.com/autowarefoundation/autoware_core/issues/151>`_)
  * feat: port autoware_point_types from universe
  * chore: remove change log and reset version to 0.0.0
  * add myself as maintainer
  * docs: finish README.md
  * style(pre-commit): autofix
  * fix: fix pre-commit.ci error
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Yutaka Kondo, cyn-liu

* feat: port autoware_point_types from autoware_universe (`#151 <https://github.com/autowarefoundation/autoware_core/issues/151>`_)
  * feat: port autoware_point_types from universe
  * chore: remove change log and reset version to 0.0.0
  * add myself as maintainer
  * docs: finish README.md
  * style(pre-commit): autofix
  * fix: fix pre-commit.ci error
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: cyn-liu

0.0.0 (2024-12-02)
------------------
