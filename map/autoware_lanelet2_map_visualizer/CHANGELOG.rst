^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_lanelet2_map_visualizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_lanelet2_map_visualizer): extract pure marker-array builder and reuse autoware_utils_visualization (`#1143 <https://github.com/autowarefoundation/autoware_core/issues/1143>`_)
  * refactor(autoware_lanelet2_map_visualizer): extract pure marker-array builder and reuse autoware_utils_visualization
  Move the LaneletMap-to-MarkerArray assembly out of the on_map_bin ROS
  callback into a pure, internally-headered free function
  create_lanelet_map_marker_array(map, viz_centerline), and have the node
  callback delegate to it then publish. This adds a directly unit-testable
  seam with no change to the node's public constructor or topics.
  Replace the two .cpp-local helpers (set_color, insert_marker_array) with
  autoware_utils_visualization::create_marker_color / append_marker_array,
  which are semantically identical (append_marker_array with no current_time
  performs the same push-back, and create_marker_color static_casts the same
  way set_color did). These helpers are internal, so the public API is
  unaffected.
  Add unit tests on the new free function asserting per-element-category
  marker namespaces and colors (road triangle, road boundaries, curbstone),
  the empty-map degenerate path, and the viz_centerline branch toggling the
  center-line markers. The new tests run without rclcpp init/shutdown.
  Behavior-preserving: the deserialized map is now passed directly as a
  LaneletMapConstPtr to the read-only queries instead of being deep-copied
  via remove_const first; the published MarkerArray is unchanged.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * Update map/autoware_lanelet2_map_visualizer/test/test_lanelet2_map_visualization.cpp
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  ---------
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* feat: add visualization support for area primitives (part of reverse maneuver feature) (`#1122 <https://github.com/autowarefoundation/autoware_core/issues/1122>`_)
  feat: add visualization for area primitives; part of reverse maneuver feature
* Contributors: Yutaka Kondo, emmeyteja, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_lanelet2_map_visualizer): add new maker (`#828 <https://github.com/mitsudome-r/autoware_core/issues/828>`_)
  * add maker
  ---------
* chore(planning, misc): remove unused header includes (`#840 <https://github.com/mitsudome-r/autoware_core/issues/840>`_)
* Contributors: Mamoru Sobue, Yuki TAKAGI, github-actions

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_lanelet2_utils): replace from/toBinMsg (`#737 <https://github.com/autowarefoundation/autoware_core/issues/737>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* Contributors: Sarun MUKDAPITAK, github-actions

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
* Merge remote-tracking branch 'origin/main' into humble
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* feat(lanelet2_map_visualizer): visualize road borders (`#575 <https://github.com/autowarefoundation/autoware_core/issues/575>`_)
  * feat(lanelet2_map_visualizer): visualize fences and road borders
  * only visualize road border
  * use a more general function
  ---------
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* Contributors: Ryohsuke Mitsudome, Zulfaqar Azmi

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* test(autoware_lanelet2_map_visualization): add unit test code (`#477 <https://github.com/autowarefoundation/autoware_core/issues/477>`_)
* feat(autoware_lanelet2_map_visualizer): porting autoware_lanelet2_map_visualizer from autoware_universe (`#428 <https://github.com/autowarefoundation/autoware_core/issues/428>`_)
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* Contributors: Bang.Liu, NorahXiong, github-actions

* fix: to be consistent version in all package.xml(s)
* test(autoware_lanelet2_map_visualization): add unit test code (`#477 <https://github.com/autowarefoundation/autoware_core/issues/477>`_)
* feat(autoware_lanelet2_map_visualizer): porting autoware_lanelet2_map_visualizer from autoware_universe (`#428 <https://github.com/autowarefoundation/autoware_core/issues/428>`_)
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* Contributors: Bang.Liu, NorahXiong, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-22)
------------------

0.2.0 (2025-02-07)
------------------

0.0.0 (2024-12-02)
------------------
