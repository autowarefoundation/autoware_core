^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_route_handler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* feat(route_handler): use a cost metric to select start lane (`#357 <https://github.com/autowarefoundation/autoware_core/issues/357>`_)
  * feat(route_handler): use a cost metric to select start lane
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Mert Çolak

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat: add setAllowArea method in route handler for downstream module route validation (`#1194 <https://github.com/autowarefoundation/autoware_core/issues/1194>`_)
  * feat: add setAllowArea method in route handler and support for finding next_lanelets_sequence when route consists areas
  * style(pre-commit): autofix
  * test(autoware_route_handler): add unit tests for allow-area route support
  * fix: address precommit-ci errors
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* perf(autoware_route_handler): drop duplicate routing graph build in setMap (`#1119 <https://github.com/autowarefoundation/autoware_core/issues/1119>`_)
  * perf(autoware_route_handler): drop duplicate routing graph build in setMap
  setMap() built the whole-map vehicle RoutingGraph twice and ran an unused
  full-map lanelet query in both overloads. Building a RoutingGraph over an
  entire HD map is one of the most expensive operations in the stack.
  - In the LaneletMapConstPtr overload, instantiate_routing_graph_and_traffic_rules
  already builds the vehicle graph with the Germany/Vehicle rules, which is
  identical to the vehicle graph rebuilt for the overall-graph container. Reuse
  the already-built routing_graph_ptr\_ as the container's vehicle slot instead of
  rebuilding it, halving the routing-graph build cost of this overload.
  - In the LaneletMapBin overload, routing_graph_ptr\_ is built with the Autoware
  participant rules (lanelet::autoware::DefaultLocation), which permit routing
  through areas, whereas the overall-graph container intentionally uses the plain
  Germany/Vehicle vehicle graph. To preserve the observable getOverallGraphPtr()
  behavior, the separate vehicle graph build is kept here; only the dead
  all_lanelets query is removed.
  - Remove the unused `all_lanelets` lanelet-layer query from both overloads.
  Add characterization tests (test/test_set_map.cpp) pinning the observable graph
  state exposed via getRoutingGraphPtr() / getOverallGraphPtr(): the overall-graph
  container always exposes two non-null graphs, and for the LaneletMapConstPtr
  overload the container's vehicle graph encodes the same per-lanelet following
  relations as routing_graph_ptr\_ and is deterministic across reconstruction.
  No public API change. Behavior preserved (verified by full package test suite).
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * test(autoware_route_handler): assert vehicle graph against independent reference (`#82 <https://github.com/autowarefoundation/autoware_core/issues/82>`_)
  Build a separate Germany/Vehicle routing graph in the test and compare the reused production routing graph against it, instead of comparing the routing graph to the container's vehicle slot (the same shared_ptr), which made the assertion f(x)==f(x). Add a pointer-identity check to document the intended reuse.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * test(autoware_route_handler): drop weak setMap characterization tests
  Remove test/test_set_map.cpp per review. On the area-free sample map, Germany/Vehicle and DefaultLocation rules produce identical following() relations, so the tests would pass even with the wrong rules and were hard to read. The production setMap() change has no observable behavior change and remains; a focused setMap test on an area-containing fixture can be added separately.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  ---------
* feat: add support for area for route planning (`#993 <https://github.com/autowarefoundation/autoware_core/issues/993>`_)
  * feat: add support for area for route planning
  * style(pre-commit): autofix
  * feat(route_handler): publish mixed lanelet/area routes in LaneletRoute segments
  * style(pre-commit): autofix
  * feat(map): visualize LaneletMap areaLayer in vector map RViz markers
  * style(pre-commit): autofix
  * revert: remove area visualization commit (ebbc254d80df34dded119e7a6ecb716ebe4d620c) to split into two PRs
  ---------
  Co-authored-by: Ryohsuke Mitsudome <ryohsuke.mitsudome@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Yutaka Kondo, emmeyteja, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_route_handler): add test for validating the `route_handler`'s reference path algorithm  (`#986 <https://github.com/mitsudome-r/autoware_core/issues/986>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* feat(autoware_route_handler): smooth the centerline from `route_handler` function (`#1029 <https://github.com/mitsudome-r/autoware_core/issues/1029>`_)
  * fix border's discontinuity
  * add route_handler constructor using laneletMapPtr explicitly
  * change waypoint-centerline trim margin
  * improve interpolation at start_s to be on waypoint
  * change use_exact condition
  * add abnormal case comment
  ---------
* chore(planning, bvp): remove unused lanelet2_extension header (`#902 <https://github.com/mitsudome-r/autoware_core/issues/902>`_)
  * remove unused lanelet2_extension in bvp modules
  * remove unused lanelet2_extension in planning components
  ---------
* fix(autoware_route_handler): fix bugprone-narrowing-conversions warnings (`#923 <https://github.com/mitsudome-r/autoware_core/issues/923>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* fix(lanelet2_utils): change is_in_lanelet argument order (`#890 <https://github.com/mitsudome-r/autoware_core/issues/890>`_)
* feat(lanelet2_extension): port lanelet2_extension utilities functions (final)  (`#838 <https://github.com/mitsudome-r/autoware_core/issues/838>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* feat(autoware_lanelet2_extension): replace remaining lanelet2_extension utilities functions (`#842 <https://github.com/mitsudome-r/autoware_core/issues/842>`_)
  * replace getArcCoordinates usage
  * replace combineLaneletsShape
  * remove null return for get_dirty_expanded_lanelet
  * change get_dirty_expanded_lanelet(s) positive right_offset (and negative left_offset) handler
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* chore: organize maintainer (`#856 <https://github.com/mitsudome-r/autoware_core/issues/856>`_)
* Contributors: NorahXiong, Sarun MUKDAPITAK, Satoshi OTA, github-actions

1.7.0 (2026-02-14)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* refactor(planning): deprecate lanelet_extension geometry conversion function (`#834 <https://github.com/autowarefoundation/autoware_core/issues/834>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* feat(lanelet2_extension): port some lanelet2_extension query functions (2) (`#809 <https://github.com/autowarefoundation/autoware_core/issues/809>`_)
* feat(lanelet2_extension): port some lanelet2_extension query functions  (`#790 <https://github.com/autowarefoundation/autoware_core/issues/790>`_)
  * port left/right/all_neighbor_lanelets and their tests
  * replace getAllNeighbors* with ported functions
  * add neighbors order description and verify in test
  * add ported functions to API table
  * add example code snippet
  * Remove optional from return value
  Assume one lanelet is always in return value (itself)
  * change usage of left/right_lanelets (always check access)
  * fix README for example code snippet
  * fix cppcheck-differential
  * fix README description
  ---------
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* refactor(planning, common): replace lanelet2_extension function (`#796 <https://github.com/autowarefoundation/autoware_core/issues/796>`_)
* Contributors: Mamoru Sobue, Ryohsuke Mitsudome, Sarun MUKDAPITAK

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_lanelet2_utils): replace from/toBinMsg (`#737 <https://github.com/autowarefoundation/autoware_core/issues/737>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* ci(pre-commit): autoupdate (`#723 <https://github.com/autowarefoundation/autoware_core/issues/723>`_)
  * pre-commit formatting changes
* Contributors: Mete Fatih Cırıt, Sarun MUKDAPITAK, github-actions

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_lanelet2_utils): replace ported functions from autoware_lanelet2_extension (`#695 <https://github.com/autowarefoundation/autoware_core/issues/695>`_)
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(route_handler): use next lane velocity when merging points (`#690 <https://github.com/autowarefoundation/autoware_core/issues/690>`_)
* feat(autoware_lanelet2_utils): porting functions from lanelet2_extension to autoware_lanelet2_utils package (`#621 <https://github.com/autowarefoundation/autoware_core/issues/621>`_)
* fix(route_handler): add same lane ID check in getRightLanelet/getLeftLanelet function (`#632 <https://github.com/autowarefoundation/autoware_core/issues/632>`_)
  * fix(route_handler): add same lane ID check in getRightLanelet/getLeftLanelet function
  * remove unnecessary changes
  ---------
* fix(route_handler): correct handling of centerline point retrieval when requested distance is longer than lanelet length (`#622 <https://github.com/autowarefoundation/autoware_core/issues/622>`_)
  * fix: correct handling of centerline point retrieval when requested distance is longer than lanelet length
  * fix: return last point
  ---------
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Sarun MUKDAPITAK, Satoshi OTA, Yukinari Hisaki, Yutaka Kondo, Zulfaqar Azmi, mitsudome-r

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(route_handler): add bicycle lane getter (`#589 <https://github.com/autowarefoundation/autoware_core/issues/589>`_)
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Mamoru Sobue, Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* perf(route_handler): improve functions to get nearest route lanelet (`#532 <https://github.com/autowarefoundation/autoware_core/issues/532>`_)
* fix(autoware_route_handler): fix deprecated autoware_utils header (`#446 <https://github.com/autowarefoundation/autoware_core/issues/446>`_)
  * fix autoware_utils header
  * style(pre-commit): autofix
  * fix to autoware_utils_geometry
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* fix(route_handler): fix error message for the route planning failure (`#463 <https://github.com/autowarefoundation/autoware_core/issues/463>`_)
  * fix(route_handler): fix error message for the route planning failure
  * use stringstream
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* fix(route_handler): getCenterLinePath with cropping waypoints (`#378 <https://github.com/autowarefoundation/autoware_core/issues/378>`_)
* feat(route_handler): remove only_route_lanes = false usage (`#450 <https://github.com/autowarefoundation/autoware_core/issues/450>`_)
* feat(route_handler, QC): add overload of get_shoulder_lanelet_sequence (`#431 <https://github.com/autowarefoundation/autoware_core/issues/431>`_)
  feat(route_handler): add get_shoulder_lanelet_sequence
* feat(route_handler): use a cost metric to select start lane (`#357 <https://github.com/autowarefoundation/autoware_core/issues/357>`_)
  * feat(route_handler): use a cost metric to select start lane
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Mamoru Sobue, Masaki Baba, Maxime CLEMENT, Mert Çolak, Takayuki Murooka, Tim Clephas, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* feat: port autoware_route_handler from Autoware Universe (`#201 <https://github.com/autowarefoundation/autoware.core/issues/201>`_)
  Co-authored-by: Mete Fatih Cırıt <xmfcx@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, mitsudome-r
