^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_map_loader
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* feat(component_interface_specs): use template type in get_qos function (`#364 <https://github.com/autowarefoundation/autoware_core/issues/364>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* feat(map_loader): add the explanation of handling use_waypoints (`#342 <https://github.com/autowarefoundation/autoware_core/issues/342>`_)
* Contributors: Takagi, Isamu, Takayuki Murooka

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(map): add node designs for the map nodes (`#1413 <https://github.com/autowarefoundation/autoware_core/issues/1413>`_)
  * feat(map): add node designs for the map nodes
  Describe the interfaces, parameters and processes of the pointcloud and
  lanelet2 map loaders, the map hash generator, the lanelet2 map
  visualizer and the map projection loader as system design node files.
  * fix(map): correct description formatting in Lanelet2MapLoader.node.yaml
  * refactor(map): rename node designs to match the node class names
  PointCloudMapLoader and Lanelet2MapVisualization follow the entity
  naming convention derived from the C++ class names, as in `#1335 <https://github.com/autowarefoundation/autoware_core/issues/1335>`_.
  * fix(map): declare the MapHashGenerator external API interfaces as remap targets
  The map hash publisher and the lanelet XML server carry the fixed API
  names as remap_target, so a system design connects them like any other
  port and the launcher remaps the node-side names.
  * fix(map): update description for map_projector_info to clarify its purpose
  ---------
* fix(map): declare the dependencies these packages use (`#1370 <https://github.com/autowarefoundation/autoware_core/issues/1370>`_)
  Each of these packages uses a package it never declares. Either it includes a
  header of that package, or it names a symbol of it while the header arrives
  through another dependency. Both build today only because some declared
  dependency re-exports the owner, so a change in an unrelated repository can
  break them without anything here changing.
  The tag follows where the dependency is used: a use in an installed header or
  in code compiled into the library takes <depend>, one reached only from test/
  takes <test_depend>. System libraries are named by the rosdep key this
  workspace already prefers.
* fix(autoware_map_loader): use CMake targets and portable logging (`#1270 <https://github.com/autowarefoundation/autoware_core/issues/1270>`_)
  * Use CMake targets and portable integer logging in autoware_map_loader
  This upstreams RoboStack downstream patch `patch/ros-rolling-autoware-map-loader.osx.patch`.
  Best-guess rationale: linking to yaml-cpp and fmt through imported targets is more robust across package managers, and portable PRIu64 logging avoids uint64_t format mismatches on platforms where unsigned long and unsigned long long differ.
  * style(pre-commit): autofix
  * Support old fmt target
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
  ---------
  Co-authored-by: Daisuke Nishimatsu <nishimarudai@gmail.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
* fix(autoware_map_loader): always publish the pointcloud map metadata (`#1316 <https://github.com/autowarefoundation/autoware_core/issues/1316>`_)
  `output/pointcloud_map_metadata` was only advertised when `enable_selected_load`
  was true, even though the metadata dict it is built from is always available and
  the differential map loader that serves the map is created unconditionally.
  A deployment that delivers the PCD map through the differential/partial services
  sets `enable_whole_load: false`, which retires `/map/pointcloud_map`. Such a
  deployment was then left with no topic proving the map module had loaded its map:
  the map component's topic monitor watches a topic nobody publishes, reports
  `NotReceived` (checked before any rate/timeout threshold, so the zeroed thresholds
  do not matter), and the map module stays in ERROR for the whole run.
  Publish the latched metadata as soon as the PCD metadata is parsed, so it is the
  delivery-independent evidence that the map is loaded and servable. This makes
  `enable_selected_load` govern only the selected-load service, as its name says.
  The new launch test pins the regression: with `enable_whole_load` and
  `enable_selected_load` both false, the metadata must still arrive on a
  transient-local subscription, and `output/pointcloud_map` must stay unadvertised.
* feat: [codecov/refactoring] [lanelet2_map_loader] Core logic isolation (`#1258 <https://github.com/autowarefoundation/autoware_core/issues/1258>`_)
  * moved all peripheral components and submodules to src/lanelet2_map_loader/utils
  * added isolated core logic header of lanelet2_map_loader.hpp
  * init lanelet2_map_loader header component as core logic
  * init lanelet2_map_loader source component as core logic
  * init lanelet2_map_loader_node header component as ROS2 node wrapper
  * init lanelet2_map_loader_node source component as ROS2 node wrapper
  * init lanelet2_map_loader_node source component as ROS2 node wrapper
  * adapted the current test suites to the new architecture
  * adapted and updatyed CMakeLists
  * removed redundant lanelet2 map loadser header .hpp
  * fixed cppcheck errors
  * [ishikawa] addressed lack of logging
  * [ishikawa] avoid copying whole map message again
  * ensure pubsub handshake inside test suite is all good with tolerance
  * [ishikawa] use rclcpp::Time::now() directly inside the core logic
  * [akamine] added the frame_id designation upon create map bin msg
  * [akamine] addressed concerns regarding nested headers + the refactoring of selected_map_loader_module
  * revamped the test suite for selected_map_loader_module (core logic test goes to unit test, node test goes to integration test)
  * [akamine] addressed the maploadexception catch
  * [akamine] explains why I set the try actch branches like that
  * [akamine] restructure - bring lanelet2_selectyed_map_loade_module.cpp/hpp outside utils to src
  * addressed cppcheck
  * style(pre-commit): autofix
  * [akamine] fully revert to original logging behavior
  * [akamine] address additional warnings and loggings
  * [akamine] address all concerns
  * fix(lanelet2_map_loader): complete agnocast application on the node wrapper
  After rebasing the core-logic isolation work onto main (which applied
  agnocast_wrapper::Node to the *legacy* node via `#1193 <https://github.com/autowarefoundation/autoware_core/issues/1193>`_), the colleague's
  agnocast intention was only half-carried-over: the ROS interfaces that this
  branch moved out of Lanelet2SelectedMapLoaderModule and into
  Lanelet2MapLoaderNode were left as plain rclcpp:: types, leaving the header
  declaration and the .cpp definition inconsistent.
  Reconcile both intentions where the interfaces now live (the node wrapper):
  - subscription/publishers/service -> AUTOWARE_SUBSCRIPTION_PTR / _PUBLISHER_PTR
  / _SERVICE_PTR
  - on_map_projector_info / on_get_selected_lanelet2_map -> agnocast
  message/server pointer macros (matching the migrated pointcloud node)
  - drop the now-unused agnocast_wrapper includes from the pure-core-logic
  Lanelet2SelectedMapLoaderModule header
  Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
  * style(pre-commit): autofix
  * [akamine] docs fixing
  * [akamine] legay dependencies fix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
* feat(pointcloud_map_loader): apply `agnocast_wrapper::Node` to  `pointcloud\_ map_loader` (`#1198 <https://github.com/autowarefoundation/autoware_core/issues/1198>`_)
  * feat(pointcloud_map_loader): apply agnocast_wrapper::Node
  * refactor(pointcloud_map_loader): publish whole/downsampled map via publish(const &)
  * refactor(pointcloud_map_loader): publish metadata via publish(const &)
  ---------
* refactor(pointcloud_map_loader): move first segment to avoid a full-cloud copy (`#1286 <https://github.com/autowarefoundation/autoware_core/issues/1286>`_)
* feat(lanelet2_map_loader): apply `agnocast_wrapper::Node` to `lanelet2_map_loader` (`#1193 <https://github.com/autowarefoundation/autoware_core/issues/1193>`_)
  * feat(lanelet2_map_loader): apply `agnocast_wrapper::Node`
  * fix build failure in test
  * fix(autoware_agnocast_wrapper): include <rclcpp/version.h> for version guards
  * fix(autoware_map_loader): skip node-based lanelet2 tests when ENABLE_AGNOCAST=1
  ---------
* feat: [codecov/refactoring] [lanelet2_map_loader] implement characterization test (`#1255 <https://github.com/autowarefoundation/autoware_core/issues/1255>`_)
  * init integration characterization test suite for ll2 map loader
  * added TEST 1 of integration test to check LL@MapLoader normal behaviors
  * added TEST 2 of integration test to check the branch where use_paypoints = false
  * added TEST 3 of integration test to check branch when enable_selected_map_loading = true
  * added TEST 4 to test the allow unsupported version = false with a stupid version number
  * update CMakeLists, all builds and tests are good now coverage also good
* refactor(`pointcloud_map_loader`): internalize selected map loader's planning helper (`#1248 <https://github.com/autowarefoundation/autoware_core/issues/1248>`_)
  * fix(`test`): re-organize folder structure to be same as `src`
  * refactor: internalize selected map loader's planning helper
  * Also merge selected tests, and move selected node test to pointcloud node test
  * style(pre-commit): autofix
  * bug: remove unused tests
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(`pointcloud_map_loader`): fix inconsistent naming in `test` folder (`#1249 <https://github.com/autowarefoundation/autoware_core/issues/1249>`_)
* refactor(`pointcloud_map_loader`): internalize partial map loader helper (`#1247 <https://github.com/autowarefoundation/autoware_core/issues/1247>`_)
* refactor(`map`): fix tests in differential map loader (`#1245 <https://github.com/autowarefoundation/autoware_core/issues/1245>`_)
  * fix(`test`): re-organize folder structure to be same as `src`
  * refactor: merge differential core/module tests into one core test and move differential node test to pointcloud module test
  * differential core+module test split removed
  * differential node/service test moved into pointcloud module test
  * refactor: hide differential helper API and test via module contract
  * style(pre-commit): autofix
  * rename: node logic test file (=> `test_pointcloud_map_loader_node.cpp`)
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(`map`): add a test code `test_pointcloud_map_loader_core.cpp` (`#1242 <https://github.com/autowarefoundation/autoware_core/issues/1242>`_)
  * feat(`map`): add a test code `test_pointcloud_map_loader_core.cpp`
  * style(pre-commit): autofix
  * bug: remove non-functional tests
  * bug: remove non-functional tests
  * fix: add error checks in test code
  * fix: re-ordering tests, to avoid confusion
  * Apply the following review comment:
  - https://github.com/autowarefoundation/autoware_core/pull/1242#pullrequestreview-4667905825
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(`map`): add a test code `test_partial_map_loader_core.cpp` (`#1241 <https://github.com/autowarefoundation/autoware_core/issues/1241>`_)
* feat(`map`): add a test code `test_differential_map_loader_core.cpp` (`#1240 <https://github.com/autowarefoundation/autoware_core/issues/1240>`_)
  add(`map`): test code `test_differential_map_loader_core.cpp`
* feat(`map`): add a test code `test_selected_map_loader_core.cpp` (`#1243 <https://github.com/autowarefoundation/autoware_core/issues/1243>`_)
* refactor(`pointcloud_map_loader`): separated core logic (`#1226 <https://github.com/autowarefoundation/autoware_core/issues/1226>`_)
  * refactor: separated core logic
  * style(pre-commit): autofix
  * refactor: merge `pointcloud_map_loader_node_core` into `pointcloud_map_loader`
  * For file name rule consistency
  * fix: by `pre-commit`
  * style(pre-commit): autofix
  * refactor: move ROS node logic to PointCloudMapLoaderNode and keep loader modules core logic-focused
  * refactor: remove `rclcpp::Logger` from core logic
  * All `RCLCPP_xxx`s are now handled in ROS node logic
  * style(pre-commit): autofix
  * bug: fix by `cppcheck`
  * refactor: merge pointcloud loader module into core loader files
  * move PointcloudMapLoaderModule declaration/implementation into pointcloud_map_loader
  * remove obsolete pointcloud_map_loader_module header/source and CMake entry
  * update includes/call sites to use merged header
  * fix test include after transitive-header removal
  * refactor: merge differential loader module into core differential loader
  * move DifferentialMapLoaderModule declaration/implementation into differential_map_loader hpp/cpp
  * remove obsolete differential_map_loader_module hpp/cpp files
  * update node and test includes to use differential_map_loader.hpp
  * preserve existing behavior and ROS service wiring
  * refactor: merge partial loader module into core partial loader
  * move PartialMapLoaderModule declaration/implementation into partial_map_loader hpp/cpp
  * remove obsolete partial_map_loader_module hpp/cpp files
  * update node and test includes to partial_map_loader.hpp
  * add explicit PCL includes in partial loader test after transitive include removal
  * refactor: merge selected loader module into core selected loader
  * move SelectedMapLoaderModule declaration/implementation into selected_map_loader hpp/cpp
  * remove obsolete selected_map_loader_module hpp/cpp files
  * update node and test includes to selected_map_loader.hpp
  * fix(`test`): restore original code (see below)
  * For regression check, we like to keep the tests with minimum change
  * We are going to add tests for core logic in the next PR
  * Perhaps we will remove node logic part tests after adding core logic tests
  * (missing fix in previous commit) restore original code
  * bug(`differential_map_loader`): fix so core logic to be node-free
  * bug: remove duplicated node logic (same as previous commit)
  * Due to initialization path change, we adjust EXPECT_FLOAT_EQ bounds for node-initialized path
  - Reason: test now initializes via PointCloudMapLoaderNode, so metadata bounds come from the actual single-file PCD (/tmp/dummy.pcd) instead of dummy_metadata_dict; expected y-bounds are -1/1.
  * bug: fix mising `tl_expected` dependency
  * style(pre-commit): autofix
  * bug: fix the following bugs
  * Inconsistent log output. Now we uses lambda function for log output.
  This makes only node logic output log.
  * Removed unused `PointcloudMapLoaderModule::PointcloudMapLoaderModule` interface
  * style(pre-commit): autofix
  * bug: fix missing log output, this is missing fix for previous commit
  * bug: remove unused dependency tl_expected
  * fix: by `pre-commit`
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_map_loader): add support of GetSelectedLanelet2Map service (`#889 <https://github.com/autowarefoundation/autoware_core/issues/889>`_)
  * fix build failure for jazzy
  * feat(autoware_map_loader): add support of GetSelectedLanelet2Map service
  * style(pre-commit): autofix
  * apply fix for pre-commit
  * fix cppcheck error
  * fix build
  * update parameters
  * set default to false
  * modify lanelet2_map_loader behavior to match with pcd_map_loader
  * fix launch files
  * update test scripts
  * rename parameter name
  * add args to autoware_core_map.launch.xml
  * update copyright year
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <ryoshuke.mitsudome@tier4.jp>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* Contributors: Junya Sasaki, Koichi Imai, Mete Fatih Cırıt, Ryohsuke Mitsudome, Taekjin LEE, Tobias Fischer, Tran Huu Nhat Huy, Yutaka Kondo, github-actions

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_map_loader): dedup PCD cell loader and cover selected module (`#1139 <https://github.com/autowarefoundation/autoware_core/issues/1139>`_)
  Consolidate the byte-identical load_point_cloud_map_cell_with_id plus the
  repeated metadata.min/max copy block from the partial, differential and
  selected loader modules into a single free function in utils, and route all
  three modules through it.
  Add test/test_selected_map_loader_module.cpp covering the previously untested
  SelectedMapLoaderModule service handler (found and not-found branches with the
  metadata bounds populated) plus a direct unit test of the create_metadata free
  function, which is now declared in the module header so it can be tested without
  instantiating the module.
  Behavior-preserving and internal-only; the public module constructor signatures
  are unchanged.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
* feat(autoware_map_loader): enable loading of multiple lanelet2 map files (`#888 <https://github.com/autowarefoundation/autoware_core/issues/888>`_)
  * feat(autoware_map_loader): enable loading of multiple lanelet map files
  * chore: update comments for arguments
  * add: test codes
  * style(pre-commit): autofix
  * docs: update README
  * fix build failure for jazzy
  * Update map/autoware_map_loader/src/lanelet2_map_loader/lanelet2_map_loader_node.cpp
  Co-authored-by: Yamato Ando <yamato.ando@gmail.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yamato Ando <yamato.ando@gmail.com>
* Contributors: Ryohsuke Mitsudome, Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_core): add USE_SCOPED_HEADER_INSTALL_DIR to map packages (`#976 <https://github.com/mitsudome-r/autoware_core/issues/976>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* Contributors: Vishal Chauhan, github-actions

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_lanelet2_utils): replace from/toBinMsg (`#737 <https://github.com/autowarefoundation/autoware_core/issues/737>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* chore: jazzy-porting: fix test depend launch-test missing (`#738 <https://github.com/autowarefoundation/autoware_core/issues/738>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Sarun MUKDAPITAK, github-actions, 心刚

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
* fix(autoware_ndt_scan_matcher): update link (`#510 <https://github.com/autowarefoundation/autoware_core/issues/510>`_)
  fix link
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
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* feat(component_interface_specs): use template type in get_qos function (`#364 <https://github.com/autowarefoundation/autoware_core/issues/364>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* feat(map_loader): add the explanation of handling use_waypoints (`#342 <https://github.com/autowarefoundation/autoware_core/issues/342>`_)
* Contributors: Takagi, Isamu, Takayuki Murooka, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat(autoware_map_loader): port autoware_map_loader from Autoware Universe to Core (`#326 <https://github.com/autowarefoundation/autoware_core/issues/326>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Ryohsuke Mitsudome
