^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_velocity_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_motion_velocity_planner): hold mutex\_ for the whole on_set_param (`#1455 <https://github.com/autowarefoundation/autoware_core/issues/1455>`_)
  on_trajectory() reads planner_data\_ and smooth_velocity_before_planning\_
  under mutex\_, but on_set_param() released the lock after
  update_module_parameters() and wrote those same members unlocked. Under a
  multi-threaded executor the two callbacks run concurrently, so hold the lock
  for the whole body.
* feat(api, motion_velocity_planner): add the node designs required by the AD API and motion planning design modules (`#1456 <https://github.com/autowarefoundation/autoware_core/issues/1456>`_)
  * feat(api): add node designs for the default AD API nodes and RViz adaptors
  * fix(autoware_motion_velocity_planner): add additional planning factors to publishers in the node design
  ---------
* fix(design): align the autoware_core node designs with the packages they describe (`#1416 <https://github.com/autowarefoundation/autoware_core/issues/1416>`_)
  * fix(autoware_velocity_smoother): correct the velocity limit message type in the node design
  The node publishes current_velocity_limit_mps as
  autoware_internal_planning_msgs/msg/VelocityLimit (node.hpp), while the
  design declared the pre-migration tier4_planning_msgs type, failing the
  connection check against downstream ports.
  * fix(autoware_motion_velocity_planner): correct the velocity limit message types and drop absent publishers in the node design
  * fix(autoware_path_generator): correct the path publisher message type in the node design
  The node publishes autoware_internal_planning_msgs/msg/PathWithLaneId on
  ~/output/path (node.hpp:84), not autoware_planning_msgs/msg/Path.
  * fix(autoware_velocity_smoother): declare the kinematic state subscriber in the node design
  The node polls /localization/kinematic_state for the ego odometry
  (node.hpp:94-97); the design did not list the input at all.
  * fix(autoware_motion_velocity_planner): name the module-side subscriber topics in the node design
  The boundary departure prevention module subscribes on absolute topic
  names, so the default ~/input/<name> remap targets addressed topics the
  node never opens and the module connections resolved to nothing. Declare
  the real names and add the steering status input the module also takes.
  * fix(autoware_gnss_poser): declare the map projector info subscriber in the node design
  The node blocks pose conversion until /map/map_projector_info arrives
  (gnss_poser_node.cpp:43); the design did not list the input at all.
  * fix(autoware_behavior_velocity_planner): add additional planning factors to publishers in the node design
  ---------
* docs(planning): point the autoware_launch config links at their current path (`#1406 <https://github.com/autowarefoundation/autoware_core/issues/1406>`_)
  `pre-commit-optional` fails on every PR with three dead links, all pointing
  into `autoware_launch/config/planning/`. That tree moved to the
  `autoware_planning_config` package:
  autoware_launch/config/planning/scenario_planning/common/
  -> autoware_universe_launch/autoware_planning_config/config/scenario_planning/common/
  `autoware_motion_velocity_planner/README.md` linked `nearest_search.param.yaml`
  and `common.param.yaml`; `autoware_path_generator/README.md` linked the former.
  All three now resolve.
  `autoware_motion_velocity_obstacle_stop_module/README.md` names the planning
  preset in prose rather than as a link, so the checker never flagged it, but the
  path was stale for the same reason and 404s too. Its target moved to
  `autoware_universe_launch/autoware_planning_config/config/preset/`.
  No `autoware_launch/config/planning` path is left in the repository.
  `pre-commit run --config .pre-commit-config-optional.yaml markdown-link-check`
  passes on all three files.
* fix(planning): point design param_files at the packages that install them (`#1387 <https://github.com/autowarefoundation/autoware_core/issues/1387>`_)
  BehaviorVelocityPlanner listed its plugin modules' param files as
  relative paths, resolving against the host node package; each module
  package installs its own config. MotionVelocityPlanner referenced the
  module packages with a _module suffix the installed filenames do not
  have.
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
* fix(planning): declare the dependencies these packages use (`#1372 <https://github.com/autowarefoundation/autoware_core/issues/1372>`_)
  Each of these packages uses a package it never declares. Either it includes a
  header of that package, or it names a symbol of it while the header arrives
  through another dependency. Both build today only because some declared
  dependency re-exports the owner, so a change in an unrelated repository can
  break them without anything here changing.
  The tag follows where the dependency is used: a use in an installed header or
  in code compiled into the library takes <depend>, one reached only from test/
  takes <test_depend>. System libraries are named by the rosdep key this
  workspace already prefers.
  A clean-context review of the pull request found five more direct uses with no manifest entry. Add one entry for each:
  - autoware_path_generator: tf2 (tf2::getYaw in src/utils.cpp)
  - autoware_motion_velocity_planner_common: tf2 (tf2::getYaw in src/planner_data.cpp and src/polygon_utils.cpp)
  - autoware_motion_velocity_obstacle_stop_module: autoware_planning_factor_interface (constructed in src/obstacle_stop_module.cpp)
  - autoware_behavior_velocity_stop_line_module: autoware_planning_factor_interface (used in src/experimental/scene.cpp)
  - autoware_velocity_smoother: rclcpp_components (register_node_macro.hpp in src/node.cpp)
* refactor: migrate node design files from autoware_universe (`#1381 <https://github.com/autowarefoundation/autoware_core/issues/1381>`_)
  Node design files for packages that moved to autoware_core, placed at
  the in-package convention <package>/design/<Name>.node.yaml.
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
* Contributors: Koichi Imai, Mete Fatih Cırıt, Taekjin LEE, Takayuki AKAMINE, github-actions

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(motion_velocity_planner): apply autoware_agnocast_wrapper for CIE (`#1187 <https://github.com/autowarefoundation/autoware_core/issues/1187>`_)
  Switch the motion_velocity_planner node registration from rclcpp_components_register_node to autoware_agnocast_wrapper_register_node (ROS2_EXECUTOR SingleThreadedExecutor, AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor) so the node can be isolated by the agnocast CIE thread configurator.
* refactor(autoware_motion_velocity_planner): extract pure node logic into testable free functions (`#1136 <https://github.com/autowarefoundation/autoware_core/issues/1136>`_)
  * refactor(autoware_motion_velocity_planner): extract pure node logic into testable free functions
  Extract three pure pieces of MotionVelocityPlannerNode logic into free
  functions in an internal header (node_utils.hpp/.cpp), turning the node
  methods into thin wrappers with no public API change:
  - process_traffic_signals: build the raw and last-observed traffic-light
  maps from a TrafficLightGroupArray, including the UNKNOWN-with-prior
  carry-over (keep prior body, refresh timestamp).
  - resample_trajectory_by_min_interval: the min-interval point decimation
  previously inlined in generate_trajectory.
  - select_pointcloud_transform_source: the TF transform-selection branch
  (at-stamp / time-zero fallback / none) from process_no_ground_pointcloud,
  expressed as a pure decision over canTransform results.
  Add focused unit tests (test_node_utils) covering the UNKNOWN carry-over,
  empty/single-point/all-close/threshold resampling cases, and the TF
  fallback/no-transform branches. Behavior preserving.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * refactor(autoware_motion_velocity_planner): address review feedback
  - node_utils.hpp: include lanelet2_core/Forward.h instead of the heavy
  primitives/Lanelet.h since only lanelet::Id is needed for the map key
  (consistent with planner_data.hpp)
  - node.cpp: correct the AtTimeZero fallback debug message; the branch is
  also taken when the stamp is valid but no transform exists at that stamp,
  so do not claim the pointcloud time is always invalid
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * refactor(autoware_motion_velocity_planner): inline pointcloud transform-source selection
  Drop the select_pointcloud_transform_source() extraction per review: the seam captured only a trivial 3-bool decision while the real tf2 coupling (which time to look up, the 0.05s timeout, fallback order and side effects) stayed in the node untested. Inline the decision back into the no-ground-pointcloud handler and remove the PointcloudTransformSource enum and its unit tests. The process_traffic_signals and resample_trajectory_by_min_interval extractions are kept.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  ---------
* fix(planning): skip processing empty no_ground_pointcloud to avoid PCL warning spam (`#1082 <https://github.com/autowarefoundation/autoware_core/issues/1082>`_)
* Contributors: Mert Yavuz, Yutaka Kondo, atsushi yano, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* test(autoware_motion_velocity_planner): add unit tests (`#518 <https://github.com/mitsudome-r/autoware_core/issues/518>`_)
  * test(autoware_motion_velocity_planner): add unit tests
  * style(pre-commit): autofix
  * fix(autoware_motion_velocity_planner): adapt tests to current codebase
  - Fix service includes: srv moved from autoware_motion_velocity_planner
  to autoware_internal_planning_msgs
  - Sync test configs with production configs (motion_velocity_planner,
  obstacle_stop, velocity_smoother params diverged significantly)
  - Add yaml-cpp link dependency for test target
  - Resolve CMakeLists.txt conflict (ament_auto_package rename)
  * style(pre-commit): autofix
  * perf(autoware_motion_velocity_obstacle_stop_module): optimize unit test logic
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
* Contributors: NorahXiong, github-actions

1.7.0 (2026-02-14)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore: reflect the move of the description packages (`#811 <https://github.com/autowarefoundation/autoware_core/issues/811>`_)
* feat(motion_velocity_planner): publish debug trajectory for each module (`#761 <https://github.com/autowarefoundation/autoware_core/issues/761>`_)
* Contributors: Maxime CLEMENT, Ryohsuke Mitsudome, Takagi, Isamu

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* docs: fix broken links (`#779 <https://github.com/autowarefoundation/autoware_core/issues/779>`_)
* chore: tf2_ros to hpp headers (`#616 <https://github.com/autowarefoundation/autoware_core/issues/616>`_)
* refactor(motion_velocity_planner): refactor time publisher (`#718 <https://github.com/autowarefoundation/autoware_core/issues/718>`_)
* Contributors: Mete Fatih Cırıt, Tim Clephas, Yuki TAKAGI, github-actions

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(motion_velocity_planner): prevent sudden yaw changes by adjusting overlap threshold for stop point insertion (`#692 <https://github.com/autowarefoundation/autoware_core/issues/692>`_)
  * fix(motion_velocity_planner): prevent sudden yaw changes by adjusting overlap threshold for stop point insertion
  ---------
* feat(motion_velocity_planner): update pointcloud preprocess (`#631 <https://github.com/autowarefoundation/autoware_core/issues/631>`_)
* fix(motion_velocity_planner): fix empty point cloud header (`#635 <https://github.com/autowarefoundation/autoware_core/issues/635>`_)
* feat(obstacle_stop): enable object specified obstacle_filtering parameter and refactor obstacle type handling (`#613 <https://github.com/autowarefoundation/autoware_core/issues/613>`_)
  * refactor obstacle_filtering structure and type handling
  ---------
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Kyoichi Sugahara, Mete Fatih Cırıt, Yuki TAKAGI, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(motion_velocity_planner, motion_velocity_planner_common): update pointcloud preprocess design (`#591 <https://github.com/autowarefoundation/autoware_core/issues/591>`_)
  * updare pcl preprocess
  ---------
* chore(motion_velocity_planner, behavior_velocity_planner): unifiy module load srv (`#585 <https://github.com/autowarefoundation/autoware_core/issues/585>`_)
  port srv
* refactor(motion_velocity_planner): restrict copy of planner_data  (`#587 <https://github.com/autowarefoundation/autoware_core/issues/587>`_)
  * restrict plannar data copy
  ---------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome, Yuki TAKAGI

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(motion_velocity_planner_node): update pcl coordinate transformation (`#519 <https://github.com/autowarefoundation/autoware_core/issues/519>`_)
  * update pcl coordinate transformation
  * add canTransform check
  ---------
* feat(autoware_motion_velocity_planner)!: only wait for the required subscriptions (`#505 <https://github.com/autowarefoundation/autoware_core/issues/505>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* fix(autoware_motion_velocity_planner): fix deprecated autoware_utils header (`#444 <https://github.com/autowarefoundation/autoware_core/issues/444>`_)
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* feat(autoware_motion_velocity_planner): point-cloud clustering optimization (`#409 <https://github.com/autowarefoundation/autoware_core/issues/409>`_)
  * Core changes for point-cloud maksing and clustering
  * fix
  * style(pre-commit): autofix
  * Update planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/planner_data.hpp
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  * fix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Arjun Jagdish Ram, Masaki Baba, Ryohsuke Mitsudome, Tim Clephas, Yuki TAKAGI, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat: autoware_motion_velocity_planner and autoware_motion_velocity_planner_common to core (`#242 <https://github.com/autowarefoundation/autoware_core/issues/242>`_)
  * feat: modify autoware_universe_utils to autoware_utils
  * feat: remove useless dependecy
  * style(pre-commit): autofix
  * feat: modify autoware_universe_utils to autoware_utils
  * feat: autoware_motion_velocity_planner_node to core
  * style(pre-commit): autofix
  * feat: autoware_motion_velocity_planner_node to core
  * feat: modify autoware_universe_utils to autoware_utils
  * feat: remove useless dependecy
  * style(pre-commit): autofix
  * feat: modify autoware_universe_utils to autoware_utils
  * style(pre-commit): autofix
  * feat: port autoware_behavior_velocity_planner to core
  * fix: apply latest changes from autoware.universe
  * fix: deadlinks in README
  * fix(autoware_motion_velocity_planner): porting autoware_motion_velocity_planner, autoware_motion_velocity_planner, sync with latest universe: v0.2
  * style(pre-commit): autofix
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/node.hpp
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/include/autoware/behavior_velocity_planner_common/utilization/util.hpp
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/src/utilization/util.cpp
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/test/src/test_util.cpp
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/debug.cpp
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/manager.cpp
  * style(pre-commit): autofix
  * Update planning/motion_velocity_planner/autoware_motion_velocity_planner/CMakeLists.txt
  * style(pre-commit): autofix
  * feat(autoware_motion_velocity_planner_node): porting autoware_motion_velocity_planner_node, autoware_motion_velocity_planner_node, remove metrics msgs publish according to pr-10342 under universe repo: v0.5
  * feat(autoware_motion_velocity_planner_common): porting autoware_motion_velocity_planner_common, autoware_motion_velocity_planner_common, port to core repo: v0.0
  * style(pre-commit): autofix
  * move
  * rename
  * fix exec
  * add maintainer
  * style(pre-commit): autofix
  * fix test_depend
  * feat(autoware_motion_velocity_planner_node): porting autoware_motion_velocity_planner_node, autoware_motion_velocity_planner_node, resolve build issue: v0.6
  ---------
  Co-authored-by: suchang <chang.su@autocore.ai>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <ryohsuke.mitsudome@tier4.jp>
  Co-authored-by: liuXinGangChina <lxg19892021@gmail.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: 心刚 <90366790+liuXinGangChina@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, storrrrrrrrm
