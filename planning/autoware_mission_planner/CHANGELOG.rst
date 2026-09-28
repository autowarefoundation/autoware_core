^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_mission_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_mission_planner): decouple core planning logic from ROS 2 node (`#1400 <https://github.com/autowarefoundation/autoware_core/issues/1400>`_)
  * refactor(autoware_mission_planner): decouple core logic
  * refactor(autoware_mission_planner): move MissionPlannerNode into mission_planner_node.cpp
  * refactor(autoware_mission_planner): rename planner_warning_message to warning_message
  * fix(autoware_mission_planner): guard optional access in MissionPlannerNode
  Fix bugprone-unchecked-optional-access errors reported by clang-tidy CI.
  Dereferences of InitializationCheckResult::waiting_message,
  SetLaneletRouteResult::route/route_marker, and
  SetWaypointRouteResult::route/route_marker were guarded by checks on
  unrelated fields (became_ready, status.success), which clang-tidy's
  dataflow analysis cannot connect to the optionals being dereferenced.
  Guard on the optionals themselves instead.
  * refactor(autoware_mission_planner): simplify check_initialization to return bool
  Return a plain bool from MissionPlanner::check_initialization() instead of
  InitializationCheckResult, and move the waiting-state info log to the node
  side with a unified message.
  * fix(autoware_mission_planner): update composable node plugin name to MissionPlannerNode
  * refactor(autoware_mission_planner): decouple error logging from route validity check
  * refactor(autoware_mission_planner): pass tf2::BufferCore into MissionPlanner
  Move the map-frame transform lookup for set_lanelet_route and
  set_waypoint_route from MissionPlannerNode into MissionPlanner itself,
  following the tf2::BufferCore injection pattern used in
  `autowarefoundation/autoware_universe#13270 <https://github.com/autowarefoundation/autoware_universe/issues/13270>`_. tf2::BufferCore has no ROS
  runtime dependency, so this keeps mission_planner.cpp free of
  rclcpp::Node/Logger while removing the duplicated try/catch lookup
  that previously lived in both service handlers.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
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
* refactor(autoware_mission_planner): create endpoints through NodeAdaptor (`#1327 <https://github.com/autowarefoundation/autoware_core/issues/1327>`_)
* refactor(autoware_mission_planner): remove pluginlib and decouple DefaultPlanner from rclcpp::Node (`#1334 <https://github.com/autowarefoundation/autoware_core/issues/1334>`_)
  * refactor(autoware_mission_planner): remove pluginlib and construct DefaultPlanner directly
  DefaultPlanner was the only implementation of PlannerPlugin and its
  class name was hardcoded, so pluginlib's dynamic class loading added
  runtime overhead without providing runtime plugin selection.
  * refactor(autoware_mission_planner): remove PlannerPlugin abstract base class
  DefaultPlanner is now constructed directly instead of via pluginlib, so
  the PlannerPlugin interface no longer serves runtime polymorphism and
  had no other implementation.
  * refactor(autoware_mission_planner): remove unused initialize overload
  The initialize(node, msg) overload had no callers; fold
  initialize_common back into the single initialize(node).
  * refactor(autoware_mission_planner): extract DefaultPlanner parameter declaration
  Move declare_parameter calls for DefaultPlannerParameters out of
  DefaultPlanner::initialize() and into the MissionPlanner constructor,
  so parameters are passed in rather than declared inside the plugin.
  * refactor(autoware_mission_planner): pass VehicleInfo into DefaultPlanner::initialize
  Construct VehicleInfo in the MissionPlanner constructor and pass it
  into DefaultPlanner::initialize() instead of retrieving it inside the
  plugin.
  * refactor(autoware_mission_planner): move vector map subscription to MissionPlanner
  DefaultPlanner subscribed to ~/input/vector_map on its own, duplicating
  the subscription already held by MissionPlanner and parsing the map
  twice. DefaultPlanner now exposes set_map() and MissionPlanner::on_map
  feeds it the map it already receives.
  * refactor(autoware_mission_planner): move goal footprint marker publishing to MissionPlanner
  DefaultPlanner::is_goal_valid() now returns the computed goal footprint
  instead of publishing it directly, and DefaultPlanner::plan() propagates
  it through a PlanResult. MissionPlanner owns the publisher and performs
  the actual publish, since default_planner.cpp does not carry the
  _node.cpp suffix and should not depend on an rclcpp::Node-owned
  publisher.
  * refactor(autoware_mission_planner): remove debug logging from DefaultPlanner::plan
  * refactor(autoware_mission_planner): move DefaultPlanner warning logs to MissionPlanner
  Return warning messages via PlanResult/GoalValidationResult instead of
  logging directly, since default_planner.cpp must not depend on
  rclcpp::Node/Logger. MissionPlanner now logs the returned message.
  This also removes the now-unused node\_ member and initialize()'s node
  argument.
  * refactor(autoware_mission_planner): replace DefaultPlanner::initialize with constructor
  Pass DefaultPlannerParameters and VehicleInfo through the constructor
  instead of a default constructor followed by a separate initialize()
  call, removing the temporarily-invalid two-phase init state.
  * refactor(autoware_mission_planner): stop using rclcpp::Node in DefaultPlanner test
  Since DefaultPlanner no longer depends on rclcpp::Node, construct
  DefaultPlannerParameters and VehicleInfo directly with values mirroring
  the yaml configs instead of declaring ROS parameters on a node. Also
  replace the gtest fixture with free helper functions and plain TEST
  cases, each constructing its own DefaultPlanner instance.
  * fix(autoware_mission_planner): concat goal validation warning message instead of dropping it
  value_or() replaced the specific warning from is_goal_valid() (e.g. "Goal's
  footprint exceeds lane!") with the generic message when a specific one was
  present, silently dropping the generic hint that used to be logged alongside
  it before the refactor. Concatenate both instead.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(autoware_mission_planner): reduce rclcpp::Node dependency in reroute safety and route state logic (`#1320 <https://github.com/autowarefoundation/autoware_core/issues/1320>`_)
  * refactor(autoware_mission_planner): extract change_state logic
  Move ROS-runtime publishing (stamping and topic publish) out of
  change_state() into an injectable on_change_state\_ callback, so the
  pure state transition logic no longer depends on rclcpp directly.
  state\_ is now RouteState::_state_type instead of the stamped message.
  * refactor(autoware_mission_planner): remove rclcpp::Logger from check_reroute_safety
  Return a RerouteSafetyResult (is_safe + reason) instead of taking a
  logger, so the free function stays independent of rclcpp. The node
  wrapper now logs the reason on failure.
  * refactor(autoware_mission_planner): return RerouteSafetyResult from MissionPlanner::check_reroute_safety
  Move the RCLCPP_ERROR logging out of check_reroute_safety and into its
  callers, so the node method itself only returns the result and the
  service handlers decide how to log and respond to an unsafe reroute.
  * refactor(autoware_mission_planner): rename change_route() to clear_route()
  * refactor(autoware_mission_planner): extract publish_route logic
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(mission_planner): extract tf logic (`#1313 <https://github.com/autowarefoundation/autoware_core/issues/1313>`_)
  * refactor(autoware_mission_planner): extract create_lanelet_route logic
  * refactor(autoware_mission_planner): extract create_waypoint_route logic
  * refactor(autoware_mission_planner): extract transform_pose logic
  Extract the pure pose-transform logic from MissionPlanner::transform_pose
  into a free helper function. The tf_buffer\_ lookup, which depends on the
  node, now happens explicitly at each call site.
  * refactor(autoware_mission_planner): pass transform_to_map into create_lanelet_route/create_waypoint_route
  Move the tf_buffer\_ lookup out of create_lanelet_route and
  create_waypoint_route and pass the resolved TransformStamped in as an
  argument, so both functions only need the transform value, not the tf
  buffer itself.
  * refactor(autoware_mission_planner): extract tf lookup to top of route service callbacks
  * refactor(mission_planner): apply suggestions from code review
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  * fix(autoware_mission_planner): avoid empty catch block flagged by clang-tidy
  bugprone-empty-catch failed CI because the catch blocks in
  on_set_lanelet_route/on_set_waypoint_route only contained a comment.
  Explicitly assign std::nullopt so the check passes.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
  Co-authored-by: Tran Huu Nhat Huy <29034232+TranHuuNhatHuy@users.noreply.github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* refactor(mission_planner): decouple arrival checker from rclcpp::Node (`#1253 <https://github.com/autowarefoundation/autoware_core/issues/1253>`_)
  * refactor(autoware_mission_planner): decouple ArrivalChecker from rclcpp::Node
  * test(autoware_mission_planner): add unit test for ArrivalChecker
  * fix(autoware_mission_planner): avoid unchecked optional access on arrival_checker\_
  Store arrival_checker\_ as a plain ArrivalChecker instead of
  std::optional<ArrivalChecker> since it is always populated in the
  constructor, resolving clang-tidy's bugprone-unchecked-optional-access
  errors on the arrival_checker\_-> call sites.
  * refactor(autoware_mission_planner): extract is_vehicle_stopped as free function
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
  Co-authored-by: Tran Huu Nhat Huy <29034232+TranHuuNhatHuy@users.noreply.github.com>
* refactor(autoware_mission_planner): remove ServiceException to improve readability (`#1233 <https://github.com/autowarefoundation/autoware_core/issues/1233>`_)
  * refactor(autoware_mission_planner): remove unused sync_call and ServiceUnready
  * refactor(autoware_mission_planner): use per-callback try/catch instead of generic exception wrapper
  The generic handle_exception() template implicitly required T to
  implement publish_processing_time(), coupling a supposedly generic
  utility to MissionPlanner's specific interface. Move try/catch and
  stop-watch timing into each service callback instead, and drop
  on_clear_route's throw in favor of a direct early return since its
  only error path is local.
  * refactor(autoware_mission_planner): let transform_pose propagate tf2::TransformException directly
  transform_pose() only translated tf2::TransformException into a
  ServiceException, coupling a TF utility function to the service
  response types. Remove that translation layer and catch
  tf2::TransformException directly in on_set_lanelet_route and
  on_set_waypoint_route instead, building the response status inline.
  service_utils::TransformError() is now unused and removed, along with
  the now-empty service_utils.cpp.
  * refactor(autoware_mission_planner): remove ServiceException, use early return in route callbacks
  Every ServiceException throw/catch pair was local to the throwing
  callback, so the exception added no value over a plain early return.
  Replace all throw sites in on_set_lanelet_route and
  on_set_waypoint_route with direct res->status assignment and return,
  matching on_clear_route. Only tf2::TransformException from
  create_route() still needs try/catch, since it crosses a function
  boundary; it now sets the response status directly instead of going
  through ServiceException. The ServiceException class and the now-
  unused ResponseStatusCode alias are removed from service_utils.hpp.
  * refactor(autoware_mission_planner): remove service_utils.hpp
  It only held a ResponseStatus alias used at 2 call sites. Reference
  autoware_common_msgs::msg::ResponseStatus directly instead and drop
  the now-empty service_utils.hpp/namespace.
  * refactor(autoware_mission_planner): use RAII guard for processing time publishing
  Replace repeated publish_processing_time(stop_watch) calls before every
  early return with a ScopedProcessingTimePublisher that publishes on
  destruction, removing duplication and preventing missed calls on new
  return paths.
  * refactor(autoware_mission_planner): extract set_fail_response helper for route responses
  Deduplicate the repeated success/code/message triples in on_set_lanelet_route and on_set_waypoint_route.
  * refactor(autoware_mission_planner): extract set_success_response helper
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* test(mission_planner): add characterization test (`#1234 <https://github.com/autowarefoundation/autoware_core/issues/1234>`_)
  * test(mission_planner): add characterization test
  * test(mission_planner):simplify test case
  * test(mission_planner): simplify test case
  * test(mission_planner): simplify map for test
  * refactor(autoware_mission_planner): simplify test helpers to accept Pose/id args directly
  Have call_set_lanelet_route(), call_set_waypoint_route(), publish_odometry(),
  and publish_autonomous_operation_mode_state() build their own request/message
  internally instead of requiring callers to pre-build them, removing repeated
  construction boilerplate at each call site.
  * test(mission_planner): extract expect_success/expect_failure helpers
  * test(mission_planner): add comment to describe test case condition
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* Contributors: Mete Fatih Cırıt, Takahisa Ishikawa, Yutaka Kondo, github-actions

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_mission_planner): extract pure check_reroute_safety free function (`#1113 <https://github.com/autowarefoundation/autoware_core/issues/1113>`_)
  * refactor(autoware_mission_planner): extract pure check_reroute_safety free function
  Extract the reroute-safety algorithm out of the MissionPlanner node method into a
  pure, dependency-injected free function declared in reroute_safety.hpp (route + lanelet
  map + scalars in, bool out). The node method now forwards to it after validating its own
  odometry / map members, so there is no public-API change (the method stays private).
  The two byte-for-byte identical start-segment distance branches (start_idx_target != 0 &&
  start_idx_original > 1 vs else) are collapsed into a single arc_length_to_lanelet_end
  helper that is parameterized only by which original-route segment supplies the primitives.
  Add table-driven unit tests over synthetic routes / lanelets covering every early-return
  branch (empty routes, null map, stopped-vehicle short-circuit, no common segment, ego not
  on first target section) and the final velocity-scaled safety-length comparison (safe /
  unsafe / velocity dependence). This is a behavior-preserving, ABI-neutral testability win
  on the package's most complex previously untested algorithm.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * test(autoware_mission_planner): pin start-segment -1 branch and empty-segment break (`#69 <https://github.com/autowarefoundation/autoware_core/issues/69>`_)
  Add a fixture case where the target route starts mid-original-route so start_idx_target != 0 && start_idx_original > 1, exercising the previously-unhit start_idx_original - 1 selector branch of check_reroute_safety; verified RED against a regression that drops the -1. Also pin the empty-primitives accumulation break.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  ---------
* feat(autoware_vehicle_info_utils): add base_pose to createFootprint (`#1072 <https://github.com/autowarefoundation/autoware_core/issues/1072>`_)
* Contributors: Sarun MUKDAPITAK, Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(mission_planner): unused using-declaration with GCC 15 (`#1000 <https://github.com/mitsudome-r/autoware_core/issues/1000>`_)
* chore(planning, bvp): remove unused lanelet2_extension header (`#902 <https://github.com/mitsudome-r/autoware_core/issues/902>`_)
  * remove unused lanelet2_extension in bvp modules
  * remove unused lanelet2_extension in planning components
  ---------
* feat(autoware_mission_planner): remove glog component (`#879 <https://github.com/mitsudome-r/autoware_core/issues/879>`_)
  feat: remove glog component
* fix(lanelet2_utils): change is_in_lanelet argument order (`#890 <https://github.com/mitsudome-r/autoware_core/issues/890>`_)
* feat(lanelet2_extension): port lanelet2_extension utilities functions (final)  (`#838 <https://github.com/mitsudome-r/autoware_core/issues/838>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* chore: organize maintainer (`#860 <https://github.com/mitsudome-r/autoware_core/issues/860>`_)
  * chore: organize maintainer
  * chore: organize maintainer
  ---------
* Contributors: Guilhem Saurel, Sarun MUKDAPITAK, Satoshi OTA, Tetsuhiro Kawaguchi, github-actions

1.7.0 (2026-02-14)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* refactor(planning): deprecate lanelet_extension geometry conversion function (`#834 <https://github.com/autowarefoundation/autoware_core/issues/834>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* refactor(planning, common): replace lanelet2_extension function (`#796 <https://github.com/autowarefoundation/autoware_core/issues/796>`_)
* Contributors: Mamoru Sobue, Ryohsuke Mitsudome

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_lanelet2_utils): replace from/toBinMsg (`#737 <https://github.com/autowarefoundation/autoware_core/issues/737>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* feat(autoware_lanelet2_utils): define remove_const in header (`#741 <https://github.com/autowarefoundation/autoware_core/issues/741>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* refactor(vehicle_info_utils): reduce autoware_utils deps (`#754 <https://github.com/autowarefoundation/autoware_core/issues/754>`_)
* chore: tf2_ros to hpp headers (`#616 <https://github.com/autowarefoundation/autoware_core/issues/616>`_)
* Contributors: Mete Fatih Cırıt, Sarun MUKDAPITAK, Tim Clephas, github-actions

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_lanelet2_utils): replace ported functions from autoware_lanelet2_extension (`#695 <https://github.com/autowarefoundation/autoware_core/issues/695>`_)
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: update maintainer (`#701 <https://github.com/autowarefoundation/autoware_core/issues/701>`_)
* feat(autoware_lanelet2_utils): porting functions from lanelet2_extension to autoware_lanelet2_utils package (`#621 <https://github.com/autowarefoundation/autoware_core/issues/621>`_)
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Sarun MUKDAPITAK, Takagi, Isamu, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(autoware_mission_planner, velocity_smoother): use transient_local for operation_mode_state (`#598 <https://github.com/autowarefoundation/autoware_core/issues/598>`_)
  subscribe transient_local with transient_local
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Kem (TiankuiXian), Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat: use component_interface_specs for mission_planner (`#546 <https://github.com/autowarefoundation/autoware_core/issues/546>`_)
* fix(mission_planner): fix check if goal footprint is inside route (`#534 <https://github.com/autowarefoundation/autoware_core/issues/534>`_)
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* fix(autoware_mission_planner): fix deprecated autoware_utils header (`#421 <https://github.com/autowarefoundation/autoware_core/issues/421>`_)
  * fix autoware_utils header
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Kosuke Takeuchi, Masaki Baba, Ryohsuke Mitsudome, Tim Clephas, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat: port simplified version of autoware_mission_planner from Autoware Universe  (`#329 <https://github.com/autowarefoundation/autoware_core/issues/329>`_)
  * feat: port autoware_mission_planner from Autoware Universe
  * chore: reset package version and remove CHANGELOG
  * chore: remove the _universe suffix from autoware_mission_planner
  * feat: repalce tier4_planning_msgs with autoware_internal_planning_msgs
  * feat: remove route_selector module
  * feat: remove reroute_availability and modified_goal subscription
  * remove unnecessary image
  * style(pre-commit): autofix
  * fix: remove unnecessary include file
  * fix: resolve useInitializationList error from cppcheck
  * Apply suggestions from code review
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Ryohsuke Mitsudome
