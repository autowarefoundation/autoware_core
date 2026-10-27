^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_velocity_smoother
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* test(autoware_velocity_smoother): add a new unit test suite for jerk smoother (`#1468 <https://github.com/autowarefoundation/autoware_core/issues/1468>`_)
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
* fix(autoware_velocity_smoother): use the mode-agnostic ok() so that resampling runs under Agnocast (`#1418 <https://github.com/autowarefoundation/autoware_core/issues/1418>`_)
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
* feat(velocity_smoother): apply agnocast_wrapper::Node to autoware_velocity_smoother (`#1264 <https://github.com/autowarefoundation/autoware_core/issues/1264>`_)
  * feat(autoware_velocity_smoother): apply agnocast_wrapper::Node and migrate polling to agnocast_wrapper::polling:: API
  * refactor: subscriber
  * style(pre-commit): autofix
  * fix: skip test in cmake
  * refatcor: move smoother constructor definitions back to cpp
  * fix: build
  * refactor: address review comments
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Mete Fatih Cırıt, Taekjin LEE, Tran Huu Nhat Huy, Yutaro Kobayashi, github-actions

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* test(autoware_velocity_smoother): cover trajectory_utils pure functions and dedup getTransVector3 (`#1115 <https://github.com/autowarefoundation/autoware_core/issues/1115>`_)
  Add a focused gtest suite for the previously untested pure functions in
  trajectory_utils: the jerk-constrained stop-distance state machine
  (calcStopDistWithJerkConstraints across its TRAPEZOID/TRIANGLE/LINEAR
  branches and the LINEAR negative-time failure path), updateStateWithJerkConstraint
  (single/multi-segment integration and the invalid-profile nullopt path),
  isValidStopDist (in-range, out-of-range, and absolute-margin behavior),
  the constant-jerk velocity profile helper, extractPathAroundIndex,
  calcArclengthArray, calcTrajectoryIntervalDistance, applyMaximumVelocityLimit,
  and calcStopDistance. Assertions pin exact numerical values derived from the
  closed-form kinematics.
  Also replace the local getTransVector3 helper with the equivalent
  autoware_utils_geometry::point_2_tf_vector, removing a byte-for-byte duplicate.
  autoware_utils_geometry is already a dependency, so no package.xml change is
  needed. Behavior-preserving: the node interface launch test still passes.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
* Contributors: Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(trajectory): refactor velocity_smoother (analytical_jerk_constrained_smoother) to experimental trajectory (`#762 <https://github.com/mitsudome-r/autoware_core/issues/762>`_)
  * feat refactor analytical_jerk_constrained_smoother
  * fix curvature and added shield
  * fix: use clamp and added 0 acc for max velocity
  * lint fix
  * fix: update dv_before and use trajectory_base
  * fix review
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* feat(velocity_smoother): migrate velocity_planning_utils and trajectory_utils to use continous Trajectory<TrajectoryPoint> (`#749 <https://github.com/mitsudome-r/autoware_core/issues/749>`_)
  * initial commit, feat: modified trajectory_utils.cpp to continous_traj
  * fix:pre-commit
  * refactor: calcStopVelocityWithConstantJerkAccLimit, added test. refactor: calcVelocityProfileWithConstantJerkAndAccelerationLimit
  * feat: converted calcstopdistance, added searchzerovelocityposition
  * style(pre-commit): autofix
  * remove findzeroposition, make another PR
  * fix: change function name to search_zero_velocity_position
  * removed unused searchvelocityidx in the header
  * fix: missing header
  * fix: remove duplicate using
  * added conditional for velocity build and condition for debug loop
  * style(pre-commit): autofix
  * fixed build
  * fix curvature calculation
  * fix include
  * revert test deletion
  * fix cmake
  * fixed some comments
  * use curvature()
  * fixed calcVelocityProfileWithConstantJerkAndAccelerationLimit
  * remove unecessary boundary workaround for curvature()
  * revert calcVelocityProfileWithConstantJerkAndAccelerationLimit
  * removed unecessary build in calcVelocityProfileWithConstantJerkAndAccelerationLimit
  * fix: extractPathAroundPosition
  * feat: removed the use of build on fallback, merge profile for build
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* fix(autoware_velocity_smoother): fix bugprone-narrowing-conversions warnings (`#919 <https://github.com/mitsudome-r/autoware_core/issues/919>`_)
  * fix(autoware_velocity_smoother): fix bugprone-narrowing-conversions warnings
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* chore: organize maintainer (`#859 <https://github.com/mitsudome-r/autoware_core/issues/859>`_)
* Contributors: Giovanni Muhammad Raditya, NorahXiong, Satoshi OTA, github-actions

1.7.0 (2026-02-14)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(velocity_smoother): remove dead store in jerk_filtered_smoother (`#814 <https://github.com/autowarefoundation/autoware_core/issues/814>`_)
  Remove unnecessary increment of `constr_idx` after the last constraint
  setup. The variable is never read after this point, making it a dead store.
  Detected by Facebook Infer static analyzer (DEAD_STORE).
  Co-authored-by: Claude Opus 4.5 <noreply@anthropic.com>
* Contributors: Ryohsuke Mitsudome, Ryuta Kambe

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore: tf2_ros to hpp headers (`#616 <https://github.com/autowarefoundation/autoware_core/issues/616>`_)
* ci(pre-commit): autoupdate (`#723 <https://github.com/autowarefoundation/autoware_core/issues/723>`_)
  * pre-commit formatting changes
* fix(velocity_smoother): missing unit tests for resampling and invalid subscribed input topic (`#562 <https://github.com/autowarefoundation/autoware_core/issues/562>`_)
  * fix(velocity_smoother): correct input trajectory topic in tests
  * fix(velocity_smoother): add missing resample unit tests
  * style(pre-commit): autofix
  * build test_resample.cpp
  * `#562 <https://github.com/autowarefoundation/autoware_core/issues/562>`_ PR remarks
  * style(pre-commit): autofix
  * use current utils geometry namespace
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* Contributors: Mete Fatih Cırıt, Tim Clephas, github-actions, ralwing

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
* Merge remote-tracking branch 'origin/main' into humble
* feat: change planning output topic name to /planning/trajectory (`#602 <https://github.com/autowarefoundation/autoware_core/issues/602>`_)
  * change planning output topic name to /planning/trajectory
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_mission_planner, velocity_smoother): use transient_local for operation_mode_state (`#598 <https://github.com/autowarefoundation/autoware_core/issues/598>`_)
  subscribe transient_local with transient_local
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* refactor: implement varying lateral acceleration and steering rate threshold in velocity smoother (`#531 <https://github.com/autowarefoundation/autoware_core/issues/531>`_)
  * refactor: implement varying steering rate threshold in velocity smoother
  * feat: implement varying lateral acceleration limit
  * fix  typo in readme
  * fix bugs in unit conversion
  * clean up the obsolete parameter in core planning launch
  ---------
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
* Contributors: Kem (TiankuiXian), Ryohsuke Mitsudome, Yukihiro Saito, Yuxuan Liu

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* fix(velocity_smoother): prevent access when vector is empty (`#438 <https://github.com/autowarefoundation/autoware_core/issues/438>`_)
  add empty check
* fix(autoware_velocity_smoother): fix deprecated autoware_utils header (`#424 <https://github.com/autowarefoundation/autoware_core/issues/424>`_)
  * fix autoware_utils header
  * style(pre-commit): autofix
  * add header for timekeeper
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Masaki Baba, Mitsuhiro Sakamoto, Tim Clephas, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat(autoware_velocity_smoother): port the package from Autoware Universe (`#299 <https://github.com/autowarefoundation/autoware_core/issues/299>`_)
* Contributors: Ryohsuke Mitsudome
