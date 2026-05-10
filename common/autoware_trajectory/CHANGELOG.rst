^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* fix(autoware_trajectory): avoid nan in align_orientation_with_trajectory_direction (`#398 <https://github.com/autowarefoundation/autoware_core/issues/398>`_)
  * fix(autoware_trajectory): avoid nan in align_orientation_with_trajectory_direction
  * tidy
  * remove eps
  ---------
* chore(autoware_trajectory): relax the warning condition of boundary check (`#393 <https://github.com/autowarefoundation/autoware_core/issues/393>`_)
  * chore(autoware_trajectory): relax the warning condition of boundary check
  * tidy
  ---------
* feat(trajectory): define distance threshold and refine restore() without API breakage(in experimental) (`#376 <https://github.com/autowarefoundation/autoware_core/issues/376>`_)
  * feat(trajectory): define distance threshold and refine restore
  * fix spell
  ---------
* feat(autoware_trajectory): add get_contained_lane_ids function (`#369 <https://github.com/autowarefoundation/autoware_core/issues/369>`_)
  * add get_contained_lane_ids
  * add unit test
  * remove assert
  ---------
* feat(trajectory): add pretty_build() function for Planning/Control component node (`#332 <https://github.com/autowarefoundation/autoware_core/issues/332>`_)
* refactor(autoware_trajectory)!: move everything to namespace experimetal (`#371 <https://github.com/autowarefoundation/autoware_core/issues/371>`_)
  refactor(autoware_trajectory)!: move everything to namespace experimental
* feat(trajectory): improve shift function and their documents (`#337 <https://github.com/autowarefoundation/autoware_core/issues/337>`_)
  * feat(trajectory): add populate function
  * update curvature figure for approximation desc
  * update align_orientation_with_trajectory_direction fig
  * finished trajectory classes
  * refactored shift
  * add comment
  * update error message
  ---------
* fix(autoware_trajectory): fix base_addition callback to work when Trajectory is moved (`#370 <https://github.com/autowarefoundation/autoware_core/issues/370>`_)
* fix(autoware_trajectory): check vector size check before accessing (`#365 <https://github.com/autowarefoundation/autoware_core/issues/365>`_)
  * fix(autoware_trajectory): check vector size check before accessing
  * update
  * minor fix
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* feat(autoware_trajectory): improve performance of get_underlying_base  (`#298 <https://github.com/autowarefoundation/autoware_core/issues/298>`_)
* feat(trajectory): add API documentation for trajectory class, add some utillity (`#295 <https://github.com/autowarefoundation/autoware_core/issues/295>`_)
* feat(trajectory): add API description, nomenclature, illustration, rename functions to align with nomenclature (`#292 <https://github.com/autowarefoundation/autoware_core/issues/292>`_)
  * feat(trajectory): add API description, nomenclature, illustration, rename functions to align with nomenclature
  * resurrect get_internal_base
  ---------
* chore: include iostream and link yaml-cpp for Jazzy (`#351 <https://github.com/autowarefoundation/autoware_core/issues/351>`_)
* Contributors: Mamoru Sobue, Tim Clephas, Yukinari Hisaki

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_trajectory): add max utilities (`#1051 <https://github.com/mitsudome-r/autoware_core/issues/1051>`_)
  * feat(autoware_trajectory): add max utilities
  * feat(autoware_trajectory): refactor max_at_bases to use compute_point and simplify logic
  ---------
* feat(autoware_trajectory): add param constraints (`#1047 <https://github.com/mitsudome-r/autoware_core/issues/1047>`_)
  * feat(autoware_trajectory): add azimuth, elevation, and curvature computation methods
  * feat(autoware_trajectory): add param constraints
  * feat(autoware_trajectory): add test for finding high curvature intervals
  ---------
* feat(autoware_trajectory): add azimuth, elevation, and curvature computation methods (`#1048 <https://github.com/mitsudome-r/autoware_core/issues/1048>`_)
* feat(autoware_trajectory): add align_orientation option to pretty_build and remove auto execution from constructor (`#1049 <https://github.com/mitsudome-r/autoware_core/issues/1049>`_)
  * feat(autoware_trajectory): add align_orientation option to pretty_build and remove auto execution from constructor
  * feat(autoware_trajectory): add align_orientation utility and integrate with pretty_build
  * Revert "feat(autoware_trajectory): add align_orientation option to pretty_build and remove auto execution from constructor"
  This reverts commit 37f2796992e17576c7a47297c8d182b793098ba1.
  * refactor(autoware_trajectory): remove unused align_orientation include from pretty_build
  ---------
* docs(autoware_trajectory): update README with TemporalTrajectory class and duplicate handling details (`#1045 <https://github.com/mitsudome-r/autoware_core/issues/1045>`_)
  * feat(autoware_trajectory): control test plotting via ENABLE_TEST_PLOT env var
  * refactor(autoware_trajectory): introduce granular epsilon constants and parameterize interpolators
  * feat(autoware_trajectory): add PCHIP interpolation implementation and example
  * refactor(autoware_trajectory): simplify crossed_with_polygon function by removing unused parameters and adjusting test cases
  * feat(autoware_trajectory): add single-point value assignment to InterpolatedArray
  * feat(autoware_trajectory): add TemporalTrajectory class
  * feat(autoware_trajectory): add temporal trajectory utilities
  * docs(autoware_trajectory): update README with TemporalTrajectory class and duplicate handling details
  ---------
* feat(autoware_trajectory): add temporal trajectory utilities (`#1044 <https://github.com/mitsudome-r/autoware_core/issues/1044>`_)
  * feat(autoware_trajectory): control test plotting via ENABLE_TEST_PLOT env var
  * refactor(autoware_trajectory): introduce granular epsilon constants and parameterize interpolators
  * feat(autoware_trajectory): add PCHIP interpolation implementation and example
  * refactor(autoware_trajectory): simplify crossed_with_polygon function by removing unused parameters and adjusting test cases
  * feat(autoware_trajectory): add single-point value assignment to InterpolatedArray
  * feat(autoware_trajectory): add TemporalTrajectory class
  * feat(autoware_trajectory): add temporal trajectory utilities
  ---------
* feat(autoware_trajectory): add TemporalTrajectory class (`#1043 <https://github.com/mitsudome-r/autoware_core/issues/1043>`_)
  * feat(autoware_trajectory): control test plotting via ENABLE_TEST_PLOT env var
  * refactor(autoware_trajectory): introduce granular epsilon constants and parameterize interpolators
  * feat(autoware_trajectory): add PCHIP interpolation implementation and example
  * refactor(autoware_trajectory): simplify crossed_with_polygon function by removing unused parameters and adjusting test cases
  * feat(autoware_trajectory): add single-point value assignment to InterpolatedArray
  * feat(autoware_trajectory): add TemporalTrajectory class
  ---------
* feat(autoware_trajectory): add single-point value assignment to InterpolatedArray (`#1042 <https://github.com/mitsudome-r/autoware_core/issues/1042>`_)
  * feat(autoware_trajectory): control test plotting via ENABLE_TEST_PLOT env var
  * refactor(autoware_trajectory): introduce granular epsilon constants and parameterize interpolators
  * feat(autoware_trajectory): add PCHIP interpolation implementation and example
  * refactor(autoware_trajectory): simplify crossed_with_polygon function by removing unused parameters and adjusting test cases
  * feat(autoware_trajectory): add single-point value assignment to InterpolatedArray
  ---------
* refactor(autoware_trajectory): simplify crossed_with_polygon function by removing unused parameters and adjusting test cases (`#1041 <https://github.com/mitsudome-r/autoware_core/issues/1041>`_)
  * feat(autoware_trajectory): control test plotting via ENABLE_TEST_PLOT env var
  * refactor(autoware_trajectory): introduce granular epsilon constants and parameterize interpolators
  * feat(autoware_trajectory): add PCHIP interpolation implementation and example
  * refactor(autoware_trajectory): simplify crossed_with_polygon function by removing unused parameters and adjusting test cases
  ---------
* feat(autoware_trajectory): add PCHIP interpolation implementation and example (`#1040 <https://github.com/mitsudome-r/autoware_core/issues/1040>`_)
  * feat(autoware_trajectory): control test plotting via ENABLE_TEST_PLOT env var
  * refactor(autoware_trajectory): introduce granular epsilon constants and parameterize interpolators
  * feat(autoware_trajectory): add PCHIP interpolation implementation and example
  * fix(autoware_trajectory): include missing iostream header for input/output operations
  ---------
* refactor(autoware_trajectory): introduce granular epsilon constants and parameterize interpolators (`#1039 <https://github.com/mitsudome-r/autoware_core/issues/1039>`_)
  * feat(autoware_trajectory): control test plotting via ENABLE_TEST_PLOT env var
  * refactor(autoware_trajectory): introduce granular epsilon constants and parameterize interpolators
  ---------
* feat(autoware_trajectory): control test plotting via ENABLE_TEST_PLOT env var (`#1038 <https://github.com/mitsudome-r/autoware_core/issues/1038>`_)
* test(autoware_trajectory): rename gtest cases to PascalCase (`#1024 <https://github.com/mitsudome-r/autoware_core/issues/1024>`_)
  Normalize test suite and test names across trajectory test files to use
  PascalCase naming.
  This keeps naming consistent with project conventions and improves test
  readability in gtest output without changing test behavior.
* refactor(autoware_trajectory): replace unreachable trajectory throws with asserts (`#1020 <https://github.com/mitsudome-r/autoware_core/issues/1020>`_)
  Swap defensive runtime exceptions for `assert` checks in trajectory
  interpolation, closest-point lookup, and pose orientation setup where
  failure is considered impossible under documented preconditions.
  Also update interpolator interface comments to describe required build
  state and clamped out-of-range behavior more explicitly.
* feat(autoware_trajectory): add offset trajectory utility (`#1013 <https://github.com/mitsudome-r/autoware_core/issues/1013>`_)
  * feat(autoware_trajectory): add offset trajectory utility
  * test(autoware_trajectory): add test for single point combined offset calculation
  * feat(autoware_trajectory): support 3d offset using full pose orientation
  Apply trajectory offsets with full quaternion rotation instead of yaw
  only so roll, pitch, and vertical offsets are respected. Update the
  utility documentation and extend tests to cover pitched and fully
  oriented trajectory points.
  * test(autoware_trajectory): consolidate offset trajectory test fixtures
  Simplify the offset test helpers by using a single trajectory point
  factory that covers position and full orientation inputs.
  This keeps the test setup consistent across yaw-only and 3D pose cases
  while preserving coverage for combined offset behavior.
  * docs(autoware_trajectory): simplify offset example trajectory setup
  Trim the example to focus on generating and visualizing offset
  trajectories from position-only points.
  Use trajectory direction alignment to derive orientations instead of
  manually assigning yaw and remove unused velocity diagnostics to keep
  the sample easier to follow.
  * docs(autoware_trajectory): add add_offset image to README
  * fix(autoware_trajectory): assert offset rebuild succeeds
  Replace the unused build result suppression with a runtime assertion on
  `build()` success when reconstructing the offset trajectory.
  This makes the invariant explicit and catches unexpected failures in a
  code path that should always preserve valid point ordering and count.
  * docs(autoware_trajectory): embed add_offset example in README
  Replace the inline `add_offset` usage snippet with a sourced excerpt from
  `example_add_offset.cpp` so the documentation stays aligned with the
  maintained example code.
  * chore(autoware_trajectory): harden add_offset example execution
  Wrap the `example_add_offset` demo in exception handling so failures from
  the embedded Python plotting setup are surfaced more cleanly.
  Update the README snippet reference to match the revised example source
  location.
  ---------
* refactor(autoware_trajectory): add effective_lanelet_id helper to ReferencePoint (`#1010 <https://github.com/mitsudome-r/autoware_core/issues/1010>`_)
* refactor(autoware_trajectory): fix clang-tidy warnings in footprint utils (`#1006 <https://github.com/mitsudome-r/autoware_core/issues/1006>`_)
  * refactor(autoware_trajectory): rename binary search functions for clarity
  * refactor(autoware_trajectory): fix clang-tidy warnings in footprint utils
  ---------
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* fix(autoware_trajectory): add Takumi Odashima as a maintainer (`#1007 <https://github.com/mitsudome-r/autoware_core/issues/1007>`_)
* refactor(autoware_trajectory): rename binary search functions for clarity (`#1005 <https://github.com/mitsudome-r/autoware_core/issues/1005>`_)
  * refactor(autoware_trajectory): rename binary search functions for clarity
  * docs(autoware_trajectory): add max_iter parameter documentation
  * fix(autoware_trajectory): simplify termination condition in binary search functions
  ---------
* feat(autoware_trajectory): handle sparse points with interpolator fallback (`#994 <https://github.com/mitsudome-r/autoware_core/issues/994>`_)
  * feat(trajectory): handle sparse points with interpolator fallback
  * feat(autoware_trajectory): skip orientation alignment when insufficient points are available
  * feat(autoware_trajectory): add minimum distance threshold check for trajectory building
  * refactor(autoware_trajectory): reorganize and rename test cases for clarity and consistency
  * fix(autoware_trajectory): cover restore for degenerate trajectories
  * refactor(autoware_trajectory): simplify restore output
  * fix(autoware_path_generator): align degenerate trajectory handling
  * feat(autoware_trajectory): enhance fallback interpolator functionality and improve documentation
  * refactor(trajectory): replace array indexing with at() for safer access
  * feat(autoware_trajectory): add error handling for empty bases in start, end, and range methods
  * fix(autoware_trajectory): guard degenerate interpolation math
  * refactor(autoware_trajectory): simplify point restoration logic and improve test clarity
  * refactor(autoware_trajectory): remove unused includes and improve code clarity
  * chore: add missing includes for string and utility in example and test files
  * fix(autoware_trajectory): initialize point coordinates in buildCroppedTrajectory test
  * revert shift changes
  * fix
  ---------
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
* refactor(autoware_trajectory): clean up test target dependencies (`#990 <https://github.com/mitsudome-r/autoware_core/issues/990>`_)
  * fix(autoware_trajectory): clean up test target dependencies
  * refactor(autoware_trajectory): remove URL
  * fix(autoware_trajectory): add temporary fix to build with jazzy
  ---------
* fix(autoware_trajectory): fix bugprone-narrowing-conversions warnings (`#920 <https://github.com/mitsudome-r/autoware_core/issues/920>`_)
* fix(autoware_trajectory): wrap examples into BUILD_TESTING in `CMakeLists.txt` (`#885 <https://github.com/mitsudome-r/autoware_core/issues/885>`_)
  * wrap examples into build_testing
  * add 'BUILD_TESTING' for clarity
  * Revert "add 'BUILD_TESTING' for clarity"
  This reverts commit 73c1deb70089f43fc731ad92c69f873524a738a7.
  ---------
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* feat(autoware_trajectory): define set stopline in Trajectory class (`#806 <https://github.com/mitsudome-r/autoware_core/issues/806>`_)
* feat(trajectory): add util function that returns first index satisfying constraint (`#881 <https://github.com/mitsudome-r/autoware_core/issues/881>`_)
* build(autoware_trajectory): add missing include (`#853 <https://github.com/mitsudome-r/autoware_core/issues/853>`_)
  fix(autoware_trajectory): add missing include
  std::clamp() added in `#791 <https://github.com/mitsudome-r/autoware_core/issues/791>`_ needs to include <algorithm>. Without it,
  GCC 14 complains with "error: 'clamp' is not a member of 'std'".
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* feat(autoware_lanelet2_extension): replace remaining lanelet2_extension utilities functions (`#842 <https://github.com/mitsudome-r/autoware_core/issues/842>`_)
  * replace getArcCoordinates usage
  * replace combineLaneletsShape
  * remove null return for get_dirty_expanded_lanelet
  * change get_dirty_expanded_lanelet(s) positive right_offset (and negative left_offset) handler
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* Contributors: Giovanni Muhammad Raditya, Mete Fatih Cırıt, Michal Sojka, Mitsuhiro Sakamoto, NorahXiong, Sarun MUKDAPITAK, Taeseung Sohn, Yukinari Hisaki, github-actions

1.7.0 (2026-02-14)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* refactor(planning): deprecate lanelet_extension geometry conversion function (`#834 <https://github.com/autowarefoundation/autoware_core/issues/834>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* chore(autoware_motion_utils, autoware_trajectory, autoware_interpolation): add maintainers for packages (`#821 <https://github.com/autowarefoundation/autoware_core/issues/821>`_)
  * add maintainers for autoware_interpolation
  * add maintainers for autoware_motion_utils
  * add maintainers for autoware_trajectory
  ---------
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* chore(autoware_trajectory): add a maintainer (`#819 <https://github.com/autowarefoundation/autoware_core/issues/819>`_)
* docs(autoware_trajectory): fix anker link for pre-commit (`#816 <https://github.com/autowarefoundation/autoware_core/issues/816>`_)
* fix(trajectory): fix potential undefined behavior in closest() (`#815 <https://github.com/autowarefoundation/autoware_core/issues/815>`_)
  Remove premature dereference of std::optional before checking if it
  has a value. The original code dereferenced the result of
  closest_with_constraint() before assigning to s, which could cause
  undefined behavior if the function returns std::nullopt.
  Detected by Facebook Infer static analyzer (OPTIONAL_EMPTY_ACCESS).
  Co-authored-by: Claude Opus 4.5 <noreply@anthropic.com>
* fix(trajectory): add at least one prev/next lanelet for generating reference_path (`#810 <https://github.com/autowarefoundation/autoware_core/issues/810>`_)
* feat(trajectory): add function to clamp `InterpolatedArray` to maximum value (`#791 <https://github.com/autowarefoundation/autoware_core/issues/791>`_)
* Contributors: Junya Sasaki, Mamoru Sobue, Mitsuhiro Sakamoto, Ryohsuke Mitsudome, Ryuta Kambe, Takagi, Isamu, mkquda

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(trajectory): added find zero velocity position for continuous trajectory (`#759 <https://github.com/autowarefoundation/autoware_core/issues/759>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* fix(autoware_trajectory): fix missing dependencies (`#734 <https://github.com/autowarefoundation/autoware_core/issues/734>`_)
* fix(autoware_trajectory): make the get_index function safe (`#568 <https://github.com/autowarefoundation/autoware_core/issues/568>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* feat(trajectory): align build_reference_path with vm-01-11 spec (`#702 <https://github.com/autowarefoundation/autoware_core/issues/702>`_)
  Co-authored-by: mitukou1109 <mitukou1109@gmail.com>
* chore: jazzy-porting, autoware_trajectory, add array size assignment to solve compile failure (`#644 <https://github.com/autowarefoundation/autoware_core/issues/644>`_)
  * build::jazzy-porting::add array size assignment to sovle compile failures with array bounds checking warnings , v0.0
  * build::jazzy-porting::use warning surpress macro for false positives of array bounds warnings, v0.1
  ---------
* fix(trajectory): copy lane id interpolator array in copy assignment (`#724 <https://github.com/autowarefoundation/autoware_core/issues/724>`_)
* ci(pre-commit): autoupdate (`#723 <https://github.com/autowarefoundation/autoware_core/issues/723>`_)
  * pre-commit formatting changes
* docs(trajectory): reorganize autoware_trajectory documentation (`#715 <https://github.com/autowarefoundation/autoware_core/issues/715>`_)
* Contributors: Giovanni Muhammad Raditya, Mamoru Sobue, Mete Fatih Cırıt, Mitsuhiro Sakamoto, Ryohsuke Mitsudome, Sarun MUKDAPITAK, Yukinari Hisaki, github-actions, 心刚

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(lanelet2_utils): organize maps by vm-map-spec id (`#716 <https://github.com/autowarefoundation/autoware_core/issues/716>`_)
* docs(autoware_trajectory): add crossed documentation (`#705 <https://github.com/autowarefoundation/autoware_core/issues/705>`_)
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(trajectory): improve crossed() to handle range, add crossed_with_polygon (`#685 <https://github.com/autowarefoundation/autoware_core/issues/685>`_)
* docs(autoware_trajectory): update API table of autoware_trajectory document (`#687 <https://github.com/autowarefoundation/autoware_core/issues/687>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_trajectory): add footprint along the trajectory (`#678 <https://github.com/autowarefoundation/autoware_core/issues/678>`_)
* feat(autoware_trajectory): add lateral_distance computation (`#655 <https://github.com/autowarefoundation/autoware_core/issues/655>`_)
* docs(autoware_trajectory): fix drawio link of self-intersecting images (`#659 <https://github.com/autowarefoundation/autoware_core/issues/659>`_)
  fix broken drawio link
* chore: jazzy-porting:fix missing header file (`#630 <https://github.com/autowarefoundation/autoware_core/issues/630>`_)
* feat(autoware_trajectory): add more test, verify implementation for find_nearest_index in autoware_trajectory packages (`#578 <https://github.com/autowarefoundation/autoware_core/issues/578>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mamoru Sobue, Mete Fatih Cırıt, Sarun MUKDAPITAK, Yutaka Kondo, mitsudome-r, 心刚

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* feat(autoware_trajectory): make `supplement_lanelet_sequence()` public (`#560 <https://github.com/autowarefoundation/autoware_core/issues/560>`_)
  * make supplement_lanelet_sequence() public
  * fix to update new end arc length
  * extend lanelet sequence after loop detection
  * include necessary header
  * use struct instead of tuple
  ---------
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* feat(autoware_trajectory): porting findNearestIndex from motion_utils to autoware_trajectory package (`#507 <https://github.com/autowarefoundation/autoware_core/issues/507>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* Contributors: Giovanni Muhammad Raditya, Mitsuhiro Sakamoto, Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(autoware_trajectory): add specialized LaneIdsInterpolator for lane ID handling (`#528 <https://github.com/autowarefoundation/autoware_core/issues/528>`_)
  * feat(autoware_trajectory): add specialized LaneIdsInterpolator for lane ID handling
  Replace generic Stairstep interpolator with domain-specific LaneIdsInterpolator
  that implements lane ID interpolation logic with preference for single lane IDs.
  - Add LaneIdsInterpolator implementation
  - Update path_point_with_lane_id to use specialized interpolator
  - Add comprehensive tests for lane ID interpolation behavior
  - Includes domain knowledge for handling lane transitions
  * Update common/autoware_trajectory/test/test_interpolator.cpp
  * Update common/autoware_trajectory/test/test_interpolator.cpp
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* feat(autoware_trajectory): implement a function to construct trajectory class for reference path (`#469 <https://github.com/autowarefoundation/autoware_core/issues/469>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
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
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* feat(autoware_trajectory): enhance shift flexibility (`#456 <https://github.com/autowarefoundation/autoware_core/issues/456>`_)
  * feat(autoware_trajectory): enhance shift flexibility
  * add test
  * fix spell miss
  * remove unused
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* chore(autoware_trajectory): update includes to maintain backward compatibility (`#455 <https://github.com/autowarefoundation/autoware_core/issues/455>`_)
* feat(autoware_trajectory): enable downcast from Trajectory<Point> to Trajectory<Pose> (`#430 <https://github.com/autowarefoundation/autoware_core/issues/430>`_)
  * feat(autoware_trajectory): enable downcast from Trajectory<Point> to Trajectory<Pose>
  * add test
  * fix test
  ---------
* chore(autoware_trajectory): fix CMake to work with differential build (`#420 <https://github.com/autowarefoundation/autoware_core/issues/420>`_)
* feat(autoware_trajectory): improve accuracy of boundary by using binary search (`#391 <https://github.com/autowarefoundation/autoware_core/issues/391>`_)
  * feat(autoware_trajectory): improve accuracy of boundary by using binary search
  * add example
  * Update common/autoware_trajectory/src/utils/find_intervals.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * Update common/autoware_trajectory/src/utils/find_intervals.cpp
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  * define macro for avoiding compiler error
  * add unit test
  * use flag instead of optional
  * add missing include
  ---------
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix(autoware_trajectory): avoid nan in align_orientation_with_trajectory_direction (`#398 <https://github.com/autowarefoundation/autoware_core/issues/398>`_)
  * fix(autoware_trajectory): avoid nan in align_orientation_with_trajectory_direction
  * tidy
  * remove eps
  ---------
* chore(autoware_trajectory): relax the warning condition of boundary check (`#393 <https://github.com/autowarefoundation/autoware_core/issues/393>`_)
  * chore(autoware_trajectory): relax the warning condition of boundary check
  * tidy
  ---------
* feat(trajectory): define distance threshold and refine restore() without API breakage(in experimental) (`#376 <https://github.com/autowarefoundation/autoware_core/issues/376>`_)
  * feat(trajectory): define distance threshold and refine restore
  * fix spell
  ---------
* feat(autoware_trajectory): add get_contained_lane_ids function (`#369 <https://github.com/autowarefoundation/autoware_core/issues/369>`_)
  * add get_contained_lane_ids
  * add unit test
  * remove assert
  ---------
* feat(trajectory): add pretty_build() function for Planning/Control component node (`#332 <https://github.com/autowarefoundation/autoware_core/issues/332>`_)
* refactor(autoware_trajectory)!: move everything to namespace experimetal (`#371 <https://github.com/autowarefoundation/autoware_core/issues/371>`_)
  refactor(autoware_trajectory)!: move everything to namespace experimental
* feat(trajectory): improve shift function and their documents (`#337 <https://github.com/autowarefoundation/autoware_core/issues/337>`_)
  * feat(trajectory): add populate function
  * update curvature figure for approximation desc
  * update align_orientation_with_trajectory_direction fig
  * finished trajectory classes
  * refactored shift
  * add comment
  * update error message
  ---------
* fix(autoware_trajectory): fix base_addition callback to work when Trajectory is moved (`#370 <https://github.com/autowarefoundation/autoware_core/issues/370>`_)
* fix(autoware_trajectory): check vector size check before accessing (`#365 <https://github.com/autowarefoundation/autoware_core/issues/365>`_)
  * fix(autoware_trajectory): check vector size check before accessing
  * update
  * minor fix
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* feat(autoware_trajectory): improve performance of get_underlying_base  (`#298 <https://github.com/autowarefoundation/autoware_core/issues/298>`_)
* feat(trajectory): add API documentation for trajectory class, add some utillity (`#295 <https://github.com/autowarefoundation/autoware_core/issues/295>`_)
* feat(trajectory): add API description, nomenclature, illustration, rename functions to align with nomenclature (`#292 <https://github.com/autowarefoundation/autoware_core/issues/292>`_)
  * feat(trajectory): add API description, nomenclature, illustration, rename functions to align with nomenclature
  * resurrect get_internal_base
  ---------
* chore: include iostream and link yaml-cpp for Jazzy (`#351 <https://github.com/autowarefoundation/autoware_core/issues/351>`_)
* Contributors: Mamoru Sobue, Tim Clephas, Yukinari Hisaki, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* feat(trajectory): remove default ctor and collect default setting in Builder (`#287 <https://github.com/autowarefoundation/autoware_core/issues/287>`_)
* fix(autoware_trajectory): fix linking issue with pybind11, and use non-deprecated tf2 headers (`#316 <https://github.com/autowarefoundation/autoware_core/issues/316>`_)
  * Fix linking issue with pybind11, and use non-deprecated tf2 headers
  * Use .hpp includes only
  * style(pre-commit): autofix
  * Remove redundant find_package(pybind11_vendor ...)
  * Undo whitespace change
  * Make pybind11 a test_depend
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Mamoru Sobue, Shane Loretz

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* feat(trajectory): improve comment, use autoware_pyplot for examples (`#282 <https://github.com/autowarefoundation/autoware.core/issues/282>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* feat(autoware_trajectory): use move semantics and return expected<T, E> for propagating failure reason (`#254 <https://github.com/autowarefoundation/autoware.core/issues/254>`_)
  Co-authored-by: Yukinari Hisaki <42021302+yhisaki@users.noreply.github.com>
* refactor(autoware_trajectory): use nodiscard for mutables, fix reference to scalar type (`#255 <https://github.com/autowarefoundation/autoware.core/issues/255>`_)
  * doc(lanelet2_utils): fix invalid drawio link and update image
  * fix
  * fix precommit errors
  ---------
  Co-authored-by: Y.Hisaki <yhisaki31@gmail.com>
* feat(autoware_trajectory): add trajectory point (`#233 <https://github.com/autowarefoundation/autoware.core/issues/233>`_)
  * add TrajectoryPoint class to templates
  * add tests
  * add method to_point for TrajectoryPoint type
  * change name of test to avoid name collision
  * add missing items
  * rename example name for clarity
  ---------
  Co-authored-by: Y.Hisaki <yhisaki31@gmail.com>
* fix(autoware_trajectory): fix a bug of align_orientation_with_trajectory_direction (`#234 <https://github.com/autowarefoundation/autoware.core/issues/234>`_)
  * fix bug of align_orientation_with_trajectory_direction
  * fixed in a better way
  * reflect comments
  * revert unnecessary changes
  ---------
* feat(autoware_trajecotry): add a conversion function from point trajectory to pose trajectory (`#207 <https://github.com/autowarefoundation/autoware.core/issues/207>`_)
  feat(autoware_trajecotry): add conversion function from point trajectory to pose trajectory
* fix(autoware_trajectory): fix a bug of example file (`#204 <https://github.com/autowarefoundation/autoware.core/issues/204>`_)
* chore(autoware_trajectory): resolve clang-tidy warning of example file (`#206 <https://github.com/autowarefoundation/autoware.core/issues/206>`_)
* feat(autoware_trajectory): add curvature_utils (`#205 <https://github.com/autowarefoundation/autoware.core/issues/205>`_)
* feat: porting `autoware_trajectory` from `autoware.universe` to `autoware.core` (`#188 <https://github.com/autowarefoundation/autoware.core/issues/188>`_)
  * add(autoware_trajectory): ported as follows (see below):
  * From `autoware.universe/common` to `autoware.core/common`
  * The history can be traced via:
  https://github.com/sasakisasaki/autoware.universe/tree/02733e7b2932ad0d1c3c9c3a2818e2e4229f2e92/common/autoware_trajectory
* Contributors: Junya Sasaki, Mamoru Sobue, Yukinari Hisaki, danielsanchezaran, mitsudome-r
