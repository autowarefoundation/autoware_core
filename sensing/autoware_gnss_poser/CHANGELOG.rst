^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_gnss_poser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_gnss_poser): stamp gnss_fixed with the fixed header stamp (`#1459 <https://github.com/autowarefoundation/autoware_core/issues/1459>`_)
  fix(autoware_gnss_poser): stamp gnss_fixed with the fix header stamp
  gnss_fixed was the only output stamped with the node clock; gnss_pose,
  gnss_pose_cov and the TF broadcast all carry the header stamp of the fix
  they were computed from. It now carries that stamp too, so the status can
  be matched to the fix and to the pose it belongs to.
  It comes before the logic is separated out because it decides what the
  pose computation answers with: once the outputs are built there, every
  one of them is stamped with the fix, and this is the only one that was
  not.
  The characterization case GnssFixed_StampIsNodeClockNotFixHeaderStamp
  pinned the old behavior and is rewritten as GnssFixed_StampIsFixHeaderStamp;
  built against this change with the old assertions, exactly that case fails
  and the other 30 pass.
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
* fix(autoware_gnss_poser): reject invalid gnss_pose_pub_method and buff_epoch at startup (`#1458 <https://github.com/autowarefoundation/autoware_core/issues/1458>`_)
  gnss_pose_pub_method is now an enum (Instant / Average / Median). A value
  outside 0..2 used to be accepted silently: every non-zero value enabled
  buffering and every value other than 1 selected the median. buff_epoch must
  now be at least 1: a zero-capacity buffer counted as "full", so the average
  published NaN coordinates on every output and the median threw
  std::out_of_range inside the fix callback. Both are rejected with
  std::invalid_argument while the node is constructed, so a misconfiguration
  fails at launch instead of at runtime.
  The schema already documented 0..2 for the method; buff_epoch's minimum moves
  from 0 to 1 and the README no longer claims that buff_epoch = 0 disables the
  method (it never did).
  Characterization tests rewritten on purpose: MethodUnknown_BehavesLikeMedian,
  MethodAverage_BuffEpochZero_PublishesNaNPosition and
  MethodInstant_BuffEpochZero_IsHarmless (all three fail against this change
  with the old assertions) are replaced by Construct_UnknownPubMethod_FailsToStart
  and Construct_BuffEpochBelowOne_FailsToStart. The other 29 cases are unchanged.
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
* test(autoware_gnss_poser): characterize orientation, antenna TF composition and covariance (`#1448 <https://github.com/autowarefoundation/autoware_core/issues/1448>`_)
  * test(autoware_gnss_poser): characterize orientation, antenna TF composition and covariance
  Third PR of the series, stacked on the position branch. Ten cases on the orientation
  sources (INS message vs. heading from displacement), the antenna -> base_link TF lookup and
  composition including the fallback to identity, the lookup time and the interpolation of a
  changing relation, the map -> gnss_base_link broadcast, and the covariance layout of
  gnss_pose_cov.
  No production code changes.
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
  * test(autoware_gnss_poser): use one tolerance for positions and one for angles
  The file compared positions with 1e-9, 1e-6 and 1e-4 m depending on the case,
  with no principle behind the difference: the tighter values were meant to show
  "the same library call", which is more than a characterization test needs.
  Computed positions now share `position_tolerance` (1e-4 m) and headings
  `angle_tolerance` (1e-6 rad), each with its rationale next to the definition;
  `expect_same_rotation` compares the rotation angle between the orientations
  so the same constant applies. Values the node copies keep exact equality, and
  the geoid-height check keeps its own 1 cm bound.
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
  * test(autoware_gnss_poser): explain the covariance constants in the assertions
  Review of `#1448 <https://github.com/autowarefoundation/autoware_core/issues/1448>`_: the orientation and covariance cases assert squared
  rmse values and two hard-coded fallbacks without saying where they come
  from. Each of them now carries the arithmetic (0.1^2 = 0.01 and so on),
  the origin of the 1.0 placeholder and of the 10.0 substitute, and the
  input covariance names the diagonal it expects to survive.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
* test(autoware_gnss_poser): characterize the position pipeline and buffering (`#1437 <https://github.com/autowarefoundation/autoware_core/issues/1437>`_)
  * test(autoware_gnss_poser): characterize the position pipeline and buffering
  Second PR of the series, stacked on the harness branch. Twelve cases on how a fixed
  NavSatFix becomes a position: projection through autoware_geography_utils, height datum
  conversion, the instantaneous method, and the average / median buffering with its full-buffer
  gate, sliding window, independence from gated and non-fixed messages, and the buff_epoch = 0
  and out-of-range method edge cases.
  No production code changes.
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
  * test(autoware_gnss_poser): address review on the position pipeline cases
  - Assert non-empty input in the mean_of / median_of test helpers instead of
  dividing by zero or indexing an empty vector.
  - Explain the 0.001-degree offsets between fixes (about 100 m, so every output
  can be matched to its own fix) at the reference constants and where the
  instant-method case uses them.
  - Explain why a 1 m bound is enough to show that the EGM2008 datum conversion
  happened (the geoid is about 36 m above the ellipsoid at the reference point).
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
  * test(autoware_gnss_poser): state what the 1 m bound in the EGM2008 case checks
  The bound only shows that the datum conversion happened (a converted z is
  about 36 m away from the input altitude, an unconverted one equals it); the
  exact value is compared with the library on the previous line, and 1.0 m has
  no meaning of its own.
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
  * test(autoware_gnss_poser): compare the datum and projector cases with literal ground truth
  The EGM2008 case compared the output with the same library call the node
  makes, which is the wrong kind of expectation for a case about the height
  conversion itself. It now expects the MGRS golden coordinates and the altitude
  lowered by the geoid height at the reference point as GeographicLib's
  GeoidEval reports it (36.12 m), within 1 cm; the separate 1 m sanity bound is
  no longer needed. The LocalCartesianUTM case keeps only its literal
  expectation (the fix sits on the map origin). Derived expectations remain
  where the case is about how positions are selected or combined, not computed.
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
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
* test(autoware_gnss_poser): add node characterization harness and gating tests (`#1427 <https://github.com/autowarefoundation/autoware_core/issues/1427>`_)
  * test(autoware_gnss_poser): add node characterization harness and gating tests
  First PR of a series that pins the observable behavior of the gnss_poser node before its
  logic is separated from the node. Adds a deterministic harness (a peer node standing in for
  every neighbor of gnss_poser, discovery-gated, event-driven synchronization) and ten cases
  on construction (including the shipped parameter file), the ROS interface, and the gates that
  decide whether a NavSatFix produces any output at all.
  The file is named test_gnss_poser_node_characterization.cpp, next to test_gnss_poser_node.cpp
  and following autoware_gyro_odometer, because node-free tests of the extracted logic will
  follow later.
  No production code changes; CMakeLists.txt only registers the new gtest target.
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
  * test(autoware_gnss_poser): address review on the characterization harness
  - Use lower snake case for the test constants, as the Autoware C++ coding
  guideline requires (kLat -> reference_latitude, kFixStampSec -> fix_stamp_sec, ...).
  - Narrow the two parameter cases to the node's contract: a configuration that
  lacks a parameter or gives it the wrong type makes the node fail to start.
  Which exception rclcpp throws is no longer asserted, and the cases are
  renamed *_FailsToStart with their comments reworded accordingly.
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
  * test(autoware_gnss_poser): satisfy the spell check in the characterization test
  - Rename `pose_covs` to `pose_cov_msgs` (consistent with `pose_cov_sub\_` and
  `last_pose_cov()`); the spell checker read the plural as an unknown word.
  - Add a `cspell:ignore` line for SBAS and GBAS, the NavSatStatus fix types
  the gate cases exercise.
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
* fix(sensing): declare the dependencies these packages use (`#1373 <https://github.com/autowarefoundation/autoware_core/issues/1373>`_)
  Each of these packages uses a package it never declares. Either it includes a
  header of that package, or it names a symbol of it while the header arrives
  through another dependency. Both build today only because some declared
  dependency re-exports the owner, so a change in an unrelated repository can
  break them without anything here changing.
  The tag follows where the dependency is used: a use in an installed header or
  in code compiled into the library takes <depend>, one reached only from test/
  takes <test_depend>. System libraries are named by the rosdep key this
  workspace already prefers.
* refactor: migrate node design files from autoware_universe (`#1381 <https://github.com/autowarefoundation/autoware_core/issues/1381>`_)
  Node design files for packages that moved to autoware_core, placed at
  the in-package convention <package>/design/<Name>.node.yaml.
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
* feat(gnss_poser): apply `agnocast_wrapper::Node` to `autoware_gnss_poser` (`#1211 <https://github.com/autowarefoundation/autoware_core/issues/1211>`_)
  * apply agnocast_wrapper::Node
  * fix
  * style(pre-commit): autofix
  * fix tests
  * fix to use wrapper tf2
  * fixed to skip tests when agnocast disabled
  * fix cpplint
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Kentaro Nagatomo, Koichi Imai, Mete Fatih Cırıt, Taekjin LEE, github-actions

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_gnss_poser): drop dead helpers, per-instance prev_position, unit-test pure statics (`#1154 <https://github.com/autowarefoundation/autoware_core/issues/1154>`_)
  Internal-only cleanup of the GNSS poser node:
  - Delete the dead get_quaternion_by_heading and get_transform helpers
  (declared and defined but never called anywhere in autoware_core).
  - Replace the function-local 'static prev_position' in callback_nav_sat_fix
  with a per-instance prev_position\_ member guarded by has_prev_position\_,
  removing the process-global mutable state shared across all instances. The
  first-sample behavior is preserved (initial difference is zero, yielding an
  identity orientation).
  - Add direct unit tests for the pure static helpers (get_median_position
  odd/even-sized buffers, get_average_position values, and
  get_quaternion_by_position_difference across all cardinal headings plus the
  identical-points edge case) via a friend test fixture, so they no longer
  rely solely on a full node + executor round-trip.
  No public API change; behavior-preserving.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
* Contributors: Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_core): add USE_SCOPED_HEADER_INSTALL_DIR to sensing packages (`#985 <https://github.com/mitsudome-r/autoware_core/issues/985>`_)
  Co-authored-by: github-actions <github-actions@github.com>
* fix(autoware_gnss_poser): fix flaky test timeout on CI (`#1018 <https://github.com/mitsudome-r/autoware_core/issues/1018>`_)
  Replace background spin() threads with main-thread spin_some() to make
  the test deterministic. The previous approach used two executors with
  two background threads per test, creating race conditions in cancel/join
  during TearDown that caused intermittent 60s timeouts on CI.
  - Use single executor with spin_some() instead of background spin()
  - Extract rebuildGnssPoserNode() to properly reset subscriptions before
  destroying the old node, preventing dangling references
  - Replace std::atomic<bool> with plain bool (single-threaded now)
  - Replace sleep_for waits with executor->spin_some() to process work
* fix(autoware_gnss_poser): fix test timeout on Humble (`#1015 <https://github.com/mitsudome-r/autoware_core/issues/1015>`_)
  fix(autoware_gnss_poser): fix test timeout on Humble by using single rclcpp init/shutdown
  The test_autoware_gnss_poser binary was timing out on the humble-above CI
  job because each test fixture performed its own rclcpp::init()/shutdown()
  cycle (9 total). On Humble, repeated init/shutdown cycles cause resource
  leaks that eventually hang the executor.
  Move rclcpp::init() and rclcpp::shutdown() to main() so there is a single
  lifecycle for the entire test binary. Also reset publisher_executor\_ in
  TearDown to avoid dangling references.
* fix(autoware_gnss_poser): fix bugprone-implicit-widening-of-multiplication-result warnings (`#912 <https://github.com/mitsudome-r/autoware_core/issues/912>`_)
  fix(autoware_gnss_poser): fix bugprone-implicit-widening-of-multiplication-result warnings
* chore(sensing): move header files from include/ to src/ (`#852 <https://github.com/mitsudome-r/autoware_core/issues/852>`_)
  * refactor(sensing): move header files from include/ to src/ for crop_box_filter and gnss_poser
  These headers are internal implementation details not used by external
  packages. Moving them to src/ clarifies they are private headers.
  * style(pre-commit): autofix
  * fix(crop_box_filter): fix for linter
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* Contributors: Mete Fatih Cırıt, NorahXiong, Takahisa Ishikawa, Vishal Chauhan, github-actions

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore: tf2_ros to hpp headers (`#616 <https://github.com/autowarefoundation/autoware_core/issues/616>`_)
* Contributors: Tim Clephas, github-actions

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
* chore: no longer support ROS 2 Galactic (`#492 <https://github.com/autowarefoundation/autoware_core/issues/492>`_)
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* test(autoware_gnss_poser): add unit tests (`#414 <https://github.com/autowarefoundation/autoware_core/issues/414>`_)
  * test(autoware_gnss_poser): add unit tests
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: NorahXiong, Tim Clephas, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* fix(autoware_gnss_poser): depend on geographiclib through its Find module (`#313 <https://github.com/autowarefoundation/autoware_core/issues/313>`_)
  fix: depend on geographiclib through it's provided Find module
* Contributors: Shane Loretz

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* feat(autoware_gnss_poser): porting from universe to core (`#166 <https://github.com/autowarefoundation/autoware.core/issues/166>`_)
* Contributors: mitsudome-r, 心刚
