^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_ekf_localizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* test(autoware_ekf_localizer): organize and remove test codes (`#1414 <https://github.com/autowarefoundation/autoware_core/issues/1414>`_)
* test(autoware_ekf_localizer): add more unit tests for `update_step()` (`#1412 <https://github.com/autowarefoundation/autoware_core/issues/1412>`_)
* feat(localization): add node designs for the pose/twist estimation nodes (`#1409 <https://github.com/autowarefoundation/autoware_core/issues/1409>`_)
  * feat(localization): add node designs for the pose/twist estimation nodes
  Declare NdtScanMatcher, EkfLocalizer, GyroOdometer, StopFilter and
  Twist2Accel node designs, with remap_target set to each node's hardcoded
  topic and service names, and extend PoseInitializer with the map, GNSS,
  stop-check inputs and the align/trigger/partial-map-load clients that
  pose_initializer.launch.xml remaps. These let a system designer module
  compose the localization stack directly from nodes.
  * feat(localization): describe node designs and bump to format 0.4.0
  ---------
* fix(autoware_ekf_localizer): fix the conditional check in `push_pose()` (`#1402 <https://github.com/autowarefoundation/autoware_core/issues/1402>`_)
* refactor(autoware_ekf_localizer): core logic isolation - PART 3 - The ADDITIONAL refactoring (`#1397 <https://github.com/autowarefoundation/autoware_core/issues/1397>`_)
  * updated EKFUpdateResult struct
  * added new internal state handlers in core header
  * update initialize to flip the flag
  * briefly moved push_pose logic from node to core
  * briefly moved push_twist logic from node to core
  * revamped reset() to activate() with bool flag param
  * update update_step() top package mega struct, including draining queues
  * finalize the wrapping of megastruct inside core
  * purge node wrapper private state variables
  * update publish_estimate_result
  * update node wrapper with reset/activate func to be cleaner
  * update timeer_callback func in node
  * update publish+_estimate_result
  * final fixes - build test all good sucecssfully with good codecov
  * style(pre-commit): autofix
  * attempted to fix the build test agnocast stuff by properly typecasting the msg
  * style(pre-commit): autofix
  * rearrange the order of queue draining blocks in update_step()
  * align yaw bias and odom to other standard passthroughs
  * style(pre-commit): autofix
  * updated update_step() interface to ingest rclcpp::Time current time, then use double time for math stuffs
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(autoware_ekf_localizer): core logic isolation - PART 2 - The REAL refactoring (`#1352 <https://github.com/autowarefoundation/autoware_core/issues/1352>`_)
  * revamped ekf_localizer.hpp header - removed rclcpp timestamps, new corewarnings
  * revamped ekf_localizer.hpp header - removed rclcpp timestamps, new corewarnings
  * revamped ekf_localizer.cpp, get rid of rclcpp time dependency, and adapt new warnings
  * removed the warning dependency inside header of node
  * revamped source of node wrapper, including ROS warnings and time assignment
  * removed friend class declaration of EKFLocalizerDiagnosticsTest for test code (we wont gonna do that here)
  * removed warning mock
  * module test - updated get current pose and twist by excluding time involvement
  * update tests - measurement_update_pose and twist are now comply to new structure
  * added 1 more assertion of no-warning stuffs on each rejection tests
  * removed redundant test_diagnostic.cpp and test_diagnostics_topic.cpp cuz we did all of em in test_ekf_localizer_integration.cpp
  * removed NoOpWhenConstructedWithNullptr module test cuz this concept is dead
  * fix various typo and misfits, now build good and test good, all good, raedy for PR
  * added new struct EKFUpdateResult to header of core logic, containing agg diags and warnings vector
  * moved major pose/tiwst measurement update funcs back to private
  * added unified orchestration funcs into public part of core
  * addednew internalized states in private sectoer
  * style(pre-commit): autofix
  * update EKFModule constructor to init those 2 new queues
  * implement new orchestration funcs in core source
  * removed aged_object_queue dependencies from the node
  * cleaned other methods inside node wrapper that are now redundant
  * removed all the obsolete, redundant part sinside node wrapper
  * restored original loggings inside core logic
  * cleaned node wrapper source from obsolete redundancies
  * exposed friend classs and friend test (this is the only way.....)
  * style(pre-commit): autofix
  * added include memory (precommit check)
  * [ishikawa] address the issue in node timer_callback() queue overflow potential bug
  * [ishikawa] remove all friend declarations, simply re-publicize the funcs
  * [sasaki] added comment to CoreWarnings to explain the throttle_ms behavior
  * fix spellcheck
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_ekf_localizer): rename of various entities inside this node (`#1380 <https://github.com/autowarefoundation/autoware_core/issues/1380>`_)
  * rename: EKFLocalizer => EKFLocalizerNode
  * rename: EKFLocalizerDiagnosticsTest => EKFLocalizerNodeDiagnosticsTest
  * renamed: EKFModule => EKFLocalizer
  * rename: TestEKFModule => TestEKFLocalizer
  * rename: ekf_module\_ => ekf_localizer\_
  * renamed: test_ekf_module.cpp => test_ekf_localizer.cpp
  * rename: test_ekf_localizer_integration.cpp => test_ekf_localizer_node_integration.cpp
  * rename: PLUGIN autoware::ekf_localizer::EKFLocalizer => PLUGIN autoware::ekf_localizer::EKFLocalizerNode
  * style(pre-commit): autofix
  * fixed CMakeLists.txt for integration test which caused agnocast error on CI
  * rename create_ekf_localizer => create_ekf_localizer_node
  * rename: ekf_localizer => ekf_localizer_node (only for those with EKFLocalizerNode class
  * rename: ekf_localizer-> => ekf_localizer_node->
  * rename: make_module => make_ekf_localizer
  * rename: module => ekf_localizer
  * further rename: ekf_localizer.get() => ekf_localizer_node.get()
  * rename: initialize_ekf_module() => initialize_ekf_localizer()
  * rename: make_module() => make_ekf_localizer()
  * rename: module\_ => ekf_localizer\_
  * [akamine] Update localization/autoware_ekf_localizer/src/ekf_localizer_node.hpp
  Co-authored-by: Takayuki AKAMINE <38586589+takam5f2@users.noreply.github.com>
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Takayuki AKAMINE <38586589+takam5f2@users.noreply.github.com>
* fix: add <fmt/format.h> for fmt 11+ compatibility (`#1385 <https://github.com/autowarefoundation/autoware_core/issues/1385>`_)
* fix(localization): declare the dependencies these packages use (`#1369 <https://github.com/autowarefoundation/autoware_core/issues/1369>`_)
  Each of these packages uses a package it never declares. Either it includes a
  header of that package, or it names a symbol of it while the header arrives
  through another dependency. Both build today only because some declared
  dependency re-exports the owner, so a change in an unrelated repository can
  break them without anything here changing.
  The tag follows where the dependency is used: a use in an installed header or
  in code compiled into the library takes <depend>, one reached only from test/
  takes <test_depend>. System libraries are named by the rosdep key this
  workspace already prefers.
* refactor(autoware_ekf_localizer): core logic isolation PART 1 - file restructuring (`#1346 <https://github.com/autowarefoundation/autoware_core/issues/1346>`_)
* feat(autoware_ekf_localizer): implement characterization test (`#1325 <https://github.com/autowarefoundation/autoware_core/issues/1325>`_)
  * inited integration test class with integration test object
  * added test case 1 of pose init gatekeeping
  * added test 2 of integration suite, build successfsfully
  * foundational fix of integration suite setup so the EKF expected results are stable
  * added test 3 of ignoring bad shits without crashing - build test good
  * added TEST 4 of confirming node handling pose queue overflow
  * added test 5 of time out cascade warning
  * fix spell check diff
  * reduced near tol threshold to 1cm
  * added std cout on diag status so we can see em clearly upon test loggings
  * revamped timeoutcascade to be moah realistic
  * revamped TEST 5 to adapt to jazzy timer being parallel
  * allow this integrtion test suite to bypass AGNOPCAST build fix lol
  * [ishikawa] replace node init/trigger sequejnce with a clean helper
  * [ishikawa] further clean the code with helpers on repeated snippetes
  * style(pre-commit): autofix
  * [ishikawa] split test 3 into 3 smaller tests (also their order)
  * style(pre-commit): autofix
  * fix some weird rebase artifacts
  * resolved git conflicts out of nowhere
  * style(pre-commit): autofix
  * fixing pre-commit ???
  * style(pre-commit): autofix
  * update warn message catch, also fix an edge case in test
  * style(pre-commit): autofix
  * [akamine] stricten the position.x assertion
  * [akamine] apply latest_odom assertion (1)
  Co-authored-by: Takayuki AKAMINE <38586589+takam5f2@users.noreply.github.com>
  * [akamine] apply latest_odom assertion (2)
  Co-authored-by: Takayuki AKAMINE <38586589+takam5f2@users.noreply.github.com>
  * [akamine] add more libs
  Co-authored-by: Takayuki AKAMINE <38586589+takam5f2@users.noreply.github.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Takayuki AKAMINE <38586589+takam5f2@users.noreply.github.com>
* refactor(ekf_localizer): isolate pose and twist subscription callback groups (`#1227 <https://github.com/autowarefoundation/autoware_core/issues/1227>`_)
  * refactor: callback-isolation
  * fix: cppcheck
  * fix: nanosecond
  * fix: address review comments on callback group isolation
  ---------
* fix(autoware_ekf_localizer): link fmt target (`#1274 <https://github.com/autowarefoundation/autoware_core/issues/1274>`_)
  This upstreams a RoboStack build fix from patch/ros-rolling-autoware-ekf-localizer.patch.
  autoware_ekf_localizer uses fmt through its dependencies and already declares fmt in package.xml. Linking fmt::fmt explicitly makes the CMake target closure complete for toolchains and package managers that do not rely on transitive link information.
  Co-authored-by: Daisuke Nishimatsu <nishimarudai@gmail.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* feat(ekf_localizer): apply `agnocast_wrapper::Node` to `ekf_localizer` (`#1190 <https://github.com/autowarefoundation/autoware_core/issues/1190>`_)
  apply agnocast_wrapper::Node
* Contributors: Dhruv Patel, Koichi Imai, Mete Fatih Cırıt, Taekjin LEE, Tobias Fischer, Tran Huu Nhat Huy, Yutaro Kobayashi, github-actions

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* test(autoware_ekf_localizer): add EKFModule unit tests via non-ROS seam (`#1098 <https://github.com/autowarefoundation/autoware_core/issues/1098>`_)
  * test(autoware_ekf_localizer): add EKFModule unit tests via non-ROS seam
  Add a non-ROS test seam so EKFModule can be constructed in a gtest
  without a live rclcpp::Node:
  - HyperParameters gains an additive default constructor and its members
  are made settable so a test can populate them by hand. The existing
  node-based constructor still initializes every field unchanged.
  - Warning gains an additive no-op constructor (null node); warn and
  warn_throttle become null-guarded, so behavior is byte-identical when
  a real node is present.
  Add test/test_ekf_module.cpp covering the previously-untested core
  algorithm branches: find_closest_delay_time_index (begin/end bounds,
  target-beyond-last, closest-of-two), accumulate_delay_time shift and
  accumulation, Simple1DFilter init and update, compensate_rph_with_delay
  zero vs non-zero angular-velocity branches, and the safety-critical
  rejection paths of measurement_update_pose/twist (delay gate, NaN/Inf
  ignore, Mahalanobis gate), asserting both the boolean return and the
  EKFDiagnosticInfo flags.
  Also drop the unused full-covariance copy in predict_with_delay and
  declare x_curr as Vector6d to avoid a dynamic-to-fixed Eigen copy.
  * fix(autoware_ekf_localizer): guard empty delay-time table in find_closest_delay_time_index (`#65 <https://github.com/autowarefoundation/autoware_core/issues/65>`_)
  Reject the extend_state_step==0 degenerate by returning a safe index from the empty accumulated_delay_times\_ table instead of dereferencing .back() (UB). Add a RED test for the empty-table case and pin the accept-path update postconditions via the pose/twist getters.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * refactor(autoware_ekf_localizer): split parameter parsing from HyperParameters data (`#87 <https://github.com/autowarefoundation/autoware_core/issues/87>`_)
  Revert the unit-test-only default-value seam on HyperParameters. The struct
  is now plain data with no rclcpp dependency and no production-looking default
  member values that could silently diverge from the YAML source of truth.
  All declare_parameter calls are mechanically moved into a free function
  load_hyper_parameters(rclcpp::Node * node) that returns a fully-populated
  instance. Production keeps a single construction path
  (params\_(load_hyper_parameters(this))) and params\_ stays const HyperParameters,
  so the production instance remains immutable. The EKFModule unit test builds the
  struct directly by hand-setting fields, with no special default constructor.
  Behavior is identical: same parameter names and defaults as the YAML.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * test(autoware_ekf_localizer): cover Warning nullptr no-op construction
  Add a direct Warning{nullptr} test asserting warn() and warn_throttle() are no-ops when constructed without a node, covering the warning.hpp partial branch flagged by Codecov, per review.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  ---------
* Contributors: Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(ekf_localizer): initialize diagnostics information before publishing them (`#680 <https://github.com/mitsudome-r/autoware_core/issues/680>`_)
  * fix: initialize diagnostics information
  before publishing them
  * chore: add comments and remove unnecessary code
  * test: reset measurement diag fields on timer early return
  * style(pre-commit): autofix
  * test(ekf_localizer): stabilize diagnostics period gtest timing
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* fix(ekf_localizer): ekf localizer diagnostics name (`#1028 <https://github.com/mitsudome-r/autoware_core/issues/1028>`_)
  * test: add diagnostics topic test and log message names
  * fix: diagnostic task names and timer-driven publish
  * feat: mirror diagnostics on diagnostics_manual alongside updater
  * refactor: drop diagnostic_updater; publish /diagnostics manually
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(autoware_core): add USE_SCOPED_HEADER_INSTALL_DIR to localization packages (`#984 <https://github.com/mitsudome-r/autoware_core/issues/984>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* fix(ekf_localizer): change diagnostic severity (`#829 <https://github.com/mitsudome-r/autoware_core/issues/829>`_)
  * change(ekf_localizer): change diagnostic severity for initialpose reception
  * feat(autoware_ekf_localizer): update README
  * chore(autoware_ekf_localizer): updated test code
  ---------
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* feat(ekf_localizer): add adjustable publishing ekf_localizaer diagnostics  (`#826 <https://github.com/mitsudome-r/autoware_core/issues/826>`_)
  * feat: adjustable publishing of diagnostics
  * test: add test for should_publish_diagnostics function
  * style(pre-commit): autofix
  * fix: cpp lint error
  * docs: update schema.json
  * test: fix parameter undefined
  * feat: latch ekf diagnostics info when error or warn occurs
  * test: latch ekf diagnostics info when error or warn occurs
  * style(pre-commit): autofix
  * feat: use diagnostic_updater
  publish on relative periodic timer
  * refactor: Add the corresponding diagnostics immediately after each process
  * refactor: remove unused includes
  * feat: publish callback_pose/callback_twist by period
  - When diagnostics_publish_period <= 0 (default): keep original behavior.
  - Timer_callback calls force_update() every tick so the latched
  ekf_localizer diagnostic is published at EKF rate.
  - Pose/twist callbacks publish callback_pose and callback_twist via
  publish_callback_return_diagnostics() and pub_diag\_.
  - Updater internal timer is disabled (setPeriod(1e9)).
  - When diagnostics_publish_period > 0: use updater for all diagnostics.
  - Latched diagnostic is published at the configured period.
  - callback_pose and callback_twist are updated in callbacks
  (last_pose_callback_time\_ / last_twist_callback_time\_) and
  published at the same period via diagnose_callback_pose and
  diagnose_callback_twist; per-callback publish is not used.
  * test: add diagnostics publish tests for period and callbacks
  Add tests that verify /diagnostics publish behavior by diagnostics_publish_period:
  - diagnostics_published_at_specified_period: when period > 0, at least one
  message is published within 250 ms at the configured rate.
  - callback_pose_and_twist_published_at_period_when_period_positive: when
  period > 0, callback_pose and callback_twist appear on /diagnostics at
  the updater period after pose and twist are published.
  - diagnostics_published_from_timer_callback_when_period_zero: when period <= 0,
  the latched ekf_localizer diagnostic is published from timer_callback
  (force_update) at EKF rate.
  - diagnostics_published_from_pose_callback_when_period_zero: when period <= 0,
  publishing pose yields callback_pose on /diagnostics.
  - diagnostics_published_from_twist_callback_when_period_zero: when period <= 0,
  publishing twist yields callback_twist on /diagnostics.
  Add get_last_diagnostics_publish_time() helper for test access.
  * style(pre-commit): autofix
  * fix: initialize diagnostic timestamps in ctor initializer list
  * feat: require positive diag rate and publish only via updater
  * test: add diagnostics period and force_update-on-ERROR tests
  * fix: stop resetting main diagnostic to OK after publish
  - Overwrite merged_diagnostic_status\_ from merge_diagnostic_status every EKF tick
  - Record merged_diagnostic_last_transition_time\_ on any merged level change; append
  error_occurrence_timestamp when non-OK
  - Call diagnostics\_.force_update() only when merged severity increases vs previous tick
  - Remove reset_diagnostics_latch_if_published and publish marker (no longer reset to OK
  after publish)
  - Use DiagnosticStatusWrapper::summary(snapshot) in diagnose()
  - Refresh test_diagnostics (helpers, expectations, slow EKF for force_update test)
  * doc: update schema.json
  * style(pre-commit): autofix
  * refactor: rename diagnostic key to last_level_transition_timestamp
  * feat: keep last_level_transition_timestamp after recovery to OK
  * style(pre-commit): autofix
  * test: rename diagnostics tests and clarify merged-status wording
  Align test naming with current behavior: merged diagnostic state is updated every
  EKF tick, not a separate latch.
  - Rename fixture to EKFLocalizerDiagnosticsTest and update friend declaration
  - Move merge_diagnostic_status test under TestEkfDiagnostics
  - Rename tests and locals from latch* to merged*
  - Rename update_diagnostics_activation_and_initialpose_only (formerly merge_ok_minimal)
  - Drop unused node_name argument from create_ekf_localizer
  - Fix comment typo: "not effect" -> "no effect"
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_ekf_localizer): adopt cie (`#962 <https://github.com/mitsudome-r/autoware_core/issues/962>`_)
* refactor(ekf_localizer): move header files of ekf_localizer (`#726 <https://github.com/mitsudome-r/autoware_core/issues/726>`_)
  * refactor: move diagnostics.hpp to source folder
  * refactor: move aged_object_queue.hpp to source folder
  * refactor: move headers of ekf_localizer to source folder
  * refactor: fix include paths due to moving headers
  * style(pre-commit): autofix
  * refactor: fix cppcheck error
  * fix: typo
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* Contributors: Motz, Takayuki AKAMINE, Tetsuhiro Kawaguchi, Vishal Chauhan, github-actions

1.7.0 (2026-02-14)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(ekf_localizer): queue pop on ekf localizer (`#679 <https://github.com/autowarefoundation/autoware_core/issues/679>`_)
  * feat: separate max_age and max_queue_size in AgedObjectQueue
  When multiple pose sources (e.g., GNSS + NDT) are active, the queue
  can legitimately grow beyond max_age. Separating these concerns allows:
  1. max_age controls how many times each element is reused
  2. max_queue_size monitors overall queue health without enforcing hard limits
  * fix: warn and pop if queue is exceeded
  * refactor: make less if statement
  * chore: fix unclear comments
  * doc: update schema.json
  * doc: modify comments
  ---------
* Contributors: Motz, Ryohsuke Mitsudome

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore: jazzy-porting: fix test depend launch-test missing (`#738 <https://github.com/autowarefoundation/autoware_core/issues/738>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: tf2_ros to hpp headers (`#616 <https://github.com/autowarefoundation/autoware_core/issues/616>`_)
* Contributors: Tim Clephas, github-actions, 心刚

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
* fix(autoware_ekf_localizer): use constexpr and string_view (`#435 <https://github.com/autowarefoundation/autoware_core/issues/435>`_)
  * fix(autoware_ekf_localizer) use constexpr and string_view
  * add std::string_view header
  * add std::string header
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* fix(autoware_ekf_localizer): modified log output section to use warning_message and throttle (`#374 <https://github.com/autowarefoundation/autoware_core/issues/374>`_)
  * fix(autoware_ekf_localizer): Modified log output section to use warning_message and throttle
  * use constexpr and string_view
  ---------
* fix(autoware_ekf_localizer): fix deprecated autoware_utils header (`#412 <https://github.com/autowarefoundation/autoware_core/issues/412>`_)
  * fix autoware_utils import
  * fix autoware_utils packages
  ---------
* Contributors: Masaki Baba, RyuYamamoto, Tim Clephas, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* chore(ekf_localizer): increase z_filter_proc_dev for large gradient road (`#211 <https://github.com/autowarefoundation/autoware.core/issues/211>`_)
  increase z_filter_proc_dev
  Co-authored-by: SakodaShintaro <shintaro.sakoda@tier4.jp>
* feat(autoware_ekf_localizer)!: porting from universe to core 2nd (`#180 <https://github.com/autowarefoundation/autoware.core/issues/180>`_)
* Contributors: Kento Yabuuchi, Motz, mitsudome-r
