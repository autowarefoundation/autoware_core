^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_gyro_odometer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* test(gyro_odometer): reorganized test suite (`#1444 <https://github.com/autowarefoundation/autoware_core/issues/1444>`_)
  The logic class can now be tested without a ROS context, so ten scenarios
  move from the node suite to the logic suite and the superseded node suite
  is deleted. New tests cover the frame check and the staleness and
  standstill boundaries. The node suite now asserts the diagnostics level
  alone. No production file is touched.
* feat(gyro_odometer): added frame consistency check and diag report (`#1424 <https://github.com/autowarefoundation/autoware_core/issues/1424>`_)
  * refactor(gyro_odometer): make OutputData a struct
  The four output messages were a tuple, so every caller had to know the
  order. Named fields say which message is which at the point of use.
  * feat(gyro_odometer): check the shared frame assumption and report it
  The previous PR made "both inputs use the same frame" an assumption of
  the logic class. This adds the check and reports the result as a new
  diagnostics entry, is_frame_id_consistent, with an ERROR naming the
  frame the output is expected in.
  * refactor(gyro_odometer): derive the reported ages instead of storing them
  latest_vehicle_twist_dt\_ and latest_imu_dt\_ were written in
  concat_gyro_and_odometer() and read again in take_status(). They are now
  locals, and take_status() works them out from the two latest stamps.
  The reported values do not change. Both are computed from the same two
  stamps, and the arrival flags never go back to false, so the only state
  where concat() skips the computation is the one before both sides have
  arrived, where the reported age was zero either way.
  * refactor(gyro_odometer): move transform_covariance to its only caller
  The node applies it to the IMU sample it has just transformed, and
  nothing in the logic class calls it. It moves next to transform_imu(),
  and its test moves with it. What the logic class expects of an IMU
  sample is now stated on input_imu().
  ---------
* test(gyro_odometer): wait for connections and fused output (`#1434 <https://github.com/autowarefoundation/autoware_core/issues/1434>`_)
  Wait for topic connections before the node tests publish their inputs. Repeat the fusion inputs until output arrives, with a five-second deadline. This handles callback order when the node discards samples before both inputs arrive.
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
* feat(gyro_odometer): remove TF and the node clock from the logic class (`#1398 <https://github.com/autowarefoundation/autoware_core/issues/1398>`_)
  * test(gyro_odometer): pin what the reported transform status depends on
  The node transforms each IMU sample into the output frame before the logic
  class receives it. A sample the node cannot transform is skipped. The logic
  class no longer deals with TF.
  An input is now stale when the two latest stamps differ by more than
  message_timeout_sec. Before, each stamp was compared against the node clock.
  The input functions no longer take a current time, so GyroOdometer receives
  messages only.
  Two effects are visible on the twist topics. An IMU sample that cannot be
  transformed no longer drops the pending vehicle twist, and the next usable
  sample fuses with it. Two inputs whose stamps agree with each other are fused
  even when they reach the node late.
  ---------
* refact(gyro_odometer): extracted logic part, which still uses TF inside (`#1389 <https://github.com/autowarefoundation/autoware_core/issues/1389>`_)
  * refactor(gyro_odometer): extract logic class still keeping transform_listener
  logic class still uses transform_listener directly, so dependency
  remains
  * refactor(gyro_odometer): inject the gyro-queue transformation as a callback
  Replace the TransformListener argument with a std::function callback, so
  gyro_odometer.hpp no longer names any ROS or TF type. The lookup moves to
  a free function in the node layer; the sequence and behaviour are
  unchanged.
  * refactor(gyro_odometer): take message_timeout_sec in the constructor
  It is fixed once the instance is built, so it does not belong in the
  per-message input calls. current_time stays a call argument: it is part
  of the event, not configuration.
  ---------
* test(gyro_odometer): pin node behaviour (`#1378 <https://github.com/autowarefoundation/autoware_core/issues/1378>`_)
  * test(gyro_odometer): pin node behaviour with a deterministically driven suite
  Only two scenarios existed, covering publish/no-publish; everything else
  on the node's topics could change unnoticed. Add a suite that pins
  queueing, averaging, timeouts, TF failure and diagnostics, driven
  synchronously with a simulated clock so message order and timing are
  deterministic instead of racing on a background-spun executor.
  * style(gyro_odometer): rename test constants to lower_snake_case
  Autoware requires lower_snake_case for constants, not the Google-style
  k-prefix used here.
  ---------
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
* refactor(autoware_gyro_odometer): renamed file name of node and logic files (`#1347 <https://github.com/autowarefoundation/autoware_core/issues/1347>`_)
  Renamed files and fixed some headers simultaneously
  in order to align with other already refactored nodes
* refactor(gyro_odometer): moved diagnostics-related classes to individual file (`#1342 <https://github.com/autowarefoundation/autoware_core/issues/1342>`_)
  Preparation for splitting autoware_gyro_odometer into a ROS layer and a
  node-independent layer. This PR does the groundwork only: it moves the diagnostics
  decision out of the fusion module and makes the non-ROS tests runnable without a ROS
  context. No behaviour changes.
  ---------
* test(gyro_odometer): improved structure of Node test (`#1333 <https://github.com/autowarefoundation/autoware_core/issues/1333>`_)
  Improved structure of Node test,
  * utilizing test fixture commonly used in other packages
  * making file structure simple
  ---------
* feat(gyro_odometer): apply `agnocast_wrapper::Node` to `gyro_odometer` (`#1191 <https://github.com/autowarefoundation/autoware_core/issues/1191>`_)
  * apply agnocast_wrapper::Node
  * style(pre-commit): autofix
  * fix
  * fix
  * fix Cmakelists.txt
  * fix
  * fix tests
  * restore loggerlevelconfigure
  * fix
  * use publish(const &)
  * run tests for test_gyro_odometer_fusion when agnocast enabled
  * fix test name
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Dhruv Patel, Kazuki Komiya, Koichi Imai, Mete Fatih Cırıt, Taekjin LEE, github-actions

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_gyro_odometer): extract pure fusion logic and fix diagnostics level (`#1112 <https://github.com/autowarefoundation/autoware_core/issues/1112>`_)
  Extract the sensor-fusion math out of the GyroOdometerNode private methods into
  pure, unit-testable free functions in a new internal header gyro_odometer_fusion.hpp:
  - transform_covariance() (moved out of the .cpp so it is reachable from tests),
  - fuse_twist() for the queue mean / covariance reduction and output-stamp selection,
  - apply_stop_compensation() for the stopped-vehicle yaw-bias clearing, and
  - determine_diagnostics() for the diagnostics level / message computation.
  The node methods now delegate to these functions and keep only I/O, TF and timeout
  handling. The refactor is behavior-preserving except for one latent bug fix:
  publish_diagnostics() previously left its local 'level' at OK, so the throttled
  WARN/ERROR console logs were unreachable and never fired. determine_diagnostics()
  now aggregates the maximum severity across triggered conditions, so the console
  WARN/ERROR logs fire as intended. The published DiagnosticStatus level and message
  text are unchanged (the TF-failure message still embeds output_frame).
  Add gtest coverage (test/test_gyro_odometer_fusion.cpp) for covariance handling,
  the stopped/moving/turning branches, the fusion mean/covariance/stamp output, and
  the diagnostics OK/WARN/ERROR/aggregation paths.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
* Contributors: Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_core): add USE_SCOPED_HEADER_INSTALL_DIR to localization packages (`#984 <https://github.com/mitsudome-r/autoware_core/issues/984>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* feat(autoware_gyro_odometer): adopt cie (`#961 <https://github.com/mitsudome-r/autoware_core/issues/961>`_)
  Co-authored-by: Koichi Imai <45482193+Koichi98@users.noreply.github.com>
  Co-authored-by: atsushi421 <atsushi.yano.2@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* fix(autoware_gyro_odometer): fix bugprone-narrowing-conversions warnings (`#934 <https://github.com/mitsudome-r/autoware_core/issues/934>`_)
  * fix(autoware_gyro_odometer): fix bugprone-narrowing-conversions warnings
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* Contributors: NorahXiong, Tetsuhiro Kawaguchi, Vishal Chauhan, github-actions

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* perf(localization, sensing): reduce subscription queue size from 100 to 10 (`#751 <https://github.com/autowarefoundation/autoware_core/issues/751>`_)
* ci(pre-commit): autoupdate (`#723 <https://github.com/autowarefoundation/autoware_core/issues/723>`_)
  * pre-commit formatting changes
* Contributors: Mete Fatih Cırıt, Yutaka Kondo, github-actions

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(gyro_odometer): make diagnostics summarized (`#626 <https://github.com/autowarefoundation/autoware_core/issues/626>`_)
  * make diagnostics summarized in gyro_odometer
  * fix diagnostics message
  ---------
  Co-authored-by: Yamato Ando <yamato.ando@tier4.jp>
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
* Contributors: Mete Fatih Cırıt, Motz, Taiki Yamada, Yutaka Kondo, mitsudome-r

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
* feat(autoware_gyro_odometer): porting the package from Autoware Universe (`#423 <https://github.com/autowarefoundation/autoware_core/issues/423>`_)
* Contributors: Tim Clephas, Yutaka Kondo, github-actions, storrrrrrrrm

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
* feat(autoware_gyro_odometer): porting the package from Autoware Universe (`#423 <https://github.com/autowarefoundation/autoware_core/issues/423>`_)
* Contributors: Tim Clephas, Yutaka Kondo, github-actions, storrrrrrrrm

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-22)
------------------

0.2.0 (2025-02-07)
------------------

0.0.0 (2024-12-02)
------------------
