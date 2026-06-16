^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_ekf_localizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

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
