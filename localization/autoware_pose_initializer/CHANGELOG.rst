^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_pose_initializer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(map_height_fitter, pose_initializer, adapi_adaptors): move the nodes to agnocast_wrapper::Node (`#1445 <https://github.com/autowarefoundation/autoware_core/issues/1445>`_)
  * feat(map_height_fitter, pose_initializer, adapi_adaptors): move the nodes to agnocast_wrapper::Node
  * refactor(map_height_fitter, pose_initializer, adapi_adaptors): use AgnocastOnlyCallbackIsolatedExecutor and drop redundant comments
  * refactor(map_height_fitter, pose_initializer, adapi_adaptors): drop the comments that restate the code
  * docs(map_height_fitter, pose_initializer, adapi_adaptors): note what the agnocast_env include provides
  * docs(map_height_fitter, pose_initializer): explain the ENABLE_AGNOCAST test gate
  ---------
* fix(autoware_pose_initializer): drop the nested spin from the user defined initial pose (`#1423 <https://github.com/autowarefoundation/autoware_core/issues/1423>`_)
  * refactor(autoware_pose_initializer): drop the nested spin from the user defined initial pose
  `PoseInitializer` applied `user_defined_initial_pose` from its constructor,
  where the executor is not running yet, so `LocalizationTriggerModule::send_request()`
  had to spin a throwaway executor over the node to collect the trigger response.
  The constructor now only validates the parameter and schedules a one-shot timer on
  `group_srv\_`, so the initialization runs under the node's own executor like every
  other path. The `need_spin` flag threaded through `change_node_trigger()`,
  `set_user_defined_initial_pose()` and `send_request()` goes with it.
  Adds a fixture for the startup path, which needs the trigger mocks in place before
  the node is constructed and has to wait for the volatile `pose_reset` subscription
  to match before the node starts spinning. It covers the configured pose reaching
  `pose_reset`, both localizers being toggled off and back on, the failure path
  leaving the node uninitialized without letting the exception escape the timer
  callback, and the two parameter validations.
  * fix(autoware_pose_initializer): claim INITIALIZING before arming the startup timer
  The UNINITIALIZED published in the constructor otherwise stands until the
  timer runs, and autoware_automatic_pose_initializer can build an AUTO
  request from it that lands after the configured pose was applied and
  replaces it.
  * test(autoware_pose_initializer): assert the startup timer shares the initialize service group
  Reads the wiring back through for_each_callback_group instead of trying to
  observe the absence of interleaving at runtime.
  ---------
  Co-authored-by: Tran Huu Nhat Huy <29034232+TranHuuNhatHuy@users.noreply.github.com>
* test(autoware_pose_initializer): implement characterization test (`#1432 <https://github.com/autowarefoundation/autoware_core/issues/1432>`_)
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
* refactor: migrate node design files from autoware_universe (`#1381 <https://github.com/autowarefoundation/autoware_core/issues/1381>`_)
  Node design files for packages that moved to autoware_core, placed at
  the in-package convention <package>/design/<Name>.node.yaml.
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
* fix(autoware_pose_initializer): remove unnecessary dependency to fmt (`#1377 <https://github.com/autowarefoundation/autoware_core/issues/1377>`_)
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
* fix(component_interface_utils): fix build errors related to NodeAdaptor (`#1339 <https://github.com/autowarefoundation/autoware_core/issues/1339>`_)
* refactor(autoware_pose_initializer): create endpoints through NodeAdaptor (`#1329 <https://github.com/autowarefoundation/autoware_core/issues/1329>`_)
  Create the initialization-state publisher and the initialize service
  through NodeAdaptor, so each names its spec once instead of restating
  the spec's name and QoS.
  This also removes a dead local: qos_state was built with depth 1,
  reliable and transient-local, and then never passed to anything, because
  the publisher beside it already called get_qos<State>().
  Move pub_reset\_ into the member-initializer list. It is unrelated to the
  specs, but with the dead qos_state lines gone it becomes the first
  statement in the constructor body, and cppcheck flags the assignment as
  useInitializationList.
* fix(`localization`): remove duplicated config files (`#1054 <https://github.com/autowarefoundation/autoware_core/issues/1054>`_)
  * chore(`localization`): remove duplicated config files
  * chore(`localization`): remove duplicated config files
  * bug(`pose_initializer`): restore the used default values (see below)
  * We can trace that the change is derived from here:
  - https://github.com/autowarefoundation/autoware_core/pull/1054/changes#diff-74292a33fd9e2f1bb16981050ebc0f61cff9abd1f50dde3be6424c3f989f854bL4-L5
  * bug(`localization`): reuse pose_initializer launch (see below)
  Until this commit, some parameters such as `ekf_enabled`, `gnss_enabled`, ... etc are not passed.
  It seems we were using the hard-coded values in the previous `pose_initializer.param.yaml`.
  So applied fixes to:
  * Pass pose initializer flags via launch include
  * Add defaults and bool params in pose initializer launch
  * bug(`localization`): fix to pass `pose_initializer` variables (see below)
  * Followed a way that of `autowarefoundation/autoware_launch`
  - https://github.com/autowarefoundation/autoware_launch/blob/6b71b90f3a1fd07c08defe974c7caff917759108/tier4_universe_launch/tier4_localization_launch/launch/pose_twist_estimator/pose_twist_estimator.launch.xml#L26-L29
  * This commit reverts the hard-coded values in the following commit
  - https://github.com/autowarefoundation/autoware_core/pull/1054/changes/da904d6555e600ca93cc32653be3292843ddc527
  * style(pre-commit): autofix
  * bug: fix my stupid mistake when conflict resolve
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Junya Sasaki, Koichi Imai, Mete Fatih Cırıt, Ryohsuke Mitsudome, Taekjin LEE, Takagi, Isamu, Tran Huu Nhat Huy, Yutaka Kondo, github-actions

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix: correct message package name in autoware_pose_initializer README (`#1185 <https://github.com/autowarefoundation/autoware_core/issues/1185>`_)
  fix message type
* fix(autoware_pose_initializer): reject stale GNSS poses and deduplicate trigger modules (`#1095 <https://github.com/autowarefoundation/autoware_core/issues/1095>`_)
  The GNSS staleness check computed the elapsed time as stamp - now(), which
  is negative for any past message, so the guard timeout < elapsed.seconds()
  was never satisfied and stale GNSS poses were never rejected. Compute the
  elapsed time as now() - stamp via a new pure is_pose_stale() helper so the
  timeout actually takes effect.
  Consolidate the byte-identical EkfLocalizationTriggerModule and
  NdtLocalizationTriggerModule into a single parameterized
  LocalizationTriggerModule(node, service_name, label), removing the
  duplicated logic while keeping the external ROS service interface
  (ekf_trigger_node / ndt_trigger_node) and response handling identical.
  Extract the 2D pose-error comparison into a pure check_pose_error() helper
  backed by autoware_utils_geometry::calc_distance2d() instead of the
  hand-rolled sqrt(pow + pow), and keep the existing PoseErrorCheckModule
  node-based API as a thin wrapper.
  Add unit tests for is_pose_stale (fresh/stale/at-timeout) and
  check_pose_error (small/large/coincident/at-threshold).
* Contributors: Kazusa Hashimoto, Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_core): add USE_SCOPED_HEADER_INSTALL_DIR to localization packages (`#984 <https://github.com/mitsudome-r/autoware_core/issues/984>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* Contributors: Vishal Chauhan, github-actions

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: update maintainer (`#701 <https://github.com/autowarefoundation/autoware_core/issues/701>`_)
* chore: jazzy-porting:fix qos profile issue (`#634 <https://github.com/autowarefoundation/autoware_core/issues/634>`_)
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
* Contributors: Mete Fatih Cırıt, Motz, Takagi, Isamu, Yutaka Kondo, mitsudome-r, 心刚

1.4.0 (2025-08-11)
------------------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* fix(autoware_pose_initializer, autoware_adapi): fix documentation link (`#547 <https://github.com/autowarefoundation/autoware_core/issues/547>`_)
* feat!: replace autoware_internal_localization_msgs with autoware_localization_msgs for InitializeLocalization service (`#542 <https://github.com/autowarefoundation/autoware_core/issues/542>`_)
  * feat!: replace autoware_internal_localization_msgs with autoware_localization_msgs for InitializeLocalization service
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_pose_initializer): fix README.md links (`#522 <https://github.com/autowarefoundation/autoware_core/issues/522>`_)
  Update README.md
* feat(localization): add autoware_pose_initializer and autoware_map_height_fitter to autoware core (`#493 <https://github.com/autowarefoundation/autoware_core/issues/493>`_)
* Contributors: Ryohsuke Mitsudome, Yutaka Kondo, Yuxuan Liu, github-actions, 心刚

* fix: to be consistent version in all package.xml(s)
* fix(autoware_pose_initializer, autoware_adapi): fix documentation link (`#547 <https://github.com/autowarefoundation/autoware_core/issues/547>`_)
* feat!: replace autoware_internal_localization_msgs with autoware_localization_msgs for InitializeLocalization service (`#542 <https://github.com/autowarefoundation/autoware_core/issues/542>`_)
  * feat!: replace autoware_internal_localization_msgs with autoware_localization_msgs for InitializeLocalization service
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_pose_initializer): fix README.md links (`#522 <https://github.com/autowarefoundation/autoware_core/issues/522>`_)
  Update README.md
* feat(localization): add autoware_pose_initializer and autoware_map_height_fitter to autoware core (`#493 <https://github.com/autowarefoundation/autoware_core/issues/493>`_)
* Contributors: Ryohsuke Mitsudome, Yutaka Kondo, Yuxuan Liu, github-actions, 心刚

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-22)
------------------

0.2.0 (2025-02-07)
------------------

0.0.0 (2024-12-02)
------------------
