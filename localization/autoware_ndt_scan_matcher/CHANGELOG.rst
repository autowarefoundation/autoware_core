^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_ndt_scan_matcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(`ndt_scan_matcher`): add tests for scan path, based on the @takam5f2's branch (`#1446 <https://github.com/autowarefoundation/autoware_core/issues/1446>`_)
  * add: tests for scan path, based on the following @takam5f2's branch
  * https://github.com/takam5f2/autoware_core/blob/5268228fadf9d74804178f8d698172a11364314b/localization/autoware_ndt_scan_matcher/test/test_ndt_scan_matcher_characteristics.cpp
  * fix: wordy comments
  * cosmetic bug: fix wrong comment
  * bug: fix wrong dependency
  * fix: simplify comments
  * fix: error log and comment, applying the following review comment
  * https://github.com/autowarefoundation/autoware_core/pull/1446#discussion_r4043894171
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Tran Huu Nhat Huy <29034232+TranHuuNhatHuy@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(ndt_scan_matcher): apply `agnocast_wrapper::Node` to `ndt_scan_matcher` (`#1215 <https://github.com/autowarefoundation/autoware_core/issues/1215>`_)
  * apply agnocast_wrapper::Node
  * style(pre-commit): autofix
  * fix tests and source
  * style(pre-commit): autofix
  * use wrapper tf2 and ok()
  * delete unnecessary comments and avoid ALLOCATE
  * fix
  * fix(ndt_scan_matcher): add missing <utility> include for std::move
  * fix(ndt_scan_matcher): remove includes made redundant by the agnocast wrapper
  * fix(ndt_scan_matcher): take the client response directly
  * fix(ndt_scan_matcher): drop the duplicate <utility> include
  * test(ndt_scan_matcher): add the node to the executor through its base interface
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: kobayu858 <yutaro.kobayashi.2@tier4.jp>
* test(autoware_ndt_scan_matcher): add characterization harness and sensor-gate tests (`#1348 <https://github.com/autowarefoundation/autoware_core/issues/1348>`_)
  Add a harness that drives NDTScanMatcher deterministically (diagnostics correlation, TF
  readiness, a differential stub map loader) and twelve cases that pin the sensor-points gates
  and the initial-pose subscriber ahead of extracting the hot path into a ROS-free core. Node-level
  tests move to ament_add_ros_isolated_gtest with their test dependencies declared. No production
  code changes.
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(`ndt_scan_matcher`): make map update module ROS-node free (`#1322 <https://github.com/autowarefoundation/autoware_core/issues/1322>`_)
  * refactor: make map update module ROS-node free
  * Move the pcd loader client, debug publisher, and `loaded_map\_` to the node;
  inject the service call (PcdLoaderFunction) and diagnostics (DiagnosticsHandlingFunction).
  * style(pre-commit): autofix
  * fix: by `pre-commit`
  * style(pre-commit): autofix
  * cosmetic: fix noisy-long comments
  * refactor: avoid doing hard diag task via callback
  * Apply the following review comment
  - https://github.com/autowarefoundation/autoware_core/pull/1322#pullrequestreview-4852039689
  * refactor: simplify `loaded_pcd_map` publish logic
  * Apply the following review comment:
  - https://github.com/autowarefoundation/autoware_core/pull/1322#pullrequestreview-4852039689
  * style(pre-commit): autofix
  * sorry: forgot applying `pre-commit`
  * style(pre-commit): autofix
  * bug: fix infinitely growing map, this is caused by my stupid mis-refactoring
  * refactor: move minor diagnostics handling into ROS-node
  * style(pre-commit): autofix
  * refactor(`autoware_ndt_scan_matcher`): simplify map update state handling
  * Extract distance_from_last_update() to remove the duplicated distance calculation
  * Drop the BuilderState struct and its need_rebuild flag by deriving the rebuild decision from out_of_map_range(position).
  * A failed rebuild no longer advances `last_update_position\_`, so it retries next tick.
  Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
  * refactor(`autoware_ndt_scan_matcher`): set debug map header.stamp inside MapUpdateModule via PointStamped
  * The stamp is now the pose timestamp rather than publish time.
  Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
  * style(pre-commit): autofix
  * add: tests for map update module
  * style(pre-commit): autofix
  * Revert "refactor(`autoware_ndt_scan_matcher`): set debug map header.stamp inside MapUpdateModule via PointStamped"
  This reverts commit c569a83f043f3c5480c2039da80e4143b853c709.
  * fix: a bug that causes heavy memory allocation
  * Apply the following PR comment:
  - https://github.com/autowarefoundation/autoware_core/pull/1322#discussion_r3819841133
  * fix: centralize publish logic for `result.loaded_pcd_map`
  * Apply the following PR review comment:
  - https://github.com/autowarefoundation/autoware_core/pull/1322#discussion_r3819937749
  * fix: tests following the corresponding revert below
  * https://github.com/autowarefoundation/autoware_core/pull/1322/commits/cb8019c576d30a38f56e0a3ab2666950e98d243f
  * revert: update logic for `last_update_position\_`
  * This is partial revert that of the following commit:
  - https://github.com/autowarefoundation/autoware_core/pull/1322/changes/14b86d5d6b98b47fd7dfb968415d72af2149c120#diff-33a0fc135682003000b9c2e2a43fa1635159795559d8d468c66d0a54011804e4L156
  * fix: simplify `publish_loaded_map_if_present`
  * Apply the following review comment:
  - https://github.com/autowarefoundation/autoware_core/pull/1322#discussion_r3892361767
  * bug: fix unexpected unassigned buffer in merged pcd
  * Apply the following review comment:
  - https://github.com/autowarefoundation/autoware_core/pull/1322#discussion_r3892274846
  * Revert "refactor(`autoware_ndt_scan_matcher`): simplify map update state handling"
  This reverts commit 14b86d5d6b98b47fd7dfb968415d72af2149c120.
  * fix: do not use optional for `stamp` in publish logic
  Co-authored-by: Motz <83898149+Motsu-san@users.noreply.github.com>
  * fix: do not use optional for `stamp` in publish logic, following the previous commit
  Co-authored-by: Motz <83898149+Motsu-san@users.noreply.github.com>
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Tran Huu Nhat Huy <29034232+TranHuuNhatHuy@users.noreply.github.com>
  Co-authored-by: Claude Opus 4.8 <noreply@anthropic.com>
  Co-authored-by: Motz <83898149+Motsu-san@users.noreply.github.com>
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
* fix(autoware_ndt_scan_matcher): use tf2 C++ header (`#1273 <https://github.com/autowarefoundation/autoware_core/issues/1273>`_)
* fix(ndt): correct sign of pitch^2 term in NDT angle Hessian (h_ang d1) (`#1217 <https://github.com/autowarefoundation/autoware_core/issues/1217>`_)
  fix(autoware_ndt_scan_matcher): correct sign of pitch^2 term in NDT angle Hessian (h_ang d1)
  computeAngleDerivatives() builds the precomputed angular Hessian table
  h_ang\_. Row 6 ("d1"), which forms the x-component of d^2T/dpitch^2, used
  +sy where the exact second derivative is -sy.
  Derivation (R = Rx(roll)*Ry(pitch)*Rz(yaw), point x):
  (R x)_x      =  cy*cz*x0 - cy*sz*x1 + sy*x2
  d/dpitch     = -sy*cz*x0 + sy*sz*x1 + cy*x2
  d2/dpitch^2  = -cy*cz*x0 + cy*sz*x1 - sy*x2    => d1 = (-cy*cz, cy*sz, -sy)
  So the third coefficient must be -sy, not +sy. The gradient (j_ang) is
  already exact, and every other h_ang block was re-derived and confirmed
  correct (roll^2, roll*pitch, roll*yaw, pitch^2 y/z, pitch*yaw, yaw^2); this
  is the sole error. The fix makes the analytic Hessian match the true second
  derivative, affecting only the Newton search direction (the optimum, where
  gradient = 0, is unchanged).
* Contributors: Junya Sasaki, Koichi Imai, Mete Fatih Cırıt, Taekjin LEE, Takayuki AKAMINE, Tobias Fischer, github-actions, ytakano

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* test(autoware_ndt_scan_matcher): unit-test covariance/oscillation math and fix no-ground hot loop (`#1108 <https://github.com/autowarefoundation/autoware_core/issues/1108>`_)
  * test(autoware_ndt_scan_matcher): unit-test covariance/oscillation math and fix no-ground hot loop
  Extract the translation-unit-local rotate_covariance free function and the
  count_oscillation logic into a testable internal seam
  (ndt_scan_matcher_helper.{hpp,cpp}) and add two fast, dependency-light
  ament_auto_add_gtest targets:
  - test_estimate_covariance covers the pure Eigen math in
  ndt_omp/estimate_covariance.cpp (calc_weight_vec, calculate_weighted_mean_and_cov,
  propose_poses_to_search, estimate_xy_covariance_by_laplace_approximation,
  rotate_covariance_to_map/base_link round-trip, adjust_diagonal_covariance) with
  hand-computed expected values.
  - test_ndt_scan_matcher_helper covers rotate_covariance (36-element covariance
  block rotation) and count_oscillation (consecutive-inversion counting).
  These functions previously had zero unit tests and were only exercised by the
  heavyweight 300s launch/integration tests.
  Behavior-preserving cleanups:
  - Hoist the constant matrix4f_to_pose(ndt_result.pose).position.z out of the
  per-point no-ground-points removal loop (it equals ndt_result.pose(2,3)),
  avoiding a full Pose + quaternion extraction per LiDAR point, and reserve()
  the output cloud. The NDTScanMatcher::count_oscillation private static member
  is retained and now delegates to the free function, so no public API changes.
  - Remove the dead, never-read Eigen::Matrix4f base_to_sensor_matrix\_ member
  (transform_sensor_measurement uses a local of the same name).
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * fix(autoware_ndt_scan_matcher): guard count_oscillation against zero-length steps and make helper header internal
  count_oscillation normalized motion vectors without guarding against
  zero-length steps (e.g. repeated/identical poses). Eigen normalized() on
  a zero vector yields NaNs, making cosine_value NaN and the oscillation
  logic unreliable. Skip zero-length steps and treat them as
  non-oscillations, resetting the consecutive-inversion count. Add unit
  tests pinning the all-identical and interleaved zero-length cases.
  Move ndt_scan_matcher_helper.hpp out of include/autoware/... into src/ so
  the helper stays an internal seam rather than part of the installed
  public header/ABI surface (USE_SCOPED_HEADER_INSTALL_DIR no longer
  installs it). Includes now use the project-local quoted form and the unit
  test includes it via ../src, matching the existing private-header
  convention in this repo.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * test(autoware_ndt_scan_matcher): assert rotation covariance against independent oracle (`#81 <https://github.com/autowarefoundation/autoware_core/issues/81>`_)
  Replace the mirrored rot * cov * rot.transpose() expected value with an explicit stdlib scalar expansion of R * cov * R^T, asserted per-entry via EXPECT_NEAR, so a transpose/sign error in rotate_covariance_to_map is caught instead of mirrored. Retain the round-trip equality as a secondary property check.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  ---------
* fix(ndt_scan_matcher): explicitly include omp.h (`#1166 <https://github.com/autowarefoundation/autoware_core/issues/1166>`_)
  * fix(ndt_scan_matcher): explicitly include omp.h
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_ndt_scan_matcher): publish all map points if publisher has subscribers (`#995 <https://github.com/autowarefoundation/autoware_core/issues/995>`_)
  * publish all map points if publisher has subscribers
  * style(pre-commit): autofix
  * fix error
  * add parameter
  * fix param name and add json
  * add loaded map clear on ndt ptr reset
  * reserve before adding points
  * add new param to autoware_core_lozalization
  * Update localization/autoware_ndt_scan_matcher/include/autoware/ndt_scan_matcher/map_update_module.hpp
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
* fix(autoware_ndt_scan_matcher): zero div if particles_num < 20 (`#996 <https://github.com/autowarefoundation/autoware_core/issues/996>`_)
  * fix zero div if particles_num < 20
  * fix zero div if particles_num < 20
  * style(pre-commit): autofix
  * clamp publish interval
  * style(pre-commit): autofix
  * add test
  * style(pre-commit): autofix
  * Update localization/autoware_ndt_scan_matcher/test/test_fixture.hpp
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
  * fix license year
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Anh Nguyen <vietanhng17@gmail.com>
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
* fix(ndt_scan_matcher): don't include removed PCL headers (`#1070 <https://github.com/autowarefoundation/autoware_core/issues/1070>`_)
  * fix(ndt_scan_matcher): don't include removed PCL headers
  pcl/filters/boost.h was removed in the PCL 1.15.0 release[1]. None of
  the Boost headers it previously included are needed for compilation,
  so the include is simply be removed from multi_voxel_grid_covariance_omp_impl.hpp.
  multi_voxel_grid_covariance_omp.h uses boost::shared_ptr so we include
  just the header for it.
  [1]: https://github.com/PointCloudLibrary/pcl/commit/3ae4df5ede96d08408b500ee83d551151dd2a3da#diff-cb7d245fd1697b5c4a72cbb9807284c28f4166cd23c267211b85a7d4dee7f507L49-L54
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Kazusa Hashimoto, Michal Sojka, Yutaka Kondo, github-actions, iwatake

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(ndt_scan_matcher): fix concurrency bugs caused by missing locks by introducing Guarded class (`#1030 <https://github.com/mitsudome-r/autoware_core/issues/1030>`_)
  * Removed `setInputSource`
  * Applied `pre-commit run -a`
  * Corrected according to cppcheck's indications
  * bug: fix wrongly remaining `getInputSource` (see below)
  * This bug is caused due to my previous conflict resolve
  * fix(agnocastlib): fix missing lock on latest_ekf_position\_ and add Guarded class
  The `latest_ekf_position\_` field in `NDTScanMatcher` was intended to be
  protected by `latest_ekf_position_mutex\_`, but a concurrency bug existed
  due to missing lock acquisitions.
  To resolve this issue, this commit introduces a new `Guarded` class. This
  class encapsulates both the mutex and the data it protects into a single
  entity. By enforcing lock acquisition whenever the internal data is accessed,
  it systematically prevents bugs caused by forgetting to acquire locks.
  * refactor(ndt_scan_matcher): apply Guarded class to last_update_position\_
  * fix(ndt_scan_matcher): fix missing lock bugs by wrapping ndt_ptr\_ with Guarded
  Refactored `ndt_ptr\_` and `ndt_ptr_mtx\_` in `NDTScanMatcher` using `Guarded`
  to fix missing lock bugs. This change was also applied to `MapUpdateModule`.
  Due to this change, some member functions were updated to take `NDT` as
  an argument. Whenever possible, these functions now take a reference to
  `NDT` instead of a `shared_ptr`. This is a best practice to prevent the
  pointer from leaking outside of `Guarded::with()`.
  Note: `MapUpdateModule::secondary_ndt_ptr\_` also has a missing lock bug.
  This will be fixed in a future commit and is not covered here.
  * refactor(ndt_scan_matcher): pass NDT by reference instead of std::shared_ptr
  When a function takes `std::shared_ptr<T>` as an argument, there should
  be a specific reason to use a pointer (such as sharing ownership).
  However, functions like `estimate_xy_covariance_by_laplace_approximation()`
  do not actually need these characteristics internally. By changing the
  arguments to take a reference instead, we reduce the risk of leaking
  pointers outside unnecessarily.
  Although this commit is not a bug fix, it is important as an example of
  a best practice when using `Guarded`: preventing internal data from
  leaking outside as much as possible.
  * refactor(ndt_scan_matcher): pre-commit
  * fix(ndt_scan_matcher): fix concurrency bug in secondary_ndt_ptr\_ using Guarded
  The `secondary_ndt_ptr\_` field in `MapUpdateModule` previously lacked
  lock protection. Since it can be accessed by multiple threads at the same
  time (from `NDTScanMatcher::service_ndt_align()` and
  `NDTScanMatcher::callback_timer()`), it needs to be protected.
  To fix this, we wrapped it with `Guarded` to add a lock.
  Generally, this data is accessed from within a *MutuallyExclusive* timer
  callback. Because it is not accessed frequently by multiple threads, we
  decided to use a larger lock scope for the basic processing.
  * refactor(ndt_scan_matcher): move heavy map update operations outside the lock
  To prevent the NDT scan matcher from being blocked during dynamic map loading,
  this commit moves the cloning and destruction of the NDT map out of the
  `ndt_ptr\_` lock scope. By using only a fast `std::swap` inside the lock,
  the align process is no longer delayed.
  * fix(ndt_scan_matcher): fix cppcheck errors
  ---------
  Co-authored-by: Shintaro Sakoda <shintaro.sakoda@tier4.jp>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: Junya Sasaki <junya.sasaki@tier4.jp>
* fix(ndt_scan_matcher): remove `setInputSource` (`#837 <https://github.com/mitsudome-r/autoware_core/issues/837>`_)
  * Removed `setInputSource`
  * Applied `pre-commit run -a`
  * Corrected according to cppcheck's indications
  * bug: fix wrongly remaining `getInputSource` (see below)
  * This bug is caused due to my previous conflict resolve
  ---------
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: Junya Sasaki <junya.sasaki@tier4.jp>
* refactor(autoware_core): add USE_SCOPED_HEADER_INSTALL_DIR to localization packages (`#984 <https://github.com/mitsudome-r/autoware_core/issues/984>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* fix(`ndt_scan_matcher`): guard input source access with mutex (`#991 <https://github.com/mitsudome-r/autoware_core/issues/991>`_)
  * bug(`ndt_scan_matcher`): fix to guard input source access with mutex
  * bug(`ndt_scan_matcher`): do not lock unnecessarily longer
  * This fix is according to the following review comment:
  ```
  Because the input\_ is copied within BaseRegType::operator=(other), we should only lock the guard when necessary.
  ```
  * bug: deep-copy to avoid data races (see below)
  * When the producer (callback timer) mutates the cloud after passing it in,
  there is a risk that the `input\_` pointcloud is updated unexpectedly after
  getting `input\_` by `getInputSource()`
  ---------
  Co-authored-by: Anh Nguyen <vietanhng17@gmail.com>
* feat(autoware_ndt_scan_matcher): apply autoware_agnocast_wrapper for CIE (`#966 <https://github.com/mitsudome-r/autoware_core/issues/966>`_)
  Co-authored-by: Koichi Imai <45482193+Koichi98@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* fix(autoware_ndt_scan_matcher): fix bugprone-narrowing-conversions warnings (`#929 <https://github.com/mitsudome-r/autoware_core/issues/929>`_)
* revert: "chore(`autoware_ndt_scan_matcher`)): move header files from `include` to `src`" (`#880 <https://github.com/mitsudome-r/autoware_core/issues/880>`_)
  This reverts commit ee1c5d1d4d70e43e8345d06402e1fcb3a4007db3.
* chore(`autoware_ndt_scan_matcher`): move header files from `include` to `src` (`#866 <https://github.com/mitsudome-r/autoware_core/issues/866>`_)
  * chore(ndt_scan_matcher): move headers from `include` to `src` not to be public
  * bug(`ndt_omp`): fix a missing copyright
  * bug(ndt_scan_matcher): remove unnecessary lines in `CMakeLists.txt`
  * style(pre-commit): autofix
  * bug: fix by `pre-commit`
  * bug: fix by `cppcheck`
  * style(pre-commit): autofix
  * bug: fix by `pre-commit`
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Junya Sasaki, NorahXiong, Ryohsuke Mitsudome, SakodaShintaro, Takumi Jin, Vishal Chauhan, atsushi yano, github-actions

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(ndt_scan_matcher): fix segmentation fault in MultiVoxelGridCovariance::createKdtree() (`#733 <https://github.com/autowarefoundation/autoware_core/issues/733>`_)
  * fix(ndt_scan_matcher): handle nullptr
  * fix: avoid wasting memory
  ---------
  Co-authored-by: Anh Nguyen <vietanhng17@gmail.com>
* perf(localization, sensing): reduce subscription queue size from 100 to 10 (`#751 <https://github.com/autowarefoundation/autoware_core/issues/751>`_)
* chore: jazzy-porting: fix test depend launch-test missing (`#738 <https://github.com/autowarefoundation/autoware_core/issues/738>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: tf2_ros to hpp headers (`#616 <https://github.com/autowarefoundation/autoware_core/issues/616>`_)
* Contributors: Shumpei Wakabayashi, Tim Clephas, Yutaka Kondo, github-actions, 心刚

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
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
* chore: jazzy-porting:fix missing header file (`#630 <https://github.com/autowarefoundation/autoware_core/issues/630>`_)
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Motz, Yutaka Kondo, mitsudome-r, 心刚

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_ndt_scan_matcher): extend timeout for test case (`#601 <https://github.com/autowarefoundation/autoware_core/issues/601>`_)
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(autoware_ndt_scan_matcher): port the package from Autoware Universe   (`#490 <https://github.com/autowarefoundation/autoware_core/issues/490>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Ryohsuke Mitsudome, github-actions

* fix: to be consistent version in all package.xml(s)
* feat(autoware_ndt_scan_matcher): port the package from Autoware Universe   (`#490 <https://github.com/autowarefoundation/autoware_core/issues/490>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Ryohsuke Mitsudome, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-22)
------------------

0.2.0 (2025-02-07)
------------------

0.0.0 (2024-12-02)
------------------
