^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_twist2accel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(twist2accel): apply `agnocast_wrapper::Node` to `twist2accel` (`#1189 <https://github.com/autowarefoundation/autoware_core/issues/1189>`_)
  * apply agnocast_wrapper::Node
  * fix to use ALLOCATE
  * fix Cmake
  * fix cpplint
  * disable test when agnocast enabled
  ---------
* refactor(autoware_twist2accel): arrange class interface (`#1186 <https://github.com/autowarefoundation/autoware_core/issues/1186>`_)
  * refactor(autoware_twist2accel): encapsulate dt and previous twist in AccelEstimator
  AccelEstimator::estimate() now takes only the current twist sample and owns
  the per-call state that previously lived in the node: it holds the previous
  twist, derives dt from successive header stamps, and returns a ready-to-publish
  AccelWithCovarianceStamped with the header copied from the input.
  Add TwistWithCovarianceStamped and Odometry overloads that normalize to a
  TwistStamped and delegate, so the node callbacks can pass the raw message
  directly and the estimate_accel helper is no longer needed.
  * style(autoware_twist2accel): remove redundant comments
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(autoware_twist2accel): extract pure AccelEstimator and add unit tests (`#1111 <https://github.com/autowarefoundation/autoware_core/issues/1111>`_)
  * refactor(autoware_twist2accel): extract pure AccelEstimator and add unit tests
  Extract the ROS-free acceleration math (dt-clamped finite difference plus
  6-channel LowpassFilter1d smoothing) out of Twist2Accel::estimate_accel into
  a pure, injectable AccelEstimator class, mirroring the sibling
  autoware_stop_filter::StopFilter pattern. estimate_accel becomes a thin
  wrapper that computes dt from the ROS clock, calls the estimator, fills the
  message, and publishes.
  This makes the previously untestable estimation logic exercisable without
  spinning a ROS node and adds gtest coverage for the no-smoothing-on-first-
  sample, finite-difference, low-pass smoothing, dt-clamp, channel-independence,
  and covariance-index branches.
  Behavior-preserving: the dt clamp (>= 1e-3), per-component LPF order, and the
  first-message zero/empty accel branch are unchanged. The accel covariance
  diagonal is now written via autoware_utils_geometry XYZRPY_COV_IDX named
  indices instead of magic i*6+i arithmetic (a gtest pins the equivalence), and
  the hot-path per-message std::make_shared copy is dropped by storing the
  previous twist as a value (std::optional). Dead tf2 / STL includes, the
  file-scope LowpassFilter1d using-declaration in the header, and the unused tf2
  package.xml dependency are removed; autoware_utils_geometry is added.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * refactor(autoware_twist2accel): clarify AccelEstimator docstring
  The class operates on geometry_msgs Twist/Accel value types, so the
  "ROS-free" wording was imprecise. Reword to state the actual property:
  no rclcpp::Node / spinning dependency.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * refactor(autoware_twist2accel): move accel covariance into pure AccelEstimator (`#85 <https://github.com/autowarefoundation/autoware_core/issues/85>`_)
  * refactor(autoware_twist2accel): move accel covariance into pure AccelEstimator
  The previous extraction left the acceleration covariance assignment in the
  node while only the acceleration values moved into the pure AccelEstimator.
  Per reviewer feedback, the covariance is core estimation logic and belongs
  with the value estimate.
  AccelEstimator::estimate now returns geometry_msgs::msg::AccelWithCovariance,
  filling the same constant diagonal covariance the node used to set (linear
  variance 1.0, angular variance 0.05, off-diagonal terms zero) via named
  constants g_linear_accel_variance / g_angular_accel_variance. The node simply
  assigns the core's AccelWithCovariance output, so the published covariance is
  byte-for-byte unchanged. The core stays pure with no new mutable shared state.
  Unit tests assert the covariance against hand-computed literals, and the node
  test adds a characterization assertion locking the published covariance.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * Potential fix for pull request finding
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
  ---------
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
  * test(autoware_twist2accel): assert covariance diagonal against named constants
  Address review: build the expected covariance array from g_linear_accel_variance /
  g_angular_accel_variance directly (pinning the index->variance structure and the
  zero off-diagonal), and drop the separate EXPECT_DOUBLE_EQ(constant, literal)
  change-detector asserts.
  ---------
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
* Contributors: Koichi Imai, Takahisa Ishikawa, Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_core): add USE_SCOPED_HEADER_INSTALL_DIR to localization packages (`#984 <https://github.com/mitsudome-r/autoware_core/issues/984>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* feat(autoware_twist2accel): adopt cie (`#964 <https://github.com/mitsudome-r/autoware_core/issues/964>`_)
* Contributors: Tetsuhiro Kawaguchi, Vishal Chauhan, github-actions

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
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* Contributors: Tim Clephas, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat(autoware_twist2accel): port autoware_twist2accel from Autoware Universe (`#302 <https://github.com/autowarefoundation/autoware_core/issues/302>`_)
* Contributors: Ryohsuke Mitsudome
