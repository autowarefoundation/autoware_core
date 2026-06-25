^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_vehicle_velocity_converter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(vehicle_velocity_converter): remove unused frame_id parameter and warning log (`#1201 <https://github.com/autowarefoundation/autoware_core/issues/1201>`_)
  refactor(vehicle_velocity_converter): remove unused frame_id parameter and warning log
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(vehicle_velocity_converter): extract node from core logic (`#1192 <https://github.com/autowarefoundation/autoware_core/issues/1192>`_)
  * refactor(vehicle_velocity_converter): extract node into vehicle_velocity_converter_node
  Split the ROS node out of the core conversion logic, mirroring the autoware_stop_filter layout:
  - vehicle_velocity_converter.{hpp,cpp}: core convert() logic only, no longer depends on rclcpp::Node
  - vehicle_velocity_converter_node.{hpp,cpp}: new VehicleVelocityConverterNode class and component registration
  - split tests into test_vehicle_velocity_converter (pure logic) and test_vehicle_velocity_converter_node (pub/sub integration)
  - update CMakeLists.txt (separate _ros library, isolated gtest) and package.xml (ament_cmake_ros test dependency)
  * refactor(vehicle_velocity_converter): make convert a member of VehicleVelocityConverter
  Turn the free convert() function into a VehicleVelocityConverter class that holds the frame-invariant settings (speed_scale_factor, stddev_vx, stddev_wz) supplied at construction, mirroring the StopFilter/StopFilterNode layout. The node now owns a single converter\_ member instead of three loose doubles, and the callback becomes converter\_.convert(*msg). Conversion behavior is unchanged.
  * test(vehicle_velocity_converter): restructure node test with a fixture into AAA shape
  Move the ROS context, executor thread and test-side pub/sub wiring into a VehicleVelocityConverterNodeTest fixture (SetUp/TearDown + start_converter_node/publish_and_wait helpers) so the test body reads as plain Arrange/Act/Assert. Coverage is unchanged.
  * test(vehicle_velocity_converter): compare float-derived twist fields with EXPECT_FLOAT_EQ
  The VelocityReport velocity fields are float while the Twist fields are double, so the converted values carry float rounding. Comparing them with EXPECT_FLOAT_EQ removes the static_cast<double>(...) on the expected literals while keeping the assertions correct. Covariance entries stay EXPECT_DOUBLE_EQ as they are pure double.
  * test(vehicle_velocity_converter): drop redundant float suffixes and align node test helper
  EXPECT_FLOAT_EQ narrows both operands to float, so the F suffix on the expected literals is unnecessary. Also make the node test's make_velocity_report take double and cast internally, matching make_report in the logic test, so call sites use plain double literals.
  * style(vehicle_velocity_converter): clarify AAA pattern
  * test(vehicle_velocity_converter): poll for DDS discovery instead of fixed sleep
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: Tran Huu Nhat Huy <29034232+TranHuuNhatHuy@users.noreply.github.com>
* refactor(autoware_vehicle_velocity_converter): extract pure convert() and use XYZRPY_COV_IDX (`#1151 <https://github.com/autowarefoundation/autoware_core/issues/1151>`_)
  Extract the VelocityReport-to-TwistWithCovarianceStamped conversion out of the
  ROS subscription callback into a pure, namespace-level free function
  convert(msg, speed_scale_factor, stddev_vx, stddev_wz); the callback now just
  calls it and publishes. This adds a deterministic, ROS-free seam for unit
  testing the conversion math.
  Replace the hand-written row-major covariance indices ([i + j * 6]) with the
  named autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX enum,
  matching the sibling autoware_gyro_odometer usage and removing the most
  error-prone magic in the file.
  Rewrite the tests as deterministic synchronous convert() calls that assert the
  full 36-element covariance (diagonal values and all off-diagonal entries zero),
  the zeroed unused twist fields, verbatim header copy, and scale-factor/sign edge
  cases, keeping one thin node-instantiation smoke test for the ROS wiring. This
  removes the executor thread, 200ms setup sleep, and 10ms poll loop.
  Behavior-preserving: the published message is byte-for-byte identical to before
  (the covariance enum indices map to the same array positions).
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
* Contributors: Takahisa Ishikawa, Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_core): add USE_SCOPED_HEADER_INSTALL_DIR to sensing packages (`#985 <https://github.com/mitsudome-r/autoware_core/issues/985>`_)
  Co-authored-by: github-actions <github-actions@github.com>
* feat(autoware_vehicle_velocity_converter): adopt cie (`#965 <https://github.com/mitsudome-r/autoware_core/issues/965>`_)
  Co-authored-by: Koichi Imai <45482193+Koichi98@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* Contributors: Tetsuhiro Kawaguchi, Vishal Chauhan, github-actions

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* perf(localization, sensing): reduce subscription queue size from 100 to 10 (`#751 <https://github.com/autowarefoundation/autoware_core/issues/751>`_)
* Contributors: Yutaka Kondo, github-actions

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(vehicle_velocity_converter): add maintainer to vehicle_velocity_converter (`#638 <https://github.com/autowarefoundation/autoware_core/issues/638>`_)
  chore: add maintainer to vehicle_velocity_converter
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Motz, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* test(autoware_vehicle_velocity_converter): add unit tests (`#478 <https://github.com/autowarefoundation/autoware_core/issues/478>`_)
  * test(autoware_vehicle_velocity_converter): add unit tests
  * style(pre-commit): autofix
  * chore(autoware_vehicle_velocity_converter): unify coding style
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* Contributors: NorahXiong, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat(autoware_vehicle_velocity_converter): port  from Autoware Universe (`#325 <https://github.com/autowarefoundation/autoware_core/issues/325>`_)
  * feat: port autoware_vehicle_velocity_converter from Autoware Universe
  * chore: reset package version and remove CHANGELOG.rst
  * fix: fix include header bracket
  * fix: resolve InitializationList cppcheck error
  ---------
* Contributors: Ryohsuke Mitsudome
