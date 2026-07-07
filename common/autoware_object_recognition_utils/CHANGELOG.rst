^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_object_recognition_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* refactor(autoware_object_recognition_utils): use `autoware_utils\_*` instead of `autoware_utils` (`#385 <https://github.com/autowarefoundation/autoware_core/issues/385>`_)
  use autoware_utils\_*
* Contributors: Yutaka Kondo

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(object_recognition_utils): allow lowercase to convert string label to label enum (`#1184 <https://github.com/autowarefoundation/autoware_core/issues/1184>`_)
  feat: allow lowercase to convert string label to label enum
* refactor(autoware_object_recognition_utils): make transform.hpp testable (`#1129 <https://github.com/autowarefoundation/autoware_core/issues/1129>`_)
  * refactor(autoware_object_recognition_utils): make transform.hpp testable
  Move the global `namespace detail` TF helpers (getTransform/getTransformMatrix)
  into autoware::object_recognition_utils::detail to stop leaking them into the
  global ::detail namespace (an ODR/symbol-collision hazard for any installed
  header consumer), and extract the per-object transform math into pure helpers
  (applyTransformToObjects / applyTransformToFeatureObjects) that take an
  already-resolved tf2::Transform (and Eigen::Matrix4f). transformObjects and
  transformObjectsWithFeature become thin lookup+apply wrappers, with their public
  template signatures unchanged. This adds an additive, buffer-free testing seam.
  Add test/src/test_transform.cpp covering the identity passthrough branch, the
  missing-transform failure path, the pure pose/covariance math (pure translation
  and 90-degree yaw), and the end-to-end lookup+apply path through a live
  tf2_ros::Buffer seeded with a static transform. transform.hpp previously had
  zero test coverage.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * refactor(autoware_object_recognition_utils): avoid redundant deep copy in transform helpers
  Pass the already-copied output_msg as the helper input on the success
  path so applyTransformToObjects / applyTransformToFeatureObjects skip
  their internal output_msg = input_msg copy (self-assignment no-op). The
  transforms are applied in-place per object, so input == output aliasing
  is safe and the output for every code path is unchanged.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * test(autoware_object_recognition_utils): cover feature-object transform via Core-local stand-in type (`#74 <https://github.com/autowarefoundation/autoware_core/issues/74>`_)
  The feature-object transform overloads (transformObjectsWithFeature and the
  extracted detail::applyTransformToFeatureObjects) are duck-typed function
  templates: they only touch msg.header, the per-object
  pose_with_covariance.pose, and feature.cluster (a PointCloud2). They are
  never instantiated inside Core, and the concrete wire type
  tier4_perception_msgs::msg::DetectedObjectsWithFeature lives outside Core.
  Instead of depending on tier4_perception_msgs (a non-Core package) or
  guarding the tests behind __has_include, instantiate the templates against
  a small Core-local stand-in struct built from Core-available message types
  (autoware_perception_msgs::msg::DetectedObject + sensor_msgs PointCloud2).
  This gives the feature path the same coverage as the objects path -- direct
  helper unit tests with hand-computed pose/cluster oracles and an end-to-end
  test through a live tf2_ros::Buffer -- with no extra dependency, so the
  tests actually compile and run in Core CI.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  ---------
* feat(object_recognition_utils): templatize `object_recognition_utils` buffer (`#1120 <https://github.com/autowarefoundation/autoware_core/issues/1120>`_)
  * templatize object_recognition_utils
  * reflect copilot review about header files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(object_recognition_utils): support ANIMAL and HAZARD labels (`#1088 <https://github.com/autowarefoundation/autoware_core/issues/1088>`_)
* Contributors: Koichi Imai, Kotaro Uetake, Yoshi Ri, Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_core): add USE_SCOPED_HEADER_INSTALL_DIR to common and testing packages (`#967 <https://github.com/mitsudome-r/autoware_core/issues/967>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* fix(autoware_object_recognition_utils): fix bugprone-narrowing-convertions warnings (`#924 <https://github.com/mitsudome-r/autoware_core/issues/924>`_)
  fix(autoware_object_recognition_utils): fix bugprone-narrowing-conversions warnings
* Contributors: NorahXiong, Vishal Chauhan, github-actions

1.7.0 (2026-02-14)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(object_recognition_utils): init empty tf2::Transform and use the transform (`#802 <https://github.com/autowarefoundation/autoware_core/issues/802>`_)
* feat(object_recognition_utils): add test for public api compile check (`#804 <https://github.com/autowarefoundation/autoware_core/issues/804>`_)
* Contributors: Mete Fatih Cırıt, Ryohsuke Mitsudome

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
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* chore: no longer support ROS 2 Galactic (`#492 <https://github.com/autowarefoundation/autoware_core/issues/492>`_)
* refactor(autoware_obect_recognition_utils): rewrite using modern C++ without API breakage (`#396 <https://github.com/autowarefoundation/autoware_core/issues/396>`_)
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* refactor(autoware_object_recognition_utils): use `autoware_utils\_*` instead of `autoware_utils` (`#385 <https://github.com/autowarefoundation/autoware_core/issues/385>`_)
  use autoware_utils\_*
* Contributors: Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* fix(autoware_object_recognition_utils): add missing include cstdint for std::uint8_t (`#314 <https://github.com/autowarefoundation/autoware_core/issues/314>`_)
* Contributors: Shane Loretz

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* feat(autoware_object_recognition_utils): move package to core (`#232 <https://github.com/autowarefoundation/autoware.core/issues/232>`_)
* Contributors: mitsudome-r, 心刚
