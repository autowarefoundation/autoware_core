^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_perception_objects_converter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_perception_objects_converter): extract pure convert() and reuse autoware_utils_uuid (`#1133 <https://github.com/autowarefoundation/autoware_core/issues/1133>`_)
  Extract the DetectedObject->PredictedObject mapping into a pure, ROS-free
  convert() / convert_object() seam in an internal conversion.hpp so the field
  mapping (header, classification, shape, kinematics, has_twist branch) is unit
  testable without spinning a node. The subscription callback now just
  take-converts-and-publishes.
  Replace the bespoke generateUUIDMsg() (boost random_generator + std::copy,
  external-linkage free function) with autoware_utils_uuid::generate_uuid(),
  drop the three boost/uuid includes, swap the 'boost' dependency for
  'autoware_utils_uuid' in package.xml, and make the UUID source injectable so
  tests can assert id assignment deterministically.
  Reserve the output objects vector and emplace converted objects to avoid
  per-object reallocations and the intermediate kinematics copy; publish via
  std::move(unique_ptr) for zero-copy intra-process delivery. Behavior preserving.
  Add a gtest covering header propagation, exact field mapping, both has_twist
  branches, distinct-UUID-per-object, and the production default generator
  (non-zero, distinct UUIDs).
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
* Contributors: Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------

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
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat: add autoware_perception_objects_converter (`#458 <https://github.com/autowarefoundation/autoware_core/issues/458>`_)
  Co-authored-by: suchang <chang.su@autocore.ai>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: github-actions, storrrrrrrrm

* fix: to be consistent version in all package.xml(s)
* feat: add autoware_perception_objects_converter (`#458 <https://github.com/autowarefoundation/autoware_core/issues/458>`_)
  Co-authored-by: suchang <chang.su@autocore.ai>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: github-actions, storrrrrrrrm

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-22)
------------------

0.2.0 (2025-02-07)
------------------

0.0.0 (2024-12-02)
------------------
