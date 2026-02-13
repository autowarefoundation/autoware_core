^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_object_recognition_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* refactor(autoware_object_recognition_utils): use `autoware_utils\_*` instead of `autoware_utils` (`#385 <https://github.com/autowarefoundation/autoware_core/issues/385>`_)
  use autoware_utils\_*
* Contributors: Yutaka Kondo

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
