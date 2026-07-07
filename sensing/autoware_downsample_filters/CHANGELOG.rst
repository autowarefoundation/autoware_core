^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_downsample_filters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(`downsample_filters`): add missing unit tests (`#1206 <https://github.com/autowarefoundation/autoware_core/issues/1206>`_)
* refactor(`downsample_filters`): split voxel-grid core logic (`#1175 <https://github.com/autowarefoundation/autoware_core/issues/1175>`_)
  * refactor(`downsample_filters`): split voxel-grid core logic
  * extract ROS-independent voxel grid core logic from the node
  * replace callback-based errors in faster voxel filter with ValidationResult
  * propagate validation reasons through core and node logging
  * style(pre-commit): autofix
  * refactor(`downsample_filters`): move field validation into validate_input
  * move x/y/z and intensity field checks from set_field_offsets to validate_input
  * simplify set_field_offsets to a plain void setter
  * remove redundant result check in filter (validation is now upstream)
  * bug: fix the following `cppcheck` error
  ```
  /__w/autoware_core/autoware_core/sensing/autoware_downsample_filters/src/voxel_grid_downsample_filter/voxel_grid_downsample_filter_core.hpp:42:20: performance: inconclusive: Technically the member function 'autoware::downsample_filters::VoxelGridDownsampleFilterCore::validate_input' can be static (but you may consider moving to unnamed namespace). [functionStatic]
  ValidationResult validate_input(const PointCloud2 & cloud) const;
  ^
  /__w/autoware_core/autoware_core/sensing/autoware_downsample_filters/src/voxel_grid_downsample_filter/voxel_grid_downsample_filter_core.cpp:31:49: note: Technically the member function 'autoware::downsample_filters::VoxelGridDownsampleFilterCore::validate_input' can be static (but you may consider moving to unnamed namespace).
  ValidationResult VoxelGridDownsampleFilterCore::validate_input(const PointCloud2 & cloud) const
  ^
  /__w/autoware_core/autoware_core/sensing/autoware_downsample_filters/src/voxel_grid_downsample_filter/voxel_grid_downsample_filter_core.hpp:42:20: note: Technically the member function 'autoware::downsample_filters::VoxelGridDownsampleFilterCore::validate_input' can be static (but you may consider moving to unnamed namespace).
  ValidationResult validate_input(const PointCloud2 & cloud) const;
  ^
  ```
  * bug: fix by `pre-commit`
  * refactor(`downsample_filters`): align voxel-grid core file naming
  * rename voxel_grid_downsample_filter_core.cpp/hpp to voxel_grid_downsample_filter.cpp/hpp
  * fix: make `set_field_offsets` private
  * Apply the following review comment:
  - https://github.com/autowarefoundation/autoware_core/pull/1175#discussion_r3433757009
  * refactor(`downsample_filters`): make voxel filter return `tl::expected`
  * change VoxelGridDownsampleFilterCore::filter to return tl::expected<PointCloud2, std::string>
  * move input validation into filter to avoid accidental validation bypass
  * simplify node call site to early-return on error
  * update unit tests to use the new expected-based interface
  This fix is proposed by this PR review comment:
  - https://github.com/autowarefoundation/autoware_core/pull/1175#discussion_r3433873891
  * refactor(`downsample_filters`): set output stamp inside core filter
  * Apply the following review comment:
  - https://github.com/autowarefoundation/autoware_core/pull/1175#discussion_r3433884564
  * style(pre-commit): autofix
  * refactor(`downsample_filters`): align core/node naming conventions
  * rename core class to VoxelGridDownsampleFilter in voxel_grid_downsample_filter.cpp/hpp
  * rename ROS2 wrapper class to VoxelGridDownsampleFilterNode in *_node.cpp/hpp
  * refactor(`downsample_filters`): avoid copy when publishing filter output
  * refactor(`downsample_filters`): inline publish of moved filter result
  * bug: fix missing package `tl_expected`
  * bug: fix build error for test code
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(`downsample_filters`): remove TF operations from voxel grid filter node (`#1161 <https://github.com/autowarefoundation/autoware_core/issues/1161>`_)
  * refactor(`downsample_filters`): remove TF operations from voxel grid filter node
  * style(pre-commit): autofix
  * bug: fix wrong conflict resolve in a previous commit
  * style(pre-commit): autofix
  * bug: fix failing tests
  * bug: remove tf-handling code from tests
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix: a bug which causes wrong `frame_id` propagation in transformation (`#1164 <https://github.com/autowarefoundation/autoware_core/issues/1164>`_)
  * bug: fix wrong `frame_id` propagation in transformation
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(`downsample_filters`): add missing tests (`#1103 <https://github.com/autowarefoundation/autoware_core/issues/1103>`_)
  * fix(`downsample_filters`): add missing tests
  * style(pre-commit): autofix
  * chore: add a comment
  * style(pre-commit): autofix
  * fix: cover more test cases
  * Apply the following review comment:
  - https://github.com/autowarefoundation/autoware_core/pull/1103#pullrequestreview-4387947773
  * style(pre-commit): autofix
  * fix: temporary disble a buggy test
  * Apply the following review comment:
  - https://github.com/autowarefoundation/autoware_core/pull/1103#discussion_r3323122457
  * fix: code duplication
  * fix: by `pre-commit`
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(voxel_grid_downsample_filter): don't use deprecated PCL header (`#1069 <https://github.com/autowarefoundation/autoware_core/issues/1069>`_)
  pcl/io/io.h is deprecated since 2011. Use the current location of the
  file.
  This fixes the following build error on my system with PCL 1.15.1:
  /build/autoware_core-release-release-jazzy-autoware_downsample_filters-1.8.0-1/src/voxel_grid_downsample_filter/voxel_grid_downsample_filter_node.cpp:24:10:
  fatal error: pcl/io/io.h: No such file or directory
  24 | #include <pcl/io/io.h>
  |          ^~~~~~~~~~~~~
* Contributors: Junya Sasaki, Michal Sojka, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_downsample_filters): typo (`#1053 <https://github.com/mitsudome-r/autoware_core/issues/1053>`_)
* refactor(autoware_core): add USE_SCOPED_HEADER_INSTALL_DIR to sensing packages (`#985 <https://github.com/mitsudome-r/autoware_core/issues/985>`_)
  Co-authored-by: github-actions <github-actions@github.com>
* fix(autoware_downsample_filters): fix bugprone-narrowing-conversions warnings (`#937 <https://github.com/mitsudome-r/autoware_core/issues/937>`_)
  * fix(autoware_downsample_filters): fix bugprone-narrowing-conversions warnings
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_downsample_filters): fix bugprone-implicit-widening-of-multiplication-result warnings (`#913 <https://github.com/mitsudome-r/autoware_core/issues/913>`_)
* Contributors: NorahXiong, Vincent Richard, Vishal Chauhan, github-actions

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_downsample_filters): fix uninitialized pointer in random_downsample_filter (`#742 <https://github.com/autowarefoundation/autoware_core/issues/742>`_)
* Contributors: Yutaka Kondo, github-actions

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
* feat(autoware_downsample_filters): re-implementation pointcloud downsample filters according to autoware_pointcloud_preprocessor in autoware_universe (`#459 <https://github.com/autowarefoundation/autoware_core/issues/459>`_)
  Co-authored-by: Ryohsuke Mitsudome <ryohsuke.mitsudome@tier4.jp>
* Contributors: NorahXiong, github-actions

* fix: to be consistent version in all package.xml(s)
* feat(autoware_downsample_filters): re-implementation pointcloud downsample filters according to autoware_pointcloud_preprocessor in autoware_universe (`#459 <https://github.com/autowarefoundation/autoware_core/issues/459>`_)
  Co-authored-by: Ryohsuke Mitsudome <ryohsuke.mitsudome@tier4.jp>
* Contributors: NorahXiong, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-22)
------------------

0.2.0 (2025-02-07)
------------------

0.0.0 (2024-12-02)
------------------
