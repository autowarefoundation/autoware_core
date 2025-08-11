^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_ground_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* feat(autoware_utils): remove managed transform buffer (`#360 <https://github.com/autowarefoundation/autoware_core/issues/360>`_)
  * feat(autoware_utils): remove managed transform buffer
  * fix(autoware_ground_filter): redundant inclusion
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Amadeusz Szymko

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
* fix: deprecation of .h files in message_filters (`#467 <https://github.com/autowarefoundation/autoware_core/issues/467>`_)
  * fix: deprecation of .h files in message_filters
  * Update perception/autoware_ground_filter/include/autoware/ground_filter/node.hpp
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* fix(autoware_ground_filter): fix deprecated autoware_utils header (`#417 <https://github.com/autowarefoundation/autoware_core/issues/417>`_)
  * fix autoware_utils header
  * style(pre-commit): autofix
  * fix autoware_utils packages
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* feat(autoware_utils): remove managed transform buffer (`#360 <https://github.com/autowarefoundation/autoware_core/issues/360>`_)
  * feat(autoware_utils): remove managed transform buffer
  * fix(autoware_ground_filter): redundant inclusion
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Amadeusz Szymko, Masaki Baba, Tim Clephas, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat: re-implementation autoware_ground_filter as alpha quality from universe (`#311 <https://github.com/autowarefoundation/autoware_core/issues/311>`_)
  * add: `scan_ground_filter` from Autoware Universe
  **Source**: Files copied from [Autoware Universe](https://github.com/autowarefoundation/autoware_universe/tree/b8ce82e3759e50f780a0941ca8698ff52aa57b97/perception/autoware_ground_segmentation).
  **Scope**: Integrated `scan_ground_filter` into `autoware.core`.
  **Dependency Changes**: Removed dependencies on `autoware_pointcloud_preprocessor` to ensure compatibility within `autoware.core`.
  **Purpose**: Focus on making `scan_ground_filter` functional within the `autoware.core` environment.
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Junya Sasaki, Ryohsuke Mitsudome
