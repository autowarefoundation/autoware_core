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
