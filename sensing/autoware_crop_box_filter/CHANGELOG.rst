^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_crop_box_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* feat(autoware_utils): remove managed transform buffer (`#360 <https://github.com/autowarefoundation/autoware_core/issues/360>`_)
  * feat(autoware_utils): remove managed transform buffer
  * fix(autoware_ground_filter): redundant inclusion
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Amadeusz Szymko

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_crop_box_filter): initialize transform variables on TF acquisition failure (`#750 <https://github.com/autowarefoundation/autoware_core/issues/750>`_)
  When TF acquisition fails during initialization, the transform matrices
  and flags were left uninitialized, potentially leading to undefined
  behavior when used in pointcloud filtering.
  This commit ensures that when TF acquisition fails:
  - need_preprocess_transform\_ and need_postprocess_transform\_ are set to false
  - eigen_transform_preprocess\_ and eigen_transform_postprocess\_ are initialized to identity matrices
  This matches the behavior when frames are equal and prevents the use of
  uninitialized values.
* Contributors: Yutaka Kondo, github-actions

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* test: add unit tests and integration test for crop box filter (`#618 <https://github.com/autowarefoundation/autoware_core/issues/618>`_)
  * refactor(crop_box_filter): extract helper function from test
  * test(crop_box_filter): improve point comparison logic and update test cases
  * test(crop_box_filter): add tests for filter logic with negative=false
  * test(crop_box_filter): add test for filtering with zero input points
  * test(crop_box_filter): refactor tests to use helper function and parameter struct
  * test(crop_box_filter): add integration tests for tf
  * test(crop_box_filter): apply clang format and linter
  * test(crop_box_filter): remove comments
  * test(crop_box_filter): update comments indent
  * style(crop_box_filter): improve comment clarity in integration test
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Takahisa Ishikawa, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* fix(autoware_crop_box_filter): fix deprecated autoware_utils header (`#418 <https://github.com/autowarefoundation/autoware_core/issues/418>`_)
  * fix autoware_utils header
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* feat(autoware_utils): remove managed transform buffer (`#360 <https://github.com/autowarefoundation/autoware_core/issues/360>`_)
  * feat(autoware_utils): remove managed transform buffer
  * fix(autoware_ground_filter): redundant inclusion
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Amadeusz Szymko, Masaki Baba, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat(autoware_crop_box_filter): reimplementation in core repo (`#279 <https://github.com/autowarefoundation/autoware_core/issues/279>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Ryohsuke Mitsudome, 心刚
