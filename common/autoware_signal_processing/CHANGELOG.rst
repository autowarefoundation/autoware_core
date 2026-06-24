^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_signal_processing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* test(autoware_signal_processing): cover Butterworth bilinear path and guard printContinuousTimeTF (`#1146 <https://github.com/autowarefoundation/autoware_core/issues/1146>`_)
  * test(autoware_signal_processing): cover Butterworth bilinear path and lowpass gain
  Strengthen test coverage without changing any public signatures or source:
  - Add ground-truth tests for the default (non-sampling) bilinear discrete-TF
  path (order 1 and 2), which previously had only a single weak end-to-end
  check via the order-5 Buttord case. Values cross-checked against
  scipy.signal.bilinear.
  - Pin the setCutOffFrequency(fc, fs) fc >= fs/2 invalid-argument guard, which
  had no test, by asserting filter_specs\_ is left unchanged.
  - Assert poly()/getAnBn() consistency through a known 2nd-order case and add a
  LowpassFilter1d::setGain test.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * test(autoware_signal_processing): address review feedback
  - Correct printFilterContinuousTimeRoots() doc comment to describe the
  continuous-time roots it actually prints, not the order/cutoff.
  - Guard printContinuousTimeTF() against indexing the denominator past its
  end when called before computeContinuousTimeTF(), and cover it with a
  regression test.
  - Add inline cspell:ignore for the math term "monic".
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  ---------
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
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* Contributors: Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat(autoware_signal_processing): port autoware_signal_processing from Autoware Universe (`#303 <https://github.com/autowarefoundation/autoware_core/issues/303>`_)
* Contributors: Ryohsuke Mitsudome
