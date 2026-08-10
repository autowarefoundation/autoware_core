^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_qp_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_qp_interface): report real OSQP solver status and drop redundant vector copies (`#1100 <https://github.com/autowarefoundation/autoware_core/issues/1100>`_)
  * fix(autoware_qp_interface): report real OSQP solver status and drop redundant vector copies
  OSQPInterface::getStatus() was hardcoded to always return "OSQP_SOLVED",
  masking max-iteration/infeasible/unsolved outcomes; only isSolved()
  discriminated. Extract a pure status_to_string(c_int) helper (internal,
  non-installed header) that maps the OSQP\_* status codes to their string
  names, and have getStatus() report latest_work_info\_.status_val.
  Also remove the needless per-call std::vector copies in updateQ/L/U/Bounds
  (the OSQP C API takes const c_float*, so the vector data is passed directly)
  and replace the q/l/u copies in the ProxQP path with
  Eigen::Map<const Eigen::VectorXd> over the const data pointers.
  Add unit tests for status_to_string across status values and a
  primal-infeasible problem asserting isSolved()==false and a non-solved
  status. Update the existing UpdateSettingsAfterInitialization test, which
  only passed due to the hardcoded status, to assert the true outcome
  (OSQP_PRIMAL_INFEASIBLE).
  Public headers and the node interface are unchanged.
  * refactor(autoware_qp_interface): keep status_to_string internal-linkage and fold status tests
  Address review: move status_to_string() into an anonymous namespace in
  osqp_interface.cpp (no exported symbol), drop the internal
  src/osqp_interface_status.hpp and its PRIVATE include dir, remove the
  change-detector StatusToString test, and fold the two behavioral tests
  (InfeasibleProblemReportsNotSolved, FreshlyConstructedIsNotSolved) into
  test_osqp_interface.cpp. The bug fix (getStatus() reporting the real solver
  status) stays covered by behavioral tests through the public API.
  ---------
* Contributors: Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_qp_interface): fix bugprone-narrowing-conversions warnings (`#922 <https://github.com/mitsudome-r/autoware_core/issues/922>`_)
  * fix(autoware_qp_interface): fix bugprone-narrowing-conversions warnings
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: NorahXiong, github-actions

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
* refactor(autoware_qp_interface): rewrite using modern C++ without API breakage (`#400 <https://github.com/autowarefoundation/autoware_core/issues/400>`_)
* Contributors: Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-21)
------------------
* chore: rename from `autoware.core` to `autoware_core` (`#290 <https://github.com/autowarefoundation/autoware.core/issues/290>`_)
* test(autoware_qp_interface): add unit tests for initializeProblem (`#237 <https://github.com/autowarefoundation/autoware.core/issues/237>`_)
  * test(autoware_qp_interface): add unit tests for QPInterface::initializeProblem
  * test(autoware_qp_interface): add unit tests for class OSQPInterface
  * test(autoware_qp_interface): add unit tests for class ProxQPInterface
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: NorahXiong, Yutaka Kondo

0.2.0 (2025-02-07)
------------------
* unify version to 0.1.0
* update changelog
* fix(autoware_qp_interface): incorrect parameter passing in delegating constructor (`#147 <https://github.com/autowarefoundation/autoware_core/issues/147>`_)
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* feat(autoware_qp_interface): porting autoware_qp_interface package from autoware_universe (`#146 <https://github.com/autowarefoundation/autoware_core/issues/146>`_)
  * feat(autoware_qp_interface): porting autoware_qp_interface package from autoware_universe
  * Delete CHANGELOG.rst since it's outdated
  * Update common/autoware_qp_interface/package.xml
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: NorahXiong, Yutaka Kondo

* fix(autoware_qp_interface): incorrect parameter passing in delegating constructor (`#147 <https://github.com/autowarefoundation/autoware_core/issues/147>`_)
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* feat(autoware_qp_interface): porting autoware_qp_interface package from autoware_universe (`#146 <https://github.com/autowarefoundation/autoware_core/issues/146>`_)
  * feat(autoware_qp_interface): porting autoware_qp_interface package from autoware_universe
  * Delete CHANGELOG.rst since it's outdated
  * Update common/autoware_qp_interface/package.xml
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: NorahXiong

0.0.0 (2024-12-02)
------------------
