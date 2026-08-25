^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_kalman_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* fix(autoware_kalman_filter): fixed clang-tidy error (`#379 <https://github.com/autowarefoundation/autoware_core/issues/379>`_)
  * fix(autoware_kalman_filter): fixed clang-tidy error
  * remove comment
  ---------
* refactor(autoware_kalman_filter): rewrite using modern C++ without API breakage (`#346 <https://github.com/autowarefoundation/autoware_core/issues/346>`_)
  * refactor using modern c++
  * remove ctor/dtor
  * precommit
  * use eigen methods
  * Update common/autoware_kalman_filter/include/autoware/kalman_filter/kalman_filter.hpp
  ---------
* chore(autoware_kalman_filter): add maintainer (`#381 <https://github.com/autowarefoundation/autoware_core/issues/381>`_)
  * chore(autoware_kalman_filter): add maintainer
  * removed the maintainer with an invalid email address.
  * added members of the Localization / Mapping team as maintainers.
  * removed the duplicate entry.
  * fixed the deletion as the wrong entry was removed
  ---------
* Contributors: RyuYamamoto, Yutaka Kondo

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* perf(autoware_kalman_filter): use Cholesky solve in update() with full guard-branch test coverage (`#1094 <https://github.com/autowarefoundation/autoware_core/issues/1094>`_)
  * test(autoware_kalman_filter): cover validation and guard branches
  Add unit tests for the previously-untested KalmanFilter dimension
  validation FALSE branches (init/predict/update), the state and covariance
  getters, and a closed-form numerical-equivalence update check on a
  correlated covariance. Add TimeDelayKalmanFilter::updateWithDelay tests
  for the non-column-vector, non-square-R, R/C-row-mismatch, and
  non-positive-definite (LLT failure) rejection paths, capturing stderr so
  test output stays clean.
  * perf(autoware_kalman_filter): solve innovation covariance via Cholesky in update()
  Mirror the optimization already applied to TimeDelayKalmanFilter::updateWithDelay
  in the base KalmanFilter::update: compute the Kalman gain with a Cholesky (LLT)
  solve of the innovation covariance S = C P C^T + R instead of an explicit matrix
  inverse. This is faster and numerically more stable, and a failed decomposition
  now rejects updates whose innovation covariance is not positive definite, which the
  explicit inverse silently accepted with a meaningless gain (potentially corrupting
  downstream state in autoware_ekf_localizer).
  Also use noalias() in predict(x_next, A, Q) to drop the intermediate Eigen
  temporaries of A P A^T in that hot per-cycle path.
  The public bool API is unchanged; behavior is identical for valid inputs (verified
  against the closed-form solution) and fails safe for non-positive-definite ones.
  * test(autoware_kalman_filter): address review feedback
  - assert update against hand-computed numeric literals instead of a re-typed copy of the solver formula
  - strengthen the two-arg init test to verify stored state/covariance feed the filter math, keeping getXelement coverage
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * test(autoware_kalman_filter): de-tautologize the kf end-to-end test (`#78 <https://github.com/autowarefoundation/autoware_core/issues/78>`_)
  Replace the recomputed-with-the-same-Kalman-formula expectations in TEST(kalman_filter, kf) with independent hand-computed numeric literals for the predict and update steps (A=B=C=I, Q=0.01I, R=0.09I), with the derivations written out in comments. The previous expectations re-typed the implementation's own predict/update expressions, so a wrong formula would have passed in both places.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * test(autoware_kalman_filter): drop trivial init getter/setter test (`#88 <https://github.com/autowarefoundation/autoware_core/issues/88>`_)
  interimadd flagged the two-argument init test as covering only
  straightforward getters/setters and empty values, with no logic to
  break. The follow-up commit had reworked it into
  init_state_cov_feed_into_filter_math, but it still ultimately exercises
  init() storing x/P round-tripped through getters, so it adds little
  value. Remove it entirely per the reviewer's request.
  The remaining tests already assert against hand-computed numeric
  literals independent of the SUT (update_matches_closed_form_with_correlated_covariance)
  and check only the boolean return of updateWithDelay for rejection
  paths, so the other two review points are already satisfied on the PR
  head.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * fix(autoware_kalman_filter): log LLT failure in update() like updateWithDelay
  Address review: KalmanFilter::update() rejected non-positive-definite innovation
  covariance silently (returned false with no message), an observability asymmetry
  with TimeDelayKalmanFilter::updateWithDelay. Emit the same std::cerr diagnostic so
  a caller that gets false can tell why the update was rejected.
  ---------
* Contributors: Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* perf(autoware_kalman_filter): optimize `updateWithDelay` by replacing sparse matrix operations (`#739 <https://github.com/mitsudome-r/autoware_core/issues/739>`_)
  * Optimize `updateWithDelay` by replacing sparse matrix operations with block arithmetic
  * style(pre-commit): autofix
  * Add test
  * style(pre-commit): autofix
  * Update README.md for (Extended Kalman Filter)
  * style(pre-commit): autofix
  * Add more detailed comments
  * style(pre-commit): autofix
  * Handle negative delay step and refactor tests
  * Replace TDKF with full term
  * style(pre-commit): autofix
  * Remove LLM like comment
  * Address PR review comments: add input validation, NaN guard, and docs fix
  - Add dimension validation for y, R, and C matrices in updateWithDelay
  - Add NaN/Inf check on Kalman gain to match base KalmanFilter::update
  - Add [[unlikely]] branch hints on all error paths
  - Clarify "diagonal block" comment terminology
  - Fix README augmented state vector (x\_{k-d} -> x\_{k-d+1}) and clarify delay range
  - Add test for C matrix dimension mismatch
  * style(pre-commit): autofix
  * fix: remove [[unlikely]] C++20 attributes for C++17 compatibility
  The [[unlikely]] attribute is a C++20 feature that causes compilation
  errors under clang-tidy which enforces strict C++17 compliance.
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yamato Ando <yamato.ando@tier4.jp>
* refactor(autoware_core): add USE_SCOPED_HEADER_INSTALL_DIR to common and testing packages (`#967 <https://github.com/mitsudome-r/autoware_core/issues/967>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* fix(autoware_kalman_filter): fix bugprone-implicit-widening-of-multiplication-result warnings (`#910 <https://github.com/mitsudome-r/autoware_core/issues/910>`_)
* fix(autoware_kalman_filter): fix bugprone-narrowing-conversions warnings (`#930 <https://github.com/mitsudome-r/autoware_core/issues/930>`_)
* Contributors: Keita Morisaki, NorahXiong, Vishal Chauhan, github-actions

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
* chore: update maintainer (`#637 <https://github.com/autowarefoundation/autoware_core/issues/637>`_)
  * chore: update maintainer
  remove Takeshi Ishita
  * chore: update maintainer
  remove Kento Yabuuchi
  * chore: update maintainer
  remove Shintaro Sakoda
  * chore: update maintainer
  remove Ryu Yamamoto
  ---------
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Motz, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* fix(autoware_kalman_filter): fixed clang-tidy error (`#379 <https://github.com/autowarefoundation/autoware_core/issues/379>`_)
  * fix(autoware_kalman_filter): fixed clang-tidy error
  * remove comment
  ---------
* refactor(autoware_kalman_filter): rewrite using modern C++ without API breakage (`#346 <https://github.com/autowarefoundation/autoware_core/issues/346>`_)
  * refactor using modern c++
  * remove ctor/dtor
  * precommit
  * use eigen methods
  * Update common/autoware_kalman_filter/include/autoware/kalman_filter/kalman_filter.hpp
  ---------
* chore(autoware_kalman_filter): add maintainer (`#381 <https://github.com/autowarefoundation/autoware_core/issues/381>`_)
  * chore(autoware_kalman_filter): add maintainer
  * removed the maintainer with an invalid email address.
  * added members of the Localization / Mapping team as maintainers.
  * removed the duplicate entry.
  * fixed the deletion as the wrong entry was removed
  ---------
* Contributors: RyuYamamoto, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-21)
------------------
* chore: rename from `autoware.core` to `autoware_core` (`#290 <https://github.com/autowarefoundation/autoware.core/issues/290>`_)
* test(autoware_kalman_filter): add tests for missed lines (`#263 <https://github.com/autowarefoundation/autoware.core/issues/263>`_)
* Contributors: NorahXiong, Yutaka Kondo

0.2.0 (2025-02-07)
------------------
* unify version to 0.1.0
* update changelog
* feat: port autoware_kalman_filter from autoware_universe (`#141 <https://github.com/autowarefoundation/autoware_core/issues/141>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* Contributors: Yutaka Kondo, cyn-liu

* feat: port autoware_kalman_filter from autoware_universe (`#141 <https://github.com/autowarefoundation/autoware_core/issues/141>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* Contributors: cyn-liu

0.0.0 (2024-12-02)
------------------
