^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_interpolation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* refactor(interpolation): use `autoware_utils\_*` instead of `autoware_utils` (`#382 <https://github.com/autowarefoundation/autoware_core/issues/382>`_)
  feat(interpolation): use split autoware utils
* fix(autoware_path_optimizer): incorrect application of input velocity due to badly mapping output trajectory to input trajectory (`#355 <https://github.com/autowarefoundation/autoware_core/issues/355>`_)
  * changes to avoid improper mapping
  * Update common/autoware_motion_utils/include/autoware/motion_utils/trajectory/trajectory.hpp
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Arjun Jagdish Ram, Takagi, Isamu

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* perf(autoware_interpolation): scalar spline-eval overloads remove per-knot vector allocations (`#1127 <https://github.com/autowarefoundation/autoware_core/issues/1127>`_)
  * perf(autoware_interpolation): scalar spline-eval overloads remove per-knot vector allocations
  Add scalar getSplineInterpolatedValue / getSplineInterpolatedDiffValue /
  getSplineInterpolatedQuadDiffValue overloads to SplineInterpolation and route the
  per-knot loops in SplineInterpolationPoints2d (point, yaw, curvature,
  updateCurvatureSpline, extendLinearlyForward, projectPointOntoSpline) through them.
  Previously each per-knot evaluation built a freshly heap-allocated single-element
  std::vector ({s}) and received another single-element std::vector back, costing O(N)
  allocations to evaluate an N-knot spline. The new scalar overloads evaluate a single
  key in place with no allocation; the existing vector overloads now delegate to them,
  removing the duplicated Horner/derivative arithmetic.
  Also drop the dead validated_query_keys copies in the three vector getters: the
  cropped result of validateKeys() was never read (the loops iterate the original
  query_keys), so the call is kept only for its throwing precondition side effect and
  the unused full-length copy of query_keys is no longer allocated. Add reserve() to
  getSplineInterpolatedYaws and the extendLinearlyForward extension vectors.
  Behavior-preserving: the scalar overloads reproduce the exact per-key arithmetic
  (get_index + Horner) of the vector loop bodies, and all per-knot call sites already
  clamp the key into the knot range before evaluating, so dropping the per-call
  validateKeys() (which never threw for in-range keys) changes nothing. The public
  getSplineInterpolatedPointAt entry point keeps the vector overload to preserve its
  validateKeys() out-of-range precondition. Public API change is additive only.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * perf(autoware_interpolation): address review feedback
  - Guard scalar spline-eval overloads against un-built (default-constructed)
  splines: get_index() now throws std::runtime_error when base_keys\_ has
  fewer than 2 knots, preventing std::clamp(lo>hi) UB and out-of-bounds
  reads. Add a unit test covering the un-built-spline path.
  - Restore validateKeys() endpoint cropping in the vector getters: iterate
  the validated (cropped) query keys instead of the raw query_keys, so
  near-boundary floating-point queries are clamped as before the refactor.
  - Reserve extended_s to target_n_knots in extendLinearlyForward() to avoid
  reallocations while appending extended knots.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  ---------
* Contributors: Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_core): add USE_SCOPED_HEADER_INSTALL_DIR to common and testing packages (`#967 <https://github.com/mitsudome-r/autoware_core/issues/967>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* fix(autoware_interpolation): fix bugprone-narrowing-conversions warnings (`#935 <https://github.com/mitsudome-r/autoware_core/issues/935>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* Contributors: NorahXiong, Vishal Chauhan, github-actions

1.7.0 (2026-02-14)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore(autoware_motion_utils, autoware_trajectory, autoware_interpolation): add maintainers for packages (`#821 <https://github.com/autowarefoundation/autoware_core/issues/821>`_)
  * add maintainers for autoware_interpolation
  * add maintainers for autoware_motion_utils
  * add maintainers for autoware_trajectory
  ---------
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* chore(autoware_interpolation): add a maintainer (`#818 <https://github.com/autowarefoundation/autoware_core/issues/818>`_)
* feat(autoware_interpolation): exposing access to coefficients (`#671 <https://github.com/autowarefoundation/autoware_core/issues/671>`_)
  * Changes to spline_interpolation code to expose access to spline coefficients
  * fixes
  * style(pre-commit): autofix
  * removed unnecessary comments
  * Added includes
  * add includes
  * added comment
  * style(pre-commit): autofix
  * unit-testing for new public functions
  * style(pre-commit): autofix
  * extra unit tests
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taiki Yamada <129915538+TaikiYamada4@users.noreply.github.com>
* perf(interpolation): optimize calc_closest_segment_indices (`#797 <https://github.com/autowarefoundation/autoware_core/issues/797>`_)
* Contributors: Arjun Jagdish Ram, Junya Sasaki, Maxime CLEMENT, Ryohsuke Mitsudome, mkquda

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
* fix(interpolation): ensure consistent output size in splineYawFromPoints (`#670 <https://github.com/autowarefoundation/autoware_core/issues/670>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Maxime CLEMENT, Mete Fatih Cırıt, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* fix(motion_utils): update motion utils trajectory (`#569 <https://github.com/autowarefoundation/autoware_core/issues/569>`_)
* Contributors: Ryohsuke Mitsudome, Yuki TAKAGI

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* chore: no longer support ROS 2 Galactic (`#492 <https://github.com/autowarefoundation/autoware_core/issues/492>`_)
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* refactor(interpolation): use `autoware_utils\_*` instead of `autoware_utils` (`#382 <https://github.com/autowarefoundation/autoware_core/issues/382>`_)
  feat(interpolation): use split autoware utils
* fix(autoware_path_optimizer): incorrect application of input velocity due to badly mapping output trajectory to input trajectory (`#355 <https://github.com/autowarefoundation/autoware_core/issues/355>`_)
  * changes to avoid improper mapping
  * Update common/autoware_motion_utils/include/autoware/motion_utils/trajectory/trajectory.hpp
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Arjun Jagdish Ram, Takagi, Isamu, Tim Clephas, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* fix(autoware_interpolation): add missing dependencies (`#235 <https://github.com/autowarefoundation/autoware.core/issues/235>`_)
* feat: port autoware_interpolation from autoware.universe (`#149 <https://github.com/autowarefoundation/autoware.core/issues/149>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* Contributors: mitsudome-r, ralwing, shulanbushangshu
