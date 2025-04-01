^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2025-03-31)
------------------
* feat(trajectory): remove default ctor and collect default setting in Builder (`#287 <https://github.com/autowarefoundation/autoware_core/issues/287>`_)
* fix(autoware_trajectory): fix linking issue with pybind11, and use non-deprecated tf2 headers (`#316 <https://github.com/autowarefoundation/autoware_core/issues/316>`_)
  * Fix linking issue with pybind11, and use non-deprecated tf2 headers
  * Use .hpp includes only
  * style(pre-commit): autofix
  * Remove redundant find_package(pybind11_vendor ...)
  * Undo whitespace change
  * Make pybind11 a test_depend
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Mamoru Sobue, Shane Loretz

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* feat(trajectory): improve comment, use autoware_pyplot for examples (`#282 <https://github.com/autowarefoundation/autoware.core/issues/282>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* feat(autoware_trajectory): use move semantics and return expected<T, E> for propagating failure reason (`#254 <https://github.com/autowarefoundation/autoware.core/issues/254>`_)
  Co-authored-by: Yukinari Hisaki <42021302+yhisaki@users.noreply.github.com>
* refactor(autoware_trajectory): use nodiscard for mutables, fix reference to scalar type (`#255 <https://github.com/autowarefoundation/autoware.core/issues/255>`_)
  * doc(lanelet2_utils): fix invalid drawio link and update image
  * fix
  * fix precommit errors
  ---------
  Co-authored-by: Y.Hisaki <yhisaki31@gmail.com>
* feat(autoware_trajectory): add trajectory point (`#233 <https://github.com/autowarefoundation/autoware.core/issues/233>`_)
  * add TrajectoryPoint class to templates
  * add tests
  * add method to_point for TrajectoryPoint type
  * change name of test to avoid name collision
  * add missing items
  * rename example name for clarity
  ---------
  Co-authored-by: Y.Hisaki <yhisaki31@gmail.com>
* fix(autoware_trajectory): fix a bug of align_orientation_with_trajectory_direction (`#234 <https://github.com/autowarefoundation/autoware.core/issues/234>`_)
  * fix bug of align_orientation_with_trajectory_direction
  * fixed in a better way
  * reflect comments
  * revert unnecessary changes
  ---------
* feat(autoware_trajecotry): add a conversion function from point trajectory to pose trajectory (`#207 <https://github.com/autowarefoundation/autoware.core/issues/207>`_)
  feat(autoware_trajecotry): add conversion function from point trajectory to pose trajectory
* fix(autoware_trajectory): fix a bug of example file (`#204 <https://github.com/autowarefoundation/autoware.core/issues/204>`_)
* chore(autoware_trajectory): resolve clang-tidy warning of example file (`#206 <https://github.com/autowarefoundation/autoware.core/issues/206>`_)
* feat(autoware_trajectory): add curvature_utils (`#205 <https://github.com/autowarefoundation/autoware.core/issues/205>`_)
* feat: porting `autoware_trajectory` from `autoware.universe` to `autoware.core` (`#188 <https://github.com/autowarefoundation/autoware.core/issues/188>`_)
  * add(autoware_trajectory): ported as follows (see below):
  * From `autoware.universe/common` to `autoware.core/common`
  * The history can be traced via:
  https://github.com/sasakisasaki/autoware.universe/tree/02733e7b2932ad0d1c3c9c3a2818e2e4229f2e92/common/autoware_trajectory
* Contributors: Junya Sasaki, Mamoru Sobue, Yukinari Hisaki, danielsanchezaran, mitsudome-r
