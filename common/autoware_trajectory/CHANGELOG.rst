^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* fix(autoware_trajectory): avoid nan in align_orientation_with_trajectory_direction (`#398 <https://github.com/youtalk/autoware_core/issues/398>`_)
  * fix(autoware_trajectory): avoid nan in align_orientation_with_trajectory_direction
  * tidy
  * remove eps
  ---------
* chore(autoware_trajectory): relax the warning condition of boundary check (`#393 <https://github.com/youtalk/autoware_core/issues/393>`_)
  * chore(autoware_trajectory): relax the warning condition of boundary check
  * tidy
  ---------
* feat(trajectory): define distance threshold and refine restore() without API breakage(in experimental) (`#376 <https://github.com/youtalk/autoware_core/issues/376>`_)
  * feat(trajectory): define distance threshold and refine restore
  * fix spell
  ---------
* feat(autoware_trajectory): add get_contained_lane_ids function (`#369 <https://github.com/youtalk/autoware_core/issues/369>`_)
  * add get_contained_lane_ids
  * add unit test
  * remove assert
  ---------
* feat(trajectory): add pretty_build() function for Planning/Control component node (`#332 <https://github.com/youtalk/autoware_core/issues/332>`_)
* refactor(autoware_trajectory)!: move everything to namespace experimetal (`#371 <https://github.com/youtalk/autoware_core/issues/371>`_)
  refactor(autoware_trajectory)!: move everything to namespace experimental
* feat(trajectory): improve shift function and their documents (`#337 <https://github.com/youtalk/autoware_core/issues/337>`_)
  * feat(trajectory): add populate function
  * update curvature figure for approximation desc
  * update align_orientation_with_trajectory_direction fig
  * finished trajectory classes
  * refactored shift
  * add comment
  * update error message
  ---------
* fix(autoware_trajectory): fix base_addition callback to work when Trajectory is moved (`#370 <https://github.com/youtalk/autoware_core/issues/370>`_)
* fix(autoware_trajectory): check vector size check before accessing (`#365 <https://github.com/youtalk/autoware_core/issues/365>`_)
  * fix(autoware_trajectory): check vector size check before accessing
  * update
  * minor fix
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* feat(autoware_trajectory): improve performance of get_underlying_base  (`#298 <https://github.com/youtalk/autoware_core/issues/298>`_)
* feat(trajectory): add API documentation for trajectory class, add some utillity (`#295 <https://github.com/youtalk/autoware_core/issues/295>`_)
* feat(trajectory): add API description, nomenclature, illustration, rename functions to align with nomenclature (`#292 <https://github.com/youtalk/autoware_core/issues/292>`_)
  * feat(trajectory): add API description, nomenclature, illustration, rename functions to align with nomenclature
  * resurrect get_internal_base
  ---------
* chore: include iostream and link yaml-cpp for Jazzy (`#351 <https://github.com/youtalk/autoware_core/issues/351>`_)
* Contributors: Mamoru Sobue, Tim Clephas, Yukinari Hisaki

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
