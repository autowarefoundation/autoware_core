^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* fix(autoware_trajectory): avoid nan in align_orientation_with_trajectory_direction (`#398 <https://github.com/autowarefoundation/autoware_core/issues/398>`_)
  * fix(autoware_trajectory): avoid nan in align_orientation_with_trajectory_direction
  * tidy
  * remove eps
  ---------
* chore(autoware_trajectory): relax the warning condition of boundary check (`#393 <https://github.com/autowarefoundation/autoware_core/issues/393>`_)
  * chore(autoware_trajectory): relax the warning condition of boundary check
  * tidy
  ---------
* feat(trajectory): define distance threshold and refine restore() without API breakage(in experimental) (`#376 <https://github.com/autowarefoundation/autoware_core/issues/376>`_)
  * feat(trajectory): define distance threshold and refine restore
  * fix spell
  ---------
* feat(autoware_trajectory): add get_contained_lane_ids function (`#369 <https://github.com/autowarefoundation/autoware_core/issues/369>`_)
  * add get_contained_lane_ids
  * add unit test
  * remove assert
  ---------
* feat(trajectory): add pretty_build() function for Planning/Control component node (`#332 <https://github.com/autowarefoundation/autoware_core/issues/332>`_)
* refactor(autoware_trajectory)!: move everything to namespace experimetal (`#371 <https://github.com/autowarefoundation/autoware_core/issues/371>`_)
  refactor(autoware_trajectory)!: move everything to namespace experimental
* feat(trajectory): improve shift function and their documents (`#337 <https://github.com/autowarefoundation/autoware_core/issues/337>`_)
  * feat(trajectory): add populate function
  * update curvature figure for approximation desc
  * update align_orientation_with_trajectory_direction fig
  * finished trajectory classes
  * refactored shift
  * add comment
  * update error message
  ---------
* fix(autoware_trajectory): fix base_addition callback to work when Trajectory is moved (`#370 <https://github.com/autowarefoundation/autoware_core/issues/370>`_)
* fix(autoware_trajectory): check vector size check before accessing (`#365 <https://github.com/autowarefoundation/autoware_core/issues/365>`_)
  * fix(autoware_trajectory): check vector size check before accessing
  * update
  * minor fix
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* feat(autoware_trajectory): improve performance of get_underlying_base  (`#298 <https://github.com/autowarefoundation/autoware_core/issues/298>`_)
* feat(trajectory): add API documentation for trajectory class, add some utillity (`#295 <https://github.com/autowarefoundation/autoware_core/issues/295>`_)
* feat(trajectory): add API description, nomenclature, illustration, rename functions to align with nomenclature (`#292 <https://github.com/autowarefoundation/autoware_core/issues/292>`_)
  * feat(trajectory): add API description, nomenclature, illustration, rename functions to align with nomenclature
  * resurrect get_internal_base
  ---------
* chore: include iostream and link yaml-cpp for Jazzy (`#351 <https://github.com/autowarefoundation/autoware_core/issues/351>`_)
* Contributors: Mamoru Sobue, Tim Clephas, Yukinari Hisaki

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* feat(autoware_trajectory): make `supplement_lanelet_sequence()` public (`#560 <https://github.com/autowarefoundation/autoware_core/issues/560>`_)
  * make supplement_lanelet_sequence() public
  * fix to update new end arc length
  * extend lanelet sequence after loop detection
  * include necessary header
  * use struct instead of tuple
  ---------
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* feat(autoware_trajectory): porting findNearestIndex from motion_utils to autoware_trajectory package (`#507 <https://github.com/autowarefoundation/autoware_core/issues/507>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* Contributors: Giovanni Muhammad Raditya, Mitsuhiro Sakamoto, Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(autoware_trajectory): add specialized LaneIdsInterpolator for lane ID handling (`#528 <https://github.com/autowarefoundation/autoware_core/issues/528>`_)
  * feat(autoware_trajectory): add specialized LaneIdsInterpolator for lane ID handling
  Replace generic Stairstep interpolator with domain-specific LaneIdsInterpolator
  that implements lane ID interpolation logic with preference for single lane IDs.
  - Add LaneIdsInterpolator implementation
  - Update path_point_with_lane_id to use specialized interpolator
  - Add comprehensive tests for lane ID interpolation behavior
  - Includes domain knowledge for handling lane transitions
  * Update common/autoware_trajectory/test/test_interpolator.cpp
  * Update common/autoware_trajectory/test/test_interpolator.cpp
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* feat(autoware_trajectory): implement a function to construct trajectory class for reference path (`#469 <https://github.com/autowarefoundation/autoware_core/issues/469>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* feat: support ROS 2 Jazzy (`#487 <https://github.com/autowarefoundation/autoware_core/issues/487>`_)
  * fix ekf_localizer
  * fix lanelet2_map_loader_node
  * MUST REVERT
  * fix pybind
  * fix depend
  * add buildtool
  * remove
  * revert
  * find_package
  * wip
  * remove embed
  * find python_cmake_module
  * public
  * remove ament_cmake_python
  * fix autoware_trajectory
  * add .lcovrc
  * fix egm
  * use char*
  * use global
  * namespace
  * string view
  * clock
  * version
  * wait
  * fix egm2008-1
  * typo
  * fixing
  * fix egm2008-1
  * MUST REVERT
  * fix egm2008-1
  * fix twist_with_covariance
  * Revert "MUST REVERT"
  This reverts commit 93b7a57f99dccf571a01120132348460dbfa336e.
  * namespace
  * fix qos
  * revert some
  * comment
  * Revert "MUST REVERT"
  This reverts commit 7a680a796a875ba1dabc7e714eaea663d1e5c676.
  * fix dungling pointer
  * fix memory alignment
  * ignored
  * spellcheck
  ---------
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* feat(autoware_trajectory): enhance shift flexibility (`#456 <https://github.com/autowarefoundation/autoware_core/issues/456>`_)
  * feat(autoware_trajectory): enhance shift flexibility
  * add test
  * fix spell miss
  * remove unused
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* chore(autoware_trajectory): update includes to maintain backward compatibility (`#455 <https://github.com/autowarefoundation/autoware_core/issues/455>`_)
* feat(autoware_trajectory): enable downcast from Trajectory<Point> to Trajectory<Pose> (`#430 <https://github.com/autowarefoundation/autoware_core/issues/430>`_)
  * feat(autoware_trajectory): enable downcast from Trajectory<Point> to Trajectory<Pose>
  * add test
  * fix test
  ---------
* chore(autoware_trajectory): fix CMake to work with differential build (`#420 <https://github.com/autowarefoundation/autoware_core/issues/420>`_)
* feat(autoware_trajectory): improve accuracy of boundary by using binary search (`#391 <https://github.com/autowarefoundation/autoware_core/issues/391>`_)
  * feat(autoware_trajectory): improve accuracy of boundary by using binary search
  * add example
  * Update common/autoware_trajectory/src/utils/find_intervals.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * Update common/autoware_trajectory/src/utils/find_intervals.cpp
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  * define macro for avoiding compiler error
  * add unit test
  * use flag instead of optional
  * add missing include
  ---------
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix(autoware_trajectory): avoid nan in align_orientation_with_trajectory_direction (`#398 <https://github.com/autowarefoundation/autoware_core/issues/398>`_)
  * fix(autoware_trajectory): avoid nan in align_orientation_with_trajectory_direction
  * tidy
  * remove eps
  ---------
* chore(autoware_trajectory): relax the warning condition of boundary check (`#393 <https://github.com/autowarefoundation/autoware_core/issues/393>`_)
  * chore(autoware_trajectory): relax the warning condition of boundary check
  * tidy
  ---------
* feat(trajectory): define distance threshold and refine restore() without API breakage(in experimental) (`#376 <https://github.com/autowarefoundation/autoware_core/issues/376>`_)
  * feat(trajectory): define distance threshold and refine restore
  * fix spell
  ---------
* feat(autoware_trajectory): add get_contained_lane_ids function (`#369 <https://github.com/autowarefoundation/autoware_core/issues/369>`_)
  * add get_contained_lane_ids
  * add unit test
  * remove assert
  ---------
* feat(trajectory): add pretty_build() function for Planning/Control component node (`#332 <https://github.com/autowarefoundation/autoware_core/issues/332>`_)
* refactor(autoware_trajectory)!: move everything to namespace experimetal (`#371 <https://github.com/autowarefoundation/autoware_core/issues/371>`_)
  refactor(autoware_trajectory)!: move everything to namespace experimental
* feat(trajectory): improve shift function and their documents (`#337 <https://github.com/autowarefoundation/autoware_core/issues/337>`_)
  * feat(trajectory): add populate function
  * update curvature figure for approximation desc
  * update align_orientation_with_trajectory_direction fig
  * finished trajectory classes
  * refactored shift
  * add comment
  * update error message
  ---------
* fix(autoware_trajectory): fix base_addition callback to work when Trajectory is moved (`#370 <https://github.com/autowarefoundation/autoware_core/issues/370>`_)
* fix(autoware_trajectory): check vector size check before accessing (`#365 <https://github.com/autowarefoundation/autoware_core/issues/365>`_)
  * fix(autoware_trajectory): check vector size check before accessing
  * update
  * minor fix
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* feat(autoware_trajectory): improve performance of get_underlying_base  (`#298 <https://github.com/autowarefoundation/autoware_core/issues/298>`_)
* feat(trajectory): add API documentation for trajectory class, add some utillity (`#295 <https://github.com/autowarefoundation/autoware_core/issues/295>`_)
* feat(trajectory): add API description, nomenclature, illustration, rename functions to align with nomenclature (`#292 <https://github.com/autowarefoundation/autoware_core/issues/292>`_)
  * feat(trajectory): add API description, nomenclature, illustration, rename functions to align with nomenclature
  * resurrect get_internal_base
  ---------
* chore: include iostream and link yaml-cpp for Jazzy (`#351 <https://github.com/autowarefoundation/autoware_core/issues/351>`_)
* Contributors: Mamoru Sobue, Tim Clephas, Yukinari Hisaki, Yutaka Kondo, github-actions

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
