^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_lanelet2_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* refactor(autoware_lanelet2_utils)!: move everything to namespace experimental (`#372 <https://github.com/autowarefoundation/autoware_core/issues/372>`_)
* refactor(autoware_lanelet2_utils): rewrite using modern C++ without API breakage (`#347 <https://github.com/autowarefoundation/autoware_core/issues/347>`_)
  * refactor using modern c++
  * precommit
  * fix
  * fix
  * precommit
  * use std::strcmp
  * precommit
  * Revert "refactor using modern c++"
  This reverts commit 3f7e4953c08f5237dc3bc75db3d896cc9c0640a3.
  ---------
* Contributors: Mamoru Sobue, Yutaka Kondo

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* feat(autoware_lanelet2_utils): add hatched_road_markings utility (`#565 <https://github.com/autowarefoundation/autoware_core/issues/565>`_)
* Contributors: Ryohsuke Mitsudome, Yukinari Hisaki

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(autoware_trajectory): implement a function to construct trajectory class for reference path (`#469 <https://github.com/autowarefoundation/autoware_core/issues/469>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* test(autoware_lanelet2_utils): fix threshold to avoid precision-related failures (`#506 <https://github.com/autowarefoundation/autoware_core/issues/506>`_)
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
* feat(autoware_lanelet2_utils): refactor interpolation and extrapolation for lanelet2 points and line (`#373 <https://github.com/autowarefoundation/autoware_core/issues/373>`_)
  * task 1
  * style(pre-commit): autofix
  * completed all task, change all function to reutn optional point
  * Added getLineStringFromArcLength and get_pose_from_2d_arc_length
  * remove unnecessary functions
  * fixed some review
  * temp
  * fix conflict
  * review fix
  * style(pre-commit): autofix
  * fixed unused function in header
  * fixed review and cleaned up unnecessary variable
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_lanelet2_utils): refactor stop line retrieval functions into lanelet2_utils (`#327 <https://github.com/autowarefoundation/autoware_core/issues/327>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* feat: add dense centerline map (`#399 <https://github.com/autowarefoundation/autoware_core/issues/399>`_)
  * Add map of　dense centerline
  * style(pre-commit): autofix
  * Correction of README.md
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(autoware_lanelet2_utils)!: move everything to namespace experimental (`#372 <https://github.com/autowarefoundation/autoware_core/issues/372>`_)
* refactor(autoware_lanelet2_utils): rewrite using modern C++ without API breakage (`#347 <https://github.com/autowarefoundation/autoware_core/issues/347>`_)
  * refactor using modern c++
  * precommit
  * fix
  * fix
  * precommit
  * use std::strcmp
  * precommit
  * Revert "refactor using modern c++"
  This reverts commit 3f7e4953c08f5237dc3bc75db3d896cc9c0640a3.
  ---------
* Contributors: Giovanni Muhammad Raditya, Mamoru Sobue, Sarun MUKDAPITAK, Yutaka Kondo, github-actions, yukage-oya

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* chore(lanelet2_utils): change header directory structure (`#274 <https://github.com/autowarefoundation/autoware.core/issues/274>`_)
* feat(lanelet2_utility): add intersection/turn_direction definition (`#244 <https://github.com/autowarefoundation/autoware.core/issues/244>`_)
* docs(lanelet2_utils): fix invalid drawio link and update image (`#251 <https://github.com/autowarefoundation/autoware.core/issues/251>`_)
  doc(lanelet2_utils): fix invalid drawio link and update image
* feat(lanelet2_utils)!: introduce lanelet2_utils by renaming from laneet2_utility (`#248 <https://github.com/autowarefoundation/autoware.core/issues/248>`_)
* Contributors: Mamoru Sobue, mitsudome-r
