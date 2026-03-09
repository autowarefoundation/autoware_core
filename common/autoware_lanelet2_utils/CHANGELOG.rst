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

1.7.0 (2026-02-14)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(lanelet2_utils): add empty input handler for get_arc_coordinates (`#831 <https://github.com/autowarefoundation/autoware_core/issues/831>`_)
  * add empty input lanelets handler for get_arc_coordinates
  * add test ensure default ArcCoordinates
  ---------
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* docs(lanelet2_utils): add succeeding/preceding lanelet sequence illustration (`#830 <https://github.com/autowarefoundation/autoware_core/issues/830>`_)
  added succeeding/preceding lanelet sequences illustration
* feat(lanelet2_extension): port some lanelet2_extension query functions (2) (`#809 <https://github.com/autowarefoundation/autoware_core/issues/809>`_)
* feat(lanelet2_extension): port remaining lanelet2_extension utilities functions (`#760 <https://github.com/autowarefoundation/autoware_core/issues/760>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* feat(lanelet2_extension): port some lanelet2_extension query functions  (`#790 <https://github.com/autowarefoundation/autoware_core/issues/790>`_)
  * port left/right/all_neighbor_lanelets and their tests
  * replace getAllNeighbors* with ported functions
  * add neighbors order description and verify in test
  * add ported functions to API table
  * add example code snippet
  * Remove optional from return value
  Assume one lanelet is always in return value (itself)
  * change usage of left/right_lanelets (always check access)
  * fix README for example code snippet
  * fix cppcheck-differential
  * fix README description
  ---------
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* revert: feat(path_generator): use `route_manager` to handle route data `#725 <https://github.com/autowarefoundation/autoware_core/issues/725>`_ (`#801 <https://github.com/autowarefoundation/autoware_core/issues/801>`_)
  This reverts commit 2403cb5952d33db8e4a403d6cc7b368521d47f5e.
* feat(path_generator): use `route_manager` to handle route data (`#725 <https://github.com/autowarefoundation/autoware_core/issues/725>`_)
* Contributors: Mitsuhiro Sakamoto, Ryohsuke Mitsudome, Sarun MUKDAPITAK

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_lanelet2_utils): define remove_const in header (`#741 <https://github.com/autowarefoundation/autoware_core/issues/741>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* chore: jazzy-porting: fix test depend launch-test missing (`#738 <https://github.com/autowarefoundation/autoware_core/issues/738>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* docs(lanelet2_utils): add example usage to autoware_lanelet2_utils documentation (`#720 <https://github.com/autowarefoundation/autoware_core/issues/720>`_)
  * add lanelet2_utils geometry functions example
  * add conversion functions example
  * add kind functions example
  * fix conversion code line after pre-commit format
  * fix code snippet title typo
  * add hatched_road_markings functions example
  * fix cppcheck
  * add topology functions example
  * add intersection functions example
  * add lane_sequence functions example
  * add nn_search functions example
  * add map_handler functions example
  * add route_manager functions example
  * fix shadow variable in example_map_handler
  * fix small typo in example_map_handler
  * remove map_handler question comment
  * modify CMakeLists to support jazzy
  * Update common/autoware_lanelet2_utils/CMakeLists.txt
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* feat(trajectory): align build_reference_path with vm-01-11 spec (`#702 <https://github.com/autowarefoundation/autoware_core/issues/702>`_)
  Co-authored-by: mitukou1109 <mitukou1109@gmail.com>
* ci(pre-commit): autoupdate (`#723 <https://github.com/autowarefoundation/autoware_core/issues/723>`_)
  * pre-commit formatting changes
* Contributors: Mamoru Sobue, Mete Fatih Cırıt, Sarun MUKDAPITAK, github-actions, 心刚

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(lanelet2_utils): organize maps by vm-map-spec id (`#716 <https://github.com/autowarefoundation/autoware_core/issues/716>`_)
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_lanelet2_utils): port get_conflicting_lanelets (`#704 <https://github.com/autowarefoundation/autoware_core/issues/704>`_)
* chore: jazzy-porting, autoware_lanelet2_utils, add missing link dependency yaml-cpp (`#686 <https://github.com/autowarefoundation/autoware_core/issues/686>`_)
  Co-authored-by: Ryohsuke Mitsudome <ryohsuke.mitsudome@tier4.jp>
* refactor(autoware_lanelet2_utils): refactor get_linestring_from_arc_length (`#688 <https://github.com/autowarefoundation/autoware_core/issues/688>`_)
  change return type of Const, and use create_safe_linestring
* feat(autoware_lanelet2_utils): port function from autoware_lanelet2_extension (`#693 <https://github.com/autowarefoundation/autoware_core/issues/693>`_)
* docs(autoware_lanelet2_utils): write documentation for geometry  (`#689 <https://github.com/autowarefoundation/autoware_core/issues/689>`_)
  * add geometry functions description
  * capitalize table column name
  * fix some descriptions
  * add geometry illustration
  * add more angle description
  ---------
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* feat(autoware_lanelet2_utils): create artificial lanelet object (`#669 <https://github.com/autowarefoundation/autoware_core/issues/669>`_)
* feat(autoware_lanelet2_utils): remove the usage of getClosestSegment and getLaneletAngle  (`#658 <https://github.com/autowarefoundation/autoware_core/issues/658>`_)
* feat(lanelet2_utils): update lanelet2_anonymizer (`#666 <https://github.com/autowarefoundation/autoware_core/issues/666>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* docs(lanelet2_utils): make table more readable (`#657 <https://github.com/autowarefoundation/autoware_core/issues/657>`_)
  docs(lanelet2_utils): divide table and make images bigger
* feat(lanelet2_utils): add RouteManager class (`#653 <https://github.com/autowarefoundation/autoware_core/issues/653>`_)
* feat(lanelet2_utils): add MapHandler class (minor change of `#651 <https://github.com/autowarefoundation/autoware_core/issues/651>`_) (`#656 <https://github.com/autowarefoundation/autoware_core/issues/656>`_)
* chore(lanelet2_utils): revert `#651 <https://github.com/autowarefoundation/autoware_core/issues/651>`_ (`#654 <https://github.com/autowarefoundation/autoware_core/issues/654>`_)
  Revert "feat(lanelet2_utils): add MapHandler class (`#651 <https://github.com/autowarefoundation/autoware_core/issues/651>`_)"
  This reverts commit b44dc53f23b81d1713fc49c91074d638cd940234.
* feat(lanelet2_utils): add MapHandler class (`#651 <https://github.com/autowarefoundation/autoware_core/issues/651>`_)
* feat(lanelet2_utils): add LaneSequence class (`#650 <https://github.com/autowarefoundation/autoware_core/issues/650>`_)
* feat(lanelet2_utils): add test_case_generator and nn_search module (`#649 <https://github.com/autowarefoundation/autoware_core/issues/649>`_)
* feat(lanelet2_utils): port message conversion from lanelet2_extension (`#648 <https://github.com/autowarefoundation/autoware_core/issues/648>`_)
  feat(route_handler): port message conversion from lanelet2_extension
* feat(autoware_lanelet2_utils): porting functions from lanelet2_extension to autoware_lanelet2_utils package (`#621 <https://github.com/autowarefoundation/autoware_core/issues/621>`_)
* feat(autoware_lanelet2_utils): unified point type conversion between lanelet types and ROS messages (`#642 <https://github.com/autowarefoundation/autoware_core/issues/642>`_)
* fix(autoware_lanelet2_utils): fix the map to pass validation (`#645 <https://github.com/autowarefoundation/autoware_core/issues/645>`_)
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mamoru Sobue, Mete Fatih Cırıt, Sarun MUKDAPITAK, Yutaka Kondo, mitsudome-r, 心刚

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
