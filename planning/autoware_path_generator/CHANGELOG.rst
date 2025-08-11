^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_path_generator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* fix(path_generator): set both lane IDs to point on border of adjacent lanes (`#384 <https://github.com/autowarefoundation/autoware_core/issues/384>`_)
  set both lane IDs to point on border of adjacent lanes
* feat(path_generator): move generate_path public (`#380 <https://github.com/autowarefoundation/autoware_core/issues/380>`_)
  * feat(path_generator): move generate_path public
  * style(pre-commit): autofix
  * fix pre-commit
  * fix test include path
  ---------
  Co-authored-by: t4-adc <grp-rd-1-adc-admin@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(autoware_trajectory)!: move everything to namespace experimetal (`#371 <https://github.com/autowarefoundation/autoware_core/issues/371>`_)
  refactor(autoware_trajectory)!: move everything to namespace experimental
* test(path_generator): add tests for path cut feature (`#268 <https://github.com/autowarefoundation/autoware_core/issues/268>`_)
  * add map for test
  * add overpass map
  * refactor & enhance base test class
  * add tests
  * style(pre-commit): autofix
  * fix year created
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * anonymize test map
  * style(pre-commit): autofix
  * add test map info to README
  * style(pre-commit): autofix
  * make tests work with autoware_trajectory
  * include necessary header
  * fix test case
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix(path_generator): deal with unintended input (`#336 <https://github.com/autowarefoundation/autoware_core/issues/336>`_)
  * prevent segfault
  * fix self-intersection search range
  * define behavior for unintended input
  * prevent segfault
  * check builder output instead of input size
  ---------
* refactor(path_generator): avoid using fixed-size array (`#353 <https://github.com/autowarefoundation/autoware_core/issues/353>`_)
  * avoid using fixed-size array
  * include necessary headers
  * avoid capturing structured bindings in lambdas
  ---------
* docs(path_generator): add description of path cut & turn signal feature (`#359 <https://github.com/autowarefoundation/autoware_core/issues/359>`_)
  * add diagrams of path cut feature
  * add diagrams of turn signal feature
  * update parameter list
  * update README
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(path_generator): avoid shortcuts at overlaps (`#352 <https://github.com/autowarefoundation/autoware_core/issues/352>`_)
  * track current lane to avoid shortcut
  * add constraints for current lane search
  ---------
* feat(autoware_path_generator): use autoware_trajectory for cropping bounds (`#349 <https://github.com/autowarefoundation/autoware_core/issues/349>`_)
* Contributors: Kazunori-Nakajima, Mamoru Sobue, Mitsuhiro Sakamoto, Yukinari Hisaki

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(path_generator): avoid far goal connection (`#594 <https://github.com/autowarefoundation/autoware_core/issues/594>`_)
  * use autoware_trajectory instead of vector of path points
  * fix goal connection algorithm
  * perform goal connection only if path reaches goal
  * fix tests
  * style(pre-commit): autofix
  * add description of goal connection feature
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
* fix(path_generator): fix start edge intersection search (`#595 <https://github.com/autowarefoundation/autoware_core/issues/595>`_)
  * return cropped line string as autoware_trajectory
  * separate intersection search into functions & fix start edge intersection search
  * fix tests
  * add tests for intersection search functions
  * ensure intersection point is not ends of start edge
  * update doxygen comment
  ---------
* fix(path_generator): merge waypoint groups with shared overlap interval (`#586 <https://github.com/autowarefoundation/autoware_core/issues/586>`_)
  * merge waypoint groups with shared overlap intervals
  * replace parameters with new ones
  * update figure in description
  * style(pre-commit): autofix
  * fix(path_generator): apply clang-tidy
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: Junya Sasaki <junya.sasaki@tier4.jp>
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* fix(path_generator): update unit tests (`#577 <https://github.com/autowarefoundation/autoware_core/issues/577>`_)
  * fix tests to use new goal connection functions
  * fix dense centerline test
  ---------
* feat(path_generator): improve goal connection for goal on the side (`#564 <https://github.com/autowarefoundation/autoware_core/issues/564>`_)
  * change goal connection method
  * apply goal connection
  * rename parameter
  * change parameter name
  ---------
* test(path_generator): add extra tests (`#449 <https://github.com/autowarefoundation/autoware_core/issues/449>`_)
  * add map for test
  * update test cases for get_turn_signal()
  * rename test route files
  * add test with dense centerline
  * move dense centerline map to sample_map directory
  * add tests for get_arc_length_on_path
  * modify path start point to be inside route lanelet
  * add tests for smooth goal connection
  * include necessary header
  * remove ineffective test case
  * fix comment
  * remove unnecessary lines
  * fix test case for refine_path_for_goal
  * rename parameters
  * add missing argument
  * remove include guard
  ---------
* fix(path_generator): fix waypoint interval calculation (`#567 <https://github.com/autowarefoundation/autoware_core/issues/567>`_)
  fix waypoint interval calculation
* fix(path_generator): support 0s turn signal search time (`#557 <https://github.com/autowarefoundation/autoware_core/issues/557>`_)
* Contributors: Kosuke Takeuchi, Mitsuhiro Sakamoto, Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(path_generator): publish processing time (`#549 <https://github.com/autowarefoundation/autoware_core/issues/549>`_)
* fix(path_generator): deal with short path (`#536 <https://github.com/autowarefoundation/autoware_core/issues/536>`_)
  * use pretty_build to handle short path
  * add error message
  ---------
* fix(path_generator): ensure refined path connects start and goal (`#511 <https://github.com/autowarefoundation/autoware_core/issues/511>`_)
  * ensure refined path connects start and goal
  * rename and add parameters for smooth goal connection
  * parameterize pre-goal offset
  * delegate validation to generate_parameter_library
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* refactor(path_generator): remove refine_goal (`#500 <https://github.com/autowarefoundation/autoware_core/issues/500>`_)
* refactor(path_generator): remove unused functions (`#499 <https://github.com/autowarefoundation/autoware_core/issues/499>`_)
  * remove unused functions
  * fix function declaration to match implementation
  * add declaration of is_trajectory_inside_lanelets
  * fix declaration of refine_path_for_goal
  ---------
* fix(path_generator): fix crop range when waypoints exist (`#461 <https://github.com/autowarefoundation/autoware_core/issues/461>`_)
  * fix crop range when waypoints exist
  * add get_arc_length_on_path function
  * fix handling of waypoints on overlapping lanes
  * check for self-intersections of bounds of lanelets extended outside route
  * fix cropping at goal
  * fix cropping trajectory
  * add comments about lanelet extension
  * fix behavior in edge cases
  ---------
* docs(path_generator): replace flowchart with raw plantuml code (`#482 <https://github.com/autowarefoundation/autoware_core/issues/482>`_)
  * add flowchart
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(path_generator): add missing autoware planning msgs (`#480 <https://github.com/autowarefoundation/autoware_core/issues/480>`_)
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* refactor(path_generator): avoid using non-API functions (`#432 <https://github.com/autowarefoundation/autoware_core/issues/432>`_)
  * avoid using lanelet::geometry::internal functions
  * use meaningful struct instead of pair
  ---------
* feat(autoware_path_generator): add smooth goal connection (`#394 <https://github.com/autowarefoundation/autoware_core/issues/394>`_)
* fix(path_generator): set both lane IDs to point on border of adjacent lanes (`#384 <https://github.com/autowarefoundation/autoware_core/issues/384>`_)
  set both lane IDs to point on border of adjacent lanes
* feat(path_generator): move generate_path public (`#380 <https://github.com/autowarefoundation/autoware_core/issues/380>`_)
  * feat(path_generator): move generate_path public
  * style(pre-commit): autofix
  * fix pre-commit
  * fix test include path
  ---------
  Co-authored-by: t4-adc <grp-rd-1-adc-admin@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(autoware_trajectory)!: move everything to namespace experimetal (`#371 <https://github.com/autowarefoundation/autoware_core/issues/371>`_)
  refactor(autoware_trajectory)!: move everything to namespace experimental
* test(path_generator): add tests for path cut feature (`#268 <https://github.com/autowarefoundation/autoware_core/issues/268>`_)
  * add map for test
  * add overpass map
  * refactor & enhance base test class
  * add tests
  * style(pre-commit): autofix
  * fix year created
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * anonymize test map
  * style(pre-commit): autofix
  * add test map info to README
  * style(pre-commit): autofix
  * make tests work with autoware_trajectory
  * include necessary header
  * fix test case
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix(path_generator): deal with unintended input (`#336 <https://github.com/autowarefoundation/autoware_core/issues/336>`_)
  * prevent segfault
  * fix self-intersection search range
  * define behavior for unintended input
  * prevent segfault
  * check builder output instead of input size
  ---------
* refactor(path_generator): avoid using fixed-size array (`#353 <https://github.com/autowarefoundation/autoware_core/issues/353>`_)
  * avoid using fixed-size array
  * include necessary headers
  * avoid capturing structured bindings in lambdas
  ---------
* docs(path_generator): add description of path cut & turn signal feature (`#359 <https://github.com/autowarefoundation/autoware_core/issues/359>`_)
  * add diagrams of path cut feature
  * add diagrams of turn signal feature
  * update parameter list
  * update README
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(path_generator): avoid shortcuts at overlaps (`#352 <https://github.com/autowarefoundation/autoware_core/issues/352>`_)
  * track current lane to avoid shortcut
  * add constraints for current lane search
  ---------
* feat(autoware_path_generator): use autoware_trajectory for cropping bounds (`#349 <https://github.com/autowarefoundation/autoware_core/issues/349>`_)
* Contributors: Kazunori-Nakajima, Kosuke Takeuchi, Mamoru Sobue, Mitsuhiro Sakamoto, Yukinari Hisaki, Yutaka Kondo, github-actions, taikitanaka3

1.0.0 (2025-03-31)
------------------
* test(autoware_path_generator): add turn signal RequiredEndPoint position test (`#323 <https://github.com/autowarefoundation/autoware_core/issues/323>`_)
  test(autoware_path_generator): add RequiredEndPoint position test
* test(path_generator): add tests for turn signal activation feature (`#253 <https://github.com/autowarefoundation/autoware_core/issues/253>`_)
  * add tests
  * style(pre-commit): autofix
  * Update planning/autoware_path_generator/test/test_turn_signal.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
* fix(autoware_path_generator): remove redundant move (`#318 <https://github.com/autowarefoundation/autoware_core/issues/318>`_)
  Remove redundant move
* fix(path_generator): fix path bound generation for overlapped lanes (`#285 <https://github.com/autowarefoundation/autoware_core/issues/285>`_)
  * fix path bound generation for overlapped lanes
  * check for intersection between start edge of drivable area and path bounds
  * fix start edge intersection search
  * temporarily disuse autoware_trajectory
  * check intersection between start edge of drivable area and center line
  * fix get_first_self_intersection_arc_length idx
  * fix redundantInitialization
  * fix structure bindings for clang-tidy
  ---------
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* Contributors: Kosuke Takeuchi, Mitsuhiro Sakamoto, Shane Loretz

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* chore: rename from `autoware.core` to `autoware_core` (`#290 <https://github.com/autowarefoundation/autoware.core/issues/290>`_)
* feat: adaptation to ROS nodes guidelines about directory structure (`#272 <https://github.com/autowarefoundation/autoware.core/issues/272>`_)
* fix(path_generator): fix path bound generation (`#267 <https://github.com/autowarefoundation/autoware.core/issues/267>`_)
  fix path bound generation
* feat(autoware_path_generator): function to smooth the path (`#227 <https://github.com/autowarefoundation/autoware.core/issues/227>`_)
  * feat: function to smooth the route (see below)
  Description:
  This commit is kind of feature porting from `autoware.universe` as follows
  * Import `PathWithLaneId DefaultFixedGoalPlanner::modifyPathForSmoothGoalConnection` from the following `autoware.universe` code
  https://github.com/autowarefoundation/autoware.universe/blob/a0816b7e3e35fbe822fefbb9c9a8132365608b49/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/src/default_fixed_goal_planner.cpp#L74-L104
  * Also import all related functions from the `autoware.universe` side
  * style(pre-commit): autofix
  * bugs: fix remaining conflicts
  * Update planning/autoware_path_generator/src/utils.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * Update planning/autoware_path_generator/src/utils.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * refactor: as follows
  * Enhance error handlings
  * Remove unused variables
  * Simplify the code
  * Improve readability a little bit
  * style(pre-commit): autofix
  * refactor: enhance error handling
  * style(pre-commit): autofix
  * bug: fix wrong function declaration in header
  * bug: fix wrong point index calculation
  * bug: remove meaningless comment
  * This comment is wrote because of my misunderstanding
  * fix: apply `pre-commit`
  * fix: smooth path before cropping trajectory points
  * bug: fix shadow variable
  * bug: fix missing parameters for `autoware_path_generator`
  * bug: fix by cpplint
  * style(pre-commit): autofix
  * bug: apply missing fix proposed by cpplint
  * style(pre-commit): autofix
  * bug: `autoware_test_utils` should be in the `test_depend`
  * fix(autoware_path_generator): add maintainer and author
  * style(pre-commit): autofix
  * fix: by pre-commit
  * Sorry, I was forgetting to do this on my local env.
  * fix: smooth path only when a goal point is included
  * bug: do error handling
  * style(pre-commit): autofix
  * bug: fix wrong distance calculation
  * The goal position is generally separate from the path points
  * fix: remove sanity check temporary as following reasons
  * CI (especially unit tests) fails due to this sanity check
  * As this is out of scope for this PR, we will fix the bug
  where the start and end are reversed in another PR
  * refactor: fix complexity
  * We should start from the simple one
  * Then we can add the necessary optimization later
  * bug: missing fixes in the include header
  * bug: inconsistent function declaration
  * The type of returned value and arguments were wrong
  * Update planning/autoware_path_generator/include/autoware/path_generator/common_structs.hpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * Update planning/autoware_path_generator/src/node.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * Update planning/autoware_path_generator/src/utils.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * Update planning/autoware_path_generator/src/utils.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * style(pre-commit): autofix
  * fix: apply comment in the following PR
  * https://github.com/autowarefoundation/autoware.core/pull/227#discussion_r1986045016
  * fix: sorry, I was missing one comment to be applied
  * style(pre-commit): autofix
  * bug: fix wrong goal point interpolation
  * feat: add test case (goal on left side)
  * bug: fix as follows
  * Prevent name duplication (path_up_to_just_before_pre_goal)
  * Fix missing left/right bound
  * Goal must have zero velocity
  * Improve readability
  * Other minor fixes
  * bug: fix duplicated zero velocity set
  * Zero velocity is set after the removed lines by this commit
  * feat: add one test case (goal on left side)
  * Update planning/autoware_path_generator/src/utils.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * fix: apply comment from reviewer
  * fix(package.xml): update maintainer for the following packages
  * `autoware_planning_test_manager`
  * `autoware_test_utils`
  * Update planning/autoware_path_generator/src/node.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * Update planning/autoware_path_generator/src/utils.cpp
  Co-authored-by: Mitsuhiro Sakamoto <50359861+mitukou1109@users.noreply.github.com>
  * Update planning/autoware_path_generator/src/utils.cpp
  Co-authored-by: Mitsuhiro Sakamoto <50359861+mitukou1109@users.noreply.github.com>
  * bug: fix missing header in the path
  * This finally causes an issue that the vehicle cannot engage
  * bug: fix an issue that smooth connection does not work
  * refactor: simplify code
  * bug: fix wrong pose at the goal (see below)
  * If we return nullopt here, the original path
  whose goal position is located at the center line is used.
  * Unless far from the goal point, the path becomes smoothed one
  whose goal position is located at the side of road correctly.
  * But as the goal approaches very closely, the goal position is
  shifted from smoothed one to the original one
  * Thus, the goal pose finally becomes wrong due to the goal position shift
  * refactor: no need this line here
  * style(pre-commit): autofix
  * bug: fix so we follow the provided review comments
  * bug: sorry, this is unsaved fix, ...
  * cosmetic: fix wrong comment
  * bug: unused function `get_goal_lanelet()` remaining
  * bug: carefully handle the pre goal velocity
  * It seems zero pre goal velocity makes scenario fail
  - We need to insert appropriate velocity for pre goal
  * Update planning/autoware_path_generator/src/utils.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * Update planning/autoware_path_generator/src/utils.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: Mitsuhiro Sakamoto <50359861+mitukou1109@users.noreply.github.com>
* feat(path_generator): publish hazard signal (`#252 <https://github.com/autowarefoundation/autoware.core/issues/252>`_)
  publish hazard signal (no command)
* fix(path_generator): set current pose appropriately in test (`#250 <https://github.com/autowarefoundation/autoware.core/issues/250>`_)
  set start pose of route as current pose
* feat(path_generator): add turn signal activation feature (`#220 <https://github.com/autowarefoundation/autoware.core/issues/220>`_)
  * add path_generator package
  fix spell check error
  include necessary headers
  change package version to 0.0.0
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  fix include guard name
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  replace flowchart uml with pre-generated image
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  style(pre-commit): autofix
  replace tier4_planning_msgs with autoware_internal_planning_msgs
  style(pre-commit): autofix
  use LaneletSequence instead of ConstLanelets
  set orientation to path points
  crop path bound to fit trajectory
  offset path bound
  no need to make return value optional
  address deprecation warning
  add doxygen comments
  support multiple previous/next lanelets
  fix path bound cut issue
  group parameters
  add turn signal activation feature
  fix turn direction check process
  consider required end point
  keep turn signal activated until reaching desired end point if without conflicts
  add missing parameters
  * add include
  * use trajectory class
  * minor change
  ---------
  Co-authored-by: mitukou1109 <mitukou1109@gmail.com>
* test(path_generator): add tests (`#215 <https://github.com/autowarefoundation/autoware.core/issues/215>`_)
  * test(path_generator): add tests
  * add tests
  * adapt test to new test manager
  * migrate to autoware_internal_planning_msgs
  * use intersection map for unit tests
  ---------
  fix pre-commit
  fix pre-commit
  * Update planning/autoware_path_generator/test/test_path_generator_node_interface.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * fix for latest
  ---------
  Co-authored-by: Mitsuhiro Sakamoto <50359861+mitukou1109@users.noreply.github.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* feat(path_generator): add path cut feature (`#216 <https://github.com/autowarefoundation/autoware.core/issues/216>`_)
  * feat(path_generator): add path cut feature
  add path_generator package
  fix spell check error
  include necessary headers
  change package version to 0.0.0
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  fix include guard name
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  replace flowchart uml with pre-generated image
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  style(pre-commit): autofix
  replace tier4_planning_msgs with autoware_internal_planning_msgs
  style(pre-commit): autofix
  use LaneletSequence instead of ConstLanelets
  set orientation to path points
  crop path bound to fit trajectory
  offset path bound
  no need to make return value optional
  address deprecation warning
  add doxygen comments
  support multiple previous/next lanelets
  fix path bound cut issue
  group parameters
  add path cut feature
  ensure s_end is not negative
  simplify return value selection
  add doxygen comments
  * ignore makeIndexedSegmenTree from spell check
  * delete comments from cspell for pre-commit
  ---------
  Co-authored-by: mitukou1109 <mitukou1109@gmail.com>
* feat(path_generator): add path_generator package (`#138 <https://github.com/autowarefoundation/autoware.core/issues/138>`_)
  * add path_generator package
  * fix spell check error
  * include necessary headers
  * change package version to 0.0.0
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  * fix include guard name
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  * replace flowchart uml with pre-generated image
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  * style(pre-commit): autofix
  * replace tier4_planning_msgs with autoware_internal_planning_msgs
  * style(pre-commit): autofix
  * use LaneletSequence instead of ConstLanelets
  * set orientation to path points
  * crop path bound to fit trajectory
  * offset path bound
  * no need to make return value optional
  * address deprecation warning
  * add doxygen comments
  * support multiple previous/next lanelets
  * fix path bound cut issue
  * group parameters
  * use autoware_utils
  * test(path_generator): add tests (`#1 <https://github.com/autowarefoundation/autoware.core/issues/1>`_)
  * add tests
  * adapt test to new test manager
  * migrate to autoware_internal_planning_msgs
  * use intersection map for unit tests
  ---------
  * fix pre-commit
  * fix pre-commit
  * Revert "fix pre-commit"
  This reverts commit 9b3ae3e93c826f571101203f2b0defc5e238741b.
  Revert "fix pre-commit"
  This reverts commit 6a3c5312920ba4551ced5247674209318b31c657.
  Revert "test(path_generator): add tests (`#1 <https://github.com/autowarefoundation/autoware.core/issues/1>`_)"
  This reverts commit 7773976d3651e7e3b0b12f405f800abebfb6abe8.
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* Contributors: Junya Sasaki, Kosuke Takeuchi, Mitsuhiro Sakamoto, NorahXiong, Yutaka Kondo, mitsudome-r
