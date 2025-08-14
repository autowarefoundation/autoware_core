^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_test_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
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
* chore: include iostream and link yaml-cpp for Jazzy (`#351 <https://github.com/autowarefoundation/autoware_core/issues/351>`_)
* Contributors: Mitsuhiro Sakamoto, Tim Clephas

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: change planning output topic name to /planning/trajectory (`#602 <https://github.com/autowarefoundation/autoware_core/issues/602>`_)
  * change planning output topic name to /planning/trajectory
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* fix(test_utils): add missing lane ids in `makeBehaviorNormalRoute()` (`#561 <https://github.com/autowarefoundation/autoware_core/issues/561>`_)
  add missing lane ids
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
* Contributors: Mitsuhiro Sakamoto, Ryohsuke Mitsudome, Yukihiro Saito

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(autoware_trajectory): implement a function to construct trajectory class for reference path (`#469 <https://github.com/autowarefoundation/autoware_core/issues/469>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* chore(autoware_test_utils): delete `launch` directory to remove dependency on `autoware_launch` (`#498 <https://github.com/autowarefoundation/autoware_core/issues/498>`_)
  remove launch
* fix(autoware_test_utils): fix linking to yaml-cpp (`#315 <https://github.com/autowarefoundation/autoware_core/issues/315>`_)
  * Fix linking to yaml-cpp
  * Remove find_package(yaml_cpp_vendor...)
  * wip
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* fix(autoware_test_utils): fix deprecated autoware_utils header (`#415 <https://github.com/autowarefoundation/autoware_core/issues/415>`_)
  * fix autoware_utils header
  * fix autoware_utils packages
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
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
* chore: include iostream and link yaml-cpp for Jazzy (`#351 <https://github.com/autowarefoundation/autoware_core/issues/351>`_)
* Contributors: Mamoru Sobue, Masaki Baba, Mitsuhiro Sakamoto, Shane Loretz, Tim Clephas, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* feat(trajectory): improve comment, use autoware_pyplot for examples (`#282 <https://github.com/autowarefoundation/autoware.core/issues/282>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
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
* fix(autoware_test_utils): set timestamp in building trajectory msg (`#245 <https://github.com/autowarefoundation/autoware.core/issues/245>`_)
* docs(autoware_test_utils): fix invalid link (`#229 <https://github.com/autowarefoundation/autoware.core/issues/229>`_)
* chore(autoware_test_utils): fix invalid link (`#221 <https://github.com/autowarefoundation/autoware.core/issues/221>`_)
* feat(autoware_test_utils): porting autoware_test_utils from universe to core (`#172 <https://github.com/autowarefoundation/autoware.core/issues/172>`_)
* Contributors: JianKangEgon, Junya Sasaki, Kosuke Takeuchi, Mamoru Sobue, Satoshi OTA, mitsudome-r
