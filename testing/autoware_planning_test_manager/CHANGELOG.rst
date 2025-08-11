^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_test_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(planning_test_manager): subscription callback (`#563 <https://github.com/autowarefoundation/autoware_core/issues/563>`_)
  * feat(planning_test_manager): subscription callback
  * fix(test_planning_test_manager): unique node names
  Update testing/autoware_planning_test_manager/test/test_planning_test_manager.cpp
  Update testing/autoware_planning_test_manager/test/test_planning_test_manager.cpp
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  ---------
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome, ralwing

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat!: remove obstacle_stop_planner and obstacle_cruise_planner (`#495 <https://github.com/autowarefoundation/autoware_core/issues/495>`_)
  * feat: remove obstacle_stop_planner and obstacle_cruise_planner
  * update
  * fix
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* Contributors: Takayuki Murooka, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* test(autoware_planning_test_manager): add tests (`#273 <https://github.com/autowarefoundation/autoware.core/issues/273>`_)
  * test(autoware_planning_test_manager): add tests
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* feat: adaptation to ROS nodes guidelines about directory structure (`#272 <https://github.com/autowarefoundation/autoware.core/issues/272>`_)
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
* feat(autoware_planning_test_manager): porting autoware_planning_test_manager package from autoware.universe (`#203 <https://github.com/autowarefoundation/autoware.core/issues/203>`_)
* Contributors: Junya Sasaki, NorahXiong, mitsudome-r
