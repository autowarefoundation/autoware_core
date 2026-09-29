^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_factor_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore: update package maintainer (`#1384 <https://github.com/autowarefoundation/autoware_core/issues/1384>`_)
  chore: update package author metadata
* fix(planning): declare the dependencies these packages use (`#1372 <https://github.com/autowarefoundation/autoware_core/issues/1372>`_)
  Each of these packages uses a package it never declares. Either it includes a
  header of that package, or it names a symbol of it while the header arrives
  through another dependency. Both build today only because some declared
  dependency re-exports the owner, so a change in an unrelated repository can
  break them without anything here changing.
  The tag follows where the dependency is used: a use in an installed header or
  in code compiled into the library takes <depend>, one reached only from test/
  takes <test_depend>. System libraries are named by the rosdep key this
  workspace already prefers.
  A clean-context review of the pull request found five more direct uses with no manifest entry. Add one entry for each:
  - autoware_path_generator: tf2 (tf2::getYaw in src/utils.cpp)
  - autoware_motion_velocity_planner_common: tf2 (tf2::getYaw in src/planner_data.cpp and src/polygon_utils.cpp)
  - autoware_motion_velocity_obstacle_stop_module: autoware_planning_factor_interface (constructed in src/obstacle_stop_module.cpp)
  - autoware_behavior_velocity_stop_line_module: autoware_planning_factor_interface (used in src/experimental/scene.cpp)
  - autoware_velocity_smoother: rclcpp_components (register_node_macro.hpp in src/node.cpp)
* feat(PlanningFactorInterface): introduce node independent PlanningFactorInterfaceBase class (`#1250 <https://github.com/autowarefoundation/autoware_core/issues/1250>`_)
  * templatize PlanningFactorInterface
  * test(autoware_planning_factor_interface): cover agnocast wrapper node in typed test
  Convert the publisher/subscriber test to a TYPED_TEST over rclcpp::Node and
  autoware::agnocast_wrapper::Node so PlanningFactorInterfaceT is exercised for
  both node instantiations, not only the rclcpp one.
  * delete tests for rclcpp::Node
  * specify node type
  ---------
* Contributors: Koichi Imai, Mete Fatih Cırıt, Satoshi OTA, github-actions

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* test(autoware_planning_factor_interface): add gtest suite and apply move-based perf wins (`#1152 <https://github.com/autowarefoundation/autoware_core/issues/1152>`_)
  * test(autoware_planning_factor_interface): add gtest suite and apply move-based perf wins
  Add the package's first gtest suite, wiring the missing BUILD_TESTING /
  ament_auto_add_gtest block in CMakeLists. The suite pins:
  - single- and two-control-point add() ControlPoint/PlanningFactor field
  construction (pose, velocity, shift_length, distance, module name,
  behavior, detail, driving direction, safety factors),
  - the templated add() overloads forwarding calcSignedArcLength results
  into ControlPoint.distance,
  - factor accumulation across multiple add() calls in insertion order,
  - the publish() contract: header.frame_id == 'map', factors forwarded
  into the PlanningFactorArray (verified via a test subscription), and
  factors\_ cleared afterwards so the next cycle starts empty.
  Apply the low-risk, behavior-preserving perf wins flagged for this
  package: move the locally-built PlanningFactor into factors\_ in both
  add() overloads, move factors\_ into msg.factors in publish() (the buffer
  is cleared immediately after), and return factors\_ by const reference
  from get_factors() to drop a deep copy per call. The publish() console
  gate now checks msg.factors (which holds the moved-from factors) so the
  output behavior is unchanged.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * style(pre-commit): autofix
  * fix(autoware_planning_factor_interface): make factor non-const for real move
  A const-qualified factor caused std::move to bind to the copy constructor,
  silently degrading the intended move into factors\_ to a copy. Drop the const
  qualifier on both add() overloads so push_back uses the move constructor.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------

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
* feat(obstacle_stop_module, motion_velocity_planner_common): add safety_factor to obstacle_stop module (`#572 <https://github.com/autowarefoundation/autoware_core/issues/572>`_)
  add safety factor, add planning_factor test
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Yuki TAKAGI, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(planning_factor): add console output option (`#513 <https://github.com/autowarefoundation/autoware_core/issues/513>`_)
  fix param json
  fix param json
  snake_case
  set default
* feat!: remove obstacle_stop_planner and obstacle_cruise_planner (`#495 <https://github.com/autowarefoundation/autoware_core/issues/495>`_)
  * feat: remove obstacle_stop_planner and obstacle_cruise_planner
  * update
  * fix
  ---------
* fix(autoware_planning_factor_interface): removed unused autoware_utils (`#440 <https://github.com/autowarefoundation/autoware_core/issues/440>`_)
  removed unused autoware_utils
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* Contributors: Kosuke Takeuchi, Masaki Baba, Takayuki Murooka, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* fix(planning_factor_interface): set control point data independently (`#291 <https://github.com/autowarefoundation/autoware_core/issues/291>`_)
  * fix(planning_factor_interface): set shift length properly
  * chore: add comment
  ---------
* Contributors: Satoshi OTA

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* chore: rename from `autoware.core` to `autoware_core` (`#290 <https://github.com/autowarefoundation/autoware.core/issues/290>`_)
* feat(autoware_planning_factor_interface): move to core from universe (`#241 <https://github.com/autowarefoundation/autoware.core/issues/241>`_)
* Contributors: Yutaka Kondo, mitsudome-r, 心刚
