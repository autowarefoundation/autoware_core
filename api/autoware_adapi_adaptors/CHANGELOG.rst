^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_adapi_adaptors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_adapi_adaptors): extract pure helpers and add unit tests (`#1147 <https://github.com/autowarefoundation/autoware_core/issues/1147>`_)
  * refactor(autoware_adapi_adaptors): extract pure helpers and add unit tests
  Extract the previously inlined, node-coupled logic of the two RViz-to-AD-API
  adaptor nodes into small internal-only free/templated helpers so it can be
  unit-tested without spinning up a ROS node, and add the package's first test/
  directory:
  - vector_to_array<T,N>() in parameter_helper.hpp: the pure size-validating
  vector-to-array covariance parsing (throw branch is now testable).
  - decide_routing_action() in routing_state_machine.hpp: the on_timer
  merge-window/state-machine, returning the next action and updated counter.
  - set_goal() / append_waypoint() in route_builder.hpp: the route-request
  building rules, including the on_waypoint frame-mismatch guard.
  The public node API/ABI is unchanged; the callbacks now delegate to these
  helpers. Behavior is preserved: the covariance size-error message is identical
  for N=36, and the timer/route-building branches map one-to-one to the previous
  inline code.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * Potential fix for pull request finding
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
  * refactor(autoware_adapi_adaptors): clarify helper naming per PR review
  Address review feedback on PR `#1147 <https://github.com/autowarefoundation/autoware_core/issues/1147>`_:
  - Rename the routing merge-window counter from request_timing_control to
  number_of_requests_for_timing_control across the extracted helper, the
  RoutingDecision struct, and the RoutingAdaptor member for an explicit name
  (sasakisasaki).
  - Rename vector_to_array's parameter from vector to values so it no longer
  reads like a type name (Copilot).
  The dimensionally-incorrect merge-window comment was already corrected in the
  preceding autofix commit. Verified with a Release build and the package gtest
  suite in the core-devel-jazzy container (37 cases, 0 failures).
  * modify naming
  ---------
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
  Co-authored-by: Takagi, Isamu <isamu.takagi@tier4.jp>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
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
* chore: jazzy-porting:fix qos profile issue (`#634 <https://github.com/autowarefoundation/autoware_core/issues/634>`_)
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Yutaka Kondo, mitsudome-r, 心刚

1.4.0 (2025-08-11)
------------------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat: port autoware_adapi_adaptors from Autoware Universe (`#530 <https://github.com/autowarefoundation/autoware_core/issues/530>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, github-actions

* fix: to be consistent version in all package.xml(s)
* feat: port autoware_adapi_adaptors from Autoware Universe (`#530 <https://github.com/autowarefoundation/autoware_core/issues/530>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-22)
------------------

0.2.0 (2025-02-07)
------------------

0.0.0 (2024-12-02)
------------------
