^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_default_adapi
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(api, motion_velocity_planner): add the node designs required by the AD API and motion planning design modules (`#1456 <https://github.com/autowarefoundation/autoware_core/issues/1456>`_)
  * feat(api): add node designs for the default AD API nodes and RViz adaptors
  * fix(autoware_motion_velocity_planner): add additional planning factors to publishers in the node design
  ---------
* feat(default_adapi): move the API nodes to agnocast_wrapper::Node (`#1438 <https://github.com/autowarefoundation/autoware_core/issues/1438>`_)
* fix(api): declare the dependencies autoware_default_adapi uses (`#1366 <https://github.com/autowarefoundation/autoware_core/issues/1366>`_)
  autoware_default_adapi uses packages it never declares. Either it includes a
  header of that package, or it names a symbol of it while the header arrives
  through another dependency. Both build today only because some declared
  dependency re-exports the owner, so a change in an unrelated repository can
  break this package without anything here changing.
  The tag follows where the dependency is used: a use in an installed header or
  in code compiled into the library takes <depend>, one reached only from test/
  takes <test_depend>.
* fix(component_interface_utils): fix build errors related to NodeAdaptor (`#1339 <https://github.com/autowarefoundation/autoware_core/issues/1339>`_)
* refactor(autoware_default_adapi): create endpoints through NodeAdaptor (`#1326 <https://github.com/autowarefoundation/autoware_core/issues/1326>`_)
  The nineteen endpoints in this package each re-derived their type, name
  and QoS by hand from a spec that already carries all three. Create them
  through NodeAdaptor instead, so each call site names its spec once.
  No wire change: every site already passed Spec::name, and the QoS each
  one built by hand is the value NodeAdaptor derives. The topic and service
  lists are identical before and after.
* Contributors: Koichi Imai, Mete Fatih Cırıt, Taekjin LEE, Takagi, Isamu, Yutaka Kondo, github-actions

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* test(autoware_default_adapi): add gtest suite for pure conversion functions (`#1141 <https://github.com/autowarefoundation/autoware_core/issues/1141>`_)
  The package previously had only a launch test asserting the interface
  version. The message-conversion translation units (route_conversion.cpp,
  localization_conversion.cpp) hold the real, pure logic yet were exercised
  only via service round-trips.
  Add an ament_auto_add_gtest suite that links against the package library
  and covers the already-pure conversion functions with value-asserting,
  table-driven cases:
  - convert_state: exhaustive internal RouteState -> external mapping,
  including the non-obvious collapses (INITIALIZING/UNSET/ROUTING->UNSET,
  REROUTING->CHANGING, ABORTED/INTERRUPTED->SET) and the default->UNKNOWN
  branch, plus stamp pass-through.
  - convert_route / RouteSegment round-trips: preferred-primitive extraction
  from alternatives, the missing-preferred branch (preferred left
  default-constructed, no erase), empty segments, and the
  RoutePrimitive.type <-> LaneletPrimitive.primitive_type field rename.
  - route convert_request overloads (lanelet, waypoint, clear) and
  convert_response field mappings.
  - localization convert_request (method=AUTO + pose copy) and
  convert_response field mappings.
  No production code changes; the functions were already pure and need no
  refactor to test.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
* Contributors: Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* docs: fix broken links (`#779 <https://github.com/autowarefoundation/autoware_core/issues/779>`_)
* ci(pre-commit): autoupdate (`#723 <https://github.com/autowarefoundation/autoware_core/issues/723>`_)
  * pre-commit formatting changes
* Contributors: Mete Fatih Cırıt, github-actions

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore: update AD API version to v1.9.1 (`#717 <https://github.com/autowarefoundation/autoware_core/issues/717>`_)
  Co-authored-by: Ryohsuke Mitsudome <ryoshuke.mitsudome@tier4.jp>
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(default_adapi): add a maintainer (`#714 <https://github.com/autowarefoundation/autoware_core/issues/714>`_)
* chore: jazzy-porting:fix qos profile issue (`#634 <https://github.com/autowarefoundation/autoware_core/issues/634>`_)
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Junya Sasaki, Mete Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo, mitsudome-r, 心刚

1.4.0 (2025-08-11)
------------------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat: release adapi v1.9.0 (`#550 <https://github.com/autowarefoundation/autoware_core/issues/550>`_)
* feat: port autoware_default_adapi from Autoware Universe (`#543 <https://github.com/autowarefoundation/autoware_core/issues/543>`_)
* Contributors: Ryohsuke Mitsudome, Takagi, Isamu, github-actions

* fix: to be consistent version in all package.xml(s)
* feat: release adapi v1.9.0 (`#550 <https://github.com/autowarefoundation/autoware_core/issues/550>`_)
* feat: port autoware_default_adapi from Autoware Universe (`#543 <https://github.com/autowarefoundation/autoware_core/issues/543>`_)
* Contributors: Ryohsuke Mitsudome, Takagi, Isamu, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-22)
------------------

0.2.0 (2025-02-07)
------------------

0.0.0 (2024-12-02)
------------------
