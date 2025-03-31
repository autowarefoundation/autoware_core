^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_mission_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat: port simplified version of autoware_mission_planner from Autoware Universe  (`#329 <https://github.com/autowarefoundation/autoware_core/issues/329>`_)
  * feat: port autoware_mission_planner from Autoware Universe
  * chore: reset package version and remove CHANGELOG
  * chore: remove the _universe suffix from autoware_mission_planner
  * feat: repalce tier4_planning_msgs with autoware_internal_planning_msgs
  * feat: remove route_selector module
  * feat: remove reroute_availability and modified_goal subscription
  * remove unnecessary image
  * style(pre-commit): autofix
  * fix: remove unnecessary include file
  * fix: resolve useInitializationList error from cppcheck
  * Apply suggestions from code review
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Ryohsuke Mitsudome
