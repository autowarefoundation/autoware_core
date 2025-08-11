^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_behavior_velocity_stop_line_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* refactor(autoware_trajectory)!: move everything to namespace experimetal (`#371 <https://github.com/autowarefoundation/autoware_core/issues/371>`_)
  refactor(autoware_trajectory)!: move everything to namespace experimental
* Contributors: Mamoru Sobue

1.4.0 (2025-08-11)
------------------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(behavior_velocity_planner_common, stop_line_module): print module/regulatory_element/lane/line ID and improve stop line module log (`#504 <https://github.com/autowarefoundation/autoware_core/issues/504>`_)
  feat(behavior_velocity_planner_common, stop_line_module): print module/regulatory_element/lane/line ID and improve stop_line_module log
* fix(autoware_behavior_velocity_planner_common): fix deprecated autoware_utils header (`#441 <https://github.com/autowarefoundation/autoware_core/issues/441>`_)
  * fix autoware_utils header
  * style(pre-commit): autofix
  * fix to autoware_utils_debug
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix(stop_line): check linked lane id of stop line (`#468 <https://github.com/autowarefoundation/autoware_core/issues/468>`_)
  * fix bug of stop_line module
  * remove unused function
  * modify based on review
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* feat(behavior_velocity_planner): extend stop line to path bound (`#367 <https://github.com/autowarefoundation/autoware_core/issues/367>`_)
  * extend stop line to path bound
  * change signature of stop line extension function
  * add tests for extendSegmentToBounds
  * add tests for getEgoAndStopPoint
  * avoid using non-API function
  * add doxygen comment
  * add test case for extendSegmentToBounds
  * deprecate extendLine() instead of removing it
  * restore parameters for extend_path
  * remove deprecated for ci build of universe
  ---------
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat(behavior_velocity_planner)!: only wait for the required subscriptions (`#433 <https://github.com/autowarefoundation/autoware_core/issues/433>`_)
* refactor(autoware_trajectory)!: move everything to namespace experimetal (`#371 <https://github.com/autowarefoundation/autoware_core/issues/371>`_)
  refactor(autoware_trajectory)!: move everything to namespace experimental
* Contributors: Kosuke Takeuchi, Mamoru Sobue, Masaki Baba, Mitsuhiro Sakamoto, Takayuki Murooka, Yukinari Hisaki, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat:  port  autoware_behavior_velocity_planner from autoware.universe to autoware.core (`#230 <https://github.com/autowarefoundation/autoware_core/issues/230>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: 心刚 <90366790+liuXinGangChina@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, storrrrrrrrm
