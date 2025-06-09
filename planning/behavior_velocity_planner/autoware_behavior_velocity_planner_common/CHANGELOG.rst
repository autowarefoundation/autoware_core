^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_behavior_velocity_planner_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.2.0 (2025-06-09)
------------------
* fix(autoware_behavior_velocity_planner_common): fix deprecated autoware_utils header (`#441 <https://github.com/autowarefoundation/autoware_core/issues/441>`_)
  * fix autoware_utils header
  * style(pre-commit): autofix
  * fix to autoware_utils_debug
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* fix(behavior_velocity_planner_common): fix argument type of findCollisionSegment (`#474 <https://github.com/autowarefoundation/autoware_core/issues/474>`_)
* fix(behavior_velocity_planner_common): support overlap lane (`#472 <https://github.com/autowarefoundation/autoware_core/issues/472>`_)
* fix(stop_line): check linked lane id of stop line (`#468 <https://github.com/autowarefoundation/autoware_core/issues/468>`_)
  * fix bug of stop_line module
  * remove unused function
  * modify based on review
  ---------
* feat(behavior_velocity_planner)!: remove unused function extendLine (`#457 <https://github.com/autowarefoundation/autoware_core/issues/457>`_)
  * remove unused function
  * remove unused parameter
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* refactor(bvp_common, QC): deprecate checkCollision() (`#429 <https://github.com/autowarefoundation/autoware_core/issues/429>`_)
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
* fix(autoware_velocity_smoother): fix deprecated autoware_utils header (`#424 <https://github.com/autowarefoundation/autoware_core/issues/424>`_)
  * fix autoware_utils header
  * style(pre-commit): autofix
  * add header for timekeeper
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Mamoru Sobue, Masaki Baba, Mitsuhiro Sakamoto, Takayuki Murooka, Tim Clephas, Yukinari Hisaki, Yutaka Kondo

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat:  port  autoware_behavior_velocity_planner from autoware.universe to autoware.core (`#230 <https://github.com/autowarefoundation/autoware_core/issues/230>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: 心刚 <90366790+liuXinGangChina@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, storrrrrrrrm
