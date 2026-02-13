^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_behavior_velocity_stop_line_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* refactor(autoware_trajectory)!: move everything to namespace experimetal (`#371 <https://github.com/autowarefoundation/autoware_core/issues/371>`_)
  refactor(autoware_trajectory)!: move everything to namespace experimental
* Contributors: Mamoru Sobue

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(vehicle_info_utils): reduce autoware_utils deps (`#754 <https://github.com/autowarefoundation/autoware_core/issues/754>`_)
* ci(pre-commit): autoupdate (`#723 <https://github.com/autowarefoundation/autoware_core/issues/723>`_)
  * pre-commit formatting changes
* feat(stop_line): add vehicle_stopped_duration_threshold parameter (`#721 <https://github.com/autowarefoundation/autoware_core/issues/721>`_)
  * feat(stop_line): add vehicle_stopped_duration_threshold parameter
  - Add new parameter vehicle_stopped_duration_threshold
  - Duration threshold for determining if the vehicle is stopped
  - Used in isVehicleStopped() function
  - Rename stop_duration_sec to required_stop_duration_sec
  - Required stop duration at the stop line
  - Used for state transition from STOPPED to START
  This change adds a configurable threshold for vehicle stopped detection
  and improves parameter naming clarity to distinguish between the two
  duration-related parameters.
  * feat(stop_line): add vehicle_stopped_duration_threshold to experimental module
  - Add vehicle_stopped_duration_threshold parameter to experimental PlannerParam
  - Use vehicle_stopped_duration_threshold in isVehicleStopped() call
  - Add parameter loading in experimental manager
  This extends the vehicle_stopped_duration_threshold functionality to
  the experimental stop line module implementation.
* Contributors: Mete Fatih Cırıt, Yukinari Hisaki, github-actions

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(behavior_velocity_planner and behavior_velocity_planner_common)!: replace PathWithLaneId with Trajectory<> class (`#681 <https://github.com/autowarefoundation/autoware_core/issues/681>`_)
  Co-authored-by: mitukou1109 <mitukou1109@gmail.com>
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mamoru Sobue, Mete Fatih Cırıt, Yutaka Kondo, mitsudome-r

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
