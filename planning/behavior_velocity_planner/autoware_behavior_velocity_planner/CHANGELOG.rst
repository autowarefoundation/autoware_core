^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_behavior_velocity_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_behavior_velocity_planner): extract node-agnostic data-ingestion helpers (`#1114 <https://github.com/autowarefoundation/autoware_core/issues/1114>`_)
  * refactor(autoware_behavior_velocity_planner): extract node-agnostic data-ingestion helpers
  Extract the byte-identical traffic-signal aggregation and velocity-buffer
  pruning logic that was copy-pasted between the legacy and experimental
  BehaviorVelocityPlannerNode variants into package-internal, node-agnostic
  free functions (data_processing::apply_traffic_signals and
  data_processing::prune_velocity_buffer).
  Both node variants now call one implementation, removing the duplicated
  logic from processOdometry/processTrafficSignals. The new free functions
  are pure transforms over PlannerData fields, which creates a unit-test seam
  that previously did not exist: the only test was a launch-style smoke test
  with zero scene plugins.
  Add gtests asserting the traffic-signal UNKNOWN-keep-last-observation
  contract (keep prior body, refresh timestamp; first UNKNOWN stored as-is)
  and the velocity-buffer time pruning (stale-from-back removal, exact-threshold
  boundary retention, future-stamp negative-time guard).
  This change is internal-only and behavior-preserving: no public node API,
  topics, or message types change.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * refactor(autoware_behavior_velocity_planner): add <deque> include and bump new-file copyright year
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  ---------
* fix(planning): skip processing empty no_ground_pointcloud to avoid PCL warning spam (`#1082 <https://github.com/autowarefoundation/autoware_core/issues/1082>`_)
* Contributors: Mert Yavuz, Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore(planning, bvp): remove unused lanelet2_extension header (`#902 <https://github.com/mitsudome-r/autoware_core/issues/902>`_)
  * remove unused lanelet2_extension in bvp modules
  * remove unused lanelet2_extension in planning components
  ---------
* fix(autoware_behavior_velocity_planner): fix bugprone-narrowing-conversions warnings (`#939 <https://github.com/mitsudome-r/autoware_core/issues/939>`_)
* chore(bvp): port missing feature in behavior_velocity_planner's experimental version (`#892 <https://github.com/mitsudome-r/autoware_core/issues/892>`_)
  change tf2_ros to hpp header (616)
* chore: organize maintainer (`#857 <https://github.com/mitsudome-r/autoware_core/issues/857>`_)
* chore(planning, misc): remove unused header includes (`#840 <https://github.com/mitsudome-r/autoware_core/issues/840>`_)
* Contributors: Mamoru Sobue, NorahXiong, Sarun MUKDAPITAK, Satoshi OTA, github-actions

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore: tf2_ros to hpp headers (`#616 <https://github.com/autowarefoundation/autoware_core/issues/616>`_)
* Contributors: Tim Clephas, github-actions

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
* feat(behavior_velocity_planner): support processing_time for behavior_velocity_planner (`#677 <https://github.com/autowarefoundation/autoware_core/issues/677>`_)
  support processing time for bvp
* feat(autoware_behavior_velocity_planner): add roundabout module param path (`#603 <https://github.com/autowarefoundation/autoware_core/issues/603>`_)
  feat(behavior_velocity_planner): add roundabout module parameter path
* chore(behavior_velocity_planner): remove bvp run_out (`#629 <https://github.com/autowarefoundation/autoware_core/issues/629>`_)
* fix(autoware_behavior_velocity_planner): enable empty no_ground_pointcloud (`#614 <https://github.com/autowarefoundation/autoware_core/issues/614>`_)
  fix psim bug
* fix(autoware_behavior_velocity_planner): let is_ready be false if  no ground pointcloud is null (`#610 <https://github.com/autowarefoundation/autoware_core/issues/610>`_)
  * fix no_ground_pointcloud bug
  * refactor code
  ---------
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Kem (TiankuiXian), Mamoru Sobue, Mete Fatih Cırıt, Sho Iwasawa, Yuki TAKAGI, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore(motion_velocity_planner, behavior_velocity_planner): unifiy module load srv (`#585 <https://github.com/autowarefoundation/autoware_core/issues/585>`_)
  port srv
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome, Yuki TAKAGI

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(planning_factor): add console output option (`#513 <https://github.com/autowarefoundation/autoware_core/issues/513>`_)
  fix param json
  fix param json
  snake_case
  set default
* feat(behavior_velocity_planner): improve module registraion/deletion log (`#503 <https://github.com/autowarefoundation/autoware_core/issues/503>`_)
  feat(behavior_velocity_planner): imporve module registraion log
  update
  update
  ho
* chore: no longer support ROS 2 Galactic (`#492 <https://github.com/autowarefoundation/autoware_core/issues/492>`_)
* fix(autoware_behavior_velocity_planner): fix deprecated autoware_utils header (`#442 <https://github.com/autowarefoundation/autoware_core/issues/442>`_)
* feat(behavior_velocity_planner)!: remove unused function extendLine (`#457 <https://github.com/autowarefoundation/autoware_core/issues/457>`_)
  * remove unused function
  * remove unused parameter
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* feat(behavior_velocity_planner)!: only wait for the required subscriptions (`#433 <https://github.com/autowarefoundation/autoware_core/issues/433>`_)
* Contributors: Kosuke Takeuchi, Masaki Baba, Mitsuhiro Sakamoto, Takayuki Murooka, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat:  port  autoware_behavior_velocity_planner from autoware.universe to autoware.core (`#230 <https://github.com/autowarefoundation/autoware_core/issues/230>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: 心刚 <90366790+liuXinGangChina@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, storrrrrrrrm
