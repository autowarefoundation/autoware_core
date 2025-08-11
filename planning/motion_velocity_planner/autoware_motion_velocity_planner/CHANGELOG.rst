^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_velocity_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(motion_velocity_planner, motion_velocity_planner_common): update pointcloud preprocess design (`#591 <https://github.com/autowarefoundation/autoware_core/issues/591>`_)
  * updare pcl preprocess
  ---------
* chore(motion_velocity_planner, behavior_velocity_planner): unifiy module load srv (`#585 <https://github.com/autowarefoundation/autoware_core/issues/585>`_)
  port srv
* refactor(motion_velocity_planner): restrict copy of planner_data  (`#587 <https://github.com/autowarefoundation/autoware_core/issues/587>`_)
  * restrict plannar data copy
  ---------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome, Yuki TAKAGI

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(motion_velocity_planner_node): update pcl coordinate transformation (`#519 <https://github.com/autowarefoundation/autoware_core/issues/519>`_)
  * update pcl coordinate transformation
  * add canTransform check
  ---------
* feat(autoware_motion_velocity_planner)!: only wait for the required subscriptions (`#505 <https://github.com/autowarefoundation/autoware_core/issues/505>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* fix(autoware_motion_velocity_planner): fix deprecated autoware_utils header (`#444 <https://github.com/autowarefoundation/autoware_core/issues/444>`_)
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* feat(autoware_motion_velocity_planner): point-cloud clustering optimization (`#409 <https://github.com/autowarefoundation/autoware_core/issues/409>`_)
  * Core changes for point-cloud maksing and clustering
  * fix
  * style(pre-commit): autofix
  * Update planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/planner_data.hpp
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  * fix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Arjun Jagdish Ram, Masaki Baba, Ryohsuke Mitsudome, Tim Clephas, Yuki TAKAGI, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat: autoware_motion_velocity_planner and autoware_motion_velocity_planner_common to core (`#242 <https://github.com/autowarefoundation/autoware_core/issues/242>`_)
  * feat: modify autoware_universe_utils to autoware_utils
  * feat: remove useless dependecy
  * style(pre-commit): autofix
  * feat: modify autoware_universe_utils to autoware_utils
  * feat: autoware_motion_velocity_planner_node to core
  * style(pre-commit): autofix
  * feat: autoware_motion_velocity_planner_node to core
  * feat: modify autoware_universe_utils to autoware_utils
  * feat: remove useless dependecy
  * style(pre-commit): autofix
  * feat: modify autoware_universe_utils to autoware_utils
  * style(pre-commit): autofix
  * feat: port autoware_behavior_velocity_planner to core
  * fix: apply latest changes from autoware.universe
  * fix: deadlinks in README
  * fix(autoware_motion_velocity_planner): porting autoware_motion_velocity_planner, autoware_motion_velocity_planner, sync with latest universe: v0.2
  * style(pre-commit): autofix
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/node.hpp
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/include/autoware/behavior_velocity_planner_common/utilization/util.hpp
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/src/utilization/util.cpp
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/test/src/test_util.cpp
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/debug.cpp
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/manager.cpp
  * style(pre-commit): autofix
  * Update planning/motion_velocity_planner/autoware_motion_velocity_planner/CMakeLists.txt
  * style(pre-commit): autofix
  * feat(autoware_motion_velocity_planner_node): porting autoware_motion_velocity_planner_node, autoware_motion_velocity_planner_node, remove metrics msgs publish according to pr-10342 under universe repo: v0.5
  * feat(autoware_motion_velocity_planner_common): porting autoware_motion_velocity_planner_common, autoware_motion_velocity_planner_common, port to core repo: v0.0
  * style(pre-commit): autofix
  * move
  * rename
  * fix exec
  * add maintainer
  * style(pre-commit): autofix
  * fix test_depend
  * feat(autoware_motion_velocity_planner_node): porting autoware_motion_velocity_planner_node, autoware_motion_velocity_planner_node, resolve build issue: v0.6
  ---------
  Co-authored-by: suchang <chang.su@autocore.ai>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <ryohsuke.mitsudome@tier4.jp>
  Co-authored-by: liuXinGangChina <lxg19892021@gmail.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: 心刚 <90366790+liuXinGangChina@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, storrrrrrrrm
