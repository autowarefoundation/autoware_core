^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_ekf_localizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.7.0 (2026-02-14)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(ekf_localizer): queue pop on ekf localizer (`#679 <https://github.com/autowarefoundation/autoware_core/issues/679>`_)
  * feat: separate max_age and max_queue_size in AgedObjectQueue
  When multiple pose sources (e.g., GNSS + NDT) are active, the queue
  can legitimately grow beyond max_age. Separating these concerns allows:
  1. max_age controls how many times each element is reused
  2. max_queue_size monitors overall queue health without enforcing hard limits
  * fix: warn and pop if queue is exceeded
  * refactor: make less if statement
  * chore: fix unclear comments
  * doc: update schema.json
  * doc: modify comments
  ---------
* Contributors: Motz, Ryohsuke Mitsudome

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore: jazzy-porting: fix test depend launch-test missing (`#738 <https://github.com/autowarefoundation/autoware_core/issues/738>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: tf2_ros to hpp headers (`#616 <https://github.com/autowarefoundation/autoware_core/issues/616>`_)
* Contributors: Tim Clephas, github-actions, 心刚

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: update maintainer (`#637 <https://github.com/autowarefoundation/autoware_core/issues/637>`_)
  * chore: update maintainer
  remove Takeshi Ishita
  * chore: update maintainer
  remove Kento Yabuuchi
  * chore: update maintainer
  remove Shintaro Sakoda
  * chore: update maintainer
  remove Ryu Yamamoto
  ---------
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Motz, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* feat: support ROS 2 Jazzy (`#487 <https://github.com/autowarefoundation/autoware_core/issues/487>`_)
  * fix ekf_localizer
  * fix lanelet2_map_loader_node
  * MUST REVERT
  * fix pybind
  * fix depend
  * add buildtool
  * remove
  * revert
  * find_package
  * wip
  * remove embed
  * find python_cmake_module
  * public
  * remove ament_cmake_python
  * fix autoware_trajectory
  * add .lcovrc
  * fix egm
  * use char*
  * use global
  * namespace
  * string view
  * clock
  * version
  * wait
  * fix egm2008-1
  * typo
  * fixing
  * fix egm2008-1
  * MUST REVERT
  * fix egm2008-1
  * fix twist_with_covariance
  * Revert "MUST REVERT"
  This reverts commit 93b7a57f99dccf571a01120132348460dbfa336e.
  * namespace
  * fix qos
  * revert some
  * comment
  * Revert "MUST REVERT"
  This reverts commit 7a680a796a875ba1dabc7e714eaea663d1e5c676.
  * fix dungling pointer
  * fix memory alignment
  * ignored
  * spellcheck
  ---------
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* fix(autoware_ekf_localizer): use constexpr and string_view (`#435 <https://github.com/autowarefoundation/autoware_core/issues/435>`_)
  * fix(autoware_ekf_localizer) use constexpr and string_view
  * add std::string_view header
  * add std::string header
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* fix(autoware_ekf_localizer): modified log output section to use warning_message and throttle (`#374 <https://github.com/autowarefoundation/autoware_core/issues/374>`_)
  * fix(autoware_ekf_localizer): Modified log output section to use warning_message and throttle
  * use constexpr and string_view
  ---------
* fix(autoware_ekf_localizer): fix deprecated autoware_utils header (`#412 <https://github.com/autowarefoundation/autoware_core/issues/412>`_)
  * fix autoware_utils import
  * fix autoware_utils packages
  ---------
* Contributors: Masaki Baba, RyuYamamoto, Tim Clephas, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* chore(ekf_localizer): increase z_filter_proc_dev for large gradient road (`#211 <https://github.com/autowarefoundation/autoware.core/issues/211>`_)
  increase z_filter_proc_dev
  Co-authored-by: SakodaShintaro <shintaro.sakoda@tier4.jp>
* feat(autoware_ekf_localizer)!: porting from universe to core 2nd (`#180 <https://github.com/autowarefoundation/autoware.core/issues/180>`_)
* Contributors: Kento Yabuuchi, Motz, mitsudome-r
