^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_map_loader
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* feat(component_interface_specs): use template type in get_qos function (`#364 <https://github.com/autowarefoundation/autoware_core/issues/364>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* feat(map_loader): add the explanation of handling use_waypoints (`#342 <https://github.com/autowarefoundation/autoware_core/issues/342>`_)
* Contributors: Takagi, Isamu, Takayuki Murooka

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_lanelet2_utils): replace from/toBinMsg (`#737 <https://github.com/autowarefoundation/autoware_core/issues/737>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* chore: jazzy-porting: fix test depend launch-test missing (`#738 <https://github.com/autowarefoundation/autoware_core/issues/738>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Sarun MUKDAPITAK, github-actions, 心刚

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
* fix(autoware_ndt_scan_matcher): update link (`#510 <https://github.com/autowarefoundation/autoware_core/issues/510>`_)
  fix link
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
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* feat(component_interface_specs): use template type in get_qos function (`#364 <https://github.com/autowarefoundation/autoware_core/issues/364>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* feat(map_loader): add the explanation of handling use_waypoints (`#342 <https://github.com/autowarefoundation/autoware_core/issues/342>`_)
* Contributors: Takagi, Isamu, Takayuki Murooka, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat(autoware_map_loader): port autoware_map_loader from Autoware Universe to Core (`#326 <https://github.com/autowarefoundation/autoware_core/issues/326>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Ryohsuke Mitsudome
