^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_pyplot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* feat(trajectory): add API description, nomenclature, illustration, rename functions to align with nomenclature (`#292 <https://github.com/autowarefoundation/autoware_core/issues/292>`_)
  * feat(trajectory): add API description, nomenclature, illustration, rename functions to align with nomenclature
  * resurrect get_internal_base
  ---------
* Contributors: Mamoru Sobue

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_pyplot): adds stem plot support (`#740 <https://github.com/autowarefoundation/autoware_core/issues/740>`_)
  feat(autoware_pyplot): add stem plot
* Contributors: Zulfaqar Azmi, github-actions

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Yutaka Kondo, mitsudome-r

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
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* feat(trajectory): add API description, nomenclature, illustration, rename functions to align with nomenclature (`#292 <https://github.com/autowarefoundation/autoware_core/issues/292>`_)
  * feat(trajectory): add API description, nomenclature, illustration, rename functions to align with nomenclature
  * resurrect get_internal_base
  ---------
* Contributors: Mamoru Sobue, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* feat(trajectory): remove default ctor and collect default setting in Builder (`#287 <https://github.com/autowarefoundation/autoware_core/issues/287>`_)
* Contributors: Mamoru Sobue

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* feat(trajectory): improve comment, use autoware_pyplot for examples (`#282 <https://github.com/autowarefoundation/autoware.core/issues/282>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* feat: port autoware_pyplot from Autoware Universe to Autoware Core (`#199 <https://github.com/autowarefoundation/autoware.core/issues/199>`_)
* Contributors: Mamoru Sobue, Ryohsuke Mitsudome, mitsudome-r
