^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_route_handler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* feat(route_handler): use a cost metric to select start lane (`#357 <https://github.com/autowarefoundation/autoware_core/issues/357>`_)
  * feat(route_handler): use a cost metric to select start lane
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Mert Çolak

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(route_handler): add bicycle lane getter (`#589 <https://github.com/autowarefoundation/autoware_core/issues/589>`_)
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Mamoru Sobue, Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* perf(route_handler): improve functions to get nearest route lanelet (`#532 <https://github.com/autowarefoundation/autoware_core/issues/532>`_)
* fix(autoware_route_handler): fix deprecated autoware_utils header (`#446 <https://github.com/autowarefoundation/autoware_core/issues/446>`_)
  * fix autoware_utils header
  * style(pre-commit): autofix
  * fix to autoware_utils_geometry
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* fix(route_handler): fix error message for the route planning failure (`#463 <https://github.com/autowarefoundation/autoware_core/issues/463>`_)
  * fix(route_handler): fix error message for the route planning failure
  * use stringstream
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* fix(route_handler): getCenterLinePath with cropping waypoints (`#378 <https://github.com/autowarefoundation/autoware_core/issues/378>`_)
* feat(route_handler): remove only_route_lanes = false usage (`#450 <https://github.com/autowarefoundation/autoware_core/issues/450>`_)
* feat(route_handler, QC): add overload of get_shoulder_lanelet_sequence (`#431 <https://github.com/autowarefoundation/autoware_core/issues/431>`_)
  feat(route_handler): add get_shoulder_lanelet_sequence
* feat(route_handler): use a cost metric to select start lane (`#357 <https://github.com/autowarefoundation/autoware_core/issues/357>`_)
  * feat(route_handler): use a cost metric to select start lane
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Mamoru Sobue, Masaki Baba, Maxime CLEMENT, Mert Çolak, Takayuki Murooka, Tim Clephas, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* feat: port autoware_route_handler from Autoware Universe (`#201 <https://github.com/autowarefoundation/autoware.core/issues/201>`_)
  Co-authored-by: Mete Fatih Cırıt <xmfcx@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, mitsudome-r
