^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_velocity_smoother
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: change planning output topic name to /planning/trajectory (`#602 <https://github.com/autowarefoundation/autoware_core/issues/602>`_)
  * change planning output topic name to /planning/trajectory
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_mission_planner, velocity_smoother): use transient_local for operation_mode_state (`#598 <https://github.com/autowarefoundation/autoware_core/issues/598>`_)
  subscribe transient_local with transient_local
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* refactor: implement varying lateral acceleration and steering rate threshold in velocity smoother (`#531 <https://github.com/autowarefoundation/autoware_core/issues/531>`_)
  * refactor: implement varying steering rate threshold in velocity smoother
  * feat: implement varying lateral acceleration limit
  * fix  typo in readme
  * fix bugs in unit conversion
  * clean up the obsolete parameter in core planning launch
  ---------
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
* Contributors: Kem (TiankuiXian), Ryohsuke Mitsudome, Yukihiro Saito, Yuxuan Liu

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* fix(velocity_smoother): prevent access when vector is empty (`#438 <https://github.com/autowarefoundation/autoware_core/issues/438>`_)
  add empty check
* fix(autoware_velocity_smoother): fix deprecated autoware_utils header (`#424 <https://github.com/autowarefoundation/autoware_core/issues/424>`_)
  * fix autoware_utils header
  * style(pre-commit): autofix
  * add header for timekeeper
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Masaki Baba, Mitsuhiro Sakamoto, Tim Clephas, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat(autoware_velocity_smoother): port the package from Autoware Universe (`#299 <https://github.com/autowarefoundation/autoware_core/issues/299>`_)
* Contributors: Ryohsuke Mitsudome
