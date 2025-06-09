^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_velocity_smoother
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.2.0 (2025-06-09)
------------------
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
* Contributors: Masaki Baba, Mitsuhiro Sakamoto, Tim Clephas, Yutaka Kondo

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat(autoware_velocity_smoother): port the package from Autoware Universe (`#299 <https://github.com/autowarefoundation/autoware_core/issues/299>`_)
* Contributors: Ryohsuke Mitsudome
