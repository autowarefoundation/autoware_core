^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_simple_pure_pursuit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome, Yukihiro Saito

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* docs(simple_pure_pursuit): add flowchart (`#481 <https://github.com/autowarefoundation/autoware_core/issues/481>`_)
  * add flowchart
  * add default value to parameter schema
  * fix parameter schema
  ---------
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* fix(autoware_simple_pure_pursuit): fix document (`#473 <https://github.com/autowarefoundation/autoware_core/issues/473>`_)
  fix deadlink
* fix(autoware_simple_pure_pursuit): fix path follower lateral deviation (`#425 <https://github.com/autowarefoundation/autoware_core/issues/425>`_)
  * fix::autoware_simple_pure_pursuit::fix path follower lateral deviation, v0.0
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* fix(autoware_simple_pure_pursuit): fix deprecated autoware_utils header (`#416 <https://github.com/autowarefoundation/autoware_core/issues/416>`_)
  * fix autoware_utils header
  * style(pre-commit): autofix
  * fix autoware_utils packages
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Masaki Baba, Mitsuhiro Sakamoto, Tim Clephas, Yukinari Hisaki, Yutaka Kondo, github-actions, 心刚

1.0.0 (2025-03-31)
------------------
* fix(autoware_simple_pure_pursuit): make control command output transient local (`#328 <https://github.com/autowarefoundation/autoware_core/issues/328>`_)
  * fix(autoware_simple_pure_pursuit): change control command topic to transient local
  * style(pre-commit): autofix
  * fix: use timestamp from subscribed message
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  * fix: use timestamp from subscribed message
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Ryohsuke Mitsudome

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* feat: add autoware_simple_pure_pursuit package (`#140 <https://github.com/autowarefoundation/autoware.core/issues/140>`_)
* Contributors: Takayuki Murooka, mitsudome-r
