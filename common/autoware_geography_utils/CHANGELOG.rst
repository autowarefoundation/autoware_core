^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_geography_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* feat(map_projection_loader): add scale_factor and remove altitude (`#340 <https://github.com/autowarefoundation/autoware_core/issues/340>`_)
* refactor(autoware_geography_utils): rewrite using modern C++ without API breakage (`#345 <https://github.com/autowarefoundation/autoware_core/issues/345>`_)
  * refactor using modern c++
  * precommit
  * revert
  * remove nodiscard
  * precommit
  ---------
* Contributors: Yamato Ando, Yutaka Kondo

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(autoware_geography_utils): disable tests for egm2008-1 (`#593 <https://github.com/autowarefoundation/autoware_core/issues/593>`_)
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* feat(map_projection_loader): add scale_factor and remove altitude (`#340 <https://github.com/autowarefoundation/autoware_core/issues/340>`_)
* refactor(autoware_geography_utils): rewrite using modern C++ without API breakage (`#345 <https://github.com/autowarefoundation/autoware_core/issues/345>`_)
  * refactor using modern c++
  * precommit
  * revert
  * remove nodiscard
  * precommit
  ---------
* Contributors: Yamato Ando, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-21)
------------------
* chore: fix CHANGELOG
* chore: rename from `autoware.core` to `autoware_core` (`#290 <https://github.com/autowarefoundation/autoware.core/issues/290>`_)
* feat(autoware_geography_utils): add support for local cartesian projection (`#181 <https://github.com/autowarefoundation/autoware.core/issues/181>`_)
  * feat(autoware_geography_utils): add support for local cartesian projection
  * feat(autoware_geography_utils): add support for local cartesian projection
  * style(pre-commit): autofix
  * feat(autoware_geography_utils): add support for local cartesian projection
  * add tests
  * fix the test
  * Update common/autoware_geography_utils/test/test_lanelet2_projector.cpp
  ---------
  Co-authored-by: Sebastian Zęderowski <szederowski@autonomous-systems.pl>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Sebastian Zęderowski, Yutaka Kondo, mitsudome-r

0.2.0 (2025-02-07)
------------------
* chore: humble to main sync (`#162 <https://github.com/autowarefoundation/autoware_core/issues/162>`_)
  * update changelog
  * 0.1.0
  ---------
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* refactor(autoware_geography_utils): apply modern C++17 style (`#109 <https://github.com/autowarefoundation/autoware_core/issues/109>`_)
  * use using
  * refactor height
  * refactor projection
  * refactor lanelet2_projector
  * set class name
  * revert string
  ---------
* test(autoware_geography_utils): add `lanelet2_projector` test (`#128 <https://github.com/autowarefoundation/autoware_core/issues/128>`_)
  * add test
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: sync files (`#115 <https://github.com/autowarefoundation/autoware_core/issues/115>`_)
  * chore: sync files
  * style(pre-commit): autofix
  * include what you use
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: M. Fatih Cırıt <mfc@autoware.org>
* docs(autoware_geography_utils): update `README.md` (`#111 <https://github.com/autowarefoundation/autoware_core/issues/111>`_)
  update readme
* feat: port autoware_geography_utils from autoware_universe (`#100 <https://github.com/autowarefoundation/autoware_core/issues/100>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Ryohsuke Mitsudome, Yutaka Kondo, awf-autoware-bot[bot]

0.1.0 (2025-01-09)
------------------
* refactor(autoware_geography_utils): apply modern C++17 style (`#109 <https://github.com/autowarefoundation/autoware_core/issues/109>`_)
  * use using
  * refactor height
  * refactor projection
  * refactor lanelet2_projector
  * set class name
  * revert string
  ---------
* test(autoware_geography_utils): add `lanelet2_projector` test (`#128 <https://github.com/autowarefoundation/autoware_core/issues/128>`_)
  * add test
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: sync files (`#115 <https://github.com/autowarefoundation/autoware_core/issues/115>`_)
  * chore: sync files
  * style(pre-commit): autofix
  * include what you use
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: M. Fatih Cırıt <mfc@autoware.org>
* docs(autoware_geography_utils): update `README.md` (`#111 <https://github.com/autowarefoundation/autoware_core/issues/111>`_)
  update readme
* feat: port autoware_geography_utils from autoware_universe (`#100 <https://github.com/autowarefoundation/autoware_core/issues/100>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Ryohsuke Mitsudome, Yutaka Kondo, awf-autoware-bot[bot]

0.0.0 (2024-12-02)
------------------
