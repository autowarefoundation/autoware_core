^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_simple_pure_pursuit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat: [codecov/refactoring] [simple_pure_pursuit] isolation of core logics, and full revamp of unit tests (`#1182 <https://github.com/autowarefoundation/autoware_core/issues/1182>`_)
  * init simpupu core logics module with Apache license
  * init simpupu core logics module with Apache license
  * init simpupu core logics header with Apache license
  * defined simpupu params struct
  * define public funcs
  * defined private funcs and entities for core logics
  * implement simple pure pursuit constructor and param setter
  * isolate create_control_command's core logics from the mess of ROS2+core logics in original node module
  * isolatecalc_longitudinal_control the same way
  * isolate calc_lateral_coltrol the same way
  * modified calc_lateral_control as for now completely immitates the original func (after discussing with the team)
  * refactored the simpupu main header - removing all core logics, leaving only ROS-related
  * refactored the simpupu main module - same way as headers - only ROS2 related + core logics inference
  * quick fixes based for clang compiler suggests
  * adapt cmakelists, package xml and header of core logics, thus wrapping up step 2
  * test_simple_pure_pursuit.cpp - first make it work on build
  * removed the old test cases in unit test module because the old ones are bundled and obsoleted with new architecture
  * added test 1 of perfectly normal case ahppy tracking
  * added test 2 of strong terminal brake at trajectory goal
  * added test 3 of strong terminal brake when trajectory is wayyyyyyyy too short
  * added test 4 of overriding with external target velocity
  * adeed test 5 of lateral offset correcting
  * added test 6 of lookahead distance clamping (the current calc logic is weird so I had to resort to infinity check)
  * added final branch coverage case of lookahead point search exceeding trajectory length case
  * some final touchups, all good I guess
  * style(pre-commit): autofix
  * deal with spell-check-differential
  * fixed that cpplink precommit test of you dont need a ; after a } thingy
  * style(pre-commit): autofix
  * fixed that cpplint precommit of Add #include <memory> for make_unique<>  [build/include_what_you_use]
  * change all  to  (yeah lol right, tbf I've used English for too long to the point of not thinking twice about plural single forms of these words
  * style(pre-commit): autofix
  * refactoring the file/class names as Ishikawa-san suggested
  * style(pre-commit): autofix
  * bring trajectory fallback to core logics
  * prevent node to return early upon false validation
  * dropping set_params() for ummitability
  * successful build
  * rebased, gonna add the precommit lint locally later
  * style(pre-commit): autofix
  * rebase
  * style(pre-commit): autofix
  * fixed spellcheck differentials
  * unified namespaces
  * Update control/autoware_simple_pure_pursuit/src/simple_pure_pursuit.hpp
  Co-authored-by: Takayuki AKAMINE <38586589+takam5f2@users.noreply.github.com>
  * [akamine] redundant function set_params() at header
  * [akamine] remove branch redundancy in clongitudinal control command core logic
  * [akamine] name the magic number -10.0 as terminal_brake_accel at simple_pure_pursuit logic header and apply it to whole node
  * [akamine] fix typo EXCEPT=>EXPECT
  * [akamine] fix class name in docstring
  * [akamine] node name inconsistency fix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Takayuki AKAMINE <38586589+takam5f2@users.noreply.github.com>
* feat: [codecod/refactoring] [simple_pure_pursuit] implement characterization test (`#1179 <https://github.com/autowarefoundation/autoware_core/issues/1179>`_)
  * init characterization test for simple pure pursuit node with Apache license
  * include generic dependencies
  * declare naemspace structures
  * added a dummy odometry generator
  * added a dummy trajectory generator
  * added a nodeoptions modifier to inject some dummy vehicle params
  * added an object to kickstart integration test env of this node
  * added sevevral destructors
  * added some helper funcs for debugging later if needdeD
  * added some more helpers to check control  retrieval states
  * init private section, added helper funcs to help cleaning and prepping between tests
  * added consts for connection and receiving timeouts and spin slepp duration
  * added test 1 basically standard happy etst case with straight seaty move
  * added test 2 as synchronization test
  * reconfigured CMakeLists to adapt to the new integ tset
  * added some test dependencies to package
  * done bug fixing and first stable build
  * terminal decellaration hardcode value confirmed, test 3 OK
  * added test 4 of empty traj, expected 0 vel 0 accel without crash or no command
  * colcon build and test successfully, all good I guess
  * final touch on comments, pretty please
  * style(pre-commit): autofix
  * added test 6 of lateral offset check as Ishikawa san suggests
  * added final test of lateral offset check as sugseted by Ishikawa san - build test done all good
  * style(pre-commit): autofix
  * Update control/autoware_simple_pure_pursuit/test/test_simple_pure_pursuit_integration.cpp
  Co-authored-by: Takayuki AKAMINE <38586589+takam5f2@users.noreply.github.com>
  * Update control/autoware_simple_pure_pursuit/CMakeLists.txt
  Co-authored-by: Takayuki AKAMINE <38586589+takam5f2@users.noreply.github.com>
  * Update control/autoware_simple_pure_pursuit/CMakeLists.txt
  Co-authored-by: Takayuki AKAMINE <38586589+takam5f2@users.noreply.github.com>
  * Update control/autoware_simple_pure_pursuit/test/test_simple_pure_pursuit_integration.cpp
  Co-authored-by: Takayuki AKAMINE <38586589+takam5f2@users.noreply.github.com>
  * Fixed all possible typos and misspells
  * maybe one last typo fix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Takayuki AKAMINE <38586589+takam5f2@users.noreply.github.com>
* Contributors: Tran Huu Nhat Huy, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_simple_pure_pursuit): fix bugprone-narrowing-conversions warnings (`#918 <https://github.com/mitsudome-r/autoware_core/issues/918>`_)
  * fix(autoware_simple_pure_pursuit): fix bugprone-narrowing-conversions warnings
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: NorahXiong, github-actions

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_simple_pure_pursuit): add empty trajectory check to prevent crash (`#744 <https://github.com/autowarefoundation/autoware_core/issues/744>`_)
  * fix(autoware_simple_pure_pursuit): add empty trajectory check to prevent crash
  * Apply suggestion from @Copilot
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  * test(autoware_simple_pure_pursuit): add test for empty trajectory check
  * Update control/autoware_simple_pure_pursuit/src/simple_pure_pursuit.cpp
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
  ---------
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* ci(pre-commit): autoupdate (`#723 <https://github.com/autowarefoundation/autoware_core/issues/723>`_)
  * pre-commit formatting changes
* Contributors: Mete Fatih Cırıt, Yutaka Kondo, github-actions

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
