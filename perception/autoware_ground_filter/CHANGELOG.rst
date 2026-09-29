^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_ground_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* feat(autoware_utils): remove managed transform buffer (`#360 <https://github.com/autowarefoundation/autoware_core/issues/360>`_)
  * feat(autoware_utils): remove managed transform buffer
  * fix(autoware_ground_filter): redundant inclusion
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Amadeusz Szymko

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* docs(mkdocs_macros): render README interfaces and parameters from node design files (`#1399 <https://github.com/autowarefoundation/autoware_core/issues/1399>`_)
  * feat(mkdocs_macros): render node design files and resolve schema $ref
  * build(pre-commit): bump autoware_system_designer to v0.4.2 for design format 0.4.0
  * docs(autoware_ground_filter): generate README interfaces and parameters from the node design file
  ---------
* fix(perception): declare the dependencies these packages use (`#1371 <https://github.com/autowarefoundation/autoware_core/issues/1371>`_)
  Each of these packages uses a package it never declares. Either it includes a
  header of that package, or it names a symbol of it while the header arrives
  through another dependency. Both build today only because some declared
  dependency re-exports the owner, so a change in an unrelated repository can
  break them without anything here changing.
  The tag follows where the dependency is used: a use in an installed header or
  in code compiled into the library takes <depend>, one reached only from test/
  takes <test_depend>. System libraries are named by the rosdep key this
  workspace already prefers.
* feat: [codecov/refactoring] [ground_filter] Step 3 - Unit test revamp (`#1216 <https://github.com/autowarefoundation/autoware_core/issues/1216>`_)
  * designated unit test suite in step 3 to be a friend class to allow access to private maths and ensure strict encapsulation
  * also expose the same for new Radial related unit tests
  * lock legacy 11 test cases for grid mode, and add new calss for radial mode unit testing
  * add test 1 of radial mode unit test
  * add test 2 of radial mode unit test
  * add test 3 of radial mode unit test
  * add test 4 of radial mode unit test
  * add test 5 of radial mode unit test
  * addressed the floating point rounding error in test 3, all good now
  * refactor(`downsample_filters`): simplify core logic (`#1219 <https://github.com/autowarefoundation/autoware_core/issues/1219>`_)
  * bug: remove unused `transform_info`
  * cosmetic: improve comments for readability
  ---------
  * moved setDataAccessor and process helper funcs back to private
  * removed the friend exposers of test classes
  * removed the friend class references in test_ground_filter
  * removed test 1 and 2 since they are basic math checkings, also revamped test 4 to be more sense
  * revamped test 5 to use the newly revamped group filter architecture:
  * also removed test 3 cuz it is too much of testing basic maths and won't really stand in the future when someone changes algo
  * added new test to verify a case when points in different azimuths towering above ground
  * within the 11 original tests, for every test involving legacy combo of setDataAcessor and process, replace it with comprehensive filter func
  * fixed the indexing issue in test ClassifyLocalAndGlobalSlopes, now all good wonderful yayyyy
  ---------
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* Tran ground filter step2 core logic isolation (`#1214 <https://github.com/autowarefoundation/autoware_core/issues/1214>`_)
  * added TEST 2 as a consolidate of 5/25 previous test_node (btw just realized precommit wont accept your commits until you fix em, damn
  * added 3rd test under indices publishing
  * added 4rd test under approximate sync functionality
  * strengthen TEST 2 with an incomparable point cloud
  * added TEST 5 to verify elevation_grid_mode is good and cool
  * rename node to ground_filter_node
  * also update node include to ground filter node include
  * updated GroundFilterParameter struct to envision the new core logic isolation
  * added an enum PointLabel to better characterize the radial/ray algo behavior for each point
  * added ray point centroids logic handlers
  * added various helper funcs to handle points addition and statistics in ray
  * added 3 more point cloud logic handling
  * fixed small error in declaration before token, now building good, moving on to others
  * removed wrong section created from wrong rebase
  * updated master process() switch in core logic
  * added convertPointCloud core logic func from the messy node
  * added calcVirtualGroundOrigin core logic func from the messy node
  * added classifyPointCloud core logic func from the messy node
  * fix case type of func, now build well and ok
  * cleaned ROS2 node wrapper header .hpp
  * cleaned ROS2 node wrapper module .cpp
  * successfully done the isolation, build ok
  * added FilterResult struct inside brain header to expedite more isolation
  * moved extractObjectPoints from the node to the core
  * moved the filter function from node to core
  * moved extractObjectPoints from node to core
  * removed all redundant useless declarations in node header
  * reorganized the CMakeLists.txt a lil bit
  * reorganized the ground_filter_node.hpp
  * reorganized the ground_filter core logic family
  * removed all the redundant stuffs in the ground_filter_node.cpp
  * finally fixed the test suite failing by reintroducing rclcpp back to test_ground_filter
* Contributors: Mete Fatih Cırıt, Taekjin LEE, Tran Huu Nhat Huy, github-actions

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix: [ground_filter] Remove dead code (`#1210 <https://github.com/autowarefoundation/autoware_core/issues/1210>`_)
* feat: [codecov/refactoring] [ground_filter] implement characterization test (`#1196 <https://github.com/autowarefoundation/autoware_core/issues/1196>`_)
* fix(autoware_ground_filter): reuse fixture node in parameter update test (`#1142 <https://github.com/autowarefoundation/autoware_core/issues/1142>`_)
  * fix(autoware_ground_filter): reuse fixture node in parameter update test
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Tran Huu Nhat Huy, Vishal Chauhan, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(ground_filter): remove unused `grid_id` (`#1025 <https://github.com/mitsudome-r/autoware_core/issues/1025>`_)
  * fix(ground_filter): remove unused `grid_id`
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_ground_filter): solve variableScope warning (`#956 <https://github.com/mitsudome-r/autoware_core/issues/956>`_)
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* test(autoware_ground_filter): add unit tests (`#497 <https://github.com/mitsudome-r/autoware_core/issues/497>`_)
  * test(autoware_ground_filter): add unit tests
  * style(pre-commit): autofix
  * fix(autoware_ground_filter): adapt tests to current codebase
  - Fix include paths after headers moved from include/ to src/
  - Add target_include_directories(PRIVATE src) for test targets
  - Fix integer division bugs (i/10 -> i/10.0f etc.) to produce
  correct floating point grid coordinates
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
* fix(ground_filter): remove unused struct member (`#812 <https://github.com/mitsudome-r/autoware_core/issues/812>`_)
* fix(autoware_ground_filter): fix bugprone-narrowing-conversions warnings (`#933 <https://github.com/mitsudome-r/autoware_core/issues/933>`_)
  * fix(autoware_ground_filter): fix bugprone-narrowing-conversions warnings
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(ground_fitler): move header files from include/ to src/ (`#863 <https://github.com/mitsudome-r/autoware_core/issues/863>`_)
  * chore(ground_fitler): move header files from include/ to src/
  * chore(ground_filter): remove unused variable suppressions in grid.hpp
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* Contributors: NorahXiong, Ryuta Kambe, Takahisa Ishikawa, github-actions

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_ground_filter): add empty point cloud check (`#746 <https://github.com/autowarefoundation/autoware_core/issues/746>`_)
  * fix(autoware_ground_filter): add empty point cloud check in isValid function
  * Update perception/autoware_ground_filter/include/autoware/ground_filter/node.hpp
  ---------
* chore: tf2_ros to hpp headers (`#616 <https://github.com/autowarefoundation/autoware_core/issues/616>`_)
* ci(pre-commit): autoupdate (`#723 <https://github.com/autowarefoundation/autoware_core/issues/723>`_)
  * pre-commit formatting changes
* Contributors: Mete Fatih Cırıt, Tim Clephas, Yutaka Kondo, github-actions

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
* fix: tf2 uses hpp headers in rolling (and is backported) (`#483 <https://github.com/autowarefoundation/autoware_core/issues/483>`_)
  * tf2 uses hpp headers in rolling (and is backported)
  * fixup! tf2 uses hpp headers in rolling (and is backported)
  ---------
* fix: deprecation of .h files in message_filters (`#467 <https://github.com/autowarefoundation/autoware_core/issues/467>`_)
  * fix: deprecation of .h files in message_filters
  * Update perception/autoware_ground_filter/include/autoware/ground_filter/node.hpp
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* fix(autoware_ground_filter): fix deprecated autoware_utils header (`#417 <https://github.com/autowarefoundation/autoware_core/issues/417>`_)
  * fix autoware_utils header
  * style(pre-commit): autofix
  * fix autoware_utils packages
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* feat(autoware_utils): remove managed transform buffer (`#360 <https://github.com/autowarefoundation/autoware_core/issues/360>`_)
  * feat(autoware_utils): remove managed transform buffer
  * fix(autoware_ground_filter): redundant inclusion
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Amadeusz Szymko, Masaki Baba, Tim Clephas, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat: re-implementation autoware_ground_filter as alpha quality from universe (`#311 <https://github.com/autowarefoundation/autoware_core/issues/311>`_)
  * add: `scan_ground_filter` from Autoware Universe
  **Source**: Files copied from [Autoware Universe](https://github.com/autowarefoundation/autoware_universe/tree/b8ce82e3759e50f780a0941ca8698ff52aa57b97/perception/autoware_ground_segmentation).
  **Scope**: Integrated `scan_ground_filter` into `autoware.core`.
  **Dependency Changes**: Removed dependencies on `autoware_pointcloud_preprocessor` to ensure compatibility within `autoware.core`.
  **Purpose**: Focus on making `scan_ground_filter` functional within the `autoware.core` environment.
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Junya Sasaki, Ryohsuke Mitsudome
