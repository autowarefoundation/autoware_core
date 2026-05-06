^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_agnocast_wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.0 (2026-05-01)
------------------
* chore: align package versions to 1.7.0 and reset changelogs
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* docs(autoware_agnocast_wrapper): use FindPackageShare and PathJoinSubstitution in Python launch examples (`#999 <https://github.com/mitsudome-r/autoware_core/issues/999>`_)
* fix(autoware_agnocast_wrapper): support jazzy (rclcpp 28~) (`#980 <https://github.com/mitsudome-r/autoware_core/issues/980>`_)
  fix(autoware_agnocast_wrapper): fix Jazzy build by using version-conditional callback type alias
* fix(autoware_agnocast_wrapper): fix false positive LD_PRELOAD warning in agnocast_env.launch.py (`#973 <https://github.com/mitsudome-r/autoware_core/issues/973>`_)
* refactor(autoware_agnocast_wrapper): remove executor threading model consistency validation (`#970 <https://github.com/mitsudome-r/autoware_core/issues/970>`_)
  * refactor(autoware_agnocast_wrapper): remove executor threading model consistency validation
  * docs(autoware_agnocast_wrapper): clarify behavior reference tables in README
  - Reword introductory sentence to clarify that only ROS2_EXECUTOR matters
  when ENABLE_AGNOCAST=0
  - Use full CMake option strings (e.g. SingleThreadedExecutor) instead of
  abbreviations in both behavior reference tables
  * style(pre-commit): autofix
  * fix(autoware_agnocast_wrapper): fix forbidden word ROS2 to ROS 2 in README
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_agnocast_wrapper): support message filter in agnocast wrapper (`#951 <https://github.com/mitsudome-r/autoware_core/issues/951>`_)
  * feat(autoware_agnocast_wrapper): add register_node macro for runtime rclcpp/agnocast switching
  Add `autoware_agnocast_wrapper_register_node` CMake macro as a drop-in
  replacement for `rclcpp_components_register_node`. When ENABLE_AGNOCAST=1,
  it generates a standalone executable that can switch between rclcpp::Node
  and agnocast::Node at runtime based on the ENABLE_AGNOCAST environment
  variable. When ENABLE_AGNOCAST is not set, it falls back to standard
  rclcpp_components_register_node behavior with zero overhead.
  Key features:
  - Configurable ROS2 and Agnocast executor types
  - Two-pass template generation (configure_file + file(GENERATE))
  - Support for both rclcpp::Node and agnocast_wrapper::Node plugins
  - Target existence validation at configure time
  - ABI consistency enforcement via autoware_agnocast_wrapper_setup()
  - Change agnocastlib from build_depend to depend
  * fix(autoware_agnocast_wrapper): add static_assert to enforce PLUGIN base class at compile time
  * fix(autoware_agnocast_wrapper): add runtime ROS2 fallback for AgnocastOnly executors
  When agnocast_only=true but ENABLE_AGNOCAST=0 at runtime, the node now
  falls back to the ROS2 executor instead of unconditionally using the
  AgnocastOnly executor.
  * fix(autoware_agnocast_wrapper): warn on mismatched executor threading models
  Emit a CMake WARNING when ROS2_EXECUTOR and AGNOCAST_EXECUTOR have
  different threading models (e.g., SingleThreadedExecutor with
  MultiThreadedAgnocastExecutor), as this silently changes behavior
  depending on the runtime ENABLE_AGNOCAST value.
  * docs(autoware_agnocast_wrapper): add executor behavior reference table to README
  * style(pre-commit): autofix
  * fix(autoware_agnocast_wrapper): replace forbidden word ROS2 with ROS 2
  * style(pre-commit): autofix
  * fix(autoware_agnocast_wrapper): remove static_assert that fails in non-template main()
  The static_assert inside if constexpr (agnocast_only) is always evaluated
  because main() is not a template function. Additionally, the component
  type header is not included in the generated source (loaded via
  class_loader at runtime), so the type cannot be resolved at compile time.
  * delete deprecated mark from publish(const MessageT&) API
  * support message_filter in agnocast_wrapper
  * style(pre-commit): autofix
  * fix cpplint
  * fix cpplint
  * add comments
  * unify header declaration
  * address message_filters.hpp review feedback
  * style(pre-commit): autofix
  * wrapper for #else
  * style(pre-commit): autofix
  * move documentation comments
  * fix to alias in #else branch
  * style(pre-commit): autofix
  * add document to README
  * style(pre-commit): autofix
  * fix to use AUTOWARE_MESSAGE_CONST_SHARED_PTR in the documenta
  * add const to AUTOWARE_MESSAGE_CONST_SHARED_PTR macro
  * fix to separate the opration from unique_ptr
  * style(pre-commit): autofix
  * fix to separate the operation from unique_ptr
  * style(pre-commit): autofix
  * fix to not allow unique ptr in subscription
  * revert unique_ptr disallowing, and add comments
  * style(pre-commit): autofix
  * delete unncessary documents sentences
  ---------
  Co-authored-by: atsushi421 <atsushi.yano.2@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_agnocast_wrapper): add const to AUTOWARE_MESSAGE_CONST_SHARED_PTR macro (`#960 <https://github.com/mitsudome-r/autoware_core/issues/960>`_)
  * add const to AUTOWARE_MESSAGE_CONST_SHARED_PTR macro
  * style(pre-commit): autofix
  * fix to separate the operation from unique_ptr
  * style(pre-commit): autofix
  * fix to not allow unique ptr in subscription
  * revert unique_ptr disallowing, and add comments
  * fix copilot review
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* docs(autoware_agnocast_wrapper): add `agnocast_wrapper_review_guide` documentation (`#957 <https://github.com/mitsudome-r/autoware_core/issues/957>`_)
  * add agnocast_review_guide documentation
  * style(pre-commit): autofix
  * fix for index
  * fix lint check
  * fix index again
  * add blank line and some other fixes
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_agnocast_wrapper): use rclcpp_components_register_nodes to avoid target name collision (`#955 <https://github.com/mitsudome-r/autoware_core/issues/955>`_)
  * fix(autoware_agnocast_wrapper): use rclcpp_components_register_nodes to avoid target name collision
  Replace rclcpp_components_register_node (singular) with
  rclcpp_components_register_nodes (plural) in the agnocast wrapper macro.
  The singular form creates both a component registration and a standalone
  executable named <EXECUTABLE>_component. This executable is never used
  but causes CMake target name collisions when a package's library target
  happens to match the generated executable name (e.g.,
  autoware_raw_vehicle_cmd_converter_node_component).
  The plural form only populates the ament resource index for component
  container support without generating the unnecessary executable.
  * refactor(autoware_agnocast_wrapper): remove unnecessary intermediate variables
  The _AGNOCAST_WRAPPER_COMPONENT and _AGNOCAST_WRAPPER_NODE variables
  were only needed because the old rclcpp_components_register_node
  (singular) macro clobbered variables in the caller's scope. Since the
  switch to rclcpp_components_register_nodes (plural), this is no longer
  the case. Use ARGS_PLUGIN and ARGS_EXECUTABLE directly.
  Also fix a stale comment that still referenced the singular macro name.
  * docs(autoware_agnocast_wrapper): fix stale comment to cover both singular and plural register_node
  ---------
* fix(autoware_agnocast_wrapper): separate to `CONST_SHARED_PTR` and mutable `SHARED_PTR` (`#953 <https://github.com/mitsudome-r/autoware_core/issues/953>`_)
  * separate to CONST_SHARED_PTR and mutable SHARED_PTR
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_agnocast_wrapper): delete `deprecated` mark from `publish(const MessageT&)` API (`#950 <https://github.com/mitsudome-r/autoware_core/issues/950>`_)
  delete deprecated mark from publish(const MessageT&) API
* feat(autoware_agnocast_wrapper): add Python version of agnocast_env.launch.xml (`#952 <https://github.com/mitsudome-r/autoware_core/issues/952>`_)
  * feat(autoware_agnocast_wrapper): add Python version of agnocast_env.launch.xml
  * docs(autoware_agnocast_wrapper): add Python launch file usage examples to README
  * docs(autoware_agnocast_wrapper): improve Python launch file examples in README
  Use os.path.join for path construction instead of string concatenation.
  * fix(autoware_agnocast_wrapper): use launch substitution for LD_PRELOAD instead of os.environ
  ---------
* feat(autoware_agnocast_wrapper): add register_node macro for runtime rclcpp/agnocast switching (`#949 <https://github.com/mitsudome-r/autoware_core/issues/949>`_)
  * feat(autoware_agnocast_wrapper): add register_node macro for runtime rclcpp/agnocast switching
  Add `autoware_agnocast_wrapper_register_node` CMake macro as a drop-in
  replacement for `rclcpp_components_register_node`. When ENABLE_AGNOCAST=1,
  it generates a standalone executable that can switch between rclcpp::Node
  and agnocast::Node at runtime based on the ENABLE_AGNOCAST environment
  variable. When ENABLE_AGNOCAST is not set, it falls back to standard
  rclcpp_components_register_node behavior with zero overhead.
  Key features:
  - Configurable ROS2 and Agnocast executor types
  - Two-pass template generation (configure_file + file(GENERATE))
  - Support for both rclcpp::Node and agnocast_wrapper::Node plugins
  - Target existence validation at configure time
  - ABI consistency enforcement via autoware_agnocast_wrapper_setup()
  - Change agnocastlib from build_depend to depend
  * fix(autoware_agnocast_wrapper): add static_assert to enforce PLUGIN base class at compile time
  * fix(autoware_agnocast_wrapper): add runtime ROS2 fallback for AgnocastOnly executors
  When agnocast_only=true but ENABLE_AGNOCAST=0 at runtime, the node now
  falls back to the ROS2 executor instead of unconditionally using the
  AgnocastOnly executor.
  * fix(autoware_agnocast_wrapper): warn on mismatched executor threading models
  Emit a CMake WARNING when ROS2_EXECUTOR and AGNOCAST_EXECUTOR have
  different threading models (e.g., SingleThreadedExecutor with
  MultiThreadedAgnocastExecutor), as this silently changes behavior
  depending on the runtime ENABLE_AGNOCAST value.
  * docs(autoware_agnocast_wrapper): add executor behavior reference table to README
  * style(pre-commit): autofix
  * fix(autoware_agnocast_wrapper): replace forbidden word ROS2 with ROS 2
  * style(pre-commit): autofix
  * fix(autoware_agnocast_wrapper): remove static_assert that fails in non-template main()
  The static_assert inside if constexpr (agnocast_only) is always evaluated
  because main() is not a template function. Additionally, the component
  type header is not included in the generated source (loaded via
  class_loader at runtime), so the type cannot be resolved at compile time.
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat: add `agnocast_env.launch.xml` and update README (`#944 <https://github.com/mitsudome-r/autoware_core/issues/944>`_)
  * add agnocast_env.launch.xml and update README
  * fix based on copilot review: for heaphook_path as an arg & fix README
  * fix to use colon for separater
  * add container_executable
  * style(pre-commit): autofix
  * add comment in document
  * delete use_agnocast_component_container_cie
  * style(pre-commit): autofix
  * handle PRs with no C++ file changes in clang-tidy step
  * add container_package variable
  * style(pre-commit): autofix
  * added dependency in package.xml
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_agnocast_wrapper): support `agnocast_wrapper::Node` (`#943 <https://github.com/mitsudome-r/autoware_core/issues/943>`_)
  * feat: add autoware_agnocast_wrapper (moved from autoware_universe)
  * Agnocast Publisher/Subscriber/PollingSubscriber for agnocast::Node
  * implement agnocast_wrapper::Node class
  * define to_rclcpp_node to be used for test
  * add autoware_utils dependency
  * delete statically defined USE_AGNOCAST_ENABLED
  * fix for copilot review
  * fix for copilot review
  * style(pre-commit): autofix
  * add USE_AGNOCAST_ENABLED to the whole node.cpp
  * style(pre-commit): autofix
  * fix copilot review for first three comments
  * style(pre-commit): autofix
  * fix for copilot review for last two comments
  * fix for copilot review
  * include some header files for cpplint fails
  * fix to delete autoware_utils and fix Cmakelists.txt for clang-tidy
  * update README
  * add @throws for documentation comment
  ---------
  Co-authored-by: atsushi421 <atsushi.yano.2@tier4.jp>
  Co-authored-by: atsushi yano <55824710+atsushi421@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_agnocast_wrapper): add publish by const ref and publisher accessor methods  (`#915 <https://github.com/mitsudome-r/autoware_core/issues/915>`_)
  * feat: add autoware_agnocast_wrapper (moved from autoware_universe)
  * add publish by const ref and publisher accessor methods
  * style(pre-commit): autofix
  * add warning when compilation and add comments
  ---------
  Co-authored-by: atsushi421 <atsushi.yano.2@tier4.jp>
  Co-authored-by: atsushi yano <55824710+atsushi421@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_agnocast_wrapper): templatize publisher/subscription for `agnocast::Node` (`#916 <https://github.com/mitsudome-r/autoware_core/issues/916>`_)
  * feat: add autoware_agnocast_wrapper (moved from autoware_universe)
  * Agnocast Publisher/Subscriber/PollingSubscriber for agnocast::Node
  * delete statically defined USE_AGNOCAST_ENABLED
  * fix for copilot review
  * style(pre-commit): autofix
  ---------
  Co-authored-by: atsushi421 <atsushi.yano.2@tier4.jp>
  Co-authored-by: atsushi yano <55824710+atsushi421@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat: add autoware_agnocast_wrapper (moved from autoware_universe) (`#905 <https://github.com/mitsudome-r/autoware_core/issues/905>`_)
  * feat: add autoware_agnocast_wrapper (moved from autoware_universe)
  * fix(autoware_agnocast_wrapper): fix minor errors found by Copilot review
  Incorporate fixes from `autowarefoundation/autoware_universe#12283 <https://github.com/autowarefoundation/autoware_universe/issues/12283>`_:
  - Add `override` to AgnocastPublisher::publish methods
  - Fix null dereference in message_ptr::operator bool() and get()
  - Align ENABLE_AGNOCAST check in CMakeLists.txt to use STREQUAL "1"
  - Add missing <type_traits> include in non-Agnocast branch
  - Fix README.md documentation errors (macro names and CMake example)
  * fix(autoware_agnocast_wrapper): fix minor errors found by Copilot review
  * ci: exclude header-only packages from clang-tidy target files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Koichi Imai, Taeseung Sohn, atsushi yano, github-actions
