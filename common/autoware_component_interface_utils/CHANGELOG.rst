^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_component_interface_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.0 (2026-09-28)
-------------------
* chore: align package versions to 1.9.0 and reset changelogs
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(component_interface_utils): let non-rclcpp node types supply the polling and response types (`#1362 <https://github.com/autowarefoundation/autoware_core/issues/1362>`_)
* feat(autoware_component_interface_utils): impl-layer interface registration and node manifest serialization (`#1332 <https://github.com/autowarefoundation/autoware_core/issues/1332>`_)
  * feat(autoware_component_interface_utils): add the interface record registry
  * test(autoware_component_interface_utils): strengthen registration test assertions
  Close four test-strength gaps found in review of the interface record
  registry commit, without touching any production code:
  - Assert minor/patch alongside major so the version-copy check in
  make_record is no longer vacuous against two specs both at 0.x.y.
  - Assert make_record's forwarded/derived fields (interface_name,
  resolved_name, type_name, kind, role, reliability, durability,
  depth), previously verified only through has_version.
  - Register two records with distinct names in
  registry_accumulates_and_returns_records and assert both the count
  and the registration order, since the prior version registered only
  one record despite the test's name.
  - Add a static_assert that HasDomainVersion is false for
  map::PointCloudMap, whose domain declares both the DEFINE_DOMAIN
  template overload and a deleted non-template overload, exercising
  the non-template-beats-template tiebreak that the existing
  single-deleted-candidate case does not cover.
  * test(autoware_component_interface_utils): remove vacuous make_record assertions
  The prior round's added assertions on the version/kind/role/interface_name
  fields happened to equal InterfaceRecord's own default initializers,
  so an omission bug in make_record could still pass:
  - major/patch on the KinematicState-derived record were both 0,
  matching the domain's 0.1.0 version and the struct's {0, 0, 0}
  defaults.
  - kind/role were asserted against Kind::Topic/Role::Provide, which
  are exactly the defaults.
  - interface_name and resolved_name were asserted from the same
  caller-supplied name, so a bug that copied one field into the
  other would go unnoticed.
  Add a local spec (test_specs::Versioned) with a non-deleted domain
  version of {2, 3, 4}, and exercise make_record with Kind::Service,
  Role::Require, and a resolved_name that differs from the spec name
  (simulating a remap). None of the resulting assertions can coincide
  with a default initializer or with each other. Verified by
  temporarily deleting/aliasing each corresponding line in
  make_record and confirming the matching assertion fails.
  * feat(autoware_component_interface_utils): register every interface at the create/init layer
  Wire InterfaceRecord registration into every publisher, subscription,
  service server, and service client creation path so a node's manifest
  is complete by the end of construction, with no bypass:
  - create_publisher_impl and create_subscription_impl now take
  NodeInterface::SharedPtr (instead of a raw node pointer) and
  register a record built from the actually-applied topic name and
  QoS after the rclcpp object is created. The subscription impl holds
  the created subscription in a local variable across the callback vs.
  polling branches so registration happens exactly once regardless of
  which form was used.
  - Client and Service constructors register a record after the
  client/service handle is created, in both the Iron+ and pre-Iron
  #if branches, using the actual resolved service name and the shared
  services QoS profile.
  - rclcpp.hpp passes the NodeInterface down to the topic impls instead
  of the bare node pointer, and its four create\_* doc comments are
  reworded to describe current behavior instead of stale phasing
  language.
  Adds two tests to test_registration.cpp: one confirming a remapped
  publisher registers with the resolved (not spec-declared) name, and
  one exercising every entry path (polling subscription, service
  server, service client, and the legacy init_pub) to confirm each
  leaves exactly one record.
  * test(autoware_component_interface_utils): pin the callback subscription branch and de-vacuous the manifest assertions
  Fix round 1 on the Task 6 registration tests, per review:
  - every_entry_path_registers previously only exercised the polling
  (nullptr callback) branch of create_subscription_impl's `if
  constexpr` split, so the callback-form branch -- the dominant form
  in universe -- was never type-checked by this package's test build.
  Add a callback-form subscription with a concrete signature (a
  generic lambda does not compile against rclcpp::function_traits) and
  bump the expected manifest size from 4 to 5, keeping it adjacent to
  the polling subscription in the asserted order.
  - Add an interface_name assertion at every index so a record swapped
  with the wrong spec, or a missing registration masked by a size
  mismatch elsewhere, would be caught. Two of those indices (the
  service server and init_pub) were previously asserted only against
  Role::Provide, which is InterfaceRecord's own default -- the same
  vacuity class Task 5 had to clear twice.
  - Add a depth assertion to create_publisher_registers_with_remap_resolved_name:
  OperationModeState declares depth 1, distinct from both
  rmw_qos_profile_default's 10 and InterfaceRecord's own default of 0,
  so it is non-vacuous evidence that the record carries the actually
  applied QoS rather than the spec-declared one.
  No non-test file changed; no existing assertion weakened or removed.
  * feat(autoware_component_interface_utils): serialize node manifests to the admission document schema
  Add to_manifest_json(), mapping InterfaceRecord entries accumulated by
  the registry into the admission schema v2 manifest document: a
  provided[]/required[] pair keyed by role, with major/minor/patch
  (Provide) or accept_major_min/accept_major_max/min_minor (Require)
  present iff the record is versioned, and reliable/best_effort +
  volatile/transient_local QoS policy strings. Any other QoS enum value
  -- including InterfaceRecord's own SYSTEM_DEFAULT default -- throws
  std::invalid_argument rather than emitting a placeholder.
  Keep this mapping in a new manifest_json.hpp rather than on rclcpp.hpp
  directly: a NodeAdaptor::manifest_json() member would force every
  consumer of rclcpp.hpp to pull in nlohmann::json. Instead
  to_manifest_json(const NodeAdaptor &, node_name) lives here as a free
  function, so rclcpp.hpp itself stays free of the dependency and only
  callers that include this header pay for it.
  Promote autoware_component_interface_specs from test_depend to depend
  now that it is exercised outside tests, add the nlohmann-json-dev
  dependency, and add the rmw dependency the package was already using
  via <rmw/types.h> without declaring. The new test needs no ROS graph,
  so it gets its own plain (non-isolated) gtest target.
  * test(autoware_component_interface_utils): cover the untested manifest schema branches
  Add cases for the two Provide/Require x versioned/unversioned
  combinations the initial suite skipped: a versioned Require record
  (asserting the accept range, not a copied major/minor/patch triple)
  and an unversioned Provide record (asserting all version keys are
  absent). Also add a durability-only throw case: to_qos_json checks
  reliability before durability, so the existing throw test could never
  reach the durability branch.
  Export nlohmann_json from this package's CMake config. The automatic
  find_package() ament_auto_find_build_dependencies() runs never
  resolves it (the package.xml dependency string and the CMake package
  name differ), so it never lands in the exported dependency list on
  its own; without this, downstream consumers of the newly public
  manifest_json.hpp only compiled by relying on nlohmann's headers
  living on a default system include path.
  * feat(autoware_component_interface_utils): deprecate the init\_* entry points
  Mark every init_pub/init_sub/init_cli/init_srv overload with
  [[deprecated]], each pointing callers at the corresponding create\_*
  that also registers the interface in the node's manifest. The relay
  helpers were rerouted onto the create\_* impls in the previous commit,
  so they remain usable without triggering these warnings. The one
  intentional legacy call kept for coverage in test_registration.cpp is
  wrapped in a narrow -Wdeprecated-declarations pragma so the package
  itself keeps building warning-free.
  Some out-of-tree consumers still call the deprecated overloads and
  build with warnings promoted to errors, so this change is source
  incompatible for them until they migrate to create\_*. It is kept as
  a separate, revertable commit for that reason.
  * test(autoware_component_interface_utils): cover relay_message and relay_service registration
  Neither relay helper was instantiated by any existing test, so a
  regression in their manifest registration order or per-side spec
  deduction would go unnoticed by the test suite. Add specs and cases
  that instantiate both relays with distinct specs per side and assert
  the exact manifest entries produced.
  * feat(autoware_component_interface_utils): add manifest drift test helper
  Add expect_manifest_matches(), a small gtest helper that parses a
  package's committed interface manifest fragment as JSON and compares
  it against a live NodeAdaptor's actual manifest, so a package can pin
  its fragment against reality in an ordinary unit test and catch drift
  before a deploy-time admission gate ever sees it.
  Cover it with test_manifest_drift: a node/adaptor fixture that
  registers one publisher, checked against a matching fragment (passes)
  and a deliberately divergent one (fails through EXPECT_NONFATAL_FAILURE,
  since the helper reports mismatches via EXPECT_EQ rather than
  throwing). Fragment files are test data only, wired in through
  compile definitions the same way the specs package already does for
  its committed manifest.
  Rewrite the README to document registration, QoS overrides, the
  fragment convention, and a full init\_* to create\_* migration table,
  and turn the relay_service group/deadlock note from a trailing
  comment into explicit prose.
  * docs(autoware_component_interface_utils): give the exact CMake install rule for manifest fragments
  The fragment-discovery section named the fixed relative path a
  manifest fragment must land at (share/<package_name>/interface_manifest_fragment.json)
  but did not show how to get it there. A subdirectory-copying install
  rule such as install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
  lands the file one level deeper instead, at
  share/<package_name>/config/interface_manifest_fragment.json, where
  the deploy-time gate's fragment discovery does not look.
  Add the exact, working install(FILES ...) rule that lands the fragment
  at the correct depth, and call out the subdirectory-copying rule as
  the trap to avoid.
  * fix(autoware_component_interface_utils): drop [[deprecated]] from init\_*
  Several autoware_universe packages in control, localization, system,
  and one rviz plugin still call init_pub/init_sub/init_cli/init_srv
  directly. This package builds with -Werror=deprecated-declarations,
  so marking those entry points [[deprecated]] turns every one of those
  call sites into a hard build failure for any consumer that builds the
  same way, well before those callers can migrate.
  Remove the [[deprecated(...)]] attributes from all seven init\_*
  overloads while keeping the create\_* migration guidance in their doc
  comments and in the README. Reword the two relay_message/relay_service
  comments and the README prose and migration table so they describe
  init\_* as the legacy form with create\_* preferred, instead of
  claiming the compiler warns on it. Drop the now-dead
  -Wdeprecated-declarations suppression pragma around the init_pub call
  in test_registration.cpp, since there is nothing left to suppress.
  The attribute comes back once the in-tree callers have migrated to
  create\_*.
  ---------
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
* fix(autoware_component_interface_utils): declare the dependencies it includes (`#1360 <https://github.com/autowarefoundation/autoware_core/issues/1360>`_)
  The package includes headers from packages it never declares. It builds today
  only because another declared dependency re-exports them, so a change in an
  unrelated repository can break it without anything here changing.
* refactor(autoware_component_interface_utils): template the adaptor and wrappers on the node type (`#1319 <https://github.com/autowarefoundation/autoware_core/issues/1319>`_)
  * refactor(autoware_component_interface_utils): template the adaptor and wrappers on the node type
  * fix(autoware_component_interface_utils): template the member-function create\_* overloads on the node type
  The three member-function overloads `#1321 <https://github.com/autowarefoundation/autoware_core/issues/1321>`_ added declare Subscription<SpecT> /
  Service<SpecT>, which default to rclcpp::Node, while delegating to overloads that
  return Subscription<SpecT, NodeT> / Service<SpecT, NodeT>. For any node type other
  than rclcpp::Node the body then fails to convert. Nothing instantiates them with a
  non-default node type yet, so the mismatch is silent until one does.
  * deduce the node pointer type separately in NodeInterface
  * stop Subscription from naming the handle's SharedPtr alias
  * fix tests
  ---------
* feat(autoware_component_interface_utils): complete create\_* and move the package off init\_* (`#1321 <https://github.com/autowarefoundation/autoware_core/issues/1321>`_)
  * feat(autoware_component_interface_utils): add member-function overloads to create\_*
  The out-parameter init_sub/init_srv forms accept a member function plus its
  instance, deducing the spec from the wrapper's smart pointer. The returning
  create_subscription/create_service forms had no counterpart, so a caller
  moving off init\_* had to spell out a std::bind or a forwarding lambda and
  repeat the message type at every call site.
  Add the three missing overloads, matching the shapes init\_* already accepts:
  a subscription callback taking the message pointer, one taking the message
  by reference, and a service callback taking request and response. The
  callback type aliases are now keyed on the spec, with the SharedPtrT-keyed
  aliases that the out-parameter forms use defined in terms of them, so the
  two spellings cannot drift into different shapes.
  The comments on the returning forms described them by an internal milestone
  name; reword them to say what they do instead.
  Tests drive a real message through both subscription overloads and a real
  request through the service overload, so the binding is asserted rather than
  just the endpoint's existence.
  * refactor(autoware_component_interface_utils): build the in-library helpers on create\_*
  relay_message, relay_service and the three member-function init_sub/init_srv
  overloads were implemented on top of the out-parameter init\_* forms. Point
  them at the creation functions instead, so nothing inside the package is
  built on the out-parameter API any more.
  The init\_* forms and the relay helpers are const while the returning create\_*
  members are not, so the delegation targets the free create\_*_impl functions
  that both families already share. No public signature changes and no
  observable behavior changes: init_X(x) and the impl call it forwarded to are
  the same call with the same arguments, and the relay helpers keep passing the
  callback group to the service only, not to the client.
  Add a runtime test for relay_message using the one core spec pair that shares
  a message type. No core spec pair shares a service type, so relay_service
  remains covered by compilation only.
  ---------
* docs(autoware_component_interface_utils): add README lost in the move to core (`#1296 <https://github.com/autowarefoundation/autoware_core/issues/1296>`_)
  common/autoware_component_interface_utils/README.md existed in
  autoware_universe but was not among the files added when the package was
  moved to autoware_core in `#1262 <https://github.com/autowarefoundation/autoware_core/issues/1262>`_, so the package lost its docs page.
  Restore it and update the content for the moved implementation:
  - Drop the "Logging for service and client" section. The core version has
  no /service_log tracer: the autoware_universe wrappers published a
  tier4_system_msgs/ServiceLog message unconditionally, and that is gone.
  The section was also already stale in autoware_universe, where it
  claimed RCLCPP_INFO while NodeInterface::log used RCLCPP_DEBUG_STREAM.
  - Document what the core version offers in its place, without
  overstating it: an opt-in, node-scoped switch
  (component_interface.service_introspection) that turns on standard
  ROS 2 service introspection for every wrapper the adaptor creates.
  Note the off default, the resulting /_service_event topic, the ROS 2
  Iron (rclcpp 21) requirement, and that introspection is not a drop-in
  replacement for /service_log.
  - Document the returning create_publisher / create_subscription /
  create_service / create_client<Spec>() forms of NodeAdaptor.
  - Note that an unready service and a timeout are still surfaced as
  Client::call exceptions.
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
* feat(autoware_component_interface_utils): move to core without service logging (`#1262 <https://github.com/autowarefoundation/autoware_core/issues/1262>`_)
  * feat(autoware_component_interface_utils): scaffold core package (no service logging)
  * feat(autoware_component_interface_utils): node interface without service_log
  * feat(autoware_component_interface_utils): service wrappers without service_log, with introspection
  Drop the /service_log ServiceLog tracer from Client/Service and enable ROS 2
  service introspection (configure_introspection) instead, gated by
  NodeInterface::introspection_state. Replace the deprecated
  rmw_qos_profile_services_default overload of create_client/create_service with
  the behavior-preserving rclcpp::ServicesQoS().
  * feat(autoware_component_interface_utils): NodeAdaptor create\_*<Spec>() and introspection hook
  * test(autoware_component_interface_utils): cover introspection branches and stabilize graph assertions
  Add coverage for the NodeInterface "metadata" and already-declared-parameter
  branches and for the Client introspection hook (symmetric with the Service one),
  and poll for graph visibility before asserting publisher/service presence so the
  tests do not flake on asynchronous discovery.
  * fix(autoware_component_interface_utils): gate service introspection behind rcl header availability
  ROS 2 service introspection (rcl/service_introspection.h and
  Client/Service::configure_introspection) is only available from Iron onward, so
  the unconditional include broke the ROS 2 Humble build. Guard the feature on
  __has_include(<rcl/service_introspection.h>) so the package builds on both Humble
  and Jazzy; introspection is simply unavailable on Humble. The introspection-
  specific tests are gated on the same macro and run on Jazzy.
  * fix(autoware_component_interface_utils): gate the QoS-service overload for ROS 2 Humble
  ROS 2 Humble create_client/create_service accept only the rmw_qos_profile_t
  overload; the rclcpp::QoS overload that rclcpp::ServicesQoS() relies on is Iron+
  (like service introspection). Gate both on RCLCPP_VERSION_GTE(21, 0, 0): Iron+
  uses rclcpp::ServicesQoS() and configures introspection, while Humble falls back
  to the still-non-deprecated rmw_qos_profile_services_default with introspection
  unavailable. Supersedes the header-only gate from the previous commit.
  * test: [TEMP experiment] point packages_above universe at youtalk delete branch
  Temporary: verifies the -above job goes green when the universe copy of
  autoware_component_interface_utils is deleted (youtalk/autoware_universe@
  refactor/finish-service-log-removal). To be reverted before upstream submission.
  * test: [TEMP experiment] also point packages_above autoware_launch at youtalk delete branch
  * fix(autoware_component_interface_utils): restore Client::async_send_request callback overload
  Universe consumers (the rviz panels, command_mode_switcher,
  default_adapi_universe, operation_mode_transition_manager, predicted_path_checker)
  call Client<Spec>::async_send_request(request, callback) with a response callback,
  so dropping this overload broke the move transparency. Restore it, ServiceLog-
  stripped. The user callback is wrapped in a concrete-signature lambda before it is
  handed to rclcpp::Client::async_send_request, which otherwise rejects a generic
  (auto-parameter) callback via its same_arguments trait.
  * chore(autoware_component_interface_utils): add original universe maintainers
  Carry over the original Universe-side maintainers (Takagi, Isamu and
  Yukihiro Saito) into package.xml alongside the current maintainer when
  moving the package into core, as requested in PR review.
  * Revert "test: [TEMP experiment] point packages_above universe at youtalk delete branch"
  This reverts commit 3ceb844b12a363fb4b318d854261b4a7d8aa35e8.
  * Revert "test: [TEMP experiment] also point packages_above autoware_launch at youtalk delete branch"
  This reverts commit af2ed95ddc3cbd586a10f3351d4368fe7e26d1f1.
  ---------
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
* Contributors: Koichi Imai, Mete Fatih Cırıt, Yutaka Kondo, github-actions
