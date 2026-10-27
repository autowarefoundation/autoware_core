^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_component_interface_specs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* feat(component_interface_specs): use template type in get_qos function (`#364 <https://github.com/autowarefoundation/autoware_core/issues/364>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* docs(autoware_component_interface_specs): fix `README.md` (`#363 <https://github.com/autowarefoundation/autoware_core/issues/363>`_)
* Contributors: Takagi, Isamu, Yutaka Kondo

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_component_interface_specs): add RIHS01 type-hash lockfile with opt-in freshness gate (`#1265 <https://github.com/autowarefoundation/autoware_core/issues/1265>`_)
  * feat(autoware_component_interface_specs): add RIHS01 type-hash generator
  * feat(autoware_component_interface_specs): commit type-hash lockfile with opt-in freshness gate
  * ci(autoware_component_interface_specs): enable type-hash freshness gate on Jazzy PRs
  * chore(autoware_component_interface_specs): use "build farm" and drop local cspell words
  Spell "buildfarm" as the two-word "build farm" everywhere it appears --
  the README and the type-hash test's comment -- so no dictionary entry is
  needed for it at all, and restore the root .cspell.json to its state on
  main by dropping both "buildfarm" and "RIHS".
  "RIHS" is a real domain term, so it belongs in the shared dictionary
  rather than in a per-repo override; it is being added there in
  `autowarefoundation/autoware-spell-check-dict#140 <https://github.com/autowarefoundation/autoware-spell-check-dict/issues/140>`_. The spell-check
  workflows pull that dictionary from the repository's main branch, so
  spell-check-differential stays red here until `#140 <https://github.com/autowarefoundation/autoware_core/issues/140>`_ merges.
  * fix(autoware_component_interface_specs): cover the sensing domain in the type-hash generator
  generate_type_hashes.cpp's own comment states its domain list mirrors
  generate_interface_manifest.cpp, but the sensing domain header was
  missing from both the #include list and the per-domain collect<>
  calls. generate_interface_manifest.cpp already includes sensing.hpp,
  so the manifest and the lockfile silently covered different type
  sets: the lockfile never hashed sensing's VehicleVelocityConverterTwist
  (geometry_msgs/msg/TwistWithCovarianceStamped).
  test_type_hashes.cpp's covers_every_manifest_type test exists to catch
  exactly this kind of divergence; it failed once the lockfile was
  regenerated for the domains registered since this generator was
  written. Add the missing include and collect<cis::sensing::Specs>
  call so both generators walk the same eight domains.
  * chore(autoware_component_interface_specs): refresh the type-hash lockfile
  Regenerate interface_type_hashes.jazzy.lock with generate_type_hashes
  now that per-domain versioning has landed for all eight domains.
  Twelve hash lines are added for types newly registered by those
  domains: MrmState, the two point-cloud-map services
  (GetDifferentialPointCloudMap, GetPartialPointCloudMap),
  TrackedObjects, TrafficLightGroupArray, five vehicle command/report
  types (ControlModeReport, GearCommand, HazardLightsCommand,
  TurnIndicatorsCommand, VelocityReport), ControlModeCommand, and
  sensing's TwistWithCovarianceStamped.
  One line is removed: sensor_msgs/msg/PointCloud2. Its PointCloudMap
  spec is deliberately excluded from the versioned Specs tuple in
  map.hpp because it carries a raw point cloud payload rather than a
  bounded interface message, so it was never part of the registered
  surface the generator walks.
  No previously committed hash changed.
  ---------
* fix(autoware_component_interface_specs): declare the dependencies it includes (`#1359 <https://github.com/autowarefoundation/autoware_core/issues/1359>`_)
  The package includes headers from packages it never declares. It builds today
  only because another declared dependency re-exports them, so a change in an
  unrelated repository can break it without anything here changing.
* feat(autoware_component_interface_specs): register and version the control boundary specs (`#1300 <https://github.com/autowarefoundation/autoware_core/issues/1300>`_)
* feat(autoware_component_interface_specs): register and version the sensing boundary specs (`#1303 <https://github.com/autowarefoundation/autoware_core/issues/1303>`_)
  * test(autoware_component_interface_specs): add a shared spec test helper header
  Add test/spec_test_utils.hpp, a header-only helper used by the
  per-domain spec tests.
  It provides has_type<T, Tuple> for asserting membership in a domain's
  Specs tuple, and expect_topic_qos<Spec>() which pins a topic spec's
  declared name/depth/reliability/durability and the rclcpp::QoS that
  get_qos<Spec>() derives from them. Every expected value is passed in
  by the caller as a literal, so the assertions never compare the spec
  against its own fields.
  Test-only and not installed: no header, node, launch, or
  on-the-wire QoS change.
  * test(autoware_component_interface_specs): share domain-version probe
  Move the has_domain_version SFINAE probe out of the map domain's test
  file and into spec_test_utils.hpp, the header already shared by all
  eight domain spec test files.
  The probe is a void_t expression-SFINAE check over the ADL-resolved
  resolve_domain_version(const Spec &). Downstream consumers rely on
  exactly this shape to decide whether a spec participates in versioned
  interface registration, so any domain that deliberately excludes a
  spec from versioning needs to type-enforce that the spec's version
  resolution is ill-formed, not merely absent from its Specs tuple.
  Keeping the only copy in one domain's test file made that domain the
  structural odd one out among its siblings; hosting it in the shared
  helper lets every domain use the same definition instead of
  reintroducing a private copy.
  has_type and expect_topic_qos are unchanged.
  * feat(autoware_component_interface_specs): register and version the sensing boundary (v0.1.0)
  Seed the sensing domain header with the vehicle-velocity-converter
  twist boundary and its per-domain version, mirroring the versioning
  structure used by the other domains.
  - Add sensing.hpp: version{0, 1, 0}, VehicleVelocityConverterTwist on
  geometry_msgs/TwistWithCovarianceStamped
  (/sensing/vehicle_velocity_converter/twist_with_covariance, depth 1,
  RELIABLE/VOLATILE), Specs tuple, and the resolve_domain_version ADL hook.
  - Add test_sensing.cpp: version, InterfaceSpec/registration (tuple_size==1),
  and the name/QoS block via get_qos<>.
  - Wire test_sensing.cpp into the gtest target.
  - Declare the geometry_msgs dependency in package.xml (sorted).
  - Extend the manifest generator to walk the sensing domain.
  Additive, observe-only, header-only data plus tests: no node, launch, or
  on-the-wire QoS change. The ObstacleGrid boundary is deferred to the
  sensing-safety track.
  * fix(autoware_component_interface_specs): correct sensing depth to 10 for vehicle velocity converter twist
  VehicleVelocityConverterTwist declared a QoS history depth of 1, but
  the real publisher, autoware_vehicle_velocity_converter, creates its
  publisher with rclcpp::QoS{10}, and the primary consumer,
  autoware_gyro_odometer, subscribes with depth 10 as well. The
  declared depth of 1 therefore did not describe the interface as
  built.
  Update the spec, its test, and the regenerated interface_manifest.json
  to depth 10. Reliability (RELIABLE) and durability (VOLATILE) are
  unchanged. Depth is not a ROS 2 QoS compatibility axis, so this
  changes how many samples are buffered, not whether endpoints can
  connect.
  * Apply suggestions from code review
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  * test(autoware_component_interface_specs): drop dangling version include
  test_sensing.cpp included version.hpp for a domain-version check that
  was removed earlier. The include has had no user since then; remove
  it.
  ---------
* feat(autoware_component_interface_specs): register and version the system boundary specs (`#1305 <https://github.com/autowarefoundation/autoware_core/issues/1305>`_)
  * test(autoware_component_interface_specs): add a shared spec test helper header
  Add test/spec_test_utils.hpp, a header-only helper used by the
  per-domain spec tests.
  It provides has_type<T, Tuple> for asserting membership in a domain's
  Specs tuple, and expect_topic_qos<Spec>() which pins a topic spec's
  declared name/depth/reliability/durability and the rclcpp::QoS that
  get_qos<Spec>() derives from them. Every expected value is passed in
  by the caller as a literal, so the assertions never compare the spec
  against its own fields.
  Test-only and not installed: no header, node, launch, or
  on-the-wire QoS change.
  * feat(autoware_component_interface_specs): register and version the system boundary (v0.1.0)
  Promote MrmState from the universe specs package into core and add
  the new HazardStatus emergency boundary, completing registration of
  the system domain's five inter-component boundaries: the existing
  OperationModeState topic and ChangeOperationMode/ChangeAutowareControl
  services, plus MrmState and HazardStatus.
  - MrmState (/system/fail_safe/mrm_state, RELIABLE/VOLATILE) is
  promoted universe -> core byte-for-byte identical to the universe
  struct; core becomes its single definition and version authority.
  - HazardStatus (/system/emergency/hazard_status,
  autoware_system_msgs/HazardStatusStamped, RELIABLE/VOLATILE) is new.
  - Register all five specs via
  AUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN(0, 1, 0,
  OperationModeState, ChangeOperationMode, ChangeAutowareControl,
  MrmState, HazardStatus), which expands to both the version{0,1,0}
  constant and the Specs tuple; version stays 0.1.0.
  - Add the hazard_status_stamped include and the utils include (for
  the get_qos test helper); domain header stays C++17-clean.
  Additive and observe-only: header-only data plus tests, no node,
  launch, or on-the-wire QoS change. Existing universe consumers of
  autoware::component_interface_specs_universe::system::MrmState keep
  compiling via a separate re-export shim in that package.
  * feat(autoware_component_interface_specs): drop HazardStatus from the system domain
  Remove the HazardStatus spec (/system/emergency/hazard_status) that
  was added earlier on this branch. A re-verification pass (two runtime
  consumer-graph checks against current images, plus static sweeps)
  found zero consumers of any kind in OSS; the producer
  hazard_status_converter is the only endpoint. The interface is
  consumed only by vendor API adapter layers, so its contract belongs
  in the vendor-owned spec partition rather than the
  autowarefoundation-owned set; it is removed here and handed off to
  that track.
  system::Specs shrinks 5 -> 4. Regenerate interface_manifest.json and
  update test_system.cpp.
  * Apply suggestions from code review
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  ---------
* feat(autoware_component_interface_specs): register and version the perception boundary specs (`#1298 <https://github.com/autowarefoundation/autoware_core/issues/1298>`_)
* feat(autoware_component_interface_specs): register and version the vehicle boundary specs (`#1304 <https://github.com/autowarefoundation/autoware_core/issues/1304>`_)
  * test(autoware_component_interface_specs): add a shared spec test helper header
  Add test/spec_test_utils.hpp, a header-only helper used by the
  per-domain spec tests.
  It provides has_type<T, Tuple> for asserting membership in a domain's
  Specs tuple, and expect_topic_qos<Spec>() which pins a topic spec's
  declared name/depth/reliability/durability and the rclcpp::QoS that
  get_qos<Spec>() derives from them. Every expected value is passed in
  by the caller as a literal, so the assertions never compare the spec
  against its own fields.
  Test-only and not installed: no header, node, launch, or
  on-the-wire QoS change.
  * feat(autoware_component_interface_specs): register and version the vehicle boundary (v0.1.0)
  Add VelocityStatus (autoware_vehicle_msgs/VelocityReport,
  /vehicle/status/velocity_status) as a new interface spec, filling the
  largest gap in the vehicle domain since velocity is consumed by every
  downstream domain. Register it alongside the existing SteeringStatus,
  GearStatus, TurnIndicatorStatus, and HazardLightStatus in
  vehicle::Specs, bringing the tuple to 5 entries under the domain's
  existing version{0, 1, 0}.
  Extend test_vehicle.cpp with version, concept_and_registration
  (InterfaceSpec + has_type + tuple_size==5 for all five specs), and
  velocity_status_qos tests, following the same test structure used
  across the other domains. No behavior change: this is additive
  header-only data (a struct + an include + a wider tuple) plus tests.
  * feat(autoware_component_interface_specs): add the control-mode status spec
  Add ControlModeStatus (/vehicle/status/control_mode, depth=1 RELIABLE
  VOLATILE on autoware_vehicle_msgs/ControlModeReport) to the vehicle
  domain and register it in vehicle::Specs (5 -> 6).
  A runtime consumer-graph check found this safety-supervision report
  consumed cross-domain by operation_mode_transition_manager and
  lane_departure_checker (control) and mrm_handler (system); it
  parallels the existing steering/gear status reports and closes a gap
  in the vehicle status contract.
  Regenerate interface_manifest.json and extend test_vehicle.cpp.
  * Apply suggestions from code review
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  ---------
* feat(autoware_component_interface_specs): register and version the map boundary specs (`#1302 <https://github.com/autowarefoundation/autoware_core/issues/1302>`_)
  * test(autoware_component_interface_specs): add a shared spec test helper header
  Add test/spec_test_utils.hpp, a header-only helper used by the
  per-domain spec tests.
  It provides has_type<T, Tuple> for asserting membership in a domain's
  Specs tuple, and expect_topic_qos<Spec>() which pins a topic spec's
  declared name/depth/reliability/durability and the rclcpp::QoS that
  get_qos<Spec>() derives from them. Every expected value is passed in
  by the caller as a literal, so the assertions never compare the spec
  against its own fields.
  Test-only and not installed: no header, node, launch, or
  on-the-wire QoS change.
  * feat(autoware_component_interface_specs): register and version the map boundary (v0.1.0)
  Register the map domain's existing VectorMap and MapProjectorInfo
  specs plus a new GetDifferentialPointCloudMap service spec into a
  versioned Specs tuple (v0.1.0), following the same per-domain
  versioning structure used across the other domains.
  PointCloudMap keeps its struct for existing consumers but is
  deliberately excluded from Specs because it carries a raw point cloud
  payload rather than a bounded interface message; the exclusion is
  marked with the interface-spec-lint suppression comment consumed by
  the autoware_interface_spec_lint package's spec_registered check (a
  separate package that lands earlier in this effort).
  Extends test_map.cpp with a version test, a concept_and_registration
  test (ServiceSpec<GetDifferentialPointCloudMap>, has_type checks for
  the three registered specs, tuple_size == 3, and the negative
  has_type<PointCloudMap, Specs> assertion), and a name check for the
  new service, while keeping the existing per-spec QoS block.
  Core-only: the autoware_universe re-export shim lands in a separate
  PR against that package.
  * fix(autoware_component_interface_specs): make the PointCloudMap version exclusion type-enforced
  PointCloudMap is excluded from the versioned `Specs` tuple and from
  interface_manifest.json because it carries a raw point cloud payload
  rather than a bounded interface message, but the namespace-level
  `resolve_domain_version(const Spec &)` template still matched it, so
  `spec_version<PointCloudMap>()` returned 0.1.0 and a void_t/
  expression-SFINAE detection trait (universe's HasDomainVersion)
  treated it as versioned. A consumer could then register PointCloudMap
  and emit a manifest record for an interface absent from the authority
  manifest -- exactly the mismatch a deploy-time provider-registration
  check is meant to catch and reject.
  Add a deleted exact-match overload
  `resolve_domain_version(const PointCloudMap &) = delete;` in the same
  namespace. The non-template exact match wins overload resolution, so
  version resolution is ill-formed for PointCloudMap: in a SFINAE
  context the detection trait now reports it as unversioned, and a
  direct call is a hard compile error. The exclusion is enforced by the
  type system rather than by tuple omission alone.
  Add a regression test mirroring the downstream detection trait that
  asserts PointCloudMap is unversioned while VectorMap stays versioned.
  * feat(autoware_component_interface_specs): add the partial pointcloud map service spec
  Add GetPartialPointCloudMap (/map/get_partial_pointcloud_map on
  autoware_map_msgs/srv/GetPartialPointCloudMap) to the map domain and
  register it in map::Specs (3 -> 4), as the companion PCD-map delivery
  path to GetDifferentialPointCloudMap.
  A runtime consumer-graph check confirmed the cross-domain consumer:
  pose_initializer (localization) calls it through the embedded
  MapHeightFitter.
  Regenerate interface_manifest.json and extend test_map.cpp.
  * chore(autoware_component_interface_specs): make comments self-contained
  State why PointCloudMap is excluded from the versioned registration
  surface directly in the comments (it carries a raw point cloud
  payload rather than a bounded interface message) instead of citing
  an external design document section.
  Also name the consuming tool in the interface-spec-lint suppression
  marker on PointCloudMap (autoware_interface_spec_lint's
  spec_registered check) so the marker is self-explanatory without
  prior knowledge of that package. The fixed-string prefix and the
  one-line-above placement are unchanged, matching the SUPPRESS_MARKER
  contract in that package's checks.py.
  * Apply suggestions from code review
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  ---------
* feat(autoware_component_interface_specs): register and version the localization boundary specs (`#1301 <https://github.com/autowarefoundation/autoware_core/issues/1301>`_)
  * test(autoware_component_interface_specs): add a shared spec test helper header
  Add test/spec_test_utils.hpp, a header-only helper used by the
  per-domain spec tests.
  It provides has_type<T, Tuple> for asserting membership in a domain's
  Specs tuple, and expect_topic_qos<Spec>() which pins a topic spec's
  declared name/depth/reliability/durability and the rclcpp::QoS that
  get_qos<Spec>() derives from them. Every expected value is passed in
  by the caller as a literal, so the assertions never compare the spec
  against its own fields.
  Test-only and not installed: no header, node, launch, or
  on-the-wire QoS change.
  * feat(autoware_component_interface_specs): register and version the localization boundary (v0.1.0)
  Lock the v0.1.0 registration contract for the localization domain,
  which already has the version{0,1,0} constant, the Specs tuple, and
  the resolve_domain_version ADL hook on
  KinematicState/Acceleration/InitializationState/Initialize wired into
  the header by an earlier commit.
  Extend test_localization.cpp only: add a version test asserting
  major/minor/patch, and a concept_and_registration test asserting
  InterfaceSpec on the three topic specs, ServiceSpec on Initialize,
  tuple membership (has_type) for all four, and tuple_size == 4.
  Pure consolidation, no new specs and no header changes:
  localization.hpp already carries the version/tuple/hook scaffolding.
  Observe-only, header-only data + tests, no behavior change.
  * Update common/autoware_component_interface_specs/test/test_localization.cpp
  ---------
* feat(autoware_component_interface_specs): register and version the planning boundary specs (`#1299 <https://github.com/autowarefoundation/autoware_core/issues/1299>`_)
  * test(autoware_component_interface_specs): add a shared spec test helper header
  Add test/spec_test_utils.hpp, a header-only helper used by the
  per-domain spec tests.
  It provides has_type<T, Tuple> for asserting membership in a domain's
  Specs tuple, and expect_topic_qos<Spec>() which pins a topic spec's
  declared name/depth/reliability/durability and the rclcpp::QoS that
  get_qos<Spec>() derives from them. Every expected value is passed in
  by the caller as a literal, so the assertions never compare the spec
  against its own fields.
  Test-only and not installed: no header, node, launch, or
  on-the-wire QoS change.
  * feat(autoware_component_interface_specs): register and version the planning boundary (v0.1.0)
  Lock the v0.1.0 registration contract for the planning domain by
  extending test_planning.cpp with explicit version and
  concept/registration checks on top of the Specs tuple and ADL version
  hook already wired into planning.hpp.
  Add a planning.version test asserting the 0.1.0 static values, and a
  planning.concept_and_registration test that checks InterfaceSpec on
  Trajectory/LaneletRoute/RouteState, ServiceSpec on
  SetLaneletRoute/SetWaypointRoute/ClearRoute, has_type membership for
  all six specs in planning::Specs, and tuple_size == 6. The existing
  QoS interface test is left untouched.
  No behavior change: header-only data plus tests, no node/launch/QoS-
  on-the-wire change. This is core-only; the autoware_universe
  re-export shim and its alias test land in a separate PR against that
  package.
  * Update common/autoware_component_interface_specs/test/test_planning.cpp
  ---------
* feat(autoware_component_interface_specs): add per-domain versioning foundation (`#1259 <https://github.com/autowarefoundation/autoware_core/issues/1259>`_)
  * feat(autoware_component_interface_specs): add per-domain versioning foundation
  Add the versioning mechanism for the component interface specs additively and
  behavior-neutrally. This lands the value types, the compile-time validation
  concepts, the per-domain Specs registry, and a committed machine-readable
  manifest, without changing any interface semantics.
  - version.hpp: Version value type with operator==/!=, an accept_major window,
  two is_compatible overloads (MAJOR-only, and a migration range), the owner
  constant, and an ADL spec_version<Spec>() resolver. C++17-clean so existing
  C++17 consumers of the domain headers keep compiling untouched.
  - concepts.hpp: InterfaceSpec / ServiceSpec / AnySpec concepts and
  all_specs_valid<Tuple>(), fully guarded by #if __cplusplus >= 202002L so the
  C++20 concept syntax never leaks into C++17 translation units.
  - The 7 domain headers each gain a static constexpr Version version{0, 1, 0},
  a using Specs = std::tuple<...> registry of the structs that already exist,
  and a resolve_domain_version ADL hook. No new spec structs are introduced.
  - generate_interface_manifest: a build-time C++20 tool that walks the Specs
  tuples and emits interface_manifest.json deterministically. It hand-emits
  JSON (no new runtime dependency) in the layout Prettier produces, so the
  committed manifest is stable across regenerations.
  - interface_manifest.json: the committed manifest, byte-identical to the
  generator output.
  - Tests test_version / test_concepts / test_manifest, built together with the
  existing per-domain tests at C++20; the generator path is passed via the
  GENERATE_TOOL compile definition.
  - README documents the manual regeneration command. The build never writes into
  the source tree; the committed manifest is regenerated by hand.
  * feat(autoware_component_interface_specs): record QoS in the manifest and declare domains with a macro
  Addresses the review on PR `#90 <https://github.com/autowarefoundation/autoware_core/issues/90>`_.
  Services do have a QoS profile. Every create_service / create_client call
  takes rmw_qos_profile_services_default unmodified, so all services really
  do run under identical conditions -- the specs just never said so. Declare
  that one shared profile once as `service_qos` in utils.hpp rather than
  repeating it on each ServiceSpec, expose it as get_service_qos() beside
  the topic-side get_qos<Spec>(), and pin it against the RMW default in
  test_service_qos.cpp so a changed default surfaces as a test failure
  instead of as silent drift between the specs and the wire.
  Emit each interface's QoS into interface_manifest.json. Reliability and
  durability are the two axes ROS 2 checks before a publisher and a
  subscription may talk at all, so a deploy-time admission gate reading the
  manifest needs them next to the type and the version. The generator now
  renders the whole document before opening the output file and rejects a
  policy it cannot name, rather than degrading an entry or truncating the
  committed file.
  Give test_manifest.cpp teeth: it now diffs the generator's output against
  the committed manifest, so a stale interface_manifest.json fails the build.
  It previously only spot-checked a few substrings, which a stale file would
  still have satisfied.
  Collapse the per-domain `version` / `Specs` / `resolve_domain_version`
  triple into AUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN, so a domain
  cannot bump its version but forget to register a spec, or register a spec
  whose version nothing can resolve.
  concepts.hpp expanded to nothing below C++20 with no way for a consumer to
  tell. It now defines AUTOWARE_COMPONENT_INTERFACE_SPECS_HAS_CONCEPTS, and a
  new C++17-pinned gtest target compiles every domain header at the standard
  autoware_package() gives consumers, so a C++20 construct leaking into that
  surface fails here rather than in a downstream package.
  Verified in the Jazzy devel container on a clean rebuild: 75 tests, 0
  failures; the three targets take -std=c++17 (consumer surface), -std=c++20
  (generator) and -std=c++20 (gtest) as intended under CMAKE_CXX_STANDARD 17;
  generator output is byte-identical to the committed manifest and unchanged
  by Prettier; clang-tidy is clean under .clang-tidy-ci (bugprone-*, warnings
  as errors), whose macro checks were confirmed live against version.hpp.
  * fix(autoware_component_interface_specs): gate C++20 test targets on a C++20-capable compiler
  The manifest generator and the concepts test compile real C++20 -- the
  `concept X = requires { ... }` syntax and tuple iteration -- via
  target_compile_features(... cxx_std_20). GCC ships standard C++20 concepts
  only from GCC 10; GCC 8/9 have the older Concepts TS instead. The Humble
  RHEL-8 binary buildfarm job runs GCC 8.5 and, unlike the Jazzy release jobs
  (which set run_package_tests:false), compiles BUILD_TESTING targets, so once
  this package's C++20 targets reach a Humble release they would fail to build
  there.
  Gate the generator and the gtest\_${PROJECT_NAME} target on GNU >= 10 so
  BUILD_TESTING degrades gracefully instead of failing to build on GCC 8.5.
  Keep gtest\_${PROJECT_NAME}_cxx17 unconditional, so the RHEL-8 job still
  compiles every domain header at the C++17 standard its consumers use. The
  C++20 gtest also bundles the plain domain value tests (test_control,
  test_version, ...); they are skipped wholesale on old GCC rather than split
  into a second C++17 target -- they assert compile-time constants identical on
  every compiler and already run on all other jobs.
  Bundle three dependency-hygiene fixes flagged in the same review:
  - Drop the vestigial `rcl` dependency; nothing includes <rcl/...>.
  - Declare `geometry_msgs`; localization.hpp uses AccelWithCovarianceStamped
  directly and was only resolving it transitively via nav_msgs.
  - The domain headers use only the RMW QoS enums, not rclcpp::QoS, so they now
  include <rmw/qos_profiles.h> instead of <rclcpp/qos.hpp>. rclcpp stays a
  package dependency for utils.hpp's get_qos<Spec>() / get_service_qos().
  The RHEL-8 GCC 8.5 build cannot be reproduced locally (the devel container is
  GCC 13), so this is not proven fixed from a local run; the Humble
  build-and-test-differential gate uses GCC 11, which supports C++20 and so
  still compiles the C++20 targets -- it is not evidence for the GCC 8.5 path.
  The real proof is the buildfarm binary job after release. Verified locally in
  the Jazzy devel container: the C++20-capable build is unchanged (75 tests, 0
  failures, all three targets), and forcing the guard false configures and
  builds only gtest\_${PROJECT_NAME}_cxx17 (generator and gtest\_${PROJECT_NAME}
  absent), which passes.
  * chore(autoware_component_interface_specs): reword CMake comments to pass cspell
  The build comments used "buildfarm" and "FTBFS", which cspell does not
  know; reword to "build farm" and "breaking the build" so
  spell-check-differential passes. Comment-only, no build change.
  ---------
* Contributors: Mete Fatih Cırıt, Yutaka Kondo, github-actions

1.9.0 (2026-06-24)
------------------

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_core): add USE_SCOPED_HEADER_INSTALL_DIR to common and testing packages (`#967 <https://github.com/mitsudome-r/autoware_core/issues/967>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* fix(component_interface_specs): change ControlCommand QoS to volatile (`#833 <https://github.com/mitsudome-r/autoware_core/issues/833>`_)
  * fix(component_interface_specs): change ControlCommand QoS durability to volatile
  Change the QoS durability of ControlCommand from TRANSIENT_LOCAL to
  VOLATILE.
  * test(component_interface_spec): update ControlCommand QoS durability to volatile on test
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* Contributors: Takahisa Ishikawa, Vishal Chauhan, github-actions

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------

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
* feat(autoware_component_interface_specs): update planning and system interface (`#544 <https://github.com/autowarefoundation/autoware_core/issues/544>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat!: replace autoware_internal_localization_msgs with autoware_localization_msgs for InitializeLocalization service (`#542 <https://github.com/autowarefoundation/autoware_core/issues/542>`_)
  * feat!: replace autoware_internal_localization_msgs with autoware_localization_msgs for InitializeLocalization service
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_component_interface_specs): add InitializationSpecs (`#508 <https://github.com/autowarefoundation/autoware_core/issues/508>`_)
  * feat(autoware_component_interface_specs): add InitializationSpecs
  * feat: add test
  * style(pre-commit): autofix
  * fix: build error
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* feat(component_interface_specs): use template type in get_qos function (`#364 <https://github.com/autowarefoundation/autoware_core/issues/364>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* docs(autoware_component_interface_specs): fix `README.md` (`#363 <https://github.com/autowarefoundation/autoware_core/issues/363>`_)
* Contributors: Ryohsuke Mitsudome, Takagi, Isamu, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* feat: add autoware_core_component_interface_specs package (`#124 <https://github.com/autowarefoundation/autoware.core/issues/124>`_)
* Contributors: Ryohsuke Mitsudome, mitsudome-r
