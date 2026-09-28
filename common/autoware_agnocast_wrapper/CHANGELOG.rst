^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_agnocast_wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_agnocast_wrapper): add the autoware_node launch action (`#1457 <https://github.com/autowarefoundation/autoware_core/issues/1457>`_)
  * feat(autoware_agnocast_wrapper): add the autoware_node launch action
  * fix(autoware_agnocast_wrapper): follow use_agnocast and keep other nodes off Agnocast
  * fix(autoware_agnocast_wrapper): accept plain strings in AutowareNode
  * fix(autoware_agnocast_wrapper): name the node in the mode and heaphook errors
  * refactor(autoware_agnocast_wrapper): drop a duplicate lookup and parse check
  * docs(autoware_agnocast_wrapper): document the autoware_node_plugins resource
  * test(autoware_agnocast_wrapper): run the autoware_node test without the install space
  ---------
* refactor(autoware_agnocast_wrapper): drop the discovery agent spawn from the launch wrapper (`#1451 <https://github.com/autowarefoundation/autoware_core/issues/1451>`_)
  * feat(autoware_agnocast_wrapper): drop the discovery agent spawn from the launch wrapper
  * style(pre-commit): autofix
  * docs(autoware_agnocast_wrapper): note that Agnocast starts the discovery agent
  * docs(autoware_agnocast_wrapper): note how the discovery agent auto-start can fail
  The auto-start only warns and continues when the agent is unavailable, so
  name the two ways it can be absent as a starting point for the reader.
  * docs(autoware_agnocast_wrapper): drop the discovery agent troubleshooting note
  The paragraph described agnocastlib internals inaccurately: the commands
  fall back to the local ioctl view rather than coming back empty, the
  AGNOCAST_NO_DISCOVERY_AGENT path short-circuits before any logging, and
  the env var predicate accepts only 1/true/yes rather than any value.
  Rather than track internals that already differ between agnocastlib
  releases, drop the paragraph. The sentence above it already carries the
  point: the wrapper no longer launches the agent because Agnocast does.
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Tran Huu Nhat Huy <29034232+TranHuuNhatHuy@users.noreply.github.com>
* feat(autoware_agnocast_wrapper): support polling_policy::All (`#1443 <https://github.com/autowarefoundation/autoware_core/issues/1443>`_)
  * feat(autoware_agnocast_wrapper): support polling_policy::All
  * test(autoware_agnocast_wrapper): poll for the All batch instead of sleeping
  * refactor(autoware_agnocast_wrapper): decide the depth rule at the call site
  * refactor(autoware_agnocast_wrapper): keep the polling helpers in the package detail namespace
  * refactor(autoware_agnocast_wrapper): spell the All vector like the other policies
  * refactor(autoware_agnocast_wrapper): point the depth rejection at polling_policy::All
  * docs(autoware_agnocast_wrapper): scope the cross-backend promise to the policies that keep it
  * docs(autoware_agnocast_wrapper): note what holding an All vector costs
  ---------
  Co-authored-by: atsushi yano <55824710+atsushi421@users.noreply.github.com>
* feat(autoware_agnocast_wrapper): add generic pubsub wrapper (`#1442 <https://github.com/autowarefoundation/autoware_core/issues/1442>`_)
  * feat: add generic pubsub wrapper
  * style(pre-commit): autofix
  * refactor(autoware_agnocast_wrapper): return client responses as std::shared_ptr (`#1419 <https://github.com/autowarefoundation/autoware_core/issues/1419>`_)
  * feat(autoware_agnocast_wrapper): add to_shared_ptr() for client responses
  * refactor(autoware_agnocast_wrapper): move to_shared_ptr() to the message_ptr layer
  * style(pre-commit): autofix
  * fix(autoware_agnocast_wrapper): reject publisher-side handles in to_shared_ptr()
  * refactor(autoware_agnocast_wrapper): return client responses as std::shared_ptr
  * style(pre-commit): autofix
  * docs(autoware_agnocast_wrapper): document the client response in the lifetime rules
  * docs(autoware_agnocast_wrapper): drop the stale client response row from the type table
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: atsushi yano <55824710+atsushi421@users.noreply.github.com>
  * feat(autoware_agnocast_wrapper): add Method 1 macros for generic pub/sub
  AUTOWARE_CREATE_GENERIC_PUBLISHER3/4(_ON_NODE) and
  AUTOWARE_CREATE_GENERIC_SUBSCRIPTION(_ON_NODE) mirror
  AUTOWARE_CREATE_PUBLISHER2/3 and AUTOWARE_CREATE_SUBSCRIPTION: under
  ENABLE_AGNOCAST=1 they forward to the create_generic_publisher()/
  create_generic_subscription() free functions, and under ENABLE_AGNOCAST=0
  they forward directly to rclcpp::Node's own native methods. This closes the
  gap where Method 1 (macro + free function, base class stays rclcpp::Node)
  had no entry point for the generic (type-erased) publisher/subscription,
  even though AUTOWARE_GENERIC_PUBLISHER_PTR/AUTOWARE_GENERIC_SUBSCRIPTION_PTR
  already existed for it.
  Adds a Method 1 round-trip test (generic_pubsub.cpp) and updates the
  create_generic_publisher()/create_generic_subscription() free function doc
  comments to point at the new macros.
  Verified with colcon build + the gtest suite (15/15 passing) under both
  ENABLE_AGNOCAST=0 and ENABLE_AGNOCAST=1.
  Addresses review comment:
  https://github.com/autowarefoundation/autoware_core/pull/1442#discussion_r-koichi98-generic_publisher-110
  * fix(autoware_agnocast_wrapper): reject qos_overriding_options on generic publisher
  rclcpp's create_generic_publisher() only honors event_callbacks,
  use_default_callbacks and callback_group; it silently drops
  qos_overriding_options (per its own doc comment), while Agnocast's
  GenericPublisher applies it. AgnocastGenericPublisher and
  ROS2GenericPublisher forwarded it through regardless, so the same
  PublisherOptions would behave differently depending on which backend
  happened to be selected at runtime.
  Both constructors now reject a non-empty qos_overriding_options with
  std::invalid_argument via a shared check_generic_publisher_options()
  helper, the same "explicitly unsupported" pattern
  polling::check_polling_qos() uses for its own backend divergence.
  Adds RejectsQosOverridingOptions / DefaultOptionsDoNotThrow tests (only
  meaningful under ENABLE_AGNOCAST=1, where these classes exist).
  Verified with colcon build + the gtest suite (15/15, 17/17 with the two
  new Agnocast-only tests) under both ENABLE_AGNOCAST=0 and
  ENABLE_AGNOCAST=1.
  Addresses review comment:
  https://github.com/autowarefoundation/autoware_core/pull/1442#discussion_r-koichi98-generic_publisher-95
  * docs(autoware_agnocast_wrapper): document generic publisher/subscription
  README.md and docs/review_guide.md enumerate the wrapper's member/macro
  surface in tables, but had no rows for the generic (type-erased) publisher/
  subscription added recently. Adds:
  - A "Generic (type-erased) publisher/subscription" row to the Node wrapper's
  supported-API table, and AUTOWARE_GENERIC_PUBLISHER_PTR/
  AUTOWARE_GENERIC_SUBSCRIPTION_PTR rows to the type-spellings table, plus a
  dedicated subsection documenting the generic-only constraints: a single
  callback shape (no message_ptr/zero-copy overload) and the
  qos_overriding_options rejection on the publisher side.
  - A Method 1 usage example for AUTOWARE_CREATE_GENERIC_PUBLISHER3/4 /
  AUTOWARE_CREATE_GENERIC_SUBSCRIPTION in the README.
  - The equivalent macro-expansion table, review checklist items, and type
  rows in docs/review_guide.md for both Method 1 and Method 2.
  Docs only; sanity-built + ran the gtest suite (15/15) to confirm nothing
  else was touched.
  Addresses review comment:
  https://github.com/autowarefoundation/autoware_core/pull/1442#discussion_r-koichi98-autoware_agnocast_wrapper-20
  * refactor(autoware_agnocast_wrapper): dispatch generic pub/sub through visit_node()
  Move the private NodeVariant node\_ and visit_node() to the top of the
  Agnocast-enabled Node class, ahead of every member function that might use
  them. This lets create_generic_publisher()/create_generic_subscription()
  dispatch through visit_node(), the same pattern create_publisher<MessageT>()/
  create_subscription<MessageT>() already use, instead of the ad hoc
  use_agnocast() + get_agnocast_node()/get_rclcpp_node() branch they had
  (which existed only to work around visit_node()'s decltype(auto) return type
  not yet being deduced when an ordinary, non-template member function
  compiled ahead of it).
  Verified this actually resolves the original compile-order problem (not
  just reorders code) by building with ENABLE_AGNOCAST=1 and exercising the
  new dispatch through a Method 2 (agnocast_wrapper::Node) round-trip test
  (NodeMemberRoundTrip in generic_pubsub.cpp).
  No behavior change: get_agnocast_node()/get_rclcpp_node() are unchanged and
  still used by the Executor-facing API.
  Verified with colcon build + the gtest suite (16/16, 18/18 with the new
  Method 2 test) under both ENABLE_AGNOCAST=0 and ENABLE_AGNOCAST=1.
  Addresses review comment:
  https://github.com/autowarefoundation/autoware_core/pull/1442#discussion_r-koichi98-node-241
  * feat(autoware_agnocast_wrapper): add options param to generic subscription depth overload (Agnocast build)
  Node::create_generic_subscription(topic, type, qos_history_depth, callback)
  had no way to pass agnocast::SubscriptionOptions, unlike the QoS-taking
  overload right next to it and unlike create_subscription<MessageT>()'s own
  depth overload. A caller needing e.g. a specific callback_group had to
  hand-build an rclcpp::QoS(rclcpp::KeepLast(n)) and use the QoS overload
  instead, even though a depth + options call already works on both backends.
  Adds a defaulted `options` parameter that forwards to the QoS-taking
  overload, mirroring create_subscription<MessageT>()'s depth overload.
  Adds NodeMemberDepthAndOptionsOverload (Agnocast-enabled build only, since
  this overload doesn't exist in the non-Agnocast build yet — see the
  follow-up commit for that).
  Verified with colcon build + the gtest suite (16/16, 19/19 with the new
  Agnocast-only test) under both ENABLE_AGNOCAST=0 and ENABLE_AGNOCAST=1.
  Addresses review comment:
  https://github.com/autowarefoundation/autoware_core/pull/1442#discussion_r-koichi98-node-336
  * feat(autoware_agnocast_wrapper): add options param to generic subscription depth overload (non-Agnocast build)
  Same gap as the previous commit, on the non-Agnocast build's Node class:
  create_generic_subscription(topic, type, qos_history_depth, callback) had
  no way to pass rclcpp::SubscriptionOptions, unlike the QoS-taking overload
  next to it. Adds a defaulted `options` parameter that forwards to it,
  completing parity with the typed create_subscription<MessageT>() depth
  overload in both builds.
  Un-guards NodeMemberDepthAndOptionsOverload (previously Agnocast-enabled
  build only) now that the overload exists in both, and switches its
  options argument to AUTOWARE_SUBSCRIPTION_OPTIONS so the same test source
  compiles either way.
  Verified with colcon build + the gtest suite under both
  ENABLE_AGNOCAST=0 (17/17) and ENABLE_AGNOCAST=1 (19/19).
  Addresses review comment:
  https://github.com/autowarefoundation/autoware_core/pull/1442#discussion_r-koichi98-node-853
  * docs(autoware_agnocast_wrapper): document @throws for generic pub/sub construction
  Both backends load topic_type's typesupport library at construction and
  throw std::runtime_error on an unknown type or missing typesupport package
  (rclcpp::GenericPublisher/GenericSubscription document this themselves),
  but neither the new GenericPublisher/GenericSubscription class docs nor the
  Node::create_generic_publisher()/create_generic_subscription() member docs
  mentioned it.
  Adds an @throws std::runtime_error doc line to both classes and to all four
  Node member overloads (Agnocast-enabled and non-Agnocast builds).
  Adds UnknownTopicTypeThrows, verifying the documented behavior actually
  happens (an unresolvable topic_type throws std::runtime_error) rather than
  just asserting the doc comment reads correctly.
  Verified with colcon build + the gtest suite (18/18, 20/20) under both
  ENABLE_AGNOCAST=0 and ENABLE_AGNOCAST=1.
  Addresses review comment:
  https://github.com/autowarefoundation/autoware_core/pull/1442#discussion_r-atsushi421-generic_publisher-33
  * fix(autoware_agnocast_wrapper): use non-deprecated serialized-message callback signature
  GenericSubscriptionCallback was std::function<void(std::shared_ptr<
  SerializedMessage>)>. rclcpp deprecates that exact form
  (AnySubscriptionCallback's SharedPtrSerializedMessageCallback) on both
  Humble and Jazzy in favor of the const form,
  SharedConstPtrSerializedMessageCallback — confirmed in both distributions'
  any_subscription_callback.hpp. Agnocast's GenericSubscription accepts the
  const form too (it dispatches on invocability), so switching is a pure
  forward-compat improvement with no loss of support on either backend.
  Changes GenericSubscriptionCallback to
  std::function<void(std::shared_ptr<const rclcpp::SerializedMessage>)>.
  No in-repo caller exists yet to update.
  Adds a static_assert pinning the exact type (not just via an `auto`-typed
  lambda, which would silently adapt to either signature) so a future edit
  can't quietly reintroduce the deprecated form.
  Verified with colcon build + the gtest suite (18/18, 20/20) under both
  ENABLE_AGNOCAST=0 and ENABLE_AGNOCAST=1.
  Addresses review comment:
  https://github.com/autowarefoundation/autoware_core/pull/1442#discussion_r-atsushi421-generic_subscription-32
  * fix(autoware_agnocast_wrapper): include <cstddef> for size_t in generic headers
  generic_publisher.hpp and generic_subscription.hpp use unqualified size_t
  in their depth-overload signatures but only included <cstdint>, <memory>
  and <string> (plus, for the subscription header, <functional>/<utility>) —
  compiling only through rclcpp's transitive includes. Sibling headers
  publisher.hpp and subscription.hpp already include <cstddef> for the same
  reason; brings these two in line.
  Header-hygiene only, no behavior change. Verified with colcon build + the
  gtest suite (18/18, 20/20) under both ENABLE_AGNOCAST=0 and
  ENABLE_AGNOCAST=1.
  Addresses review comment:
  https://github.com/autowarefoundation/autoware_core/pull/1442#discussion_r-atsushi421-generic_publisher-26
  * style(autoware_agnocast_wrapper): use member-initializer for ROS2GenericSubscription
  ROS2GenericSubscription default-constructed subscription\_ and then assigned
  into it in the constructor body, even though the body does nothing else
  first. AgnocastGenericSubscription right above it already builds
  subscription\_ via a member-initializer instead. Switches ROS2Generic
  Subscription to the same pattern for consistency, avoiding the unnecessary
  default-construct-then-assign.
  Pure style change, no behavior difference. The existing round-trip tests
  in generic_pubsub.cpp already exercise this constructor at runtime and
  continue to pass, serving as the regression check here.
  Verified with colcon build + the gtest suite (18/18, 20/20) under both
  ENABLE_AGNOCAST=0 and ENABLE_AGNOCAST=1.
  Addresses review comment:
  https://github.com/autowarefoundation/autoware_core/pull/1442#discussion_r-atsushi421-generic_subscription-105
  * feat(agnocast_wrapper): accept ConstSharedPtr synchronizer callbacks (`#1433 <https://github.com/autowarefoundation/autoware_core/issues/1433>`_)
  * feat(agnocast_wrapper): accept ConstSharedPtr synchronizer callbacks
  registerCallback() took only callbacks taking AUTOWARE_MESSAGE_CONST_SHARED_PTR,
  so a call site whose signature is fixed elsewhere -- an overridden virtual, a
  pluginlib boundary, a third-party callback -- had to bridge the Agnocast pointer
  by hand. It now takes MessageT::ConstSharedPtr callbacks as well, aliased
  straight out of the ipc_shared_ptr: no copy, one allocation per message instead
  of the two a hand-rolled bridge costs. The AUTOWARE_MESSAGE_CONST_SHARED_PTR
  form is probed first, so a callable accepting both keeps resolving to it.
  The probe is std::is_invocable_v<std::decay_t<C> &, ...>, which is what the
  std::function behind the registration accepts: const C & turns away a callable
  whose operator() is non-const, and a bare std::decay_t<C> asks about a prvalue,
  which differs for a ref-qualified operator(). The adapter dispatches through
  std::visit rather than get_if, so a further AnyCallback alternative fails to
  compile instead of throwing on the dispatch path.
  test/cases/message_filters.cpp pins the two decisions the wrapper takes. Which
  shape a registration resolves to is decided at compile time and asserted there:
  every registerCallback() overload is instantiated in both shapes, along with the
  operator() qualifiers std::function accepts, and a callable taking both shapes
  carries a ConstSharedPtr overload that does not compile, so resolving the wrong
  way breaks the build. Which backend a Subscriber holds is the one run-time case.
  Pairing and delivering messages is the backends' own, left to their suites.
  * docs(agnocast_wrapper): say what the adapter does with both shapes
  The comment described the adapter as bridging upstream's arguments to
  message_ptr, which covered every path before this PR and now covers two of
  four: the ConstSharedPtr alternative hands over to_std_shared_ptr() on the
  agnocast side and passes the pointer through untouched on the rclcpp side,
  and that pass-through is the path this PR adds.
  * docs(agnocast_wrapper): name the callables the disabled build takes
  The usage example offered a member-function pointer, a std::bind result, "or
  any other callable convertible to" the two-argument signature. The last one
  holds only where the wrapper's own Synchronizer runs. At ENABLE_AGNOCAST=0 it
  is a direct alias of upstream's, whose Signal9::addCallback(C &) forwards nine
  placeholders to the callable, so a bare lambda does not compile; a
  member-function pointer takes upstream's two-argument overload, and a bind
  result drops the arguments it has no placeholder for.
  * refactor(agnocast_wrapper): keep the callback shape on the adapter's type
  Which of the two shapes a registration resolves to is a property of the
  callable, but it was erased into a std::function at registration and read back
  out of a variant on every delivery, so the same two-way decision stood in
  makeCallback, bindMemberCallback and both invoke bodies.
  Template the adapter on the callable instead. The shape becomes one
  static constexpr bool and one if constexpr per invoke, and the variant, both
  std::function aliases, both probes and both visits go away. Only the vector
  holding the adapters needs the type erased, so AdapterBase carries nothing but
  a virtual destructor -- upstream still calls the invoke members directly on the
  concrete adapter type. std::invoke covers the member-function-pointer case, so
  the instance travels alongside the callable instead of inside a lambda.
  * fix(agnocast_wrapper): reject synchronizer callbacks upstream cannot deliver
  std::shared_ptr<const M> converts to std::weak_ptr<const M> and to
  std::shared_ptr<const void>, so asking is_invocable_v alone accepted a callback
  taking either. Upstream message_filters has no ParameterAdapter for them, so
  such a registration compiled in the agnocast-enabled build and failed inside
  signal9.h in the other one.
  Exclude both, as subscription.hpp already does for the same shape.
  * fix(agnocast_wrapper): take only member functions alongside an instance
  Upstream reaches the two-argument form through addCallback(void (T::*)(P0, P1),
  T *), which a callable other than a member function pointer does not match, so
  it never becomes a candidate. The wrapper takes the callable as a template
  parameter and asks std::is_invocable_v whether it accepts the instance first,
  which is also true of a plain callable declaring T * as its first parameter --
  and std::invoke then calls it that way, so such a registration compiled here
  and had no counterpart in the other build.
  Require a member function pointer where an instance is passed.
  * docs(agnocast_wrapper): give the delivered message its real lifetime
  "preserving zero-copy semantics during the callback" reads like the borrowed
  reference a const MessageT & subscription callback gets, which must not be
  stored. Both shapes here are owning handles that may be kept past the callback.
  What they must not outlive is the Agnocast subscription, which the Subscriber
  drops on unsubscribe() and on a further subscribe() as well as on destruction,
  so naming the Subscriber alone missed two ways to reach a released handle --
  and the class documents calling subscribe() repeatedly. Type spellings, where
  that rule and its consequence live, did not list this path at all.
  * refactor(agnocast_wrapper): drop the unused Callback alias
  The adapter stores the callable itself, so nothing constructs this std::function
  any more. Nothing outside the class ever named it either: ::Callback has no
  occurrence in autoware_core, autoware_universe or the launcher.
  * docs(agnocast_wrapper): say what happens past the retention limit
  The rule said how long a delivered message may be held but not what breaking it
  costs, and the failure is a process exit on whichever thread drops the last
  copy, far from the release that caused it.
  * test(agnocast_wrapper): justify the qualifiers by what now holds the callable
  The note explained the pinned operator() qualifiers by the std::function the
  registration used to store, which d0e47eb3 removed. The set is unchanged: the
  adapter holds the callable and invokes it as a non-const lvalue.
  ---------
  * fix(autoware_agnocast_wrapper): reject qos_overriding_options on generic subscription
  The same backend divergence `#787 <https://github.com/autowarefoundation/autoware_core/issues/787>`_dd79a fixed for the generic publisher
  exists on the subscription side: rclcpp's create_generic_subscription()
  only honors event_callbacks, use_default_callbacks and callback_group —
  it silently drops qos_overriding_options — while Agnocast's
  GenericSubscription applies it. AgnocastGenericSubscription and
  ROS2GenericSubscription forwarded it through regardless, via
  to_rclcpp_subscription_options(), so the same SubscriptionOptions would
  behave differently depending on which backend happened to be selected at
  runtime.
  Both constructors now reject a non-empty qos_overriding_options with
  std::invalid_argument via a shared check_generic_subscription_options()
  helper, mirroring check_generic_publisher_options(). callback_group and
  ignore_local_publications still forward through
  to_rclcpp_subscription_options() as before; only qos_overriding_options is
  rejected.
  Updates the README/review_guide wording ("on the publisher side" ->
  "on both the publisher and the subscription") to match.
  Adds RejectsQosOverridingOptions / DefaultOptionsDoNotThrow tests for the
  subscription, mirroring the existing publisher-side tests.
  Verified with colcon build + the gtest suite (18/18, 23/23 with the new
  Agnocast-only tests) under both ENABLE_AGNOCAST=0 and ENABLE_AGNOCAST=1.
  * fix(autoware_agnocast_wrapper): skip generic pub/sub tests without the agnocast heaphook
  CI runs the ENABLE_AGNOCAST=1 job with ENABLE_AGNOCAST=1 at test runtime
  too (colcon-test's env, same as colcon-build's), but never sets
  LD_PRELOAD=libagnocast_heaphook.so. Constructing an Agnocast endpoint in
  that state exits the whole process from validate_ld_preload() instead of
  throwing, so GenericPubSubMethod1Test.MacroRoundTrip took the entire test
  binary down (ctest return code 1) the moment it tried to build a real
  Agnocast publisher/subscriber, and every later test in the run never
  executed either.
  polling_subscriber.cpp and service_introspection.cpp already guard against
  exactly this with an agnocast_heaphook_loaded() check in SetUp() that
  GTEST_SKIPs instead. generic_pubsub.cpp's tests never got that guard.
  Adds the same guard here: a shared GenericPubSubTestBase fixture (aliased
  to each existing test suite name, so no test names change) skips via
  heaphook_probe.hpp's agnocast_heaphook_loaded() the same way, and every
  TEST() becomes TEST_F() against it.
  Verified this actually reproduces and fixes the CI failure: rebuilt with
  ENABLE_AGNOCAST=1 and ran the suite with runtime ENABLE_AGNOCAST=1 and no
  LD_PRELOAD (the CI condition) — before this change that crashes the binary
  the same way CI did; after, all 8 generic pub/sub cases report SKIPPED and
  the binary exits 0, the same behavior PollingSubscriberTest/
  ServiceIntrospectionTest already have in that state.
  Also verified with colcon build + the gtest suite (18/18, 23/23) under
  both ENABLE_AGNOCAST=0 and ENABLE_AGNOCAST=1 with the runtime env unset,
  confirming the tests still actually run and pass in that configuration.
  * fix(autoware_agnocast_wrapper): reject qos_overriding_options in the non-Agnocast build too
  check_generic_publisher_options()/check_generic_subscription_options()
  lived inside #ifdef USE_AGNOCAST_ENABLED, so they only reconciled the
  divergence between AgnocastGenericPublisher/Subscription and
  ROS2GenericPublisher/Subscription within an ENABLE_AGNOCAST=1 build.
  Node::create_generic_publisher()/create_generic_subscription() in the
  non-Agnocast build passed qos_overriding_options straight through to
  rclcpp unchecked, so the same caller code setting it would build and run
  under ENABLE_AGNOCAST=0 but throw under ENABLE_AGNOCAST=1 — the same
  inconsistency the original fix was meant to close, just moved to a
  different axis (build config instead of runtime backend).
  Moves the checks outside the #ifdef, in both generic_publisher.hpp and
  generic_subscription.hpp, as check_generic_publisher_qos_overriding_options()
  /check_generic_subscription_qos_overriding_options(). They now take
  rclcpp::QosOverridingOptions directly instead of the whole
  agnocast::PublisherOptions/SubscriptionOptions, so they have no dependency
  on Agnocast-only types and compile in both builds. Calls the appropriate
  one from Node::create_generic_publisher()/create_generic_subscription() in
  node.hpp's non-Agnocast branch (the Agnocast-enabled branch already
  delegates to AgnocastGenericPublisher/ROS2GenericPublisher, which check
  internally).
  Adds NodeMemberPublisherRejectsQosOverridingOptions/
  NodeMemberSubscriptionRejectsQosOverridingOptions — unconditional (using
  AUTOWARE_PUBLISHER_OPTIONS/AUTOWARE_SUBSCRIPTION_OPTIONS so the same test
  source runs in both builds) — and confirmed they fail without this change
  under ENABLE_AGNOCAST=0 (the gap this closes) and pass with it.
  Verified with colcon build + the gtest suite (20/20, 25/25) under both
  ENABLE_AGNOCAST=0 and ENABLE_AGNOCAST=1.
  * fix(autoware_agnocast_wrapper): route Method 1 generic macros through the qos_overriding_options check
  AUTOWARE_CREATE_GENERIC_PUBLISHER4/AUTOWARE_CREATE_GENERIC_SUBSCRIPTION
  under ENABLE_AGNOCAST=0 expanded to this->create_generic_publisher()/
  create_generic_subscription() directly — rclcpp::Node's own native
  methods — bypassing check_generic_publisher_qos_overriding_options()/
  check_generic_subscription_qos_overriding_options() entirely. d10b74b0
  closed this gap for Method 2 (agnocast_wrapper::Node members) but Method 1
  (the macros, used on a plain rclcpp::Node) still let the same
  qos_overriding_options silently pass through under ENABLE_AGNOCAST=0 while
  throwing under ENABLE_AGNOCAST=1.
  Gives the non-Agnocast build its own create_generic_publisher(rclcpp::Node
  *, ...)/create_generic_subscription(rclcpp::Node *, ...) free functions —
  same name as the Agnocast-enabled build's, mirroring how
  AUTOWARE_GENERIC_PUBLISHER_PTR etc. already resolve to a different type
  per build — that check first, then forward to rclcpp. Routes the non-
  Agnocast macros through them instead of calling `this`/`node` directly,
  matching how AUTOWARE_CREATE_CLIENT*/AUTOWARE_CREATE_SERVICE* already stay
  identical text in both #ifdef branches because clients/services always go
  through a wrapper function regardless of build.
  Adds MacroPublisherRejectsQosOverridingOptions/
  MacroSubscriptionRejectsQosOverridingOptions (Method 1, via
  GenericPubSubMethod1Node), unconditional like the Method 2 tests, and
  confirmed they fail to throw without this change under ENABLE_AGNOCAST=0
  (the exact gap flagged) and pass with it.
  Verified with colcon build + the gtest suite (22/22, 27/27) under both
  ENABLE_AGNOCAST=0 and ENABLE_AGNOCAST=1.
  * test(autoware_agnocast_wrapper): cover the _ON_NODE generic macros
  AUTOWARE_CREATE_GENERIC_PUBLISHER3/4_ON_NODE and
  AUTOWARE_CREATE_GENERIC_SUBSCRIPTION_ON_NODE had no test coverage at all
  (not even a basic round-trip), so 23f9b329's qos_overriding_options fix for
  Method 1 was verified only through the this-implicit macros
  (AUTOWARE_CREATE_GENERIC_PUBLISHER3/4, AUTOWARE_CREATE_GENERIC_SUBSCRIPTION),
  even though the _ON_NODE variants route through the exact same wrapper free
  functions and had the identical bug before that commit.
  Adds MacroOnNodeRoundTrip (basic pub/sub functionality, outside a node
  subclass, matching the _ON_NODE macros' documented use case) and
  MacroOnNodePublisherRejectsQosOverridingOptions/
  MacroOnNodeSubscriptionRejectsQosOverridingOptions, confirmed to fail
  without 23f9b329 under ENABLE_AGNOCAST=0 and pass with it, same as the
  this-implicit macro tests already do.
  Verified with colcon build + the gtest suite (25/25, 30/30) under both
  ENABLE_AGNOCAST=0 and ENABLE_AGNOCAST=1.
  * fix(autoware_agnocast_wrapper): route the generic subscription depth overload through the QoS overload's check
  Node::create_generic_subscription()'s depth (qos_history_depth) overload in the
  non-Agnocast build called node\_->create_generic_subscription() directly instead of
  delegating to the QoS-taking overload above it, so it skipped
  check_generic_subscription_qos_overriding_options() entirely: qos_overriding_options
  was silently ignored through this overload under ENABLE_AGNOCAST=0, while the
  Agnocast-enabled build's equivalent depth overload already delegated correctly and
  threw. Found by re-auditing every node\_->create_generic_publisher()/
  create_generic_subscription() call site in node.hpp for the same bypass pattern
  after a reviewer caught this one instance (the publisher's depth overload has no
  options parameter to leak through, so it was not affected).
  Change it to delegate to the QoS overload, mirroring the Agnocast-enabled branch's
  own depth overload and the free-function depth overloads in generic_publisher.hpp/
  generic_subscription.hpp.
  Added NodeMemberSubscriptionDepthOverloadRejectsQosOverridingOptions, which fails
  without this fix and passes with it; confirmed by reverting the fix and re-running.
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaro Kobayashi <129580202+kobayu858@users.noreply.github.com>
  Co-authored-by: Koichi Imai <45482193+Koichi98@users.noreply.github.com>
  Co-authored-by: atsushi yano <55824710+atsushi421@users.noreply.github.com>
* feat(agnocast_wrapper): accept ConstSharedPtr synchronizer callbacks (`#1433 <https://github.com/autowarefoundation/autoware_core/issues/1433>`_)
  * feat(agnocast_wrapper): accept ConstSharedPtr synchronizer callbacks
  registerCallback() took only callbacks taking AUTOWARE_MESSAGE_CONST_SHARED_PTR,
  so a call site whose signature is fixed elsewhere -- an overridden virtual, a
  pluginlib boundary, a third-party callback -- had to bridge the Agnocast pointer
  by hand. It now takes MessageT::ConstSharedPtr callbacks as well, aliased
  straight out of the ipc_shared_ptr: no copy, one allocation per message instead
  of the two a hand-rolled bridge costs. The AUTOWARE_MESSAGE_CONST_SHARED_PTR
  form is probed first, so a callable accepting both keeps resolving to it.
  The probe is std::is_invocable_v<std::decay_t<C> &, ...>, which is what the
  std::function behind the registration accepts: const C & turns away a callable
  whose operator() is non-const, and a bare std::decay_t<C> asks about a prvalue,
  which differs for a ref-qualified operator(). The adapter dispatches through
  std::visit rather than get_if, so a further AnyCallback alternative fails to
  compile instead of throwing on the dispatch path.
  test/cases/message_filters.cpp pins the two decisions the wrapper takes. Which
  shape a registration resolves to is decided at compile time and asserted there:
  every registerCallback() overload is instantiated in both shapes, along with the
  operator() qualifiers std::function accepts, and a callable taking both shapes
  carries a ConstSharedPtr overload that does not compile, so resolving the wrong
  way breaks the build. Which backend a Subscriber holds is the one run-time case.
  Pairing and delivering messages is the backends' own, left to their suites.
  * docs(agnocast_wrapper): say what the adapter does with both shapes
  The comment described the adapter as bridging upstream's arguments to
  message_ptr, which covered every path before this PR and now covers two of
  four: the ConstSharedPtr alternative hands over to_std_shared_ptr() on the
  agnocast side and passes the pointer through untouched on the rclcpp side,
  and that pass-through is the path this PR adds.
  * docs(agnocast_wrapper): name the callables the disabled build takes
  The usage example offered a member-function pointer, a std::bind result, "or
  any other callable convertible to" the two-argument signature. The last one
  holds only where the wrapper's own Synchronizer runs. At ENABLE_AGNOCAST=0 it
  is a direct alias of upstream's, whose Signal9::addCallback(C &) forwards nine
  placeholders to the callable, so a bare lambda does not compile; a
  member-function pointer takes upstream's two-argument overload, and a bind
  result drops the arguments it has no placeholder for.
  * refactor(agnocast_wrapper): keep the callback shape on the adapter's type
  Which of the two shapes a registration resolves to is a property of the
  callable, but it was erased into a std::function at registration and read back
  out of a variant on every delivery, so the same two-way decision stood in
  makeCallback, bindMemberCallback and both invoke bodies.
  Template the adapter on the callable instead. The shape becomes one
  static constexpr bool and one if constexpr per invoke, and the variant, both
  std::function aliases, both probes and both visits go away. Only the vector
  holding the adapters needs the type erased, so AdapterBase carries nothing but
  a virtual destructor -- upstream still calls the invoke members directly on the
  concrete adapter type. std::invoke covers the member-function-pointer case, so
  the instance travels alongside the callable instead of inside a lambda.
  * fix(agnocast_wrapper): reject synchronizer callbacks upstream cannot deliver
  std::shared_ptr<const M> converts to std::weak_ptr<const M> and to
  std::shared_ptr<const void>, so asking is_invocable_v alone accepted a callback
  taking either. Upstream message_filters has no ParameterAdapter for them, so
  such a registration compiled in the agnocast-enabled build and failed inside
  signal9.h in the other one.
  Exclude both, as subscription.hpp already does for the same shape.
  * fix(agnocast_wrapper): take only member functions alongside an instance
  Upstream reaches the two-argument form through addCallback(void (T::*)(P0, P1),
  T *), which a callable other than a member function pointer does not match, so
  it never becomes a candidate. The wrapper takes the callable as a template
  parameter and asks std::is_invocable_v whether it accepts the instance first,
  which is also true of a plain callable declaring T * as its first parameter --
  and std::invoke then calls it that way, so such a registration compiled here
  and had no counterpart in the other build.
  Require a member function pointer where an instance is passed.
  * docs(agnocast_wrapper): give the delivered message its real lifetime
  "preserving zero-copy semantics during the callback" reads like the borrowed
  reference a const MessageT & subscription callback gets, which must not be
  stored. Both shapes here are owning handles that may be kept past the callback.
  What they must not outlive is the Agnocast subscription, which the Subscriber
  drops on unsubscribe() and on a further subscribe() as well as on destruction,
  so naming the Subscriber alone missed two ways to reach a released handle --
  and the class documents calling subscribe() repeatedly. Type spellings, where
  that rule and its consequence live, did not list this path at all.
  * refactor(agnocast_wrapper): drop the unused Callback alias
  The adapter stores the callable itself, so nothing constructs this std::function
  any more. Nothing outside the class ever named it either: ::Callback has no
  occurrence in autoware_core, autoware_universe or the launcher.
  * docs(agnocast_wrapper): say what happens past the retention limit
  The rule said how long a delivered message may be held but not what breaking it
  costs, and the failure is a process exit on whichever thread drops the last
  copy, far from the release that caused it.
  * test(agnocast_wrapper): justify the qualifiers by what now holds the callable
  The note explained the pinned operator() qualifiers by the std::function the
  registration used to store, which d0e47eb3 removed. The set is unchanged: the
  adapter holds the callable and invokes it as a non-const lvalue.
  ---------
* refactor(autoware_agnocast_wrapper): return client responses as std::shared_ptr (`#1419 <https://github.com/autowarefoundation/autoware_core/issues/1419>`_)
  * feat(autoware_agnocast_wrapper): add to_shared_ptr() for client responses
  * refactor(autoware_agnocast_wrapper): move to_shared_ptr() to the message_ptr layer
  * style(pre-commit): autofix
  * fix(autoware_agnocast_wrapper): reject publisher-side handles in to_shared_ptr()
  * refactor(autoware_agnocast_wrapper): return client responses as std::shared_ptr
  * style(pre-commit): autofix
  * docs(autoware_agnocast_wrapper): document the client response in the lifetime rules
  * docs(autoware_agnocast_wrapper): drop the stale client response row from the type table
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: atsushi yano <55824710+atsushi421@users.noreply.github.com>
* feat(autoware_agnocast_wrapper): forward configure_introspection on client and service (`#1429 <https://github.com/autowarefoundation/autoware_core/issues/1429>`_)
  * feat(autoware_agnocast_wrapper): forward configure_introspection on client and service
  autoware_component_interface_utils calls configure_introspection() on the client
  and service handles it creates, unconditionally on rclcpp 21+. The handle type is
  deduced from NodeT, so with NodeT = autoware::agnocast_wrapper::Node it is the
  wrapper's Client / Service, which had no such method -- a compile error on Jazzy,
  invisible on Humble where the surrounding #if erases the call.
  Both backends implement it from Iron (rclcpp 21) onward, so the wrapper only has
  to forward. The addition is gated on RCLCPP_VERSION_GTE(21, 0, 0), the spelling
  the call site already uses; a static_assert in the agnocast half pins that gate
  against agnocast's own AGNOCAST_HAS_SERVICE_INTROSPECTION, so a future divergence
  is a message rather than "no member named configure_introspection".
  A null clock and a KeepAll QoS are rejected on the handle rather than left to the
  backends, which disagree on both: agnocast throws std::invalid_argument, while
  rclcpp dereferences the null clock and accepts a KeepAll that agnocast cannot
  serve. The public method is therefore not virtual; backends override
  configure_introspection_impl(), the shape wait_for_service() already uses, and
  that hook is protected so the checks cannot be reached around.
  The heaphook probe the new cases need moves to test/heaphook_probe.hpp and is
  shared with the polling-subscriber cases instead of copied.
  * refactor(autoware_agnocast_wrapper): check the introspection arguments in one place
  * fix(autoware_agnocast_wrapper): declare the service introspection hook before public
  * test(autoware_agnocast_wrapper): assert on both introspection handles in one case each
  * docs(autoware_agnocast_wrapper): document the backend exceptions and thread-safety of configure_introspection
  * fix(autoware_agnocast_wrapper): report the introspection gate mismatch at preprocessing time
  * test(autoware_agnocast_wrapper): compile exactly one introspection skip per build
  * docs(autoware_agnocast_wrapper): say what Agnocast does with a KeepAll introspection QoS
  ---------
  Co-authored-by: atsushi yano <55824710+atsushi421@users.noreply.github.com>
* feat(agnocast_wrapper): add a backend-agnostic async parameters client (`#1430 <https://github.com/autowarefoundation/autoware_core/issues/1430>`_)
  * feat(autoware_agnocast_wrapper): add AsyncParametersClient
  rclcpp::AsyncParametersClient cannot be built on an Agnocast node: it reaches
  the remote node's parameter services through NodeServicesInterface::add_client(),
  which Agnocast does not support. Agnocast ships its own, so reading another
  node's parameters in both modes needs a wrapper that picks the backend at
  construction.
  Shaped like tf2.hpp and diagnostic_updater.hpp: one concrete class defined in
  both halves of the #ifdef, a std::variant on the Agnocast side, non-copyable and
  non-movable, and spelled the same in both builds so it needs no AUTOWARE\_ macro.
  Of the six parameter service calls both backends offer it exposes only
  get_parameters(), which is all the first caller uses, alongside
  wait_for_service() and service_is_ready().
  One backend difference it settles, and one it leaves alone.
  A transient-local QoS cannot work on either backend. A node's parameter services
  are always volatile -- rclcpp offers no way to change them -- so a
  transient-local client never matches one, and neither backend says so: rclcpp
  leaves the client mute and Agnocast rewrites the durability away. The
  constructor rejects anything but volatile.
  The exception type for an unusable remote_node_name stays each backend's own;
  exception types are not normalized anywhere in the wrapper.
  Two test cases, covering the wrapper's own behaviour rather than either
  backend's: the compile-time surface and the QoS check from both sides. They pass
  at ENABLE_AGNOCAST=0, at =1 with the runtime disabled, and at =1 with the 2.4.0
  kernel module and heaphook loaded.
  * docs(agnocast_wrapper): say where the parameters client's wait honours its timeout
  agnocast::AsyncParametersClient stops as soon as agnocast::ok() is false, which
  it is in any process that did not call agnocast::init() -- a component container
  brings up the rclcpp context alone. There the backend returns false after one
  readiness probe, and a caller cannot tell that from a timeout.
  Documented rather than worked around. The wrapper polled service_is_ready()
  until 5b59273f, and the only configuration that rescues is a Method 2 node in a
  component container, which `autowarefoundation/agnocast#1517 <https://github.com/autowarefoundation/agnocast/issues/1517>`_ has to make work
  first; autoware_agnocast_wrapper_register_node() gives an AgnocastOnly
  executable agnocast::init() and a working upstream wait. Leaving the workaround
  out also keeps this consistent with agnocast_wrapper::Client::wait_for_service(),
  which forwards straight through and has the same gap.
  * refactor(agnocast_wrapper): give the parameters client's QoS handling one home
  checked_qos() and the RCLCPP_VERSION_MAJOR >= 28 argument block were copy-pasted
  into both halves of the #ifdef, which is why the agnocast-disabled copy had to
  point back at the other for its rationale. They move out to two inline free
  functions above the split, the shape polling::check_polling_qos() and
  detail::to_qos() already use in this package: detail::checked_parameters_qos()
  and detail::to_rclcpp_parameters_qos(), the latter owning the version gate.
  What is rejected does not change here.
  * fix(agnocast_wrapper): reject the parameter QoS policies the backends disagree on
  Two corrections to what detail::checked_parameters_qos() rejects, and one to why
  it rejects at all.
  A durability of SystemDefault or Unknown was rejected along with transient-local
  by comparing against Volatile. Both resolve to volatile in every RMW and would
  match a parameter service, so the check compares against TransientLocal, the one
  value that does not.
  Reliability was not checked and diverges the same way. rclcpp's parameter
  services are RELIABLE, so a best-effort client's request writer never matches
  their reliable reader and wait_for_service() fails without a word; Agnocast does
  not use reliability for matching, so the same QoS works at runtime
  ENABLE_AGNOCAST=1. BestEffort is now rejected -- BestEffort specifically, not
  != Reliable, since SystemDefault and Unknown resolve to reliable.
  The rationale said a transient-local QoS "cannot work" on either backend. It
  does work on Agnocast: agnocast::Client rewrites the durability to volatile
  (`autowarefoundation/agnocast#1255 <https://github.com/autowarefoundation/agnocast/issues/1255>`_). rclcpp is the one that silently never
  matches, so rejecting here is what makes the two builds agree rather than a
  restatement of something both backends already refuse. Saying it the old way
  invited a later reader to drop the check.
  * perf(agnocast_wrapper): stop the parameters client copying the callable
  std::function copies allocate once the captures outgrow the small-buffer size,
  and the chain was caller -> wrapper -> backend -> the backend's response lambda,
  with only the wrapper handing its by-value argument on as an lvalue. It moves
  now, at both get_parameters() and, for the callback group's refcount, at both
  constructors -- the ternary evaluates one branch, so moving in both is safe.
  * fix(agnocast_wrapper): include what the parameters client uses
  <type_traits> is left over from the if constexpr that went away with the polling
  wait; nothing in either build names anything from it. std::in_place_type and
  int64_t were reaching the header transitively instead, so <cstdint> joins the
  <utility> that came in with the moves.
  * docs(agnocast_wrapper): stop the parameters client's mirror repeating itself
  The agnocast-disabled class carried the agnocast-enabled build's member docs
  byte for byte, including a "Both backends" that describes nothing in a build
  with one backend. tf2.hpp's mirror keeps none of them and says once that the
  signatures match, so this one does the same, holding on to only the reason it
  composes rather than derives.
  The class doc also gains the @invariant that tf2.hpp and diagnostic_updater.hpp
  both state and this one had left implicit.
  * docs(agnocast_wrapper): warn about the parameters client's callback-group traps
  Two things a caller cannot see from the signature.
  Blocking on the future from inside a callback deadlocks: the executor thread
  that would run the response callback, and so satisfy the promise, is the one
  sitting in the wait. Written as a rule rather than a condition because the
  shape of the trap differs -- a single-threaded executor stops whatever the
  group type is, a multi-threaded one only when the group is MutuallyExclusive --
  and it is not Agnocast-specific: rclcpp's executor gates the same way.
  Leaving group null puts the response subscription in the node's default
  MutuallyExclusive group, and at ENABLE_AGNOCAST=1 an Agnocast subscription
  sharing such a group with an rclcpp entity aborts the process. A Reentrant
  group is accepted.
  * docs(agnocast_wrapper): say which backend the parameters client's lifetime rules bind
  The @pre read as if both backends were equally fragile. Only Agnocast is:
  ::agnocast::ClientBase keeps the node as a raw pointer and dereferences it on
  the response error path, while rclcpp holds interface shared_ptrs that keep the
  node alive, so the wrong destruction order is survivable there.
  Destroying the client itself needs a rule too, and it is not a precondition of
  construction, so it sits in the class doc: a response already queued for
  dispatch is not withdrawn when the client goes away, and running it afterwards
  touches freed state. Nothing to fix here -- the enqueued callable would have to
  re-check the callback registry under the lock, which is agnocastlib's to do.
  * test(agnocast_wrapper): exercise the parameters client past its constructor
  wait_for_service() is a member template, so nothing instantiated it: misspell
  the member it forwards to and the constructor-only cases still compiled, in both
  builds. A case that calls it and service_is_ready() against an absent remote
  node closes that, and takes std::visit past the constructor while it is there.
  Zero is the timeout to use -- the one value both upstream clients define as a
  non-blocking probe, so the case reads the same in all three modes.
  The best-effort rejection added with the reliability check gets a case too, and
  the accepting one now spells both policies out.
  * docs(agnocast_wrapper): add parameter clients to the migration checklist
  A leftover rclcpp::AsyncParametersClient in a Method 2 node fails the same way
  as a leftover timer or tf2 listener: it builds and runs at ENABLE_AGNOCAST=0,
  and reaches the node through get_rclcpp_node(), which throws in agnocast mode.
  It belongs in the checklist item that already names the others.
  * docs(agnocast_wrapper): say what leaves the parameters client's indefinite wait
  A negative duration is the default, and on the Agnocast backend the wait ends
  only when agnocast::ok() goes false. Naming rclcpp::shutdown() as the thing that
  does not do it leaves a caller without a remedy, so the line names the two that
  do: SIGINT and SIGTERM, which agnocast::init() installs a handler for and whose
  own thread shuts the context down without the blocked thread's help, and this
  package's shutdown() when another thread calls it.
  ---------
  Co-authored-by: atsushi yano <55824710+atsushi421@users.noreply.github.com>
* feat(agnocast_wrapper): give the subscription a polling mode (`#1363 <https://github.com/autowarefoundation/autoware_core/issues/1363>`_)
  * fix(autoware_agnocast_wrapper): carry ignore_local_publications to the DDS path
  An agnocast-enabled build with the DDS backend selected translates the Agnocast
  subscription options into the rclcpp ones by hand, and ROS2Subscription copied
  only callback_group and qos_overriding_options. ignore_local_publications was
  dropped, so a subscription asking to ignore same-process publishers received
  them anyway.
  TransformListener already copied all three. Factor that into
  to_rclcpp_subscription_options() and use it in both places, so the next field
  Agnocast gains is added once.
  * feat(agnocast_wrapper): give the subscription a polling mode
  AgnocastSubscription holds either an agnocast::Subscription or an
  agnocast::TakeSubscription, never both: agnocast fixes the delivery mode at
  construction, so a subscription created with a callback cannot be polled.
  create_subscription() therefore also has callback-less overloads. Under
  Agnocast they register no eventfd, so publishers skip the subscription when
  signalling; under rclcpp the wrapper builds the usual
  no-op-callback-in-an-unspun-group idiom, so the call site is the same in both
  builds. A SubscriptionOptions in the callback position has to select these
  rather than be deduced as a callback, hence the is_subscription_callback_v
  guard on every callback overload, in the ENABLE_AGNOCAST=0 Node too.
  take() refuses a callback subscription in both builds. rclcpp would allow it,
  but its take() consumes the message the callback was going to receive, so the
  same call would lose messages in one build and throw in the other.
  Options keep the existing per-build spelling: the agnocast-enabled Node takes
  agnocast::SubscriptionOptions and the disabled one rclcpp::SubscriptionOptions,
  as AUTOWARE_SUBSCRIPTION_OPTIONS already resolves them. Polling is exposed as
  take(out, info) only, the shape component_interface_utils calls; a
  pointer-returning take_data() would keep Agnocast's zero copy, but nothing
  polls through that package today and rclcpp::Subscription has no such member,
  so adding it would break the =0 build's promise that the same source compiles
  against both Nodes.
  * fix(agnocast_wrapper): refuse the caller's callback group on a polling subscription
  The callback-less create_subscription() only built a no-exec group when the
  caller supplied none, so a group made with the default
  automatically_add_to_executor_with_node = true was honoured: the executor then
  dispatched the no-op callback and consumed every message, and take() returned
  false forever with nothing reported.
  Always build the group here and warn that the caller's is ignored, which is
  what agnocast::TakeSubscription does on the other backend, and assert(false)
  in the callback that can now never run, as
  autoware_utils_rclcpp::InterProcessPollingSubscriber does.
  * fix(agnocast_wrapper): disable intra-process delivery on a polling subscription
  take() drops a sample whose publisher is matched intra-process, expecting the
  intra-process waitable to deliver it instead. That waitable sits in the polling
  subscription's own callback group, which is deliberately outside the executor,
  so on a node with intra-process comms enabled every same-process publisher was
  lost: the DDS path returned false forever while the Agnocast path delivered
  them.
  * refactor(agnocast_wrapper): take the not-pollable topic name off the handle
  topic_name\_ existed only for that message, so every callback subscription paid
  for a string it would never read, and the copy it held was the name as passed
  rather than the remapped one. Both handles already expose the resolved name.
  * fix(agnocast_wrapper): fill the message info the Agnocast take() leaves behind
  The override never wrote to its out-parameter, so a caller's
  default-constructed rclcpp::MessageInfo came back holding whatever the stack
  did -- and a real value on the rclcpp path, with nothing to warn them. Agnocast
  carries none of the fields, so zero it and report the two sequence numbers as
  unsupported rather than as sequence number zero.
  * docs(agnocast_wrapper): document the callback-less subscription and take()
  The Node API table listed only the QoS and depth overloads, and take() appeared
  nowhere. Say which of the two ways to poll a new caller should reach for, so
  they do not read as interchangeable: the polling subscriber keeps the zero copy
  and the re-delivery policy, and this form is for callers that need an
  rclcpp::Subscription-shaped handle.
  * fix(agnocast_wrapper): reword the comment cspell rejects
  "unspun" is not in the Autoware dictionary.
  ---------
* feat(agnocast_wrapper): accept rclcpp-shaped client and service calls (`#1394 <https://github.com/autowarefoundation/autoware_core/issues/1394>`_)
  * feat(agnocast_wrapper): accept rclcpp-shaped client and service calls
  Client gains SharedResponse, for generic code that has to take the response
  pointer type off the client rather than spell it, and async_send_request()
  overloads taking a std::shared_ptr request by const reference, for callers that
  hold the request as an lvalue and cannot change its type. By const reference
  rather than by value so that async_send_request(std::move(req)) still binds to
  the rvalue-reference overloads. Only the Agnocast backend copies the request, to
  get the payload into shared memory; the DDS backends forward the pointer, as
  rclcpp::Client does.
  create_client() and create_service() also accept an rmw_qos_profile_t, because
  rclcpp::Node only grew the rclcpp::QoS overloads in Iron (rclcpp 21) and callers
  written against Humble still pass rmw_qos_profile_services_default. The profile
  is passed through unchanged: QoSInitialization::from_rmw() would report a
  SYSTEM_DEFAULT or UNKNOWN history as KEEP_LAST and drop the depth of a KEEP_ALL
  profile.
  * fix(agnocast_wrapper): reject a null request before a backend allocates for it
  The std::shared_ptr overloads made async_send_request(nullptr) compile in the
  Agnocast build, where it used to be rejected because message_ptr's shared_ptr
  constructor is explicit. Dereferencing it would have crashed after a
  shared-memory slot was already borrowed.
  * refactor(agnocast_wrapper): keep to_qos out of the public namespace
  Its only callers are the rmw_qos_profile_t overloads, which are transitional.
  Leaving it where downstream code can find it by name would make removing them a
  breaking change.
  * docs(agnocast_wrapper): say why the request-owning step is a hook
  * docs(agnocast_wrapper): note that the client response alias is const
  rclcpp::Client carries the same name for a non-const pointer, so generic code
  templated over both node types compiles in the rclcpp instantiation and fails
  only in the wrapper one if it writes through the response.
  * docs(agnocast_wrapper): name the call that only ENABLE_AGNOCAST=0 accepts
  An owned request passed as an lvalue binds to the std::shared_ptr overload
  there, because the two spellings collapse onto one type, and does not compile at
  =1. Overload resolution cannot tell the two apart in that build, so the
  difference can only be written down.
  * docs(agnocast_wrapper): say that the Agnocast backend copies the request
  Publisher::publish(const MessageT &) documents the same thing on the publisher
  side.
  ---------
  Co-authored-by: atsushi yano <55824710+atsushi421@users.noreply.github.com>
* feat(autoware_agnocast_wrapper): forward get_topic_name() from polling subscribers (`#1426 <https://github.com/autowarefoundation/autoware_core/issues/1426>`_)
  * feat(autoware_agnocast_wrapper): forward get_topic_name() from polling subscribers
  * docs(autoware_agnocast_wrapper): note that polling subscribers expose get_topic_name()
  ---------
* fix(autoware_agnocast_wrapper): cache the last message in the polling subscriber (`#1407 <https://github.com/autowarefoundation/autoware_core/issues/1407>`_)
  * refactor(autoware_agnocast_wrapper): mirror the autoware_utils_rclcpp polling policies
  * fix(autoware_agnocast_wrapper): reject an unsupported polling policy on a static_assert
  AgnocastPollingPolicy's primary template was only declared, so a policy with
  no specialization made AgnocastPollingSubscriber::policy\_ an incomplete type.
  create_polling_subscriber<M, polling_policy::All> then failed on that error as
  well as on the static_assert that states the reason, and only in an
  ENABLE_AGNOCAST=1 build.
  Define the primary template so the type stays complete and carry the reason in
  a static_assert of its own. take_data() is a stub: the static_assert rejects
  the instantiation before the virtual take_data_impl() that calls it is built.
  * docs(autoware_agnocast_wrapper): note that take_data() is not synchronized
  The migration checklist is where the calling context of a polling subscriber is
  decided, and take_data() carries the same single-thread rule as
  autoware_utils_rclcpp's polling subscriber.
  * test(autoware_agnocast_wrapper): wait on the subscriber count instead of republishing
  Republishing until a message landed left one more message in flight, which the
  tests absorbed with a fixed 200 ms sleep before asserting on what the next take
  returns. Wait for the publisher to see the subscriber instead, then publish once:
  a same-process subscriber shows up in get_intra_process_subscription_count() on
  the agnocast backend and in get_subscription_count() on the ROS 2 backend, so
  their sum works on both.
  * test(autoware_agnocast_wrapper): pin that Latest replaces the cached message
  The case asserted only that the same message comes back, so an implementation
  that remembered the first message forever kept every case green. Publish a
  second message and assert both that take_data() returns it and that a further
  take_data() re-delivers it rather than the first.
  * refactor(autoware_agnocast_wrapper): make take_data() the virtual itself
  The non-virtual take_data() existed to compute the policy-dependent
  allow_same_message default before forwarding to the virtual take_data_impl().
  That parameter is gone, so the indirection only forwards.
  * fix(autoware_agnocast_wrapper): reject a polling QoS whose history depth is not 1
  Only depth > 1 was rejected, and only on the agnocast path, so KeepAll and
  KeepLast(0) reached the backends: the agnocast take loop never iterates at depth
  0 while the ROS 2 backend delivers normally. Check in create_polling_subscriber()
  so the rejection depends on neither the build nor use_agnocast(), and name the
  topic and the actual depth, which a node with several polling subscribers needs.
  * docs(autoware_agnocast_wrapper): state the polling subscriber's member order constraint
  The cached message pins a kmod entry under the subscription's id, and releasing
  it after the subscription is gone aborts the process. Only the declaration order
  keeps that from happening.
  ---------
  Co-authored-by: atsushi yano <55824710+atsushi421@users.noreply.github.com>
* feat(autoware_agnocast_wrapper): add mode-agnostic init() and shutdown() (`#1415 <https://github.com/autowarefoundation/autoware_core/issues/1415>`_)
  * feat(autoware_agnocast_wrapper): add mode-agnostic init() and shutdown()
  * docs(autoware_agnocast_wrapper): state the real reason for destroying the node first
  * refactor(autoware_agnocast_wrapper): declare use_agnocast() regardless of the build mode
  * fix(autoware_agnocast_wrapper): make the context flag atomic and latch shutdown()
  * fix(autoware_agnocast_wrapper): rename the env reader to satisfy clang-tidy naming
  * docs(autoware_agnocast_wrapper): correct the context claims and record the sim-time constraint
  * docs(autoware_agnocast_wrapper): note that the agnocast context carries no arguments over
  * docs(autoware_agnocast_wrapper): note how the discarded arguments can be recovered
  * docs(autoware_agnocast_wrapper): note that agnocast_only needs ENABLE_AGNOCAST and a fallback
  ---------
* feat(agnocast_wrapper): expose endpoint name and QoS on the wrappers (`#1395 <https://github.com/autowarefoundation/autoware_core/issues/1395>`_)
  * feat(agnocast_wrapper): expose endpoint name and QoS on the wrappers
  Subscription gains get_topic_name() and get_actual_qos(), Publisher gains
  get_actual_qos() and Service gains get_service_name(), so callers can read the
  remap-resolved name and the effective QoS off a wrapper endpoint the way they
  already can off an rclcpp one.
  * fix(agnocast_wrapper): forward qos_overriding_options to ROS2Subscription
  ROS2Publisher forwards it and the Agnocast path carries it through
  to_agnocast_subscription_options(), so a subscription created with overrides was
  the one endpoint that silently ignored them.
  * docs(agnocast_wrapper): scope the accessor note to the handles that have them
  polling::PollingSubscriber exposes neither accessor, and neither does the
  autoware_utils_rclcpp class it wraps, so "the endpoint handles" overstated it.
  * docs(agnocast_wrapper): move the accessor note to the end of the section
  It describes members on constructed handles, so it no longer splits the two
  type-spelling notes.
  * chore(agnocast_wrapper): require agnocastlib 2.4.0
  The accessors this package forwards to arrived in 2.4.0. Without the floor an
  older install fails inside the headers with a template error instead of at
  configure time with the version it found.
  ---------
* feat(agnocast_wrapper): accept std::shared_ptr<const MessageT> callbacks (`#1396 <https://github.com/autowarefoundation/autoware_core/issues/1396>`_)
  * feat(agnocast_wrapper): accept std::shared_ptr<const MessageT> callbacks
  For interfaces whose callback signature cannot be templated on the pointer
  type -- notably autoware_component_interface_utils, which binds member
  functions taking Message::ConstSharedPtr.
  The payload is not copied: to_std_shared_ptr() wraps the ipc_shared_ptr in a
  holder and returns an aliasing pointer over it, so the kernel-side reference
  lives as long as the last copy. AgnocastPollingSubscriber::take_data_impl()
  already hand-rolled that construction, so it now shares the helper.
  * refactor(agnocast_wrapper): factor out the subscription callback-shape traits
  Mirrors the named traits the service side already uses, corrects the
  ROS2Subscription comment (the class is the Agnocast build's DDS path, not
  the ENABLE_AGNOCAST=0 build), spells the accepted type as
  std::shared_ptr<const MessageT> in the static_assert, marks
  to_std_shared_ptr as subscriber-side only, and documents the third
  callback shape in the README and the review guide.
  * docs(agnocast_wrapper): trim the ConstSharedPtr callback note
  * docs(agnocast_wrapper): drop the unfounded macro preference
  * docs(agnocast_wrapper): drop the interface qualifier
  * docs(agnocast_wrapper): trim the to_std_shared_ptr doc comment
  * docs(agnocast_wrapper): trim the callback-shape trait comments
  * docs(agnocast_wrapper): state the owning-handle lifetime constraint
  An owning handle must not outlive the subscription that delivered it on the
  Agnocast path. Also corrects the take_data() contract, which claimed the
  returned pointer had the same lifetime semantics in both modes.
  * perf(agnocast_wrapper): register read-only subscription callbacks as shared-const
  rclcpp copies the message to satisfy a unique_ptr callback: on every
  inter-process delivery, and intra-process for every subscription but the
  last. Only AUTOWARE_MESSAGE_UNIQUE_PTR needs ownership.
  * fix(agnocast_wrapper): tighten the shared-const callback trait
  is_invocable_v also accepts parameters that a shared_ptr<const MessageT>
  merely converts to, such as std::weak_ptr. rclcpp rejects those shapes,
  so taking them would compile only at ENABLE_AGNOCAST=1.
  * refactor(agnocast_wrapper): move to_std_shared_ptr into detail
  It takes a raw agnocast::ipc_shared_ptr, which no wrapper API hands out,
  so it has no user-facing use.
  * fix(agnocast_wrapper): make the shared-const callback shape purely additive
  A callback invocable with const MessageT & stays on the const-reference
  branch, so the shape is claimed only by callbacks that no other branch
  accepts.
  * fix(agnocast_wrapper): include message_ptr.hpp where to_std_shared_ptr is used
  * docs(agnocast_wrapper): stop naming a single owning callback form
  * docs(agnocast_wrapper): note the plain ConstSharedPtr callback beside the type spellings
  ---------
  Co-authored-by: atsushi yano <55824710+atsushi421@users.noreply.github.com>
* refactor(autoware_agnocast_wrapper): split monolithic header into per-topic headers (`#1386 <https://github.com/autowarefoundation/autoware_core/issues/1386>`_)
  * refactor(autoware_agnocast_wrapper): move header contents into per-topic files
  * refactor(autoware_agnocast_wrapper): give each split header its own includes
  * docs(autoware_agnocast_wrapper): describe each split header
  * refactor(autoware_agnocast_wrapper): tighten the split headers' includes
  * docs(autoware_agnocast_wrapper): fix up the split headers' descriptions
  ---------
  Co-authored-by: atsushi yano <55824710+atsushi421@users.noreply.github.com>
* docs(autoware_agnocast_wrapper): fix dead link in review guide (`#1365 <https://github.com/autowarefoundation/autoware_core/issues/1365>`_)
* fix(autoware_agnocast_wrapper): declare the dependencies it includes (`#1358 <https://github.com/autowarefoundation/autoware_core/issues/1358>`_)
  fix(autoware_agnocast_wrapper): declare the dependencies it uses
  The package includes a header of rcl, and names rcl_interfaces types in an
  installed header while that header arrives through another dependency. It
  builds today only because rclcpp re-exports them, so a change in an unrelated
  repository can break it without anything here changing.
* docs(autoware_agnocast_wrapper): document undocumented behaviors and APIs (`#1309 <https://github.com/autowarefoundation/autoware_core/issues/1309>`_)
  * docs(autoware_agnocast_wrapper): fix statements that no longer match the implementation
  * docs(autoware_agnocast_wrapper): document undocumented behaviors and APIs
  * docs(autoware_agnocast_wrapper): drop Method 1 polling subscriber checklist item
  ---------
* docs(autoware_agnocast_wrapper): fix statements that no longer match the implementation (`#1308 <https://github.com/autowarefoundation/autoware_core/issues/1308>`_)
* refactor(autoware_agnocast_wrapper): unify message_filters #else namespace to C++17 style (`#1285 <https://github.com/autowarefoundation/autoware_core/issues/1285>`_)
* feat(autoware_agnocast_wrapper): let Node derive from enable_shared_from_this (`#1294 <https://github.com/autowarefoundation/autoware_core/issues/1294>`_)
  Nodes that hand themselves to helpers as a shared_ptr need shared_from_this(), which
  rclcpp::Node provides. The wrapper Node did not, so migrating such a node failed to compile.
  Both the agnocast and the non-agnocast Node need it: neither is an rclcpp::Node alias.
* docs(autoware_agnocast_wrapper): fix dead agnocast link and point docs at agnocast_doc (`#1291 <https://github.com/autowarefoundation/autoware_core/issues/1291>`_)
  * docs(autoware_agnocast_wrapper): fix broken message_filters guide link
  * docs(autoware_agnocast_wrapper): point agnocast links at agnocast_doc
  ---------
* feat(autoware_agnocast_wrapper): support `ok()` (`#1232 <https://github.com/autowarefoundation/autoware_core/issues/1232>`_)
  support ok
* fix(autoware_agnocast_wrapper): parenthesize node in *_ON_NODE macros (`#1284 <https://github.com/autowarefoundation/autoware_core/issues/1284>`_)
* refactor(autoware_agnocast_wrapper): remove old polling subscriber API (`#1283 <https://github.com/autowarefoundation/autoware_core/issues/1283>`_)
* fix(autoware_agnocast_wrapper): build ROS2 polling with upstream autoware_utils (`#1280 <https://github.com/autowarefoundation/autoware_core/issues/1280>`_)
  * fix(autoware_agnocast_wrapper): build ROS2 polling with upstream autoware_utils
  * docs(autoware_agnocast_wrapper): note unused allow_same_message in rclcpp polling
  ---------
* fix(autoware_agnocast_wrapper): include <rclcpp/version.h> for version guards (`#1281 <https://github.com/autowarefoundation/autoware_core/issues/1281>`_)
* feat(autoware_agnocast_wrapper): polling bridge `polling:` namespace (`#1277 <https://github.com/autowarefoundation/autoware_core/issues/1277>`_)
  * polling bridge polling: namespace
  * update README
  * apply fix for allow_same_message
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_agnocast_wrapper): `declare_paramter` return type to auto (`#1278 <https://github.com/autowarefoundation/autoware_core/issues/1278>`_)
  fix declare_paramter return type to auto
* fix(autoware_agnocast_wrapper): expose same `diagnostic_updater `API in in both `ENABLE_AGNOCAST`=0/1 builds (`#1251 <https://github.com/autowarefoundation/autoware_core/issues/1251>`_)
  * fix(autoware_agnocast_wrapper): expose same diagnostic_updater API in both ENABLE_AGNOCAST=0/1 builds
  * restore comments
  * fix copilot review
  * add doxygen comments
  * keep type alias private
  * fix buffer initialization
  ---------
* fix(autoware_agnocast_wrapper): unify Client/Service across `ENABLE_AGNOCAST`=0/1 builds (`#1254 <https://github.com/autowarefoundation/autoware_core/issues/1254>`_)
  * fix(autoware_agnocast_wrapper): unify Client/Service across ENABLE_AGNOCAST=0/1 builds
  * scope this branch down to Client/Service type unification only
  * style(pre-commit): autofix
  * fix comments
  * callback outsde try-catch
  * fix comments based on the reviews
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(autoware_agnocast_wrapper): move set_period/polling_policy after the non-Agnocast macro block (`#1252 <https://github.com/autowarefoundation/autoware_core/issues/1252>`_)
  move set_period/polling_policy after the non-Agnocast macro block
* fix(autoware_agnocast_wrapper): fix for `take_data` (`#1235 <https://github.com/autowarefoundation/autoware_core/issues/1235>`_)
  * fix for take_data
  * delete unnecessary comments
  ---------
* feat(autoware_agnocast_wrapper): support policy as `autoware_utils_rclcpp` for polling subscriber (`#1230 <https://github.com/autowarefoundation/autoware_core/issues/1230>`_)
  * fix for polling subscriber
  * style(pre-commit): autofix
  * assert when All is used and add default qos_depthg
  * style(pre-commit): autofix
  * fix for take_data
  * fix for copilot review
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_agnocast_wrapper): add default `qos_depth` as in `autoware_utils` (`#1231 <https://github.com/autowarefoundation/autoware_core/issues/1231>`_)
  * add default qos_depth as in autoware_utils
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_agnocast_wrapper): support const reference subscription callbacks (`#1228 <https://github.com/autowarefoundation/autoware_core/issues/1228>`_)
  * feat(autoware_agnocast_wrapper): support const reference subscription callbacks
  Accept plain rclcpp-style `const MessageT &` callbacks in
  create_subscription, in addition to AUTOWARE_MESSAGE_UNIQUE_PTR and
  AUTOWARE_MESSAGE_CONST_SHARED_PTR. The subscription dereferences the
  received pointer before invoking the callback, so zero-copy is preserved
  on the Agnocast path (the reference points into shared memory). This lets
  nodes that only read the message inside the callback keep their original
  signature instead of adopting wrapper-specific types.
  Claude-Session: https://claude.ai/code/session_011PvbUwvaBZwXQTGYEs8vqx
  * fix: address review findings
  - Pass the message via std::as_const so generic callbacks cannot mutate
  the shared-memory entry through the const-ref dispatch path
  - Consolidate the duplicated subscription callback trait logic into
  namespace-scope helpers, matching the service-side convention
  - Document the callback-scoped lifetime of the const-ref argument
  Claude-Session: https://claude.ai/code/session_01X9QJop3EYJGEVGPF3zshjN
  * style(pre-commit): autofix
  * refactor: restore inline subscription callback trait expressions
  Revert the namespace-scope trait helper consolidation from the previous
  review-fix commit; keep the std::as_const dispatch fix and the lifetime
  comments unchanged.
  Claude-Session: https://claude.ai/code/session_01X9QJop3EYJGEVGPF3zshjN
  * docs: state the callback-scoped reference lifetime in README
  Claude-Session: https://claude.ai/code/session_01X9QJop3EYJGEVGPF3zshjN
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_agnocast_wrapper): expose the same API in both ENABLE_AGNOCAST=0/1 build (`#1207 <https://github.com/autowarefoundation/autoware_core/issues/1207>`_)
  * expose same api
  * fix doc comments and fix to move
  * style(pre-commit): autofix
  * added comments for non-compatibility of message_filter subscriber
  * align with ENABLE_AGNOCAST=1 block
  * fix cpplint
  * fix clang-tidy
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Koichi Imai, Mete Fatih Cırıt, Tetsuhiro Kawaguchi, Yutaro Kobayashi, atsushi yano, github-actions

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_agnocast_wrapper): split service ptr macros into server/client variants (`#1205 <https://github.com/autowarefoundation/autoware_core/issues/1205>`_)
  * refactor(autoware_agnocast_wrapper): split service ptr macros into
  server/client variants
  * Apply feedback review
  * Remove const from is_shared_ptr_service_callback_v
  ---------
* feat(autoware_agnocast_wrapper): add overload for service (`#1203 <https://github.com/autowarefoundation/autoware_core/issues/1203>`_)
  * add create_service overload
  * add assert
  * fix copilot review
  ---------
* feat(autoware_agnocast_wrapper): add service and client support (`#1074 <https://github.com/autowarefoundation/autoware_core/issues/1074>`_)
  * Add service and client support to agnocast wrapper
  * Mark service and client support as experimental in README
  * Add missing macros
  * Fix colcon flag typo in README
  * fix
  * style(pre-commit): autofix
  * fix
  * style(pre-commit): autofix
  * Add a comment
  * Adjust create_service/create_client calls depending on rclcpp version
  * fix cpplint errors
  * style(pre-commit): autofix
  * Fix type deduction in create_client and create_service
  * Add functional header
  * Fix cpplint errors
  * Remove experimental tag
  * Add RCLCPP version check for client and service macros
  * Propagate exception in client async callbacks
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Koichi Imai <koichi.imai.2@tier4.jp>
  Co-authored-by: Koichi Imai <45482193+Koichi98@users.noreply.github.com>
* feat(autoware_agnocast_wrapper): support more args for synchronizer (`#1104 <https://github.com/autowarefoundation/autoware_core/issues/1104>`_)
  support 8 args
* feat(autoware_agnocast_wrapper): adopt glog to agnocast main templete (`#1180 <https://github.com/autowarefoundation/autoware_core/issues/1180>`_)
  feat(autoware_agnocast_wrapper): use glog (`#88 <https://github.com/autowarefoundation/autoware_core/issues/88>`_)
  * feat: use glog
  * fix: tag name
  * feat: link glog
  * fix
  * fix: add ament auto library
  ---------
* fix(autoware_agnocast_wrapper): get_name,namespace,get_fully_qualified_name (`#1178 <https://github.com/autowarefoundation/autoware_core/issues/1178>`_)
  fix get_name,namespace,get_fully_qualified_name
* feat(autoware_agnocast_wrapper): spawn agnocast_discovery_agent from launch wrapper (`#1084 <https://github.com/autowarefoundation/autoware_core/issues/1084>`_)
  Spawn exactly one agnocast_discovery_agent per ros2 launch tree from
  agnocast_env.launch.{py,xml} when ENABLE_AGNOCAST=1, deduplicated tree-wide via
  the LaunchContext globals and pinned to namespace="/".
* feat(autoware_agnocast_wrapper): add ON_NODE macros (`#1170 <https://github.com/autowarefoundation/autoware_core/issues/1170>`_)
* feat(autoware_agnocast_wrapper): update `agnocast_env.launch` to enable override `use_agnocast` (`#1123 <https://github.com/autowarefoundation/autoware_core/issues/1123>`_)
  * update agnocast_env.launch to override use_agnocast
  * style(pre-commit): autofix
  * more description in README
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Takumi Jin <87105992+ruth561@users.noreply.github.com>
* docs(autoware_agnocast_wrapper): update `autoware_agnocast_wrapper` review_guide.md (`#1121 <https://github.com/autowarefoundation/autoware_core/issues/1121>`_)
  * update agnocast_wrapper review guide
  * add callback arguments migration check
  * fix example code
  ---------
  Co-authored-by: atsushi yano <55824710+atsushi421@users.noreply.github.com>
* feat(autoware_agnocast_wrapper): align message_filters `registerCallback` with upstream API (`#1091 <https://github.com/autowarefoundation/autoware_core/issues/1091>`_)
  * fix to align with upstream API
  * fix
  * fix
  * style(pre-commit): autofix
  * add comments
  * move when capture
  * style(pre-commit): autofix
  * delete redundant comments
  * use unique_ptr
  * fix to use copy-capture for upstream-conn, and add try-catch
  * fix to use rvalue
  * add Note for return value conn
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_agnocast_wrapper): introduce diagnostic_updater API (`#1087 <https://github.com/autowarefoundation/autoware_core/issues/1087>`_)
  * introduce diagnostic_updater
  * fix to use in_place_type
  * update comments and README
  * style(pre-commit): autofix
  * add documentation comments
  * add documentation comment for starting_up_status
  * fix cpplint
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Takumi Jin <87105992+ruth561@users.noreply.github.com>
* feat(autoware_agnocast_wrapper): introduce Timer API (`#1076 <https://github.com/autowarefoundation/autoware_core/issues/1076>`_)
  * introduce timer api
  * added comments about exposing unsupported API through rclcpp Timer
  * add comments for Timer
  * add comments for create_timer and set_period
  * set_period check and add comments for Exception
  * delete const from time_until_trigger and is_canceled
  * unuse is_using_agnocast
  * add README and comments for set_period
  ---------
  Co-authored-by: atsushi yano <55824710+atsushi421@users.noreply.github.com>
  Co-authored-by: Takumi Jin <87105992+ruth561@users.noreply.github.com>
* feat(autoware_agnocast_wrapper): introduce `ExactTime` for message_filter (`#1077 <https://github.com/autowarefoundation/autoware_core/issues/1077>`_)
  * support ExactTime
  * delete duplication
  * update README
  * delete detail namespace
  * fix doxygen
  * add documentatino comments
  * add noexcept
  * add const& for policy
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_agnocast_wrapper): introduce tf2 API (`#1085 <https://github.com/autowarefoundation/autoware_core/issues/1085>`_)
  * introduce tf2
  * style(pre-commit): autofix
  * use AGNOCAST\_*_OPTIONS
  * fix cpplint
  * use in_place_type
  * style(pre-commit): autofix
  * add ignore_local_publication
  * style(pre-commit): autofix
  * add documentation comments
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_agnocast_wrapper): refer to ROS_DISTRO for heaphook_path (`#1037 <https://github.com/autowarefoundation/autoware_core/issues/1037>`_)
  refer to ROS_DISTRO for heaphook_path
* refactor(autoware_agnocast_wrapper): to use `std::variant` in message_filter (`#1086 <https://github.com/autowarefoundation/autoware_core/issues/1086>`_)
  * refactor to use std::variant
  * style(pre-commit): autofix
  * fix
  * fix for default constructor
  * unuse unique_ptr
  * style(pre-commit): autofix
  * unallow move for ApproximateTimeSynchronizer
  * delete is_using_agnocast
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_agnocast_wrapper): fix message_filter subscriber to take agnocast_wrapper::Node (`#1078 <https://github.com/autowarefoundation/autoware_core/issues/1078>`_)
  * fix to take agnocast_wrapper::Node
  * fix copilot review
  * fix half-initialized and add comments
  * add node-side invariant docs
  * add documentation comment
  ---------
  Co-authored-by: atsushi yano <55824710+atsushi421@users.noreply.github.com>
* chore: update `autoware_agnocast_wrapper` maintiners (`#1081 <https://github.com/autowarefoundation/autoware_core/issues/1081>`_)
  * update autoware_agnocast_wrapper maintiners
  * fix email
  ---------
* fix(autoware_agnocast_wrapper): fix message_ptr to respect unique semantics (`#1001 <https://github.com/autowarefoundation/autoware_core/issues/1001>`_)
  * fix(autoware_agnocast_wrapper): fix message_ptr to respect unique semantics
  * Apply feedback review
  * Apply feedback review
  * Partial specialization of agnocast_message/ros2_message by ownership
  ---------
  Co-authored-by: Koichi Imai <45482193+Koichi98@users.noreply.github.com>
* Contributors: Guojun Wu, Keita Morisaki, Koichi Imai, Tetsuhiro Kawaguchi, github-actions

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
