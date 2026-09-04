# autoware_agnocast_wrapper

The purpose of this package is to integrate Agnocast, a zero-copy middleware, into each topic in Autoware with minimal side effects. Agnocast is a library designed to work alongside ROS 2, enabling true zero-copy publish/subscribe communication for all ROS 2 message types, including unsized message types.

- Agnocast Repository: <https://github.com/autowarefoundation/agnocast>
- Agnocast Documentation: <https://autowarefoundation.github.io/agnocast_doc/main/>
- Discussion on Agnocast Integration into Autoware: <https://github.com/orgs/autowarefoundation/discussions/5835>
- [Review Guide for Agnocast Wrapper PRs](docs/review_guide.md)

This package provides macros that wrap functions for publish/subscribe operations and smart pointer types for handling ROS 2 messages. When Autoware is built using the default build command, Agnocast is **not enabled**. However, setting the environment variable `ENABLE_AGNOCAST=1` enables Agnocast and results in a build that includes its integration. This design ensures backward compatibility for users who are unaware of Agnocast, minimizing disruption.

## Two Integration Approaches

This package provides two approaches for integrating Agnocast. Both will coexist for the foreseeable future.

### 1. Node Wrapper (`agnocast_wrapper::Node`)

Use this when you want the **entire node** to transparently switch between `rclcpp::Node` and `agnocast::Node` at runtime. The node wrapper automatically selects the correct underlying implementation based on the `ENABLE_AGNOCAST` environment variable.

`agnocast_wrapper::Node` does **not** publicly derive from `rclcpp::Node`. It exposes a curated subset
of the `rclcpp::Node` surface and forwards each member to the underlying implementation (`rclcpp::Node`
or `agnocast::Node`). The **member names and argument lists are identical in both builds**
(`ENABLE_AGNOCAST=0` and `=1`), so a node written against it compiles unchanged either way — provided
you spell the handle, options and message types with the `AUTOWARE_*` macros (see
[Type spellings](#type-spellings)). If you need an API that is not listed below, extend the wrapper, or
reach the underlying node via `get_rclcpp_node()` (declared in both builds, but it throws when the node
is in Agnocast mode — see the [build-modes table](#build-modes-agnocast-disabled-vs-agnocast-enabled)).

#### Supported API surface

The following members / free functions are provided. Unless noted, signatures mirror their
`rclcpp::Node` counterparts.

| Category        | Members                                                                                                                                                                                                                                                                                                                                                                                                               |
| --------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Construction    | `Node(name, options)`, `Node(name, namespace, options)`, virtual destructor, `SharedPtr`. Non-copyable and non-movable (copying would alias one backend behind two wrappers). Derives from `std::enable_shared_from_this<Node>`, so `shared_from_this()` is available when the node is owned by a `shared_ptr`                                                                                                        |
| Basic info      | `get_name()`, `get_namespace()`, `get_fully_qualified_name()`, `get_logger()`                                                                                                                                                                                                                                                                                                                                         |
| Time            | `get_clock()`, `now()`                                                                                                                                                                                                                                                                                                                                                                                                |
| Node interfaces | `get_node_base_interface()`, `get_node_topics_interface()`, `get_node_parameters_interface()` (partial — only these three)                                                                                                                                                                                                                                                                                            |
| Callback groups | `create_callback_group()`                                                                                                                                                                                                                                                                                                                                                                                             |
| Parameters      | `declare_parameter()` (typed + `ParameterValue`/`ParameterType` overloads), `has_parameter()`, `undeclare_parameter()`, `get_parameter()` / `get_parameters()` (typed + prefix overloads), `set_parameter()` / `set_parameters()` / `set_parameters_atomically()`, `describe_parameter(s)()`, `get_parameter_types()`, `list_parameters()`, `add_on_set_parameters_callback()`, `remove_on_set_parameters_callback()` |
| Publisher       | `create_publisher<MessageT>()` (`QoS` and depth overloads) — see [Publisher API](#publisher-api)                                                                                                                                                                                                                                                                                                                      |
| Subscription    | `create_subscription<MessageT>()` (`QoS` and depth overloads)                                                                                                                                                                                                                                                                                                                                                         |
| Client          | `create_client<ServiceT>()` — takes `rclcpp::QoS` (the wrapper normalizes the Humble vs. Jazzy QoS-argument difference)                                                                                                                                                                                                                                                                                               |
| Service         | `create_service<ServiceT>()` — `message_ptr` callback form and an rclcpp-style `shared_ptr` callback form                                                                                                                                                                                                                                                                                                             |
| Timer           | `create_wall_timer()`; free `create_timer(node, clock, period, cb, group)` and free `set_period(timer, period)` (see [Timer notes](#timer-notes))                                                                                                                                                                                                                                                                     |
| Underlying node | `get_rclcpp_node()`; `get_agnocast_node()` (agnocast-enabled build only — not declared in an agnocast-disabled build, so calling it there is a compile error); free `to_rclcpp_node(node)`                                                                                                                                                                                                                            |
| Context         | free `init()`, `shutdown()` and `ok()` — mode-agnostic replacements for the rclcpp equivalents (see [Context notes](#context-notes))                                                                                                                                                                                                                                                                                  |

> `OnSetParametersCallbackType` is aliased in this namespace and resolves to the correct rclcpp type
> for both Humble (rclcpp 16.x) and Jazzy (rclcpp 28+).

Polling subscribers are **not** a `Node` member. Use the free function
`polling::create_polling_subscriber<MessageT>(node, topic, qos)` — see
[Polling Subscriber](#polling-subscriber-polling-namespace).

#### Type spellings

The member _names_ and argument lists above are the same in both builds, but the handle, options and
message types they use are not the same C++ types. Always spell them with the `AUTOWARE_*` macros so the
same source compiles in both builds:

| What                             | Spell it as                            | `ENABLE_AGNOCAST=0`                  | `ENABLE_AGNOCAST=1`                            |
| -------------------------------- | -------------------------------------- | ------------------------------------ | ---------------------------------------------- |
| `create_publisher` result        | `AUTOWARE_PUBLISHER_PTR(M)`            | `rclcpp::Publisher<M>::SharedPtr`    | `agnocast_wrapper::Publisher<M>::SharedPtr`    |
| `create_subscription` result     | `AUTOWARE_SUBSCRIPTION_PTR(M)`         | `rclcpp::Subscription<M>::SharedPtr` | `agnocast_wrapper::Subscription<M>::SharedPtr` |
| `create_wall_timer` result       | `AUTOWARE_TIMER_PTR`                   | `rclcpp::TimerBase::SharedPtr`       | `agnocast_wrapper::Timer::SharedPtr`           |
| `create_publisher` options arg   | `AUTOWARE_PUBLISHER_OPTIONS`           | `rclcpp::PublisherOptions`           | `agnocast::PublisherOptions`                   |
| `create_subscription` options    | `AUTOWARE_SUBSCRIPTION_OPTIONS`        | `rclcpp::SubscriptionOptions`        | `agnocast::SubscriptionOptions`                |
| Owning subscription callback arg | `AUTOWARE_MESSAGE_CONST_SHARED_PTR(M)` | `std::shared_ptr<const M>`           | `message_ptr<const M, Shared>`                 |

A subscription callback may also take the plain `MessageT::ConstSharedPtr`; it needs no macro because it is spelled the same in both builds.

**On the Agnocast path an owning handle must not outlive the subscription that delivered it.** This covers `AUTOWARE_MESSAGE_CONST_SHARED_PTR`, a callback taking `MessageT::ConstSharedPtr`, and the pointer returned by `polling::take_data()`. Reading it afterwards can return recycled memory, and releasing it can abort the process. Members are destroyed in reverse declaration order, so declare the subscription **before** any member that caches a message:

```cpp
AUTOWARE_SUBSCRIPTION_PTR(PointCloud2) sub_;   // declared first -> destroyed last
std::shared_ptr<const PointCloud2> latest_;    // destroyed first -> safe
```

The DDS path lets the same pointer be held indefinitely, so a node validated only with `ENABLE_AGNOCAST=0` will not show the problem.

`AUTOWARE_CLIENT_PTR(S)` / `AUTOWARE_SERVICE_PTR(S)` and the `AUTOWARE_CLIENT_*FUTURE*` macros resolve to
the wrapper's own `Client<S>` / `Service<S>` types in **both** builds, so client and service code needs no
per-build spelling. See [Key Macros](docs/review_guide.md#3-key-macros) for the full macro list.

Publisher, subscription, client and service handles carry the same read-back accessors in both builds: `get_topic_name()` and `get_actual_qos()` on a publisher or a subscription, `get_service_name()` on a client or a service. The polling subscriber carries `get_topic_name()` but not `get_actual_qos()`. `get_actual_qos()` is the one whose meaning differs: on the Agnocast path it reports the QoS as requested, not RMW-resolved.

On rclcpp 21 (Iron) and newer, a client or service handle also carries `configure_introspection(clock, qos, state)`, which forwards to the `rclcpp` or the Agnocast counterpart. That lets utilities written against `rclcpp::Node` — `autoware_component_interface_utils` in particular — enable ROS 2 service introspection on a handle without knowing which backend is behind it. It is **not declared** on Humble (rclcpp 16), which has no service introspection at all, so gate any call on `RCLCPP_VERSION_MAJOR >= 21` as the wrapper does. The two backends do not fail alike. A null clock throws `std::invalid_argument` on the Agnocast path, whatever the state asked for, while the `rclcpp` path dereferences it unchecked. The clock and the QoS are only consumed where the event publisher is created, on a transition out of `RCL_SERVICE_INTROSPECTION_OFF`: only there does the Agnocast path reject a QoS it cannot use (`std::invalid_argument`) or fail to load the event typesupport (`std::runtime_error`, once per handle), and a call that only moves between `METADATA` and `CONTENTS`, or that turns introspection off, looks at neither.

#### Build modes: agnocast-disabled vs agnocast-enabled

Which of the two `Node` **class definitions** is compiled is a **build-time** choice, selected by the
`USE_AGNOCAST_ENABLED` preprocessor macro. The API surface above is identical in both, so this choice
only affects the backend and the underlying-node accessors below.

This is a separate axis from the **runtime backend selection**: in the agnocast-enabled build, each
node _instance_ additionally picks `rclcpp::Node` vs `agnocast::Node` at construction from the
`ENABLE_AGNOCAST` environment variable read at runtime (`use_agnocast()`), fixed for the node's
lifetime. The runtime value selects the backend; it does _not_ change which `Node` definition was
compiled or which methods are declared — e.g. `get_agnocast_node()` is declared in every
agnocast-enabled build and instead throws at runtime when the node is not in Agnocast mode.

|                        | Agnocast-disabled build<br>(`USE_AGNOCAST_ENABLED` undefined) | Agnocast-enabled build<br>(`USE_AGNOCAST_ENABLED` defined)                                                                             |
| ---------------------- | ------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| Backend                | Always an owned `rclcpp::Node`.                               | `rclcpp::Node` or `agnocast::Node`, chosen at construction from the runtime `ENABLE_AGNOCAST` value and fixed for the node's lifetime. |
| `get_rclcpp_node()`    | Always returns the owned node.                                | Declared; returns the `rclcpp::Node`, but **throws** `std::runtime_error` if the node is in Agnocast mode.                             |
| `get_agnocast_node()`  | **Not declared** — calling it is a compile error.             | Declared regardless of the runtime backend; returns the `agnocast::Node`, but **throws** if the node is not in Agnocast mode.          |
| `to_rclcpp_node(node)` | Always succeeds.                                              | Forwards to `get_rclcpp_node()` (same throw condition).                                                                                |

In both builds `agnocast_wrapper::Node` does not derive from `rclcpp::Node`, so hand it to an executor or
utility via `get_node_base_interface()` (e.g. `executor.add_node(node->get_node_base_interface())`).

```cpp
#include <autoware/agnocast_wrapper/node.hpp>

class MyNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit MyNode(const rclcpp::NodeOptions & options)
  : Node("my_node", options)
  {
    pub_ = create_publisher<std_msgs::msg::String>("output", 10);
    sub_ = create_subscription<std_msgs::msg::String>(
      "input", 10,
      [this](AUTOWARE_MESSAGE_CONST_SHARED_PTR(std_msgs::msg::String) && msg) { /* ... */ });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() { /* ... */ });
  }

private:
  AUTOWARE_PUBLISHER_PTR(std_msgs::msg::String) pub_;
  AUTOWARE_SUBSCRIPTION_PTR(std_msgs::msg::String) sub_;
  AUTOWARE_TIMER_PTR timer_;
};
```

#### Timer notes

`create_timer()` is provided as a **free function** (not a member) because `rclcpp::Node::create_timer` was added in Jazzy and does not exist on Humble. The free form is portable across both:

```cpp
timer_ = autoware::agnocast_wrapper::create_timer(
  this, this->get_clock(), rclcpp::Duration::from_seconds(0.1), [this]() { /* ... */ });
```

`set_period()` is likewise a **free function**. `rclcpp::TimerBase` has no `set_period` member, so the free form is the only portable spelling across both builds:

```cpp
autoware::agnocast_wrapper::set_period(timer_, std::chrono::milliseconds(200));
```

#### Context notes

An AgnocastOnly executable brings up the agnocast context; every other executable brings up the rclcpp
one. `ok()` reports whichever is alive.

Use `init()` / `shutdown()` / `ok()` from this namespace rather than the `rclcpp` equivalents. `ok()`
matters even if you never call `init()` yourself: in an AgnocastOnly executable `rclcpp::ok()` reports
`false` while the process is healthy. `shutdown()` takes no argument — it tears down whatever `init()`
brought up.

`init()`'s `agnocast_only` flag is a property of the executable, not of the environment. Pass `true` if
and only if the main spins one of agnocast's `AgnocastOnly*` executors:

| Your `main()`                                                              | `agnocast_only` |
| -------------------------------------------------------------------------- | --------------- |
| No Agnocast executor at all (a test main, a tool, an rclcpp-only node)     | omit it         |
| A non-AgnocastOnly Agnocast executor (`SingleThreadedAgnocastExecutor`, …) | omit it         |
| An `AgnocastOnly*` executor                                                | `true`          |

`true` selects the agnocast context only when `ENABLE_AGNOCAST` is 1, so a main that passes it needs
an rclcpp executor to fall back to when Agnocast is off.
`autoware_agnocast_wrapper_register_node()` fills the flag in and generates that fallback; a main that
has to serve both modes belongs to that macro rather than being hand-written.

At `ENABLE_AGNOCAST=1` a node deriving from `agnocast_wrapper::Node` needs the agnocast context even
when the executable does not bring it up, because it spins agnocast-only executors internally — for the
`use_sim_time` clock thread, and for a tf listener with `spin_thread`. Until
[agnocast#1517](https://github.com/autowarefoundation/agnocast/pull/1517) brings that context up
lazily, such a node aborts at construction unless the executable is registered with an `AgnocastOnly*`
executor.

#### Publisher API

A wrapper publisher exposes three `publish()` overloads, all supported in both builds:

| Call                                                                   | Behavior                                                       |
| ---------------------------------------------------------------------- | -------------------------------------------------------------- |
| `ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub)` → `pub->publish(std::move(msg))` | Zero-copy: the message is built in place in shared memory.     |
| `ALLOCATE_OUTPUT_MESSAGE_SHARED(pub)` → `pub->publish(std::move(msg))` | Same, for the shared-ownership form.                           |
| `pub->publish(msg)` (`const MessageT &`)                               | Copies `msg` into a freshly allocated message, then publishes. |

Prefer the allocate-then-move form when you are constructing the outgoing message anyway; the
`const MessageT &` overload suits a message you already hold and must keep.

#### CMake setup

To use the Node wrapper in your package, add the following to your `CMakeLists.txt`:

```cmake
find_package(autoware_agnocast_wrapper REQUIRED)
ament_target_dependencies(my_node_component autoware_agnocast_wrapper)
autoware_agnocast_wrapper_setup(my_node_component)
```

`autoware_agnocast_wrapper_setup()` is required: it defines `USE_AGNOCAST_ENABLED` on the target, which
`ament_target_dependencies()` does not propagate. Apply it to every target that includes a wrapper header;
`autoware_agnocast_wrapper_register_node()` does it for the targets it handles.

#### Registering a Node with `autoware_agnocast_wrapper_register_node`

Instead of calling `rclcpp_components_register_node` directly, use the `autoware_agnocast_wrapper_register_node` macro to register your component node. This macro:

1. Registers the component with `rclcpp_components` (for component container support)
2. Creates a standalone executable that can switch between `rclcpp::Node` and `agnocast::Node` at runtime based on the `ENABLE_AGNOCAST` environment variable

When `ENABLE_AGNOCAST` is not set or set to `0`, this macro falls back to standard `rclcpp_components_register_node` behavior.

```cmake
find_package(autoware_agnocast_wrapper REQUIRED)

ament_auto_add_library(my_node_component SHARED src/my_node.cpp)
ament_target_dependencies(my_node_component autoware_agnocast_wrapper)

autoware_agnocast_wrapper_register_node(my_node_component
  PLUGIN "my_package::MyNode"
  EXECUTABLE my_node
)
```

**Parameters:**

| Parameter           | Required | Default                          | Description                                         |
| ------------------- | -------- | -------------------------------- | --------------------------------------------------- |
| `PLUGIN`            | Yes      | -                                | Fully qualified class name of the component         |
| `EXECUTABLE`        | Yes      | -                                | Executable name for the node                        |
| `ROS2_EXECUTOR`     | No       | `SingleThreadedExecutor`         | Executor to use when `ENABLE_AGNOCAST=0` at runtime |
| `AGNOCAST_EXECUTOR` | No       | `SingleThreadedAgnocastExecutor` | Executor to use when `ENABLE_AGNOCAST=1` at runtime |

**Valid executor values:**

- `ROS2_EXECUTOR`: `SingleThreadedExecutor`, `MultiThreadedExecutor`
- `AGNOCAST_EXECUTOR`: `SingleThreadedAgnocastExecutor`, `MultiThreadedAgnocastExecutor`, `CallbackIsolatedAgnocastExecutor`, `AgnocastOnlySingleThreadedExecutor`, `AgnocastOnlyMultiThreadedExecutor`, `AgnocastOnlyCallbackIsolatedExecutor`

**Node class requirements:**

The required PLUGIN base class depends on the `AGNOCAST_EXECUTOR` type. The generated template enforces this via `if constexpr` at compile time:

| AGNOCAST_EXECUTOR         | Required PLUGIN base class          |
| ------------------------- | ----------------------------------- |
| `AgnocastOnly*` executors | `autoware::agnocast_wrapper::Node`  |
| Other agnocast executors  | Any `rclcpp::Node`-compatible class |

Non-`AgnocastOnly` executors use `NodeInstanceWrapper::get_node_base_interface()` directly, which works with any node type (`rclcpp::Node`, `agnocast_wrapper::Node`, etc.) without requiring a cast. `AgnocastOnly` executors require `get_agnocast_node()`, which is only available on `autoware::agnocast_wrapper::Node`.

**Behavior reference:**

The tables below show the complete behavior for each configuration. When `ENABLE_AGNOCAST=0` at build time, only `ROS2_EXECUTOR` matters. When `ENABLE_AGNOCAST=1`, the behavior depends on both `ROS2_EXECUTOR` and `AGNOCAST_EXECUTOR`, and can be switched at runtime via the `ENABLE_AGNOCAST` environment variable.

Build-time `ENABLE_AGNOCAST=0` (or unset):

| ROS2_EXECUTOR            | Runtime behavior         |
| ------------------------ | ------------------------ |
| `SingleThreadedExecutor` | `SingleThreadedExecutor` |
| `MultiThreadedExecutor`  | `MultiThreadedExecutor`  |

Runtime `ENABLE_AGNOCAST` has no effect in this mode — no switchable template is generated.

Build-time `ENABLE_AGNOCAST=1`:

| ROS 2<br>\_EXECUTOR      | AGNOCAST<br>\_EXECUTOR                 | Runtime<br>`ENABLE_AGNOCAST=0` | Runtime<br>`ENABLE_AGNOCAST=1`         |
| ------------------------ | -------------------------------------- | ------------------------------ | -------------------------------------- |
| `SingleThreadedExecutor` | `SingleThreadedAgnocastExecutor`       | `SingleThreadedExecutor`       | `SingleThreadedAgnocastExecutor`       |
| `MultiThreadedExecutor`  | `MultiThreadedAgnocastExecutor`        | `MultiThreadedExecutor`        | `MultiThreadedAgnocastExecutor`        |
| `MultiThreadedExecutor`  | `CallbackIsolatedAgnocastExecutor`     | `MultiThreadedExecutor`        | `CallbackIsolatedAgnocastExecutor`     |
| `SingleThreadedExecutor` | `AgnocastOnlySingleThreadedExecutor`   | `SingleThreadedExecutor`       | `AgnocastOnlySingleThreadedExecutor`   |
| `MultiThreadedExecutor`  | `AgnocastOnlyMultiThreadedExecutor`    | `MultiThreadedExecutor`        | `AgnocastOnlyMultiThreadedExecutor`    |
| `MultiThreadedExecutor`  | `AgnocastOnlyCallbackIsolatedExecutor` | `MultiThreadedExecutor`        | `AgnocastOnlyCallbackIsolatedExecutor` |
| `SingleThreadedExecutor` | `MultiThreadedAgnocastExecutor`        | `SingleThreadedExecutor`       | `MultiThreadedAgnocastExecutor`        |
| `SingleThreadedExecutor` | `CallbackIsolatedAgnocastExecutor`     | `SingleThreadedExecutor`       | `CallbackIsolatedAgnocastExecutor`     |
| `SingleThreadedExecutor` | `AgnocastOnlyMultiThreadedExecutor`    | `SingleThreadedExecutor`       | `AgnocastOnlyMultiThreadedExecutor`    |
| `SingleThreadedExecutor` | `AgnocastOnlyCallbackIsolatedExecutor` | `SingleThreadedExecutor`       | `AgnocastOnlyCallbackIsolatedExecutor` |
| `MultiThreadedExecutor`  | `SingleThreadedAgnocastExecutor`       | `MultiThreadedExecutor`        | `SingleThreadedAgnocastExecutor`       |
| `MultiThreadedExecutor`  | `AgnocastOnlySingleThreadedExecutor`   | `MultiThreadedExecutor`        | `AgnocastOnlySingleThreadedExecutor`   |

Example with `agnocast_wrapper::Node` (AgnocastOnly executor):

```cmake
autoware_agnocast_wrapper_register_node(my_node_component
  PLUGIN "my_package::MyNode"
  EXECUTABLE my_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)
```

Example with `rclcpp::Node` (no node changes required):

```cmake
autoware_agnocast_wrapper_register_node(my_node_component
  PLUGIN "my_package::MyNode"
  EXECUTABLE my_node
  AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor
)
```

### 2. Macro + Free Function API

Use this when only **specific topics** need Agnocast on an existing `rclcpp::Node`, without converting the entire node to `agnocast_wrapper::Node`.

You can immediately understand how to use the macros just by looking at `macros.hpp`. A typical callback and publisher setup looks like this:

```cpp
#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>

pub_output_ = AUTOWARE_CREATE_PUBLISHER3(
  PointCloud2,
  "output",
  rclcpp::SensorDataQoS().keep_last(max_queue_size_),
  pub_options
);

void onPointCloud(AUTOWARE_MESSAGE_UNIQUE_PTR(const PointCloud2) && input_msg) {
  auto output = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_output_);
  ...
  pub_output_->publish(std::move(output));
}
```

Subscription callbacks that only read the message inside the callback can also keep the plain rclcpp `const MessageT &` signature:

```cpp
void onPointCloud(const PointCloud2 & input_msg) {
  ...
}
```

Zero-copy is preserved on the Agnocast path: the subscription dereferences the received pointer before invoking the callback, so the reference points directly into shared memory. The referenced entry is kept alive only while the callback runs: the reference is valid for the duration of the callback and must not be stored or used after the callback returns. Use `AUTOWARE_MESSAGE_CONST_SHARED_PTR` instead when the callback needs to keep the message alive beyond the callback without a copy.

A callback may also take the plain rclcpp `MessageT::ConstSharedPtr`:

```cpp
void onPointCloud(const PointCloud2::ConstSharedPtr input_msg) {
  ...
}
```

The payload is not copied here either, and the pointer may be kept alive beyond the callback, at the same cost as `AUTOWARE_MESSAGE_CONST_SHARED_PTR` — one heap allocation per message, and copies of the pointer are free.

To use the macros provided by this package in your own package, include the following lines in your `CMakeLists.txt`:

```cmake
find_package(autoware_agnocast_wrapper REQUIRED)
ament_target_dependencies(target autoware_agnocast_wrapper)
target_include_directories(target PRIVATE ${autoware_agnocast_wrapper_INCLUDE_DIRS})
autoware_agnocast_wrapper_setup(target)
```

## Message Filters Support

This package provides wrapper types for `message_filters` (`Subscriber`, `Synchronizer`) in the `autoware::agnocast_wrapper::message_filters` namespace. These wrappers transparently switch between `::message_filters` and `agnocast::message_filters` at runtime.

### Current limitations

- Only `ApproximateTime` and `ExactTime` synchronization policies are supported.
- 2 to 8 message types per `Synchronizer`. Upstream `message_filters` supports up to 9, but the registration path this wrapper uses caps it at 8.
- `connectInput()` is not supported; pass `Subscriber` references at construction time.
- `Subscriber::subscribe()` (and the topic-taking constructor) takes an `autoware::agnocast_wrapper::Node *`, so this wrapper requires a Method 2 node. There is no overload for a plain `rclcpp::Node`.

### Usage example

```cpp
#include <autoware/agnocast_wrapper/message_filters.hpp>

using namespace autoware::agnocast_wrapper::message_filters;

// 1. Create subscribers
Subscriber<sensor_msgs::msg::Image> image_sub;
Subscriber<sensor_msgs::msg::CameraInfo> info_sub;
image_sub.subscribe(node, "/camera/image", rmw_qos_profile_sensor_data);
info_sub.subscribe(node, "/camera/info", rmw_qos_profile_sensor_data);

// 2. Create synchronizer
using Policy = sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
Synchronizer<Policy> sync(Policy(10), image_sub, info_sub);

// 3. Register callback. Mirrors `::message_filters::Synchronizer::registerCallback` —
//    pass a member-function pointer and `this`, or a `std::bind` result, or any other
//    callable convertible to `void(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(M0) &,
//                                    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(M1) &)`.
//    Returns a `::message_filters::Connection` for later `.disconnect()`.
auto conn = sync.registerCallback(&MyNode::onSynchronized, this);
// Note: `conn` going out of scope does NOT unregister the callback.
// Call conn.disconnect() explicitly if you need to remove it later.
// Equivalent form (still supported):
// sync.registerCallback(std::bind(
//   &MyNode::onSynchronized, this, std::placeholders::_1, std::placeholders::_2));
```

The callback method signature should use `const` references:

```cpp
void onSynchronized(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::Image) & img,
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::CameraInfo) & info);
```

### Migration guide (from `::message_filters`)

| Before                                                    | After                                                                                 |
| --------------------------------------------------------- | ------------------------------------------------------------------------------------- |
| `#include <message_filters/subscriber.h>`                 | `#include <autoware/agnocast_wrapper/message_filters.hpp>`                            |
| `message_filters::Subscriber<M>`                          | `autoware::agnocast_wrapper::message_filters::Subscriber<M>`                          |
| `message_filters::Synchronizer<Policy>`                   | `autoware::agnocast_wrapper::message_filters::Synchronizer<Policy>`                   |
| `message_filters::sync_policies::ApproximateTime<M0, M1>` | `autoware::agnocast_wrapper::message_filters::sync_policies::ApproximateTime<M0, M1>` |
| `message_filters::sync_policies::ExactTime<M0, M1>`       | `autoware::agnocast_wrapper::message_filters::sync_policies::ExactTime<M0, M1>`       |

## tf2 Support

This package provides wrapper types for tf2 (`TransformListener`, `TransformBroadcaster`, `StaticTransformBroadcaster`, `Buffer`) in the `autoware::agnocast_wrapper` namespace. The listener and broadcasters transparently switch between their `tf2_ros` and `agnocast` implementations at runtime, depending on whether the given node is running in Agnocast mode.

The node-taking constructors require a Method 2 node (`autoware::agnocast_wrapper::Node`). This is needed because an AgnocastOnly executor does not spin a plain `tf2_ros::TransformListener` (a ROS 2 subscription); routing `/tf` through Agnocast keeps tf callbacks firing.

All four wrapper types are non-copyable and non-movable (the backend is chosen at construction and bound by reference), so hold them by value or in a `unique_ptr`.

`Buffer` aliases to `agnocast::Buffer` in Agnocast-enabled builds and `tf2_ros::Buffer` otherwise. The agnocast variant intentionally omits APIs that would silently break under an AgnocastOnly executor (currently `waitForTransform` / `setCreateTimerInterface` and the `/tf2_frames` debug service), so misuse is caught at compile time.

### Usage example

```cpp
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/tf2.hpp>

class MyNode : public autoware::agnocast_wrapper::Node
{
public:
  MyNode()
  : autoware::agnocast_wrapper::Node("my_node"), tf_buffer_(this->get_clock())
  {
    // `*this` is a node derived from autoware::agnocast_wrapper::Node.
    tf_listener_ = std::make_unique<autoware::agnocast_wrapper::TransformListener>(
      tf_buffer_, *this);
    tf_broadcaster_ = std::make_unique<autoware::agnocast_wrapper::TransformBroadcaster>(*this);
  }

private:
  autoware::agnocast_wrapper::Buffer tf_buffer_;
  std::unique_ptr<autoware::agnocast_wrapper::TransformListener> tf_listener_;
  std::unique_ptr<autoware::agnocast_wrapper::TransformBroadcaster> tf_broadcaster_;
};
```

### Migration guide (from `tf2_ros`)

| Before                                                | After                                                    |
| ----------------------------------------------------- | -------------------------------------------------------- |
| `#include <tf2_ros/transform_listener.hpp>`           | `#include <autoware/agnocast_wrapper/tf2.hpp>`           |
| `#include <tf2_ros/buffer.hpp>`                       | `#include <autoware/agnocast_wrapper/tf2.hpp>`           |
| `#include <tf2_ros/transform_broadcaster.hpp>`        | `#include <autoware/agnocast_wrapper/tf2.hpp>`           |
| `#include <tf2_ros/static_transform_broadcaster.hpp>` | `#include <autoware/agnocast_wrapper/tf2.hpp>`           |
| `tf2_ros::TransformListener`                          | `autoware::agnocast_wrapper::TransformListener`          |
| `tf2_ros::Buffer`                                     | `autoware::agnocast_wrapper::Buffer`                     |
| `tf2_ros::TransformBroadcaster`                       | `autoware::agnocast_wrapper::TransformBroadcaster`       |
| `tf2_ros::StaticTransformBroadcaster`                 | `autoware::agnocast_wrapper::StaticTransformBroadcaster` |

## Diagnostic Updater Support

This package provides a wrapper `autoware::agnocast_wrapper::diagnostic_updater::Updater` for `diagnostic_updater::Updater`. The wrapper transparently switches between `diagnostic_updater::Updater` and `agnocast::Updater` at runtime, so nodes inheriting from `autoware::agnocast_wrapper::Node` can use the same idiom in both modes.

The `diagnostic_updater.period` and `diagnostic_updater.use_fqn` parameters are declared identically in both modes, so behavior remains consistent.

### Current limitations

- Only the `Updater(autoware::agnocast_wrapper::Node*, double)` constructor is supported. The upstream interface-pointer constructor and `Updater(NodeT, double)` template overload are intentionally hidden in both modes, so source code stays portable between agnocast-enabled and disabled builds.
- The wrapper does **not** inherit from `DiagnosticTaskVector`, so `getTasks()` is not available.
- The wrapper is non-copyable and non-movable; `verbose_` is bound by reference to the underlying impl.

### Usage example

```cpp
#include <autoware/agnocast_wrapper/diagnostic_updater.hpp>

class MyNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit MyNode(const rclcpp::NodeOptions & options)
  : Node("my_node", options), updater_(this)
  {
    updater_.setHardwareID("my_hardware");
    updater_.add("status", this, &MyNode::diagnose);
  }

private:
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & stat) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "running");
  }

  autoware::agnocast_wrapper::diagnostic_updater::Updater updater_;
};
```

### Migration guide (from `diagnostic_updater::Updater`)

| Before                                                 | After                                                                     |
| ------------------------------------------------------ | ------------------------------------------------------------------------- |
| `#include <diagnostic_updater/diagnostic_updater.hpp>` | `#include <autoware/agnocast_wrapper/diagnostic_updater.hpp>`             |
| `diagnostic_updater::Updater updater_{this};`          | `autoware::agnocast_wrapper::diagnostic_updater::Updater updater_{this};` |

The `add()` / `removeByName()` / `setHardwareID()` / `setHardwareIDf()` / `broadcast()` / `force_update()` / `setPeriod()` / `getPeriod()` APIs and the `verbose_` field behave the same as the upstream `diagnostic_updater::Updater`.

> **Note:** `DiagnosticTask` subclasses (e.g. `FrequencyStatus`, `TimeStampStatus`, `Heartbeat`) defined in `diagnostic_updater` can be added via `updater_.add(task)` unchanged.

## Polling Subscriber (`polling::` namespace)

`autoware::agnocast_wrapper::polling::create_polling_subscriber<MessageT>(node, topic, qos)` creates a polling (take-based) subscriber whose `take_data()` returns a plain `std::shared_ptr<const MessageT>` in **both** `ENABLE_AGNOCAST` modes. In agnocast mode the message stays in shared memory and is aliased into the returned `shared_ptr` (zero-copy, no payload copy); in rclcpp mode it reuses `autoware_utils_rclcpp::InterProcessPollingSubscriber`.

### `take_data()` contract

- Returns the latest message, or `nullptr` when none is available.
- Re-delivery is governed by the **policy tag** and is identical across backends:
  - `polling_policy::Latest` (default): re-delivers the cached message.
  - `polling_policy::Newest`: returns `nullptr` until a new message arrives.
- `polling_policy::All` is rejected at compile time (`take_data()` returns a single message, not a vector).
- The QoS history depth must be 1; `create_polling_subscriber()` throws `std::invalid_argument` otherwise. A deeper queue makes `take_data()` lag behind the newest message, and depth 0 is not delivered at all in agnocast mode.
- The returned `std::shared_ptr` may be held across cycles, but in agnocast mode it must not outlive the polling subscriber (see [Type spellings](#type-spellings)).

### Usage example

```cpp
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>

class MyNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit MyNode(const rclcpp::NodeOptions & options) : Node("my_node", options)
  {
    namespace polling = autoware::agnocast_wrapper::polling;
    sub_ = polling::create_polling_subscriber<nav_msgs::msg::Odometry>(this, "~/input/odometry", 1);
  }

  void on_timer()
  {
    const std::shared_ptr<const nav_msgs::msg::Odometry> msg = sub_->take_data();
    if (!msg) {
      return;
    }
    // use msg->...
  }

private:
  autoware::agnocast_wrapper::polling::PollingSubscriber<nav_msgs::msg::Odometry>::SharedPtr sub_;
};
```

## How to Enable/Disable Agnocast on Build

To build Autoware **with** Agnocast:

```bash
export ENABLE_AGNOCAST=1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

To build Autoware **without** Agnocast (default behavior):

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

To explicitly **disable** Agnocast when it has been previously enabled:

```bash
unset ENABLE_AGNOCAST
# or
export ENABLE_AGNOCAST=0
```

To rebuild a specific package **without** Agnocast after it was previously built with Agnocast:

```bash
rm -Rf ./install/<package_name> ./build/<package_name>
export ENABLE_AGNOCAST=0
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select <package_name>
```

To rebuild a specific package **with** Agnocast after it was previously built without it:

```bash
rm -Rf ./install/<package_name> ./build/<package_name>
export ENABLE_AGNOCAST=1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select <package_name>
```

Please note that the `ENABLE_AGNOCAST` environment variable may not behave as expected in the following scenario:

- Package A depends on build artifacts from Package B
- Both A and B were previously built with Agnocast **enabled**
- Rebuilding only Package A with `ENABLE_AGNOCAST=0` will not be sufficient, as compile options enabling Agnocast may propagate from Package B

Example:

- A = `autoware_occupancy_grid_map_outlier_filter`
- B = `autoware_pointcloud_preprocessor`

In such cases, rebuild both A and B with Agnocast **disabled** to ensure consistency. As a best practice, we recommend keeping the value of `ENABLE_AGNOCAST` consistent within a workspace to avoid unintentional mismatches and simplify build management.

## How to Enable Agnocast at Runtime

When Agnocast is enabled at build time, the heaphook shared library must be preloaded at runtime via `LD_PRELOAD`, and component containers must be replaced with their Agnocast equivalents. This package provides `agnocast_env.launch.xml` (and its Python equivalent `agnocast_env.launch.py`) which handles both of these concerns based on the `ENABLE_AGNOCAST` environment variable.

### Discovery agent

Including `agnocast_env.launch.xml` / `.py` also launches the `agnocast_discovery_agent` node
automatically, once per launch tree and only when `ENABLE_AGNOCAST=1`. It publishes the local Agnocast
state on `/_agnocast_discovery`, which the `ros2 topic list_agnocast` / `info_agnocast` / `hz_agnocast`
commands rely on.

### Provided Variables

After including `agnocast_env.launch.xml` (or `agnocast_env.launch.py`), the following variables are available (in Python launch files, reference them via `LaunchConfiguration`):

| Variable               | Description                                                                              |
| ---------------------- | ---------------------------------------------------------------------------------------- |
| `ld_preload_value`     | `LD_PRELOAD` value with the heaphook library prepended (when Agnocast is enabled)        |
| `container_package`    | Resolved component container package name (`rclcpp_components` or `agnocast_components`) |
| `container_executable` | Resolved component container executable name                                             |

### Launch Arguments

| Argument                 | Default                                                                     | Description                                                                                             |
| ------------------------ | --------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------- |
| `agnocast_heaphook_path` | `/opt/ros/$ROS_DISTRO/lib/libagnocast_heaphook.so` (falls back to `humble`) | Path to the heaphook shared library                                                                     |
| `use_multithread`        | `false`                                                                     | Use the multi-threaded component container (`component_container_mt`)                                   |
| `use_agnocast`           | `$(env ENABLE_AGNOCAST 0)`                                                  | Per-node override (`1`/`0`). Usually left unset; defaults to the `ENABLE_AGNOCAST` environment variable |

The `container_executable` is resolved as follows:

| `use_multithread` | `ENABLE_AGNOCAST=0`      | `ENABLE_AGNOCAST=1`                |
| ----------------- | ------------------------ | ---------------------------------- |
| `false`           | `component_container`    | `agnocast_component_container`     |
| `true`            | `component_container_mt` | `agnocast_component_container_cie` |

### Examples (XML)

Basic usage with a single node:

```xml
<include file="$(find-pkg-share autoware_agnocast_wrapper)/launch/agnocast_env.launch.xml"/>

<node pkg="my_package" exec="my_node" name="my_node">
  <env name="LD_PRELOAD" value="$(var ld_preload_value)"/>
</node>
```

Using a component container with multi-threading:

```xml
<include file="$(find-pkg-share autoware_agnocast_wrapper)/launch/agnocast_env.launch.xml">
  <arg name="use_multithread" value="true"/>
</include>

<node_container pkg="$(var container_package)" exec="$(var container_executable)" name="my_container">
  <env name="LD_PRELOAD" value="$(var ld_preload_value)"/>
</node_container>
```

Disabling Agnocast for a single include (debugging / emergency fallback):

```xml
<include file="$(find-pkg-share autoware_agnocast_wrapper)/launch/agnocast_env.launch.xml">
  <arg name="use_agnocast" value="0"/>
</include>

<node pkg="my_package" exec="my_node" name="my_node">
  <env name="LD_PRELOAD" value="$(var ld_preload_value)"/>
</node>
```

Even when the workspace is built with `ENABLE_AGNOCAST=1`, passing `use_agnocast` to a single include forces that node (or container) back to the plain `rclcpp` path without touching the rest of the launch tree. Use it to temporarily disable Agnocast for one node while debugging, or as an emergency fallback when a specific node misbehaves under Agnocast.

### Examples (Python)

A Python launch file (`agnocast_env.launch.py`) is also provided with the same functionality. It accepts the same `use_agnocast` argument and sets the same launch configurations (`ld_preload_value`, `container_package`, `container_executable`) that can be referenced via `LaunchConfiguration`.

Basic usage with a single node:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    agnocast_env = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("autoware_agnocast_wrapper"),
                "launch",
                "agnocast_env.launch.py",
            ])
        ),
    )

    my_node = Node(
        package="my_package",
        executable="my_node",
        name="my_node",
        additional_env={"LD_PRELOAD": LaunchConfiguration("ld_preload_value")},
    )

    return LaunchDescription([agnocast_env, my_node])
```

Using a component container with multi-threading:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    agnocast_env = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("autoware_agnocast_wrapper"),
                "launch",
                "agnocast_env.launch.py",
            ])
        ),
        launch_arguments={"use_multithread": "true"}.items(),
    )

    container = ComposableNodeContainer(
        name="my_container",
        namespace="",
        package=LaunchConfiguration("container_package"),
        executable=LaunchConfiguration("container_executable"),
        additional_env={"LD_PRELOAD": LaunchConfiguration("ld_preload_value")},
        composable_node_descriptions=[],
    )

    return LaunchDescription([agnocast_env, container])
```

The same `use_agnocast` override works here too, via `launch_arguments={"use_agnocast": "0"}.items()`.

This ensures that only the intended nodes receive the heaphook, rather than all nodes in the launch tree.
