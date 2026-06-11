# autoware_agnocast_wrapper

The purpose of this package is to integrate Agnocast, a zero-copy middleware, into each topic in Autoware with minimal side effects. Agnocast is a library designed to work alongside ROS 2, enabling true zero-copy publish/subscribe communication for all ROS 2 message types, including unsized message types.

- Agnocast Repository: <https://github.com/tier4/agnocast>
- Discussion on Agnocast Integration into Autoware: <https://github.com/orgs/autowarefoundation/discussions/5835>
- [Review Guide for Agnocast Wrapper PRs](docs/review_guide.md)

This package provides macros that wrap functions for publish/subscribe operations and smart pointer types for handling ROS 2 messages. When Autoware is built using the default build command, Agnocast is **not enabled**. However, setting the environment variable `ENABLE_AGNOCAST=1` enables Agnocast and results in a build that includes its integration. This design ensures backward compatibility for users who are unaware of Agnocast, minimizing disruption.

## Two Integration Approaches

This package provides two approaches for integrating Agnocast. Both will coexist for the foreseeable future.

### 1. Node Wrapper (`agnocast_wrapper::Node`)

Use this when you want the **entire node** to transparently switch between `rclcpp::Node` and `agnocast::Node` at runtime. The node wrapper automatically selects the correct underlying implementation based on the `ENABLE_AGNOCAST` environment variable.

Currently supported APIs:

- Publisher / Subscription / PollingSubscriber
- Timer (`create_wall_timer`, free `create_timer()`, free `set_period()`)
- Parameters
- Logger, Clock
- Callback groups
- Node interfaces (partial: `get_node_base_interface()`, `get_node_topics_interface()`, `get_node_parameters_interface()`)

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

To use the Node wrapper in your package, add the following to your `CMakeLists.txt`:

```cmake
find_package(autoware_agnocast_wrapper REQUIRED)
ament_target_dependencies(my_node_component autoware_agnocast_wrapper)
```

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

You can immediately understand how to use the macros just by looking at `autoware_agnocast_wrapper.hpp`. A typical callback and publisher setup looks like this:

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
- Maximum 2 message types per `Synchronizer`.
- `connectInput()` is not supported; pass `Subscriber` references at construction time.

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
