# autoware_agnocast_wrapper

The purpose of this package is to integrate Agnocast, a zero-copy middleware, into each topic in Autoware with minimal side effects. Agnocast is a library designed to work alongside ROS 2, enabling true zero-copy publish/subscribe communication for all ROS 2 message types, including unsized message types.

- Agnocast Repository: <https://github.com/tier4/agnocast>
- Discussion on Agnocast Integration into Autoware: <https://github.com/orgs/autowarefoundation/discussions/5835>

This package provides macros that wrap functions for publish/subscribe operations and smart pointer types for handling ROS 2 messages. When Autoware is built using the default build command, Agnocast is **not enabled**. However, setting the environment variable `ENABLE_AGNOCAST=1` enables Agnocast and results in a build that includes its integration. This design ensures backward compatibility for users who are unaware of Agnocast, minimizing disruption.

## Two Integration Approaches

This package provides two approaches for integrating Agnocast. Both will coexist for the foreseeable future.

### 1. Node Wrapper (`agnocast_wrapper::Node`)

Use this when you want the **entire node** to transparently switch between `rclcpp::Node` and `agnocast::Node` at runtime. The node wrapper automatically selects the correct underlying implementation based on the `ENABLE_AGNOCAST` environment variable.

Currently supported APIs:

- Publisher / Subscription / PollingSubscriber
- Parameters
- Logger, Clock
- Callback groups
- Node interfaces (partial: `get_node_base_interface()`, `get_node_topics_interface()`, `get_node_parameters_interface()`)

> **Note:** Timer (`create_wall_timer`, `create_timer`) is not yet supported and will be added in a future update.

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
      "input", 10, [this](std::unique_ptr<const std_msgs::msg::String> msg) { /* ... */ });
  }

private:
  autoware::agnocast_wrapper::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  autoware::agnocast_wrapper::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};
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
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --package-select <package_name>
```

To rebuild a specific package **with** Agnocast after it was previously built without it:

```bash
rm -Rf ./install/<package_name> ./build/<package_name>
export ENABLE_AGNOCAST=1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --package-select <package_name>
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

When Agnocast is enabled at build time, the heaphook shared library must be preloaded at runtime via `LD_PRELOAD`, and component containers must be replaced with their Agnocast equivalents. This package provides `agnocast_env.launch.xml` which handles both of these concerns based on the `ENABLE_AGNOCAST` environment variable.

### Provided Variables

After including `agnocast_env.launch.xml`, the following variables are available:

| Variable               | Description                                                                              |
| ---------------------- | ---------------------------------------------------------------------------------------- |
| `ld_preload_value`     | `LD_PRELOAD` value with the heaphook library prepended (when Agnocast is enabled)        |
| `container_package`    | Resolved component container package name (`rclcpp_components` or `agnocast_components`) |
| `container_executable` | Resolved component container executable name                                             |

### Launch Arguments

| Argument                 | Default                                       | Description                                                           |
| ------------------------ | --------------------------------------------- | --------------------------------------------------------------------- |
| `agnocast_heaphook_path` | `/opt/ros/humble/lib/libagnocast_heaphook.so` | Path to the heaphook shared library                                   |
| `use_multithread`        | `false`                                       | Use the multi-threaded component container (`component_container_mt`) |

The `container_executable` is resolved as follows:

| `use_multithread` | `ENABLE_AGNOCAST=0`      | `ENABLE_AGNOCAST=1`                |
| ----------------- | ------------------------ | ---------------------------------- |
| `false`           | `component_container`    | `agnocast_component_container`     |
| `true`            | `component_container_mt` | `agnocast_component_container_cie` |

### Examples

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

This ensures that only the intended nodes receive the heaphook, rather than all nodes in the launch tree.
