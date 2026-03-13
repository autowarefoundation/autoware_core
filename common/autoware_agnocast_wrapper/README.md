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
ament_target_dependencies(target autoware_agnocast_wrapper)
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

When Agnocast is enabled at build time, the heaphook shared library must be preloaded at runtime via `LD_PRELOAD`. This package provides a launch file that computes the appropriate `LD_PRELOAD` value based on the `ENABLE_AGNOCAST` environment variable.

In your node's launch file, first include the launch file, then apply `ld_preload_value` to each node that uses Agnocast:

```xml
<include file="$(find-pkg-share autoware_agnocast_wrapper)/launch/agnocast_env.launch.xml"/>

<node pkg="my_package" exec="my_node" name="my_node">
  <env name="LD_PRELOAD" value="$(var ld_preload_value)"/>
</node>
```

This ensures that only the intended nodes receive the heaphook, rather than all nodes in the launch tree.
