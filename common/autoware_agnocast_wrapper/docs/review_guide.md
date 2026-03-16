This document serves as a guide for reviewing PRs that apply [autoware_agnocast_wrapper](https://github.com/autowarefoundation/autoware_core/tree/main/common/autoware_agnocast_wrapper). **Part 1** provides a step-by-step walkthrough of what to check during review. **Part 2** compiles background knowledge referenced by each review step — refer to it as needed.

## Related Links

| Resource                                            | URL                                                                                                                                                              |
| --------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| autoware_agnocast_wrapper (source code)             | [autowarefoundation/autoware_core/.../autoware_agnocast_wrapper](https://github.com/autowarefoundation/autoware_core/tree/main/common/autoware_agnocast_wrapper) |
| Agnocast repository                                 | [autowarefoundation/agnocast](https://github.com/autowarefoundation/agnocast)                                                                                    |
| Agnocast README                                     | [agnocast/README.md](https://github.com/autowarefoundation/agnocast/blob/main/README.md)                                                                         |
| Autoware Discussion: Agnocast introduction proposal | [Discussion #5835 - Introduce True Zero-Copy Publish/Subscribe IPC to Autoware](https://github.com/orgs/autowarefoundation/discussions/5835)                     |
| Agnocast ROS 2 rosdistro support                    | [Issue #5968 - Use ROS 2 packages in Agnocast released via rosdistro](https://github.com/autowarefoundation/autoware/issues/5968)                                |

## Table of Contents

### Part 1: Review Guide

- Prerequisites for Review
- Review Procedure Guide
  - Common Prerequisites
  - Method 1: Macro + Free Function API Review Procedure
  - Method 2: agnocast_wrapper::Node Inheritance Review Procedure

### Part 2: Background Knowledge Reference

- What is autoware_agnocast_wrapper?
- Behavior Changes via ENABLE_AGNOCAST Environment Variable
- Key Macros
- Two Integration Methods: Free Functions vs agnocast_wrapper::Node
- component_container and autoware_agnocast_wrapper_register_node
- Executor Types and Selection
- component_container Selection (agnocast_env.launch.xml)
- Build and Execution Procedures
- References

---

# Part 1: Review Guide

## 1. Prerequisites for Review

### Overview of autoware_agnocast_wrapper

[autoware_agnocast_wrapper](https://github.com/autowarefoundation/autoware_core/tree/main/common/autoware_agnocast_wrapper) is a wrapper package for applying the zero-copy middleware [Agnocast](https://github.com/autowarefoundation/agnocast) to Autoware nodes. Build-time and runtime behavior is controlled by the `ENABLE_AGNOCAST` environment variable:

- **Built with `ENABLE_AGNOCAST=0` (or unset)**: Standard ROS 2 build. Macros expand directly to rclcpp APIs, and no Agnocast-related code is included
- **Built with `ENABLE_AGNOCAST=1`**: Agnocast-enabled build. At runtime, communication switches between Agnocast and ROS 2 depending on the `ENABLE_AGNOCAST` value

In other words, users who build with `ENABLE_AGNOCAST=0` are not affected at all, and backward compatibility is maintained (see Part 2 Section 2 for details).

### Agnocast Bridge: Why Other Nodes Are Not Affected

Another important prerequisite when reviewing Agnocast-related PRs is the existence of the **Agnocast Bridge**.

The [Agnocast Bridge](https://github.com/autowarefoundation/agnocast/blob/main/docs/agnocast_ros2_bridge.md) automatically forwards messages bidirectionally between Agnocast nodes and standard ROS 2 nodes:

- **R2A (ROS 2 → Agnocast)**: Forwards messages from ROS 2 publishers to Agnocast subscribers
- **A2R (Agnocast → ROS 2)**: Forwards messages from Agnocast publishers to ROS 2 subscribers

Message circulation (echo-back) is automatically prevented by the Bridge's internal logic.

**This means:**

- Even if Agnocast is applied to a node, communication with neighboring nodes that still use standard ROS 2 will not be disrupted, as the Bridge automatically mediates
- **During review, there is no need to check whether connected nodes have been adapted for Agnocast**
- Each node can be independently adapted for Agnocast

---

## 2. Review Procedure Guide

This section provides a step-by-step review procedure for PRs that apply autoware_agnocast_wrapper. The checklist differs depending on the integration method (Method 1: Macro + Free Function API / Method 2: agnocast_wrapper::Node Inheritance), so each is documented separately.

Background knowledge referenced by each step is compiled in Part 2: Background Knowledge Reference. When you need to check details such as macro expansion results or executor types during review, follow the links annotated on each step to the relevant Part 2 section.

### Common Prerequisites

#### Step 0: Identify the Integration Method

autoware_agnocast_wrapper has two integration patterns: applying Agnocast to specific topics while keeping the existing node as-is (Method 1), or replacing the node's base class entirely (Method 2). You can distinguish them by checking the node's base class.

- Base class remains **`rclcpp::Node`** → **Method 1: Macro + Free Function API**
- Base class changed to **`autoware::agnocast_wrapper::Node`** → **Method 2: agnocast_wrapper::Node Inheritance**

```cpp
// Method 1
class MyNode : public rclcpp::Node

// Method 2
class MyNode : public autoware::agnocast_wrapper::Node
```

Once identified, proceed to the corresponding review procedure below.

---

### Method 1: Macro + Free Function API Review Procedure

#### Step 1: Code Changes (see Part 2 Section 3)

- [ ] `#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>` has been added
- [ ] **Base class remains `rclcpp::Node`**
- [ ] Member variable types: `rclcpp::Publisher<M>::SharedPtr` → `AUTOWARE_PUBLISHER_PTR(M)` etc.
- [ ] Creation: `this->create_publisher` → `AUTOWARE_CREATE_PUBLISHER2` / `AUTOWARE_CREATE_PUBLISHER3` etc.
- [ ] Callback arguments: `const SharedPtr` / `UniquePtr` → `AUTOWARE_MESSAGE_CONST_SHARED_PTR` / `AUTOWARE_MESSAGE_UNIQUE_PTR`
- [ ] Message allocation (if publisher exists): `std::make_unique<M>()` → `ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_)`
- [ ] Options type: `rclcpp::SubscriptionOptions` → `AUTOWARE_SUBSCRIPTION_OPTIONS`

#### Step 2: Verification (see Part 2 Section 2, Part 2 Section 8)

Items 1 and 2 are **recommended**, item 3 is **if possible** (requires Agnocast environment).

**1. Build and run with `ENABLE_AGNOCAST=0` (or unset) (recommended)**

The most important test: verify that the build does not break in environments without Agnocast.

```bash
unset ENABLE_AGNOCAST  # or export ENABLE_AGNOCAST=0
colcon build --symlink-install --packages-select <target_package>
# Run and verify that standard ROS 2 behavior works correctly
```

- [ ] Build succeeds
- [ ] `ros2 topic echo` etc. confirms that topic pub/sub works as expected

**2. Build with `ENABLE_AGNOCAST=1`, run with `ENABLE_AGNOCAST=0` (recommended)**

Verify that the Agnocast-enabled build works correctly in ROS 2 fallback mode.

> **Note:** When switching the `ENABLE_AGNOCAST` value and rebuilding, delete the `build/` and `install/` directories of both the target package and `autoware_agnocast_wrapper` before building. Since `ENABLE_AGNOCAST` is an environment variable (not a CMake variable), the previous setting may remain in the build cache, causing inconsistencies.

```bash
export ENABLE_AGNOCAST=1
rm -rf build/autoware_agnocast_wrapper install/autoware_agnocast_wrapper
rm -rf build/<target_package> install/<target_package>
colcon build --symlink-install --packages-select autoware_agnocast_wrapper <target_package>
export ENABLE_AGNOCAST=0  # Fall back to ROS 2 at runtime
# Run and verify behavior
```

- [ ] Build succeeds
- [ ] `ros2 topic echo` etc. confirms that topic pub/sub works as expected

**3. Build and run with `ENABLE_AGNOCAST=1` (if possible)**

Verify that Agnocast communication works correctly. Requires Agnocast environment setup (see below).

```bash
export ENABLE_AGNOCAST=1
colcon build --symlink-install --packages-select <target_package>
# Run with ENABLE_AGNOCAST=1
```

- [ ] `ros2 topic list_agnocast` shows `(Agnocast enabled)` for the target topic
- [ ] `ros2 topic info_agnocast /target_topic` shows expected Agnocast Publisher/Subscriber counts
- [ ] Subscriber-side callbacks are working correctly

> **Regarding `ros2 topic echo` / `ros2 topic hz`:**
> These commands are generally usable for Agnocast topics thanks to the Bridge feature.
>
> However, when all subscribers for a topic have been converted to Agnocast, the Bridge may not yet have been created when `ros2 topic echo` connects, causing it to time out before being recognized as a ROS 2 subscriber. In such cases, use the `ros2 topic list_agnocast` / `ros2 topic info_agnocast` commands, or directly verify subscriber-side callback behavior. Native commands such as `ros2 topic echo_agnocast` are planned for future release.

#### Agnocast Environment Prerequisites

To perform verification item 3, the following Agnocast environment is required:

**Agnocast kernel module check:**

```bash
$ lsmod | grep agnocast
agnocast              835584  0
```

If not shown:

```bash
sudo add-apt-repository ppa:t4-system-software/agnocast
sudo apt update
sudo apt install agnocast-kmod-v2.3  # Match the version in autoware.repos
sudo modprobe agnocast
```

**Agnocast heaphook check:**

```bash
ls /opt/ros/humble/lib/libagnocast_heaphook.so
```

If not found:

```bash
sudo apt install agnocast-heaphook-v2.3  # Match the version in autoware.repos
```

> The version can be found in the `middleware/external/agnocast` section of [`autoware.repos`](https://github.com/autowarefoundation/autoware/blob/main/repositories/autoware.repos). For instructions on building and installing from source, see the [Agnocast README](https://github.com/autowarefoundation/agnocast/blob/main/README.md).

---

### Method 2: agnocast_wrapper::Node Inheritance Review Procedure

Node-wide migration to `agnocast_wrapper::Node` (see Part 2 Section 4 Method 2).

#### Step 1: Code Changes (see Part 2 Section 3, Part 2 Section 4 Method 2)

- [ ] `#include <autoware/agnocast_wrapper/node.hpp>` has been added
- [ ] Base class has been changed to `autoware::agnocast_wrapper::Node`
- [ ] Member variable types: `AUTOWARE_*_PTR` macros or `autoware::agnocast_wrapper::Publisher<M>::SharedPtr` etc.
- [ ] Creation: Use `agnocast_wrapper::Node` member functions `create_publisher` / `create_subscription` directly (**`AUTOWARE_CREATE_*` macros are not needed**)
- [ ] Message allocation (if publisher exists): `std::make_unique<M>()` → `ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_)`
- [ ] If the original CMakeLists.txt used `rclcpp_components_register_node()`, it has been replaced with `autoware_agnocast_wrapper_register_node()`

#### Step 2: Verification

- [ ] **Build and run with `ENABLE_AGNOCAST=0` (recommended)**: Build succeeds and standard ROS 2 behavior works
- [ ] **Build with `ENABLE_AGNOCAST=1`, run with `ENABLE_AGNOCAST=0` (recommended)**: Build succeeds and ROS 2 fallback works
- [ ] **Build and run with `ENABLE_AGNOCAST=1` (if possible)**: Agnocast communication works via `ros2 topic list_agnocast` / `ros2 topic info_agnocast` (environment setup required)

---

# Part 2: Background Knowledge Reference

> The following sections compile the background knowledge referenced by the review guide steps above.

## 1. What is autoware_agnocast_wrapper?

[autoware_agnocast_wrapper](https://github.com/autowarefoundation/autoware_core/tree/main/common/autoware_agnocast_wrapper) is a package for integrating [Agnocast](https://github.com/autowarefoundation/agnocast), a zero-copy middleware, into each Autoware topic with minimal impact.

Agnocast is an rclcpp-compatible zero-copy IPC middleware that enables true zero-copy Publish/Subscribe communication for all ROS 2 message types, including variable-length message types already generated by rosidl (see: [Autoware Discussion #5835](https://github.com/orgs/autowarefoundation/discussions/5835)).

**Key features:**

- Agnocast can be enabled/disabled at both build time and runtime
- Can be applied to existing `rclcpp::Node`-based code with minimal changes
- Maintains backward compatibility for users unfamiliar with Agnocast

## 2. Behavior Changes via ENABLE_AGNOCAST Environment Variable

autoware_agnocast_wrapper behaves differently depending on the `ENABLE_AGNOCAST` environment variable.

### Build Time

| ENABLE_AGNOCAST | Behavior                                                                                                                                     |
| --------------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| Unset or `0`    | Standard ROS 2 build. `USE_AGNOCAST_ENABLED` is not defined. Macros expand directly to rclcpp APIs.                                          |
| `1`             | Agnocast-enabled build. `USE_AGNOCAST_ENABLED` is defined. Macros expand to wrapper classes, and runtime-switchable templates are generated. |

### Runtime (when built with ENABLE_AGNOCAST=1)

| ENABLE_AGNOCAST | Behavior                        |
| --------------- | ------------------------------- |
| Unset or `0`    | Communicates via ROS 2 (rclcpp) |
| `1`             | Communicates via Agnocast       |

**Notes:**

- If built with `ENABLE_AGNOCAST=0`, setting `ENABLE_AGNOCAST=1` at runtime will NOT enable Agnocast (switching code is not generated at build time)
- After building with `ENABLE_AGNOCAST=1`, you can fall back to ROS 2 by running with `ENABLE_AGNOCAST=0`

## 3. Key Macros

All macros below are defined in [`autoware_agnocast_wrapper.hpp`](https://github.com/autowarefoundation/autoware_core/blob/main/common/autoware_agnocast_wrapper/include/autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp).

### Message Pointer Types

| Macro                                     | ENABLE_AGNOCAST=1 (Agnocast) | ENABLE_AGNOCAST=0 (ROS 2)     |
| ----------------------------------------- | ---------------------------- | ----------------------------- |
| `AUTOWARE_MESSAGE_UNIQUE_PTR(MsgT)`       | `message_ptr<MsgT, Unique>`  | `std::unique_ptr<MsgT>`       |
| `AUTOWARE_MESSAGE_SHARED_PTR(MsgT)`       | `message_ptr<MsgT, Shared>`  | `std::shared_ptr<MsgT>`       |
| `AUTOWARE_MESSAGE_CONST_SHARED_PTR(MsgT)` | `message_ptr<MsgT, Shared>`  | `std::shared_ptr<const MsgT>` |

> `AUTOWARE_MESSAGE_SHARED_PTR` is for publishers (mutable messages), while `AUTOWARE_MESSAGE_CONST_SHARED_PTR` is for subscriptions (read-only messages).

### Publisher/Subscriber Types

| Macro                                   | ENABLE_AGNOCAST=1 (Agnocast)         | ENABLE_AGNOCAST=0 (ROS 2)                        |
| --------------------------------------- | ------------------------------------ | ------------------------------------------------ |
| `AUTOWARE_PUBLISHER_PTR(MsgT)`          | `Publisher<MsgT>::SharedPtr`         | `rclcpp::Publisher<MsgT>::SharedPtr`             |
| `AUTOWARE_SUBSCRIPTION_PTR(MsgT)`       | `Subscription<MsgT>::SharedPtr`      | `rclcpp::Subscription<MsgT>::SharedPtr`          |
| `AUTOWARE_POLLING_SUBSCRIBER_PTR(MsgT)` | `PollingSubscriber<MsgT>::SharedPtr` | `InterProcessPollingSubscriber<MsgT>::SharedPtr` |

### Publisher/Subscriber Creation

| Macro                                                                   | ENABLE_AGNOCAST=1 (Agnocast)                                 | ENABLE_AGNOCAST=0 (ROS 2)                                 |
| ----------------------------------------------------------------------- | ------------------------------------------------------------ | --------------------------------------------------------- |
| `AUTOWARE_CREATE_SUBSCRIPTION(msg_type, topic, qos, callback, options)` | `agnocast_wrapper::create_subscription<msg_type>(...)`       | `this->create_subscription<msg_type>(...)`                |
| `AUTOWARE_CREATE_PUBLISHER2(msg_type, topic, qos)`                      | `agnocast_wrapper::create_publisher<msg_type>(...)`          | `this->create_publisher<msg_type>(...)`                   |
| `AUTOWARE_CREATE_PUBLISHER3(msg_type, topic, qos, options)`             | `agnocast_wrapper::create_publisher<msg_type>(...)`          | `this->create_publisher<msg_type>(...)`                   |
| `AUTOWARE_CREATE_POLLING_SUBSCRIBER(msg_type, topic, qos)`              | `agnocast_wrapper::create_polling_subscriber<msg_type>(...)` | `InterProcessPollingSubscriber::create_subscription(...)` |

### Message Allocation

| Macro                                       | ENABLE_AGNOCAST=1 (Agnocast)                                               | ENABLE_AGNOCAST=0 (ROS 2)            |
| ------------------------------------------- | -------------------------------------------------------------------------- | ------------------------------------ |
| `ALLOCATE_OUTPUT_MESSAGE_UNIQUE(publisher)` | `publisher->allocate_output_message_unique()` (allocates in shared memory) | `std::make_unique<ROSMessageType>()` |
| `ALLOCATE_OUTPUT_MESSAGE_SHARED(publisher)` | `publisher->allocate_output_message_shared()` (allocates in shared memory) | `std::make_shared<ROSMessageType>()` |

### Options Types

| Macro                           | ENABLE_AGNOCAST=1 (Agnocast)    | ENABLE_AGNOCAST=0 (ROS 2)     |
| ------------------------------- | ------------------------------- | ----------------------------- |
| `AUTOWARE_SUBSCRIPTION_OPTIONS` | `agnocast::SubscriptionOptions` | `rclcpp::SubscriptionOptions` |
| `AUTOWARE_PUBLISHER_OPTIONS`    | `agnocast::PublisherOptions`    | `rclcpp::PublisherOptions`    |

## 4. Two Integration Methods: Free Functions vs agnocast_wrapper::Node

autoware_agnocast_wrapper provides two integration approaches.

### Method 1: Macro + Free Function API

Used to apply Agnocast to specific topics only on an existing `rclcpp::Node`. Wraps individual Publishers/Subscribers without changing the entire node.

**Usage example:**

```cpp
#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>

class MyNode : public rclcpp::Node  // Remains rclcpp::Node
{
  AUTOWARE_PUBLISHER_PTR(PointCloud2) pub_;

  void setup() {
    pub_ = AUTOWARE_CREATE_PUBLISHER3(PointCloud2, "output", qos, options);
  }

  void callback(AUTOWARE_MESSAGE_UNIQUE_PTR(const PointCloud2) && msg) {
    auto output = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_);
    // ... processing ...
    pub_->publish(std::move(output));
  }
};
```

**CMakeLists.txt:**

```cmake
find_package(autoware_agnocast_wrapper REQUIRED)
ament_target_dependencies(target autoware_agnocast_wrapper)
autoware_agnocast_wrapper_setup(target)
```

### Method 2: agnocast_wrapper::Node Inheritance

Used to transparently switch the entire node between `rclcpp::Node` and [`agnocast::Node`](https://github.com/autowarefoundation/agnocast/blob/main/docs/agnocast_node_interface_comparison.md). The node wrapper automatically selects the appropriate implementation based on the `ENABLE_AGNOCAST` environment variable at runtime.

**Usage example:**

```cpp
#include <autoware/agnocast_wrapper/node.hpp>

class MyNode : public autoware::agnocast_wrapper::Node  // Inherits Node wrapper
{
public:
  explicit MyNode(const rclcpp::NodeOptions & options)
  : Node("my_node", options)
  {
    pub_ = create_publisher<std_msgs::msg::String>("output", 10);
    sub_ = create_subscription<std_msgs::msg::String>(
      "input", 10, [this](auto msg) { /* ... */ });
  }

private:
  autoware::agnocast_wrapper::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  autoware::agnocast_wrapper::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};
```

### Comparison Table

|                        | Macro + Free Function | agnocast_wrapper::Node   |
| ---------------------- | --------------------- | ------------------------ |
| Base class             | `rclcpp::Node`        | `agnocast_wrapper::Node` |
| Scope of changes       | Specific topics only  | Entire node              |
| Amount of code changes | Small                 | Medium                   |
| AgnocastOnly Executor  | Not available         | Available                |

### Method 2 Behavior with ENABLE_AGNOCAST=0

When built with `ENABLE_AGNOCAST=0` (or unset), [`node.hpp`](https://github.com/autowarefoundation/autoware_core/blob/main/common/autoware_agnocast_wrapper/include/autoware/agnocast_wrapper/node.hpp) defines `autoware::agnocast_wrapper::Node` as a simple typedef for `rclcpp::Node`:

```cpp
// node.hpp when ENABLE_AGNOCAST=0
using Node = rclcpp::Node;
```

Therefore, code using Method 2 compiles and runs without issues as a regular `rclcpp::Node` when built with `ENABLE_AGNOCAST=0`. Backward compatibility is maintained even in environments without Agnocast support.

## 5. component_container and autoware_agnocast_wrapper_register_node

### autoware_agnocast_wrapper_register_node Macro

A CMake macro used in place of `rclcpp_components_register_node`. It generates different targets depending on the `ENABLE_AGNOCAST` setting at build time.

**ENABLE_AGNOCAST=0 (or unset):**

- Delegates to `rclcpp_components_register_node` (standard behavior)
- Generated target: `<EXECUTABLE>` only

**ENABLE_AGNOCAST=1:**

- Generates two targets:
  1. `<EXECUTABLE>_component`: Standard rclcpp_components executable (for containers)
  2. `<EXECUTABLE>`: Runtime-switchable standalone executable

**Usage example:**

```cmake
autoware_agnocast_wrapper_register_node(my_node_component
  PLUGIN "my_package::MyNode"
  EXECUTABLE my_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR SingleThreadedAgnocastExecutor
)
```

### Parameters

| Parameter           | Required | Description                                                                              |
| ------------------- | -------- | ---------------------------------------------------------------------------------------- |
| `PLUGIN`            | Yes      | Fully qualified class name of the component                                              |
| `EXECUTABLE`        | Yes      | Executable name for the node                                                             |
| `ROS2_EXECUTOR`     | No       | Executor when `ENABLE_AGNOCAST=0` at runtime (default: `SingleThreadedExecutor`)         |
| `AGNOCAST_EXECUTOR` | No       | Executor when `ENABLE_AGNOCAST=1` at runtime (default: `SingleThreadedAgnocastExecutor`) |

## 6. Executor Types and Selection

### ROS2_EXECUTOR (when ENABLE_AGNOCAST=0 at runtime)

| Executor                 | Description                                      |
| ------------------------ | ------------------------------------------------ |
| `SingleThreadedExecutor` | Single-threaded, executes callbacks sequentially |
| `MultiThreadedExecutor`  | Multi-threaded, executes callbacks in parallel   |

### AGNOCAST_EXECUTOR (when ENABLE_AGNOCAST=1 at runtime)

| Executor                               | Description                                                                                                |
| -------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| `SingleThreadedAgnocastExecutor`       | Single-threaded. Processes both ROS 2 and Agnocast callbacks                                               |
| `MultiThreadedAgnocastExecutor`        | Multi-threaded. Processes both ROS 2 and Agnocast callbacks                                                |
| `CallbackIsolatedAgnocastExecutor`     | Multi-threaded (callback isolated). Processes both ROS 2 and Agnocast callbacks                            |
| `AgnocastOnlySingleThreadedExecutor`   | Single-threaded. Processes Agnocast callbacks only. **Requires agnocast_wrapper::Node**                    |
| `AgnocastOnlyMultiThreadedExecutor`    | Multi-threaded. Processes Agnocast callbacks only. **Requires agnocast_wrapper::Node**                     |
| `AgnocastOnlyCallbackIsolatedExecutor` | Multi-threaded (callback isolated). Processes Agnocast callbacks only. **Requires agnocast_wrapper::Node** |

> **About AgnocastOnly executors:** The PLUGIN must inherit from `autoware::agnocast_wrapper::Node`. The `get_agnocast_node()` method is used to add the node to the executor.

### Threading Model Warning

If `ROS2_EXECUTOR` and `AGNOCAST_EXECUTOR` have different threading models, a WARNING is issued at CMake time. This warns that threading behavior will implicitly change depending on the runtime `ENABLE_AGNOCAST` value.

**Example:** `ROS2_EXECUTOR=SingleThreadedExecutor` + `AGNOCAST_EXECUTOR=MultiThreadedAgnocastExecutor`

- With `ENABLE_AGNOCAST=0`: single-threaded
- With `ENABLE_AGNOCAST=1`: multi-threaded

## 7. component_container Selection (agnocast_env.launch.xml)

By including `agnocast_env.launch.xml`, the appropriate component container is automatically selected based on the `ENABLE_AGNOCAST` environment variable.

### Provided Variables

| Variable               | Description                                  |
| ---------------------- | -------------------------------------------- |
| `ld_preload_value`     | `LD_PRELOAD` value with heaphook prepended   |
| `container_package`    | `rclcpp_components` or `agnocast_components` |
| `container_executable` | Container executable name to use             |

### container_executable Resolution

| use_multithread | ENABLE_AGNOCAST=0        | ENABLE_AGNOCAST=1                  |
| --------------- | ------------------------ | ---------------------------------- |
| `false`         | `component_container`    | `agnocast_component_container`     |
| `true`          | `component_container_mt` | `agnocast_component_container_cie` |

### Usage Examples

Both an XML launch file (`agnocast_env.launch.xml`) and a Python launch file (`agnocast_env.launch.py`) are provided. Choose whichever matches the format of your existing launch files. Both provide the same variables (`ld_preload_value` / `container_package` / `container_executable`).

**XML launch file:**

```xml
<include file="$(find-pkg-share autoware_agnocast_wrapper)/launch/agnocast_env.launch.xml">
  <arg name="use_multithread" value="true"/>
</include>

<node_container pkg="$(var container_package)" exec="$(var container_executable)" name="my_container">
  <env name="LD_PRELOAD" value="$(var ld_preload_value)"/>
</node_container>
```

**Python launch file:**

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# Include agnocast_env.launch.py
agnocast_env = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('autoware_agnocast_wrapper'),
            'launch', 'agnocast_env.launch.py'
        )
    ),
    launch_arguments={'use_multithread': 'true'}.items(),
)

# Use the provided variables
container = ComposableNodeContainer(
    name='my_container',
    package=LaunchConfiguration('container_package'),
    executable=LaunchConfiguration('container_executable'),
    additional_env={'LD_PRELOAD': LaunchConfiguration('ld_preload_value')},
    # ...
)
```

### Parameters

| Parameter                | Default                                       | Description                               |
| ------------------------ | --------------------------------------------- | ----------------------------------------- |
| `agnocast_heaphook_path` | `/opt/ros/humble/lib/libagnocast_heaphook.so` | Path to the heaphook library              |
| `use_multithread`        | `false`                                       | Whether to use a multi-threaded container |

## 8. Build and Execution Procedures

### Build with Agnocast Disabled (default)

```bash
unset ENABLE_AGNOCAST  # or export ENABLE_AGNOCAST=0
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Build with Agnocast Enabled

```bash
export ENABLE_AGNOCAST=1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Run with Agnocast Enabled

```bash
export ENABLE_AGNOCAST=1
source ~/autoware/install/setup.bash
ros2 launch autoware_launch ...
```

### Run with Agnocast Disabled (after building with ENABLE_AGNOCAST=1)

```bash
export ENABLE_AGNOCAST=0  # or unset ENABLE_AGNOCAST
source ~/autoware/install/setup.bash
ros2 launch autoware_launch ...
```

## 9. References

- [autoware_agnocast_wrapper Source Code & README](https://github.com/autowarefoundation/autoware_core/tree/main/common/autoware_agnocast_wrapper) - The package itself. The official README contains detailed build/execution procedures and executor configuration
- [Agnocast Repository](https://github.com/autowarefoundation/agnocast) - Agnocast source code including kernel module, heaphook, bridge, etc.
- [Agnocast README](https://github.com/autowarefoundation/agnocast/blob/main/README.md) - Agnocast architecture, supported environments, installation procedures
- [agnocast::Node and rclcpp::Node Interface Comparison](https://github.com/autowarefoundation/agnocast/blob/main/docs/agnocast_node_interface_comparison.md) - API differences between `agnocast::Node` and `rclcpp::Node`
- [Agnocast Autoware Integration Guide](https://github.com/autowarefoundation/agnocast/blob/main/docs/autoware_integration.md) - Detailed documentation on integrating Agnocast into Autoware
- [Agnocast message_filters User Guide](https://github.com/autowarefoundation/agnocast/blob/main/docs/message_filters_user_guide.md) - How to use Agnocast's message_filters
- [Agnocast ROS 2 Bridge](https://github.com/autowarefoundation/agnocast/blob/main/docs/agnocast_ros2_bridge.md) - Bridge communication between Agnocast nodes and standard ROS 2 nodes
- [Callback Isolated Executor for Agnocast](https://github.com/autowarefoundation/agnocast/blob/main/docs/callback_isolated_executor_for_agnocast.md) - Design of CallbackIsolatedAgnocastExecutor
- [Autoware Discussion #5835: Introduce True Zero-Copy Publish/Subscribe IPC to Autoware](https://github.com/orgs/autowarefoundation/discussions/5835) - Agnocast introduction proposal for Autoware. Design philosophy, performance comparison, Agnocast Bridge mechanics, etc.
- [Issue #5968: Use ROS 2 packages in Agnocast released via rosdistro](https://github.com/autowarefoundation/autoware/issues/5968) - Tracking issue for Agnocast package rosdistro release support
