# autoware_planning_factor_interface

## Overview

The `PlanningFactorInterface` is a C++ class designed to facilitate the addition and publication of planning factors.

## Design

The `PlanningFactorInterface` class is designed to be lightweight and efficient, with the following key components:

- **Add:** Methods to add planning factors to the interface.

- **Publisher:** The class includes a publisher for `PlanningFactorArray` messages, which are used to distribute planning factors to other nodes in the system.

The design emphasizes flexibility and ease of use, allowing developers to quickly integrate new planning factors into autoware.

## Usage

### Including the Header

To use the `PlanningFactorInterface`, include the header file in your code:

```cpp
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
```

### Creating an Instance

Instantiate the `PlanningFactorInterface` by providing a node and a name for the factor module:

```cpp

class AvoidancePlanner
{
public:
  AvoidancePlanner(rclcpp::Node & node)
  : planning_factor_interface_{std::make_unique<
      autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "avoidance_planner")}
```

You can also enable console output for debugging by setting the appropriate parameters:

```cpp
// Enable console output with a 1000ms throttle duration
planning_factor_interface_ = std::make_unique<
  autoware::planning_factor_interface::PlanningFactorInterface>(
  &node, "avoidance_planner", true, 1000);
```

### Adding Planning Factors

```cpp
planning_factor_interface_->add(
        traj_points, ego_pose, stop_pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::NONE,
        autoware_internal_planning_msgs::msg::SafetyFactorArray{});
```

### Publishing Factors

After adding planning factors, you can publish them by calling the `publish` method:

```cpp
// Publish the added factors
planning_factor_interface_->publish();
```
