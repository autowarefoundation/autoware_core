# autoware_qos_utils

Autoware QoS Utils provides QoS compatibility utilities for different ROS 2 distributions.

## Overview

This package provides compatibility functions for QoS (Quality of Service) configuration across different ROS 2 distributions, particularly addressing the API changes introduced in ROS 2 Jazzy.

## Features

- **QoS Compatibility**: Provides unified QoS configuration functions that work across ROS 2 distributions
- **Service QoS**: Compatible service QoS configuration for both old and new ROS 2 APIs
- **Topic QoS**: Compatible default topic QoS configuration
- **Header-Only**: Pure header-only library with no compilation overhead

## Usage

### Service QoS

```cpp
#include <autoware/qos_utils/qos_compatibility.hpp>

// Create a service client with compatible QoS
auto client = node->create_client<MyService>(
  "my_service",
  AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE()
);

// Create a service with compatible QoS
auto service = node->create_service<MyService>(
  "my_service",
  callback,
  AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE()
);
```

### Topic QoS

```cpp
#include <autoware/qos_utils/qos_compatibility.hpp>

// Create a publisher with compatible QoS
auto publisher = node->create_publisher<MyMessage>(
  "my_topic",
  AUTOWARE_DEFAULT_TOPIC_QOS_PROFILE()
);

// Create a subscription with compatible QoS
auto subscription = node->create_subscription<MyMessage>(
  "my_topic",
  10,
  callback,
  AUTOWARE_DEFAULT_TOPIC_QOS_PROFILE()
);
```

## API Reference

### Macros

- `AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE()`: Expands to service QoS profile compatible with current ROS 2 distribution
- `AUTOWARE_DEFAULT_TOPIC_QOS_PROFILE()`: Expands to topic QoS profile compatible with current ROS 2 distribution

## ROS 2 Distribution Compatibility

| ROS 2 Distribution | Service QoS                                   | Topic QoS                           |
| ------------------ | --------------------------------------------- | ----------------------------------- |
| Humble and earlier | `rclcpp::ServicesQoS().get_rmw_qos_profile()` | `rmw_qos_profile_default`           |
| Jazzy and later    | `rclcpp::ServicesQoS()`                       | `rclcpp::QoS(rclcpp::KeepLast(10))` |

### Implementation Details

The package uses conditional compilation to provide the appropriate QoS configuration:

- **Service QoS**: Uses C++ interface (`rclcpp::ServicesQoS()`) for better compatibility
- **Topic QoS**: Uses appropriate interface based on ROS 2 distribution
- **Zero Runtime Overhead**: All macros expand to direct expressions for optimal performance

## Dependencies

- `rclcpp`: ROS 2 C++ client library

## License

Apache License 2.0
