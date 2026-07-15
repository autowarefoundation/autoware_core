// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__INTERFACE_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

// ROS 2 service introspection (rcl/service_introspection.h and
// Client/Service::configure_introspection) is available from Iron onward; ROS 2
// Humble does not ship it. Gate the feature on the header so this package builds
// on both distributions — service introspection is simply unavailable on Humble.
#if __has_include(<rcl/service_introspection.h>)
#include <rcl/service_introspection.h>
#define AUTOWARE_COMPONENT_INTERFACE_UTILS_HAS_SERVICE_INTROSPECTION 1
#else
#define AUTOWARE_COMPONENT_INTERFACE_UTILS_HAS_SERVICE_INTROSPECTION 0
#endif

#include <memory>
#include <string>

namespace autoware::component_interface_utils
{

/// Node-scoped adaptor context. Holds the node pointer and, where ROS 2 service
/// introspection is available, the introspection state resolved once from the
/// "component_interface.service_introspection" string parameter
/// ("off" | "metadata" | "contents", default "off"). Service-call tracing is
/// provided by ROS 2 service introspection (see the service wrappers), not a
/// custom log topic.
struct NodeInterface
{
  using SharedPtr = std::shared_ptr<NodeInterface>;

  explicit NodeInterface(rclcpp::Node * node) : node(node)
  {
#if AUTOWARE_COMPONENT_INTERFACE_UTILS_HAS_SERVICE_INTROSPECTION
    const std::string param = "component_interface.service_introspection";
    const std::string mode = node->has_parameter(param)
                               ? node->get_parameter(param).as_string()
                               : node->declare_parameter<std::string>(param, "off");
    if (mode == "contents") {
      introspection_state = RCL_SERVICE_INTROSPECTION_CONTENTS;
    } else if (mode == "metadata") {
      introspection_state = RCL_SERVICE_INTROSPECTION_METADATA;
    } else {
      introspection_state = RCL_SERVICE_INTROSPECTION_OFF;
    }
#endif
  }

  rclcpp::Node * node;
#if AUTOWARE_COMPONENT_INTERFACE_UTILS_HAS_SERVICE_INTROSPECTION
  rcl_service_introspection_state_t introspection_state = RCL_SERVICE_INTROSPECTION_OFF;
#endif
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__INTERFACE_HPP_
