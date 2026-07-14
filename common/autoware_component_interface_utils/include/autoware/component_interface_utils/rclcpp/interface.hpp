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

#include <rcl/service_introspection.h>

#include <memory>
#include <string>

namespace autoware::component_interface_utils
{

/// Node-scoped adaptor context. Holds the node pointer and the resolved
/// service-introspection state. Service tracing is provided by ROS 2 service
/// introspection (see configure_introspection in the service wrappers), not a
/// custom log topic. The state is read once from the
/// "component_interface.service_introspection" string parameter
/// ("off" | "metadata" | "contents"), defaulting to "off".
struct NodeInterface
{
  using SharedPtr = std::shared_ptr<NodeInterface>;

  explicit NodeInterface(rclcpp::Node * node) : node(node)
  {
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
  }

  rclcpp::Node * node;
  rcl_service_introspection_state_t introspection_state = RCL_SERVICE_INTROSPECTION_OFF;
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__INTERFACE_HPP_
