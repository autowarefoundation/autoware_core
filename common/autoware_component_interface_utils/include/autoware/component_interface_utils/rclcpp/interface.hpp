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

#include "autoware/component_interface_utils/rclcpp/registration.hpp"

#include <rclcpp/rclcpp.hpp>

#include <rclcpp/version.h>

// ROS 2 Iron (rclcpp 21) introduced both service introspection
// (Client/Service::configure_introspection, rcl/service_introspection.h) and the
// rclcpp::QoS overloads of create_client/create_service. ROS 2 Humble (rclcpp 16)
// has neither: it ships no rcl/service_introspection.h and exposes only the
// rmw_qos_profile_t QoS overload. Gate both on the rclcpp version so this package
// builds on Humble and Jazzy alike; service introspection is unavailable on Humble.
#if RCLCPP_VERSION_GTE(21, 0, 0)
#include <rcl/service_introspection.h>
#define AUTOWARE_COMPONENT_INTERFACE_UTILS_RCLCPP_GE_IRON 1
#else
#define AUTOWARE_COMPONENT_INTERFACE_UTILS_RCLCPP_GE_IRON 0
#endif

#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace autoware::component_interface_utils
{

#if AUTOWARE_COMPONENT_INTERFACE_UTILS_RCLCPP_GE_IRON
/// True when the client/service handle supports ROS 2 service introspection. That is an rcl
/// feature, so a node type whose services do not pass through rcl -- Agnocast -- has no
/// counterpart and simply cannot be introspected. Spelled as a detection idiom so this package
/// stays independent of any particular node type.
template <class T, class = void>
struct has_configure_introspection : std::false_type
{
};
template <class T>
struct has_configure_introspection<
  T, std::void_t<decltype(std::declval<T &>().configure_introspection(
       std::declval<rclcpp::Clock::SharedPtr>(), std::declval<const rclcpp::QoS &>(),
       std::declval<rcl_service_introspection_state_t>()))>> : std::true_type
{
};
#endif

/// Node-scoped adaptor context. Holds the node pointer and, where ROS 2 service
/// introspection is available, the introspection state resolved once from the
/// "component_interface.service_introspection" string parameter
/// ("off" | "metadata" | "contents", default "off"). Service-call tracing is
/// provided by ROS 2 service introspection (see the service wrappers), not a
/// custom log topic.
template <class NodeT = rclcpp::Node>
struct NodeInterface
{
  using SharedPtr = std::shared_ptr<NodeInterface>;
  using NodeType = NodeT;

  /// D is deduced separately from NodeT so a derived node does not hijack it, as in NodeAdaptor.
  template <class D>
  explicit NodeInterface(D * node) : node(node)
  {
#if AUTOWARE_COMPONENT_INTERFACE_UTILS_RCLCPP_GE_IRON
    const std::string param = "component_interface.service_introspection";
    const std::string mode = node->has_parameter(param)
                               ? node->get_parameter(param).as_string()
                               : node->template declare_parameter<std::string>(param, "off");
    if (mode == "contents") {
      introspection_state = RCL_SERVICE_INTROSPECTION_CONTENTS;
    } else if (mode == "metadata") {
      introspection_state = RCL_SERVICE_INTROSPECTION_METADATA;
    } else {
      introspection_state = RCL_SERVICE_INTROSPECTION_OFF;
    }
#endif
  }

  NodeT * node;
#if AUTOWARE_COMPONENT_INTERFACE_UTILS_RCLCPP_GE_IRON
  rcl_service_introspection_state_t introspection_state = RCL_SERVICE_INTROSPECTION_OFF;
#endif

  /// Append one endpoint record. Called by the create/init implementation
  /// layer; thread-safe because composed nodes may construct interfaces from
  /// multiple callback groups.
  void register_interface(InterfaceRecord record)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    records_.push_back(std::move(record));
  }

  /// Snapshot of everything this node registered.
  std::vector<InterfaceRecord> manifest() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return records_;
  }

private:
  mutable std::mutex mutex_;
  std::vector<InterfaceRecord> records_;
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__INTERFACE_HPP_
