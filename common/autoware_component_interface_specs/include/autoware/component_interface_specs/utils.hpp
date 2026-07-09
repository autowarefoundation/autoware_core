// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS__UTILS_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS__UTILS_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rmw/qos_profiles.h>

#include <cstddef>

namespace autoware::component_interface_specs
{

template <typename T>
rclcpp::QoS get_qos()
{
  return rclcpp::QoS{T::depth}.reliability(T::reliability).durability(T::durability);
}

/// ROS 2 gives services a QoS profile just like topics: `rmw_qos_profile_services_default`,
/// which is what every `create_service` / `create_client` call in Autoware takes. No call
/// site overrides it, so all services do run under identical conditions. Service specs
/// therefore carry no QoS members of their own -- the one profile they all share is
/// declared here instead, and `test_service_qos.cpp` asserts it still equals the RMW
/// default it mirrors, so a change to that default surfaces as a test failure rather than
/// as silent drift between the specs and the wire.
namespace service_qos
{
static constexpr std::size_t depth = 10;
static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
}  // namespace service_qos

/// The QoS every service in this package runs on, as the topic-side `get_qos<T>()` returns
/// the QoS of a single topic spec.
inline rclcpp::QoS get_service_qos()
{
  return rclcpp::QoS{service_qos::depth}
    .reliability(service_qos::reliability)
    .durability(service_qos::durability);
}

}  // namespace autoware::component_interface_specs

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS__UTILS_HPP_
