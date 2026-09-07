// Copyright 2026 TIER IV, Inc.
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

#pragma once

// Shared by client.hpp and service.hpp: the version gate for ROS 2 service introspection, and the
// argument checking a handle does before it hands the call to a backend.

#include <rclcpp/rclcpp.hpp>

#include <rclcpp/version.h>

// ROS 2 Iron (rclcpp 21) introduced service introspection. Humble (rclcpp 16) ships no
// rcl/service_introspection.h, and neither rclcpp nor agnocast declares configure_introspection()
// there.
#if RCLCPP_VERSION_GTE(21, 0, 0)
#include <rcl/service_introspection.h>
#endif

#include <stdexcept>
#include <string>

#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/agnocast_service_event_publisher.hpp>

#if RCLCPP_VERSION_GTE(21, 0, 0) && !AGNOCAST_HAS_SERVICE_INTROSPECTION
#error AGNOCAST_HAS_SERVICE_INTROSPECTION (agnocast_service_event_publisher.hpp) no longer agrees \
  with RCLCPP_VERSION_GTE(21, 0, 0); align the two gates
#endif

#endif

#if RCLCPP_VERSION_GTE(21, 0, 0)

namespace autoware::agnocast_wrapper::detail
{

/// @brief Reject the arguments the two backends do not handle alike, so that a handle behaves the
/// same whichever one is behind it.
///
/// @throws std::invalid_argument if @p clock is null, including when turning introspection off,
/// or if @p qos uses KeepAll, which degrades to depth 0 through agnocast's ioctl so that backend
/// would retain nothing.
inline void check_introspection_args(
  const char * service_name, const rclcpp::Clock::SharedPtr & clock, const rclcpp::QoS & qos)
{
  if (clock == nullptr) {
    throw std::invalid_argument(
      std::string("configure_introspection(") + service_name +
      "): a clock is required, including when turning introspection off");
  }
  if (qos.history() == rclcpp::HistoryPolicy::KeepAll) {
    throw std::invalid_argument(
      std::string("configure_introspection(") + service_name +
      "): KeepAll history is not supported, use KeepLast instead");
  }
}

}  // namespace autoware::agnocast_wrapper::detail

#endif
