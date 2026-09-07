// Copyright 2025 TIER IV, Inc.
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

// Runtime mode query, init()/shutdown() and ok().

#include <string>
#include <vector>

namespace autoware::agnocast_wrapper
{

/// @brief Whether this process runs on Agnocast, from the ENABLE_AGNOCAST environment variable.
/// Read once and fixed for the lifetime of the process; always false in a non-Agnocast build.
bool use_agnocast();

/// @brief Mode-agnostic replacement for rclcpp::init(). Brings up the context ok() reports on: the
/// agnocast one for an AgnocastOnly executable, the rclcpp one for every other.
///
/// @param agnocast_only True if and only if this executable spins one of agnocast's AgnocastOnly*
///   executors. It selects the agnocast context only when ENABLE_AGNOCAST is 1, so such an
///   executable needs an rclcpp executor to fall back to otherwise;
///   autoware_agnocast_wrapper_register_node() fills the flag in and generates that fallback.
/// @return The arguments left once the ROS ones are removed, to hand to
///   rclcpp::NodeOptions::arguments(). Empty when the agnocast context is the one brought up, which
///   parses the command line itself and carries no arguments over.
std::vector<std::string> init(int argc, char const * const * argv, bool agnocast_only = false);

/// @brief Mode-agnostic replacement for rclcpp::shutdown(). Tears down whichever context init()
/// brought up.
void shutdown();

/// @brief Mode-agnostic replacement for rclcpp::ok().
///
/// An executable spinning an AgnocastOnly* executor brings up only the agnocast context; every
/// other executable brings up the rclcpp one. Reports whichever is alive.
bool ok();

}  // namespace autoware::agnocast_wrapper
