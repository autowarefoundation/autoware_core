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

#include "autoware/agnocast_wrapper/runtime.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

#ifdef USE_AGNOCAST_ENABLED
#include <agnocast/agnocast.hpp>

#include <cstdlib>
#endif

namespace autoware::agnocast_wrapper
{

#ifdef USE_AGNOCAST_ENABLED

namespace
{

// Defaults to zero if the environment variable is missing or invalid.
int get_ENABLE_AGNOCAST()
{
  const char * env = std::getenv("ENABLE_AGNOCAST");
  if (env) {
    return std::atoi(env);
  }
  return 0;
}

// Set by init() so that shutdown() cannot disagree about which context is up.
bool g_agnocast_only_context = false;

}  // namespace

bool use_agnocast()
{
  static const int sv = get_ENABLE_AGNOCAST();
  return sv == 1;
}

std::vector<std::string> init(int argc, char const * const * argv, const bool agnocast_only)
{
  g_agnocast_only_context = use_agnocast() && agnocast_only;
  if (g_agnocast_only_context) {
    agnocast::init(argc, argv);
    return {};
  }
  return rclcpp::init_and_remove_ros_arguments(argc, argv);
}

void shutdown()
{
  if (g_agnocast_only_context) {
    agnocast::shutdown();
    return;
  }
  rclcpp::shutdown();
}

bool ok()
{
  return rclcpp::ok() || agnocast::ok();
}

#else

std::vector<std::string> init(int argc, char const * const * argv, const bool /* agnocast_only */)
{
  return rclcpp::init_and_remove_ros_arguments(argc, argv);
}

void shutdown()
{
  rclcpp::shutdown();
}

bool ok()
{
  return rclcpp::ok();
}

#endif

}  // namespace autoware::agnocast_wrapper
