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

#ifdef USE_AGNOCAST_ENABLED

namespace autoware::agnocast_wrapper
{

// Defaults to zero if the environment variable is missing or invalid.
inline int get_ENABLE_AGNOCAST()
{
  const char * env = std::getenv("ENABLE_AGNOCAST");
  if (env) {
    return std::atoi(env);
  }
  return 0;
}

inline bool use_agnocast()
{
  static const int sv = get_ENABLE_AGNOCAST();
  return sv == 1;
}

/// @brief Mode-agnostic replacement for rclcpp::ok().
///
/// An AgnocastOnly executable initializes only the agnocast context, while mixed-mode and
/// non-Agnocast executables initialize only the rclcpp context. Exactly one is alive in any
/// mode, so the disjunction answers "is this process still running" everywhere.
inline bool ok()
{
  return rclcpp::ok() || agnocast::ok();
}

}  // namespace autoware::agnocast_wrapper

#else

namespace autoware::agnocast_wrapper
{

/// @brief Mode-agnostic replacement for rclcpp::ok() (non-Agnocast build).
inline bool ok()
{
  return rclcpp::ok();
}

}  // namespace autoware::agnocast_wrapper

#endif
