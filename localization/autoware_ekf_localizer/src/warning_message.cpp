// Copyright 2023 Autoware Foundation
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

#include "autoware/ekf_localizer/warning_message.hpp"

#include <fmt/core.h>

#include <string>
#include <string_view>

namespace autoware::ekf_localizer
{

std::string pose_delay_step_warning_message(
  const double delay_time, const double delay_time_threshold)
{
  constexpr std::string_view s =
    "Pose delay exceeds the compensation limit, ignored. "
    "delay: {:.3f}[s], limit: {:.3f}[s]";
  return fmt::format(s, delay_time, delay_time_threshold);
}

std::string twist_delay_step_warning_message(
  const double delay_time, const double delay_time_threshold)
{
  constexpr std::string_view s =
    "Twist delay exceeds the compensation limit, ignored. "
    "delay: {:.3f}[s], limit: {:.3f}[s]";
  return fmt::format(s, delay_time, delay_time_threshold);
}

std::string pose_delay_time_warning_message(const double delay_time)
{
  constexpr std::string_view s =
    "Pose time stamp is inappropriate, set delay to 0[s]. delay = {:.3f}";
  return fmt::format(s, delay_time);
}

std::string twist_delay_time_warning_message(const double delay_time)
{
  constexpr std::string_view s =
    "Twist time stamp is inappropriate, set delay to 0[s]. delay = {:.3f}";
  return fmt::format(s, delay_time);
}

std::string mahalanobis_warning_message(const double distance, const double max_distance)
{
  constexpr std::string_view s = "The Mahalanobis distance {:.4f} is over the limit {:.4f}.";
  return fmt::format(s, distance, max_distance);
}

std::string large_ekf_dt_waring_message(const double ekf_dt)
{
  constexpr std::string_view s = "Large ekf_dt_ detected!! ({:.4f} sec) Capped to 10.0 seconds";
  return fmt::format(s, ekf_dt);
}

std::string too_slow_ekf_dt_waring_message(const double ekf_dt)
{
  constexpr std::string_view s =
    "EKF period may be too slow to finish pose smoothing!! ({:.4f} sec)";
  return fmt::format(s, ekf_dt);
}

}  // namespace autoware::ekf_localizer
