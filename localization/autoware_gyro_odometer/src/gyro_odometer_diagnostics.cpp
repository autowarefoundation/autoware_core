// Copyright 2026 Autoware Foundation
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

#include "gyro_odometer_diagnostics.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <string>

namespace autoware::gyro_odometer
{

DiagnosticsResult determine_diagnostics(const DiagnosticsState & state)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  DiagnosticsResult result;

  const auto raise = [&result](const int8_t level, const std::string & message) {
    result.entries.push_back({level, message});
    result.level = std::max(result.level, level);
    result.log_message += message;
    result.log_message += "; ";
  };

  if (!state.vehicle_twist_arrived) {
    raise(DiagnosticStatus::WARN, "Twist msg has not been arrived yet.");
  }
  if (!state.imu_arrived) {
    raise(DiagnosticStatus::WARN, "IMU msg has not been arrived yet.");
  }
  if (state.latest_vehicle_twist_dt > state.message_timeout_sec) {
    const std::string message = fmt::format(
      "Vehicle twist msg is timeout. vehicle_twist_dt: {}[sec], tolerance {}[sec]",
      state.latest_vehicle_twist_dt, state.message_timeout_sec);
    raise(DiagnosticStatus::ERROR, message);
  }
  if (state.latest_imu_dt > state.message_timeout_sec) {
    const std::string message = fmt::format(
      "IMU msg is timeout. imu_dt: {}[sec], tolerance {}[sec]", state.latest_imu_dt,
      state.message_timeout_sec);
    raise(DiagnosticStatus::ERROR, message);
  }
  if (!state.is_succeed_transform_imu) {
    raise(
      DiagnosticStatus::ERROR,
      "Please publish TF from " + state.output_frame + " to frame of IMU.");
  }

  return result;
}

}  // namespace autoware::gyro_odometer
