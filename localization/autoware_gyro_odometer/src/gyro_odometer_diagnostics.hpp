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

#ifndef GYRO_ODOMETER_DIAGNOSTICS_HPP_
#define GYRO_ODOMETER_DIAGNOSTICS_HPP_

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace autoware::gyro_odometer
{

/// \brief A single diagnostics finding: a severity level paired with its human-readable message.
struct DiagnosticsEntry
{
  int8_t level{diagnostic_msgs::msg::DiagnosticStatus::OK};
  std::string message;
};

/// \brief Inputs needed to decide the gyro_odometer diagnostics level and messages.
struct DiagnosticsState
{
  bool vehicle_twist_arrived{false};
  bool imu_arrived{false};
  bool is_succeed_transform_imu{false};
  bool is_frame_id_consistent{true};
  double latest_vehicle_twist_dt{0.0};
  double latest_imu_dt{0.0};
  double message_timeout_sec{0.0};
  std::string output_frame;
};

/// \brief Result of evaluating the diagnostics state.
///
/// \c entries holds one element per triggered condition (level + message), preserving the node's
/// historical check order. \c level is the maximum severity across all entries. \c log_message is
/// every entry message concatenated, each terminated by "; ".
struct DiagnosticsResult
{
  std::vector<DiagnosticsEntry> entries;
  int8_t level{diagnostic_msgs::msg::DiagnosticStatus::OK};
  std::string log_message;
};

/// \brief Evaluate the diagnostics state into per-condition entries, an aggregated level, and a
/// concatenated log message.
///
/// Pure function: no node, clock, or interface dependency. The messages are byte-for-byte identical
/// to the node's original strings (the TF-failure message embeds \c state.output_frame).
DiagnosticsResult determine_diagnostics(const DiagnosticsState & state);

}  // namespace autoware::gyro_odometer

#endif  // GYRO_ODOMETER_DIAGNOSTICS_HPP_
