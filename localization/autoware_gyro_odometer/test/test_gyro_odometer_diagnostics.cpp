// Copyright 2025 Autoware Foundation
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

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <gtest/gtest.h>

namespace autoware::gyro_odometer
{

// Pure-decision tests: no ROS context, deterministic state in / entries out.

// determine_diagnostics: everything healthy -> OK, no entries, empty log.
TEST(GyroOdometerDiagnostics, DetermineDiagnosticsOkWhenHealthy)
{
  DiagnosticsState state;
  state.vehicle_twist_arrived = true;
  state.imu_arrived = true;
  state.is_succeed_transform_imu = true;
  state.latest_vehicle_twist_dt = 0.01;
  state.latest_imu_dt = 0.01;
  state.message_timeout_sec = 1.0;
  state.output_frame = "base_link";

  const DiagnosticsResult result = determine_diagnostics(state);

  EXPECT_EQ(result.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_TRUE(result.entries.empty());
  EXPECT_TRUE(result.log_message.empty());
}

// determine_diagnostics: missing inputs raise WARN. This pins the bug fix: the aggregated level
// must reflect the highest triggered severity (previously the local 'level' stayed OK and the WARN
// log was unreachable).
TEST(GyroOdometerDiagnostics, DetermineDiagnosticsWarnWhenNotArrived)
{
  DiagnosticsState state;
  state.vehicle_twist_arrived = false;
  state.imu_arrived = false;
  state.is_succeed_transform_imu = true;
  state.latest_vehicle_twist_dt = 0.0;
  state.latest_imu_dt = 0.0;
  state.message_timeout_sec = 1.0;

  const DiagnosticsResult result = determine_diagnostics(state);

  EXPECT_EQ(result.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  ASSERT_EQ(result.entries.size(), 2u);
  EXPECT_EQ(result.entries[0].level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(result.entries[1].level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

// determine_diagnostics: a timeout raises ERROR.
TEST(GyroOdometerDiagnostics, DetermineDiagnosticsErrorOnTimeout)
{
  DiagnosticsState state;
  state.vehicle_twist_arrived = true;
  state.imu_arrived = true;
  state.is_succeed_transform_imu = true;
  state.latest_vehicle_twist_dt = 2.0;  // > timeout
  state.latest_imu_dt = 0.0;
  state.message_timeout_sec = 1.0;

  const DiagnosticsResult result = determine_diagnostics(state);

  EXPECT_EQ(result.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  ASSERT_EQ(result.entries.size(), 1u);
  EXPECT_EQ(result.entries[0].level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

// determine_diagnostics: TF failure raises ERROR.
TEST(GyroOdometerDiagnostics, DetermineDiagnosticsErrorOnTransformFailure)
{
  DiagnosticsState state;
  state.vehicle_twist_arrived = true;
  state.imu_arrived = true;
  state.is_succeed_transform_imu = false;  // TF lookup failed
  state.latest_vehicle_twist_dt = 0.0;
  state.latest_imu_dt = 0.0;
  state.message_timeout_sec = 1.0;
  state.output_frame = "base_link";

  const DiagnosticsResult result = determine_diagnostics(state);

  EXPECT_EQ(result.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  ASSERT_EQ(result.entries.size(), 1u);
  EXPECT_EQ(result.entries[0].level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

// determine_diagnostics: when both WARN and ERROR conditions trigger, the aggregated level is the
// ERROR maximum, while every entry keeps its own level and the entry order is preserved.
TEST(GyroOdometerDiagnostics, DetermineDiagnosticsAggregatesToMaxSeverity)
{
  DiagnosticsState state;
  state.vehicle_twist_arrived = false;  // WARN
  state.imu_arrived = true;
  state.is_succeed_transform_imu = false;  // ERROR
  state.latest_vehicle_twist_dt = 0.0;
  state.latest_imu_dt = 0.0;
  state.message_timeout_sec = 1.0;
  state.output_frame = "base_link";

  const DiagnosticsResult result = determine_diagnostics(state);

  EXPECT_EQ(result.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  ASSERT_EQ(result.entries.size(), 2u);
  EXPECT_EQ(result.entries[0].level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(result.entries[1].level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

}  // namespace autoware::gyro_odometer
