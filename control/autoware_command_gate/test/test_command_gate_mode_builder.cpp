// Copyright 2026 The Autoware Contributors
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

#include "command_gate_mode_builder.hpp"

#include <builtin_interfaces/msg/time.hpp>

#include <builtin_interfaces/msg/time.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>

#include <gtest/gtest.h>

namespace autoware::control::command_gate
{

TEST(CommandGateModeBuilder, MakeStop)
{
  CommandGateModeBuilder builder;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 42;
  stamp.nanosec = 100;

  const auto outputs = builder.make_stop(stamp);

  EXPECT_EQ(outputs.state.stamp.sec, stamp.sec);
  EXPECT_EQ(outputs.state.stamp.nanosec, stamp.nanosec);
  EXPECT_EQ(outputs.state.mode, autoware_adapi_v1_msgs::msg::OperationModeState::STOP);
  EXPECT_FALSE(outputs.state.is_autoware_control_enabled);
  EXPECT_FALSE(outputs.state.is_in_transition);
  EXPECT_TRUE(outputs.state.is_stop_mode_available);
  EXPECT_TRUE(outputs.state.is_autonomous_mode_available);
  EXPECT_TRUE(outputs.state.is_local_mode_available);
  EXPECT_TRUE(outputs.state.is_remote_mode_available);

  EXPECT_EQ(outputs.gear.stamp.sec, stamp.sec);
  EXPECT_EQ(outputs.gear.stamp.nanosec, stamp.nanosec);
  EXPECT_EQ(outputs.gear.command, autoware_vehicle_msgs::msg::GearCommand::PARK);

  EXPECT_TRUE(outputs.status.success);
  EXPECT_EQ(outputs.status.code, 0);
  EXPECT_EQ(outputs.status.message, "Switched to STOP");
}

TEST(CommandGateModeBuilder, MakeAutonomous)
{
  CommandGateModeBuilder builder;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 99;
  stamp.nanosec = 1;

  const auto outputs = builder.make_autonomous(stamp);

  EXPECT_EQ(outputs.state.stamp.sec, stamp.sec);
  EXPECT_EQ(outputs.state.stamp.nanosec, stamp.nanosec);
  EXPECT_EQ(outputs.state.mode, autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS);
  EXPECT_TRUE(outputs.state.is_autoware_control_enabled);
  EXPECT_FALSE(outputs.state.is_in_transition);
  EXPECT_TRUE(outputs.state.is_stop_mode_available);
  EXPECT_TRUE(outputs.state.is_autonomous_mode_available);
  EXPECT_TRUE(outputs.state.is_local_mode_available);
  EXPECT_TRUE(outputs.state.is_remote_mode_available);

  EXPECT_EQ(outputs.gear.stamp.sec, stamp.sec);
  EXPECT_EQ(outputs.gear.stamp.nanosec, stamp.nanosec);
  EXPECT_EQ(outputs.gear.command, autoware_vehicle_msgs::msg::GearCommand::DRIVE);

  EXPECT_TRUE(outputs.status.success);
  EXPECT_EQ(outputs.status.code, 0);
  EXPECT_EQ(outputs.status.message, "Switched to AUTONOMOUS");
}

}  // namespace autoware::control::command_gate
