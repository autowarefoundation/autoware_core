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

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_common_msgs/msg/response_status.hpp>
#include <autoware_system_msgs/srv/change_operation_mode.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>

#include <gtest/gtest.h>

namespace autoware::control::command_gate
{

using Request = autoware_system_msgs::srv::ChangeOperationMode::Request;

namespace
{
Request make_request(uint16_t mode)
{
  Request request;
  request.mode = mode;
  return request;
}
}  // namespace

TEST(CommandGateModeBuilder, MakeStop)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 42;
  stamp.nanosec = 100;

  const auto outputs = CommandGateModeBuilder::make_stop(stamp);

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
}

TEST(CommandGateModeBuilder, MakeAutonomous)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 99;
  stamp.nanosec = 1;

  const auto outputs = CommandGateModeBuilder::make_autonomous(stamp);

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
}

TEST(CommandGateModeBuilder, MakeLocal)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 7;
  stamp.nanosec = 77;

  const auto outputs = CommandGateModeBuilder::make_local(stamp);

  EXPECT_EQ(outputs.state.stamp.sec, stamp.sec);
  EXPECT_EQ(outputs.state.stamp.nanosec, stamp.nanosec);
  EXPECT_EQ(outputs.state.mode, autoware_adapi_v1_msgs::msg::OperationModeState::LOCAL);
  EXPECT_FALSE(outputs.state.is_autoware_control_enabled);
  EXPECT_FALSE(outputs.state.is_in_transition);
  EXPECT_TRUE(outputs.state.is_stop_mode_available);
  EXPECT_TRUE(outputs.state.is_autonomous_mode_available);
  EXPECT_TRUE(outputs.state.is_local_mode_available);
  EXPECT_TRUE(outputs.state.is_remote_mode_available);

  EXPECT_EQ(outputs.gear.stamp.sec, stamp.sec);
  EXPECT_EQ(outputs.gear.stamp.nanosec, stamp.nanosec);
  EXPECT_EQ(outputs.gear.command, autoware_vehicle_msgs::msg::GearCommand::NONE);
}

TEST(CommandGateModeBuilder, MakeRemote)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 13;
  stamp.nanosec = 130;

  const auto outputs = CommandGateModeBuilder::make_remote(stamp);

  EXPECT_EQ(outputs.state.stamp.sec, stamp.sec);
  EXPECT_EQ(outputs.state.stamp.nanosec, stamp.nanosec);
  EXPECT_EQ(outputs.state.mode, autoware_adapi_v1_msgs::msg::OperationModeState::REMOTE);
  EXPECT_FALSE(outputs.state.is_autoware_control_enabled);
  EXPECT_FALSE(outputs.state.is_in_transition);
  EXPECT_TRUE(outputs.state.is_stop_mode_available);
  EXPECT_TRUE(outputs.state.is_autonomous_mode_available);
  EXPECT_TRUE(outputs.state.is_local_mode_available);
  EXPECT_TRUE(outputs.state.is_remote_mode_available);

  EXPECT_EQ(outputs.gear.stamp.sec, stamp.sec);
  EXPECT_EQ(outputs.gear.stamp.nanosec, stamp.nanosec);
  EXPECT_EQ(outputs.gear.command, autoware_vehicle_msgs::msg::GearCommand::NONE);
}

TEST(CommandGateModeBuilder, CreateModeOutputStop)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 5;
  stamp.nanosec = 50;

  const auto outputs =
    CommandGateModeBuilder::create_mode_output(make_request(Request::STOP), stamp);

  ASSERT_TRUE(outputs.has_value());
  EXPECT_EQ(outputs->state.mode, autoware_adapi_v1_msgs::msg::OperationModeState::STOP);
  EXPECT_FALSE(outputs->state.is_autoware_control_enabled);
  EXPECT_EQ(outputs->gear.command, autoware_vehicle_msgs::msg::GearCommand::PARK);
  EXPECT_EQ(outputs->state.stamp.sec, stamp.sec);
  EXPECT_EQ(outputs->state.stamp.nanosec, stamp.nanosec);
  EXPECT_EQ(outputs->gear.stamp.sec, stamp.sec);
  EXPECT_EQ(outputs->gear.stamp.nanosec, stamp.nanosec);
}

TEST(CommandGateModeBuilder, CreateModeOutputAutonomous)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 99;
  stamp.nanosec = 1;

  const auto outputs =
    CommandGateModeBuilder::create_mode_output(make_request(Request::AUTONOMOUS), stamp);

  ASSERT_TRUE(outputs.has_value());
  EXPECT_EQ(outputs->state.mode, autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS);
  EXPECT_TRUE(outputs->state.is_autoware_control_enabled);
  EXPECT_EQ(outputs->gear.command, autoware_vehicle_msgs::msg::GearCommand::DRIVE);
  EXPECT_EQ(outputs->state.stamp.sec, stamp.sec);
  EXPECT_EQ(outputs->state.stamp.nanosec, stamp.nanosec);
}

TEST(CommandGateModeBuilder, CreateModeOutputLocal)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 7;
  stamp.nanosec = 77;

  const auto outputs =
    CommandGateModeBuilder::create_mode_output(make_request(Request::LOCAL), stamp);

  ASSERT_TRUE(outputs.has_value());
  EXPECT_EQ(outputs->state.mode, autoware_adapi_v1_msgs::msg::OperationModeState::LOCAL);
  EXPECT_EQ(outputs->gear.command, autoware_vehicle_msgs::msg::GearCommand::NONE);
  EXPECT_EQ(outputs->state.stamp.sec, stamp.sec);
  EXPECT_EQ(outputs->state.stamp.nanosec, stamp.nanosec);
}

TEST(CommandGateModeBuilder, CreateModeOutputRemote)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 13;
  stamp.nanosec = 130;

  const auto outputs =
    CommandGateModeBuilder::create_mode_output(make_request(Request::REMOTE), stamp);

  ASSERT_TRUE(outputs.has_value());
  EXPECT_EQ(outputs->state.mode, autoware_adapi_v1_msgs::msg::OperationModeState::REMOTE);
  EXPECT_EQ(outputs->gear.command, autoware_vehicle_msgs::msg::GearCommand::NONE);
  EXPECT_EQ(outputs->state.stamp.sec, stamp.sec);
  EXPECT_EQ(outputs->state.stamp.nanosec, stamp.nanosec);
}

TEST(CommandGateModeBuilder, CreateModeOutputUnknownReturnsNullopt)
{
  builtin_interfaces::msg::Time stamp;

  // 0 is reserved (no mode constant) and any value outside STOP/AUTONOMOUS/LOCAL/REMOTE
  // must publish nothing so the node only reports a PARAMETER_ERROR.
  EXPECT_FALSE(CommandGateModeBuilder::create_mode_output(make_request(0), stamp).has_value());
  EXPECT_FALSE(CommandGateModeBuilder::create_mode_output(make_request(5), stamp).has_value());
  EXPECT_FALSE(CommandGateModeBuilder::create_mode_output(make_request(255), stamp).has_value());
}

TEST(CommandGateModeBuilder, CreateResponseStop)
{
  builtin_interfaces::msg::Time stamp;

  const auto response = CommandGateModeBuilder::create_response(make_request(Request::STOP), stamp);

  EXPECT_TRUE(response.status.success);
  EXPECT_EQ(response.status.code, 0);
  EXPECT_EQ(response.status.message, "Switched to STOP");
}

TEST(CommandGateModeBuilder, CreateResponseAutonomous)
{
  builtin_interfaces::msg::Time stamp;

  const auto response =
    CommandGateModeBuilder::create_response(make_request(Request::AUTONOMOUS), stamp);

  EXPECT_TRUE(response.status.success);
  EXPECT_EQ(response.status.code, 0);
  EXPECT_EQ(response.status.message, "Switched to AUTONOMOUS");
}

TEST(CommandGateModeBuilder, CreateResponseLocal)
{
  builtin_interfaces::msg::Time stamp;

  const auto response =
    CommandGateModeBuilder::create_response(make_request(Request::LOCAL), stamp);

  EXPECT_TRUE(response.status.success);
  EXPECT_EQ(response.status.code, 0);
  EXPECT_EQ(response.status.message, "Switched to LOCAL");
}

TEST(CommandGateModeBuilder, CreateResponseRemote)
{
  builtin_interfaces::msg::Time stamp;

  const auto response =
    CommandGateModeBuilder::create_response(make_request(Request::REMOTE), stamp);

  EXPECT_TRUE(response.status.success);
  EXPECT_EQ(response.status.code, 0);
  EXPECT_EQ(response.status.message, "Switched to REMOTE");
}

TEST(CommandGateModeBuilder, CreateResponseUnknownReportsParameterError)
{
  builtin_interfaces::msg::Time stamp;

  // Unknown modes must surface a PARAMETER_ERROR while create_mode_output publishes nothing.
  for (const uint16_t mode :
       {static_cast<uint16_t>(0), static_cast<uint16_t>(5), static_cast<uint16_t>(255)}) {
    const auto response = CommandGateModeBuilder::create_response(make_request(mode), stamp);
    EXPECT_FALSE(response.status.success);
    EXPECT_EQ(response.status.code, autoware_common_msgs::msg::ResponseStatus::PARAMETER_ERROR);
    EXPECT_EQ(response.status.message, "Unknown operation mode requested.");
  }
}

}  // namespace autoware::control::command_gate
