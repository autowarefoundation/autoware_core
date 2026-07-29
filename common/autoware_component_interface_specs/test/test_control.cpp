// Copyright 2023 The Autoware Contributors
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

#include "autoware/component_interface_specs/concepts.hpp"
#include "autoware/component_interface_specs/control.hpp"
#include "autoware/component_interface_specs/version.hpp"
#include "gtest/gtest.h"
#include "spec_test_utils.hpp"

#include <tuple>

namespace specs = autoware::component_interface_specs;
namespace tu = autoware::component_interface_specs::test_utils;

TEST(control, version)
{
  static_assert(specs::control::version.major == 0);
  static_assert(specs::control::version.minor == 1);
  static_assert(specs::control::version.patch == 0);
  EXPECT_EQ(specs::control::version.major, 0);
}

TEST(control, concept_and_registration)
{
  using specs::control::ControlCommand;
  using specs::control::GearCommand;
  using specs::control::HazardLightsCommand;
  using specs::control::Specs;
  using specs::control::TurnIndicatorsCommand;

  static_assert(specs::InterfaceSpec<ControlCommand>);
  static_assert(specs::InterfaceSpec<GearCommand>);
  static_assert(specs::InterfaceSpec<TurnIndicatorsCommand>);
  static_assert(specs::InterfaceSpec<HazardLightsCommand>);

  static_assert(tu::has_type<ControlCommand, Specs>::value);
  static_assert(tu::has_type<GearCommand, Specs>::value);
  static_assert(tu::has_type<TurnIndicatorsCommand, Specs>::value);
  static_assert(tu::has_type<HazardLightsCommand, Specs>::value);
  static_assert(std::tuple_size_v<Specs> == 4);
  SUCCEED();
}

TEST(control, interface)
{
  using autoware::component_interface_specs::control::ControlCommand;
  size_t depth = 1;
  EXPECT_EQ(ControlCommand::depth, depth);
  EXPECT_EQ(ControlCommand::reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(ControlCommand::durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);

  const auto qos = autoware::component_interface_specs::get_qos<ControlCommand>();
  EXPECT_EQ(qos.depth(), depth);
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST(control, gear_command_qos)
{
  using specs::control::GearCommand;
  tu::expect_topic_qos<GearCommand>(
    "/control/command/gear_cmd", 1, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(control, turn_indicators_command_qos)
{
  using specs::control::TurnIndicatorsCommand;
  tu::expect_topic_qos<TurnIndicatorsCommand>(
    "/control/command/turn_indicators_cmd", 1, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(control, hazard_lights_command_qos)
{
  using specs::control::HazardLightsCommand;
  tu::expect_topic_qos<HazardLightsCommand>(
    "/control/command/hazard_lights_cmd", 1, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE);
}
