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

TEST(control, concept_and_registration)
{
  using specs::control::ControlCommand;
  using specs::control::ControlModeRequest;
  using specs::control::GearCommand;
  using specs::control::HazardLightsCommand;
  using specs::control::PredictedTrajectory;
  using specs::control::Specs;
  using specs::control::TurnIndicatorsCommand;

  static_assert(specs::InterfaceSpec<ControlCommand>);
  static_assert(specs::InterfaceSpec<GearCommand>);
  static_assert(specs::InterfaceSpec<TurnIndicatorsCommand>);
  static_assert(specs::InterfaceSpec<HazardLightsCommand>);
  static_assert(specs::InterfaceSpec<PredictedTrajectory>);
  static_assert(specs::ServiceSpec<ControlModeRequest>);

  static_assert(tu::has_type<ControlCommand, Specs>::value);
  static_assert(tu::has_type<GearCommand, Specs>::value);
  static_assert(tu::has_type<TurnIndicatorsCommand, Specs>::value);
  static_assert(tu::has_type<HazardLightsCommand, Specs>::value);
  static_assert(tu::has_type<PredictedTrajectory, Specs>::value);
  static_assert(tu::has_type<ControlModeRequest, Specs>::value);
  static_assert(std::tuple_size_v<Specs> == 6);
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
