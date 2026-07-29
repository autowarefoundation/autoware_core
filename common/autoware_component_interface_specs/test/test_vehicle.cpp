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
#include "autoware/component_interface_specs/vehicle.hpp"
#include "autoware/component_interface_specs/version.hpp"
#include "gtest/gtest.h"
#include "spec_test_utils.hpp"

#include <tuple>

namespace specs = autoware::component_interface_specs;
namespace tu = autoware::component_interface_specs::test_utils;

TEST(vehicle, version)
{
  static_assert(specs::vehicle::version.major == 0);
  static_assert(specs::vehicle::version.minor == 1);
  static_assert(specs::vehicle::version.patch == 0);
  EXPECT_EQ(specs::vehicle::version.major, 0);
}

TEST(vehicle, concept_and_registration)
{
  using specs::vehicle::ControlModeStatus;
  using specs::vehicle::GearStatus;
  using specs::vehicle::HazardLightStatus;
  using specs::vehicle::Specs;
  using specs::vehicle::SteeringStatus;
  using specs::vehicle::TurnIndicatorStatus;
  using specs::vehicle::VelocityStatus;

  static_assert(specs::InterfaceSpec<VelocityStatus>);
  static_assert(specs::InterfaceSpec<SteeringStatus>);
  static_assert(specs::InterfaceSpec<GearStatus>);
  static_assert(specs::InterfaceSpec<TurnIndicatorStatus>);
  static_assert(specs::InterfaceSpec<HazardLightStatus>);
  static_assert(specs::InterfaceSpec<ControlModeStatus>);

  static_assert(tu::has_type<VelocityStatus, Specs>::value);
  static_assert(tu::has_type<SteeringStatus, Specs>::value);
  static_assert(tu::has_type<GearStatus, Specs>::value);
  static_assert(tu::has_type<TurnIndicatorStatus, Specs>::value);
  static_assert(tu::has_type<HazardLightStatus, Specs>::value);
  static_assert(tu::has_type<ControlModeStatus, Specs>::value);
  static_assert(std::tuple_size_v<Specs> == 6);
  SUCCEED();
}

TEST(vehicle, velocity_status_qos)
{
  using specs::vehicle::VelocityStatus;
  tu::expect_topic_qos<VelocityStatus>(
    "/vehicle/status/velocity_status", 1, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(vehicle, control_mode_status_qos)
{
  using specs::vehicle::ControlModeStatus;
  tu::expect_topic_qos<ControlModeStatus>(
    "/vehicle/status/control_mode", 1, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(vehicle, interface)
{
  {
    using autoware::component_interface_specs::vehicle::SteeringStatus;
    size_t depth = 1;
    EXPECT_EQ(SteeringStatus::depth, depth);
    EXPECT_EQ(SteeringStatus::reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(SteeringStatus::durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);

    const auto qos = autoware::component_interface_specs::get_qos<SteeringStatus>();
    EXPECT_EQ(qos.depth(), depth);
    EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
    EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
  }

  {
    using autoware::component_interface_specs::vehicle::GearStatus;
    size_t depth = 1;
    EXPECT_EQ(GearStatus::depth, depth);
    EXPECT_EQ(GearStatus::reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(GearStatus::durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);

    const auto qos = autoware::component_interface_specs::get_qos<GearStatus>();
    EXPECT_EQ(qos.depth(), depth);
    EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
    EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
  }

  {
    using autoware::component_interface_specs::vehicle::TurnIndicatorStatus;
    size_t depth = 1;
    EXPECT_EQ(TurnIndicatorStatus::depth, depth);
    EXPECT_EQ(TurnIndicatorStatus::reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(TurnIndicatorStatus::durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);

    const auto qos = autoware::component_interface_specs::get_qos<TurnIndicatorStatus>();
    EXPECT_EQ(qos.depth(), depth);
    EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
    EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
  }

  {
    using autoware::component_interface_specs::vehicle::HazardLightStatus;
    size_t depth = 1;
    EXPECT_EQ(HazardLightStatus::depth, depth);
    EXPECT_EQ(HazardLightStatus::reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(HazardLightStatus::durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);

    const auto qos = autoware::component_interface_specs::get_qos<HazardLightStatus>();
    EXPECT_EQ(qos.depth(), depth);
    EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
    EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
  }
}
