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

#include "autoware/component_interface_specs/concepts.hpp"
#include "autoware/component_interface_specs/sensing.hpp"
#include "autoware/component_interface_specs/version.hpp"
#include "gtest/gtest.h"
#include "spec_test_utils.hpp"

#include <tuple>

namespace specs = autoware::component_interface_specs;
namespace tu = autoware::component_interface_specs::test_utils;

TEST(sensing, version)
{
  static_assert(specs::sensing::version.major == 0);
  static_assert(specs::sensing::version.minor == 1);
  static_assert(specs::sensing::version.patch == 0);
  EXPECT_EQ(specs::sensing::version.major, 0);
}

TEST(sensing, concept_and_registration)
{
  using specs::sensing::Specs;
  using specs::sensing::VehicleVelocityConverterTwist;

  static_assert(specs::InterfaceSpec<VehicleVelocityConverterTwist>);
  static_assert(tu::has_type<VehicleVelocityConverterTwist, Specs>::value);
  static_assert(std::tuple_size_v<Specs> == 1);
  SUCCEED();
}

TEST(sensing, vehicle_velocity_converter_twist_qos)
{
  using specs::sensing::VehicleVelocityConverterTwist;
  tu::expect_topic_qos<VehicleVelocityConverterTwist>(
    "/sensing/vehicle_velocity_converter/twist_with_covariance", 10,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}
