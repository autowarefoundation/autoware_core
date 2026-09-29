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
#include "autoware/component_interface_specs/localization.hpp"
#include "gtest/gtest.h"
#include "spec_test_utils.hpp"

#include <tuple>

namespace specs = autoware::component_interface_specs;
namespace tu = autoware::component_interface_specs::test_utils;

TEST(localization, concept_and_registration)
{
  using specs::localization::Acceleration;
  using specs::localization::InitializationState;
  using specs::localization::Initialize;
  using specs::localization::KinematicState;
  using specs::localization::Specs;

  static_assert(specs::InterfaceSpec<KinematicState>);
  static_assert(specs::InterfaceSpec<Acceleration>);
  static_assert(specs::InterfaceSpec<InitializationState>);
  static_assert(specs::ServiceSpec<Initialize>);

  static_assert(tu::has_type<KinematicState, Specs>::value);
  static_assert(tu::has_type<Acceleration, Specs>::value);
  static_assert(tu::has_type<InitializationState, Specs>::value);
  static_assert(tu::has_type<Initialize, Specs>::value);
  static_assert(std::tuple_size_v<Specs> == 4);
  SUCCEED();
}

TEST(localization, interface)
{
  {
    using specs::localization::KinematicState;
    tu::expect_topic_qos<KinematicState>(
      "/localization/kinematic_state", 1, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

  {
    using specs::localization::Acceleration;
    tu::expect_topic_qos<Acceleration>(
      "/localization/acceleration", 1, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

  {
    using specs::localization::InitializationState;
    tu::expect_topic_qos<InitializationState>(
      "/localization/initialization_state", 1, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }
}
