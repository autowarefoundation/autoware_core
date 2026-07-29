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
#include "autoware/component_interface_specs/system.hpp"
#include "autoware/component_interface_specs/version.hpp"
#include "gtest/gtest.h"
#include "spec_test_utils.hpp"

#include <tuple>

namespace specs = autoware::component_interface_specs;
namespace tu = autoware::component_interface_specs::test_utils;

TEST(system, interface)
{
  {
    using autoware::component_interface_specs::system::OperationModeState;
    OperationModeState state;
    size_t depth = 1;
    EXPECT_EQ(state.depth, depth);
    EXPECT_EQ(state.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(state.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }
}

TEST(system, version)
{
  static_assert(specs::system::version.major == 0);
  static_assert(specs::system::version.minor == 1);
  static_assert(specs::system::version.patch == 0);
  EXPECT_EQ(specs::system::version.major, 0);
}

TEST(system, concept_and_registration)
{
  using specs::system::ChangeAutowareControl;
  using specs::system::ChangeOperationMode;
  using specs::system::MrmState;
  using specs::system::OperationModeState;
  using specs::system::Specs;

  static_assert(specs::InterfaceSpec<MrmState>);
  static_assert(specs::InterfaceSpec<OperationModeState>);
  static_assert(specs::ServiceSpec<ChangeOperationMode>);
  static_assert(specs::ServiceSpec<ChangeAutowareControl>);

  static_assert(tu::has_type<OperationModeState, Specs>::value);
  static_assert(tu::has_type<ChangeOperationMode, Specs>::value);
  static_assert(tu::has_type<ChangeAutowareControl, Specs>::value);
  static_assert(tu::has_type<MrmState, Specs>::value);
  static_assert(std::tuple_size_v<Specs> == 4);
  SUCCEED();
}

TEST(system, mrm_state_qos)
{
  using specs::system::MrmState;
  tu::expect_topic_qos<MrmState>(
    "/system/fail_safe/mrm_state", 1, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE);
}
