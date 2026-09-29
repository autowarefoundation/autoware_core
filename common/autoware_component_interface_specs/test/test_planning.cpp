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
#include "autoware/component_interface_specs/planning.hpp"
#include "gtest/gtest.h"
#include "spec_test_utils.hpp"

#include <tuple>

namespace specs = autoware::component_interface_specs;
namespace tu = autoware::component_interface_specs::test_utils;

TEST(planning, concept_and_registration)
{
  using specs::planning::ClearRoute;
  using specs::planning::LaneletRoute;
  using specs::planning::RouteState;
  using specs::planning::SetLaneletRoute;
  using specs::planning::SetWaypointRoute;
  using specs::planning::Specs;
  using specs::planning::Trajectory;

  static_assert(specs::InterfaceSpec<Trajectory>);
  static_assert(specs::InterfaceSpec<LaneletRoute>);
  static_assert(specs::InterfaceSpec<RouteState>);
  static_assert(specs::ServiceSpec<SetLaneletRoute>);
  static_assert(specs::ServiceSpec<SetWaypointRoute>);
  static_assert(specs::ServiceSpec<ClearRoute>);

  static_assert(tu::has_type<Trajectory, Specs>::value);
  static_assert(tu::has_type<LaneletRoute, Specs>::value);
  static_assert(tu::has_type<RouteState, Specs>::value);
  static_assert(tu::has_type<SetLaneletRoute, Specs>::value);
  static_assert(tu::has_type<SetWaypointRoute, Specs>::value);
  static_assert(tu::has_type<ClearRoute, Specs>::value);
  static_assert(std::tuple_size_v<Specs> == 6);
  SUCCEED();
}

TEST(planning, interface)
{
  {
    using specs::planning::LaneletRoute;
    tu::expect_topic_qos<LaneletRoute>(
      "/planning/route", 1, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }

  {
    using specs::planning::Trajectory;
    tu::expect_topic_qos<Trajectory>(
      "/planning/trajectory", 1, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }
}
