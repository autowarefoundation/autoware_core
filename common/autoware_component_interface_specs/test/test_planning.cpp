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

#include "autoware/component_interface_specs/planning.hpp"
#include "gtest/gtest.h"

TEST(planning, interface)
{
  {
    using autoware::component_interface_specs::planning::LaneletRoute;
    size_t depth = 1;
    EXPECT_EQ(LaneletRoute::depth, depth);
    EXPECT_EQ(LaneletRoute::reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(LaneletRoute::durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    const auto qos = autoware::component_interface_specs::get_qos<LaneletRoute>();
    EXPECT_EQ(qos.depth(), depth);
    EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
    EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
  }

  {
    using autoware::component_interface_specs::planning::Trajectory;
    size_t depth = 1;
    EXPECT_EQ(Trajectory::depth, depth);
    EXPECT_EQ(Trajectory::reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(Trajectory::durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);

    const auto qos = autoware::component_interface_specs::get_qos<Trajectory>();
    EXPECT_EQ(qos.depth(), depth);
    EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
    EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
  }
}
