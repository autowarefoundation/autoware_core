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

#include "autoware/component_interface_specs/control.hpp"
#include "gtest/gtest.h"

TEST(control, interface)
{
  using autoware::component_interface_specs::control::ControlCommand;
  size_t depth = 1;
  EXPECT_EQ(ControlCommand::depth, depth);
  EXPECT_EQ(ControlCommand::reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(ControlCommand::durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  const auto qos = autoware::component_interface_specs::get_qos<ControlCommand>();
  EXPECT_EQ(qos.depth(), depth);
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
}
