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

#include "autoware/component_interface_specs/map.hpp"
#include "gtest/gtest.h"

TEST(map, interface)
{
  {
    using autoware::component_interface_specs::map::MapProjectorInfo;
    size_t depth = 1;
    EXPECT_EQ(MapProjectorInfo::depth, depth);
    EXPECT_EQ(MapProjectorInfo::reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(MapProjectorInfo::durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    const auto qos = autoware::component_interface_specs::get_qos<MapProjectorInfo>();
    EXPECT_EQ(qos.depth(), depth);
    EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
    EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
  }

  {
    using autoware::component_interface_specs::map::PointCloudMap;
    size_t depth = 1;
    EXPECT_EQ(PointCloudMap::depth, depth);
    EXPECT_EQ(PointCloudMap::reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(PointCloudMap::durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    const auto qos = autoware::component_interface_specs::get_qos<PointCloudMap>();
    EXPECT_EQ(qos.depth(), depth);
    EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
    EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
  }

  {
    using autoware::component_interface_specs::map::VectorMap;
    size_t depth = 1;
    EXPECT_EQ(VectorMap::depth, depth);
    EXPECT_EQ(VectorMap::reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(VectorMap::durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    const auto qos = autoware::component_interface_specs::get_qos<VectorMap>();
    EXPECT_EQ(qos.depth(), depth);
    EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
    EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
  }
}
