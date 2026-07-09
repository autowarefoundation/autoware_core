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

#include "autoware/component_interface_specs/utils.hpp"
#include "gtest/gtest.h"

#include <rmw/qos_profiles.h>

namespace cis = autoware::component_interface_specs;

// Service specs carry no QoS members: every service in this package runs on ROS 2's default
// service profile, which no call site overrides. `service_qos` restates that profile so the
// manifest can record it. Pin the two together here -- if a ROS release ever changes the
// default, this fails loudly instead of letting the manifest describe a profile the services
// no longer run on.
TEST(service_qos, mirrors_the_rmw_service_default)
{
  EXPECT_EQ(rmw_qos_profile_services_default.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(cis::service_qos::depth, rmw_qos_profile_services_default.depth);
  EXPECT_EQ(cis::service_qos::reliability, rmw_qos_profile_services_default.reliability);
  EXPECT_EQ(cis::service_qos::durability, rmw_qos_profile_services_default.durability);
}

TEST(service_qos, get_service_qos_returns_the_declared_policies)
{
  const auto qos = cis::get_service_qos();
  EXPECT_EQ(qos.depth(), cis::service_qos::depth);
  EXPECT_EQ(qos.history(), rclcpp::HistoryPolicy::KeepLast);
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
}
