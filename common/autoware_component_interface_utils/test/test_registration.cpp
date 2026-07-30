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

#include "autoware/component_interface_utils/rclcpp.hpp"
#include "autoware/component_interface_utils/rclcpp/registration.hpp"
#include "gtest/gtest.h"

#include <autoware/component_interface_specs/localization.hpp>
#include <autoware/component_interface_specs/version.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/response_status.hpp>

#include <memory>
#include <string>

namespace ciu = autoware::component_interface_utils;

namespace test_specs
{
// A local, unversioned spec: no resolve_domain_version overload anywhere.
// The message type only needs to exist; ResponseStatus comes from a dependency
// this package already declares (autoware_adapi_v1_msgs).
struct Unversioned
{
  using Message = autoware_adapi_v1_msgs::msg::ResponseStatus;
  static constexpr char name[] = "/test/unversioned";
  static constexpr std::size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

// A spec whose domain explicitly opted out via a deleted overload
// (the map::PointCloudMap pattern): the trait must also be false.
struct Excluded
{
  using Message = autoware_adapi_v1_msgs::msg::ResponseStatus;
  static constexpr char name[] = "/test/excluded";
  static constexpr std::size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};
constexpr autoware::component_interface_specs::Version resolve_domain_version(const Excluded &) =
  delete;
}  // namespace test_specs

static_assert(
  ciu::HasDomainVersion<autoware::component_interface_specs::localization::KinematicState>::value);
static_assert(!ciu::HasDomainVersion<test_specs::Unversioned>::value);
static_assert(!ciu::HasDomainVersion<test_specs::Excluded>::value);

TEST(registration, registry_accumulates_and_returns_records)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_registry");
  ciu::NodeInterface interface(node.get());

  ciu::InterfaceRecord record;
  record.kind = ciu::InterfaceRecord::Kind::Topic;
  record.role = ciu::InterfaceRecord::Role::Provide;
  record.interface_name = "/test/topic";
  record.resolved_name = "/test/topic";
  interface.register_interface(record);

  const auto manifest = interface.manifest();
  ASSERT_EQ(manifest.size(), 1u);
  EXPECT_EQ(manifest[0].interface_name, "/test/topic");
  EXPECT_EQ(manifest[0].role, ciu::InterfaceRecord::Role::Provide);
  rclcpp::shutdown();
}

TEST(registration, make_record_resolves_the_domain_version_only_where_declared)
{
  using KinematicState = autoware::component_interface_specs::localization::KinematicState;
  const auto versioned = ciu::make_record<KinematicState>(
    ciu::InterfaceRecord::Kind::Topic, ciu::InterfaceRecord::Role::Provide, KinematicState::name,
    "nav_msgs/msg/Odometry", rmw_qos_profile_default);
  EXPECT_TRUE(versioned.has_version);
  const auto v = resolve_domain_version(KinematicState{});
  EXPECT_EQ(versioned.major, v.major);

  const auto plain = ciu::make_record<test_specs::Unversioned>(
    ciu::InterfaceRecord::Kind::Topic, ciu::InterfaceRecord::Role::Provide,
    test_specs::Unversioned::name, "std_msgs/msg/Empty", rmw_qos_profile_default);
  EXPECT_FALSE(plain.has_version);
}
