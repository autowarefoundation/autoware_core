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
#include <autoware/component_interface_specs/map.hpp>
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

// A local, versioned spec whose domain version has three distinct, non-zero
// components. Asserting against {2, 3, 4} can never coincide with
// InterfaceRecord's own {0, 0, 0} defaults, unlike a real domain (all of which
// are currently 0.x.y).
struct Versioned
{
  using Message = autoware_adapi_v1_msgs::msg::ResponseStatus;
  static constexpr char name[] = "/test/versioned";
  static constexpr std::size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};
constexpr autoware::component_interface_specs::Version resolve_domain_version(const Versioned &)
{
  return {2, 3, 4};
}
}  // namespace test_specs

static_assert(
  ciu::HasDomainVersion<autoware::component_interface_specs::localization::KinematicState>::value);
static_assert(!ciu::HasDomainVersion<test_specs::Unversioned>::value);
static_assert(!ciu::HasDomainVersion<test_specs::Excluded>::value);
// PointCloudMap's domain declares the DEFINE_DOMAIN template overload for every spec
// in its Specs tuple *and* a deleted non-template overload for this one spec; the
// trait must still resolve to false, exercising the non-template-beats-template
// overload-resolution tiebreak rather than the single-deleted-candidate case above.
static_assert(
  !ciu::HasDomainVersion<autoware::component_interface_specs::map::PointCloudMap>::value);

TEST(registration, registry_accumulates_and_returns_records)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_registry");
  ciu::NodeInterface interface(node.get());

  ciu::InterfaceRecord first_record;
  first_record.kind = ciu::InterfaceRecord::Kind::Topic;
  first_record.role = ciu::InterfaceRecord::Role::Provide;
  first_record.interface_name = "/test/topic";
  first_record.resolved_name = "/test/topic";
  interface.register_interface(first_record);

  ciu::InterfaceRecord second_record;
  second_record.kind = ciu::InterfaceRecord::Kind::Service;
  second_record.role = ciu::InterfaceRecord::Role::Require;
  second_record.interface_name = "/test/service";
  second_record.resolved_name = "/test/service";
  interface.register_interface(second_record);

  const auto manifest = interface.manifest();
  ASSERT_EQ(manifest.size(), 2u);
  EXPECT_EQ(manifest[0].interface_name, "/test/topic");
  EXPECT_EQ(manifest[0].role, ciu::InterfaceRecord::Role::Provide);
  EXPECT_EQ(manifest[1].interface_name, "/test/service");
  EXPECT_EQ(manifest[1].role, ciu::InterfaceRecord::Role::Require);
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
  EXPECT_EQ(versioned.minor, v.minor);

  // The fields make_record forwards rather than resolves from the domain
  // version: the caller-supplied name/type and the QoS actually applied
  // (rmw_qos_profile_default is depth 10 / RELIABLE / VOLATILE, which differs
  // from InterfaceRecord's own defaults of 0 / SYSTEM_DEFAULT / SYSTEM_DEFAULT).
  EXPECT_EQ(versioned.resolved_name, KinematicState::name);
  EXPECT_EQ(versioned.type_name, "nav_msgs/msg/Odometry");
  EXPECT_EQ(versioned.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(versioned.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
  EXPECT_EQ(versioned.depth, 10u);

  // A local spec whose domain version has three distinct, non-zero components
  // (so major/minor/patch cannot coincide with InterfaceRecord's defaults),
  // called with Kind::Service/Role::Require (both differing from
  // InterfaceRecord's Kind::Topic/Role::Provide defaults) and a resolved_name
  // that differs from the spec-declared name (as under a remap). This pins
  // interface_name and resolved_name apart: a bug that copied one field into
  // the other would fail one of the two assertions below.
  const auto remapped = ciu::make_record<test_specs::Versioned>(
    ciu::InterfaceRecord::Kind::Service, ciu::InterfaceRecord::Role::Require, "/remapped/odom",
    "nav_msgs/msg/Odometry", rmw_qos_profile_default);
  EXPECT_TRUE(remapped.has_version);
  EXPECT_EQ(remapped.major, 2);
  EXPECT_EQ(remapped.minor, 3);
  EXPECT_EQ(remapped.patch, 4);
  EXPECT_EQ(remapped.interface_name, test_specs::Versioned::name);
  EXPECT_EQ(remapped.resolved_name, "/remapped/odom");
  EXPECT_EQ(remapped.kind, ciu::InterfaceRecord::Kind::Service);
  EXPECT_EQ(remapped.role, ciu::InterfaceRecord::Role::Require);

  const auto plain = ciu::make_record<test_specs::Unversioned>(
    ciu::InterfaceRecord::Kind::Topic, ciu::InterfaceRecord::Role::Provide,
    test_specs::Unversioned::name, "std_msgs/msg/Empty", rmw_qos_profile_default);
  EXPECT_FALSE(plain.has_version);
}
