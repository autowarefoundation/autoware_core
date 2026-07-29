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
#include "autoware/component_interface_specs/map.hpp"
#include "autoware/component_interface_specs/version.hpp"
#include "gtest/gtest.h"
#include "spec_test_utils.hpp"

#include <tuple>
#include <type_traits>
#include <utility>

namespace specs = autoware::component_interface_specs;
namespace tu = autoware::component_interface_specs::test_utils;

// Local mirror of universe's HasDomainVersion: a void_t/expression-SFINAE probe over
// the ADL-resolved resolve_domain_version(const Spec &). Downstream consumers use
// exactly this shape to decide whether a spec participates in versioned interface
// registration, so PointCloudMap's exclusion (it carries a raw point cloud payload
// rather than a bounded interface message) must be type-enforced here as an
// ill-formed version resolution, not merely as a tuple omission. Otherwise a
// consumer registering PointCloudMap would emit a manifest record for an interface
// absent from the authority manifest.
namespace
{
template <class, class = void>
struct has_domain_version : std::false_type
{
};
template <class S>
struct has_domain_version<
  S, std::void_t<decltype(resolve_domain_version(std::declval<const S &>()))>> : std::true_type
{
};
}  // namespace

// PointCloudMap is intentionally excluded from the versioned surface, so version
// resolution must be ill-formed for it and the detection trait must report it as
// unversioned.
static_assert(
  !has_domain_version<specs::map::PointCloudMap>::value,
  "PointCloudMap must not resolve a domain version (raw point cloud, excluded from versioning)");
// Positive control: a genuinely registered spec is still detected as versioned.
static_assert(
  has_domain_version<specs::map::VectorMap>::value, "VectorMap must resolve a domain version");

TEST(map, concept_and_registration)
{
  using specs::map::GetDifferentialPointCloudMap;
  using specs::map::GetPartialPointCloudMap;
  using specs::map::MapProjectorInfo;
  using specs::map::PointCloudMap;
  using specs::map::Specs;
  using specs::map::VectorMap;

  static_assert(specs::InterfaceSpec<MapProjectorInfo>);
  static_assert(specs::InterfaceSpec<VectorMap>);
  static_assert(specs::ServiceSpec<GetDifferentialPointCloudMap>);
  static_assert(specs::ServiceSpec<GetPartialPointCloudMap>);

  static_assert(tu::has_type<MapProjectorInfo, Specs>::value);
  static_assert(tu::has_type<VectorMap, Specs>::value);
  static_assert(tu::has_type<GetDifferentialPointCloudMap, Specs>::value);
  static_assert(tu::has_type<GetPartialPointCloudMap, Specs>::value);
  static_assert(std::tuple_size_v<Specs> == 4);

  // PointCloudMap stays available to existing consumers but is deliberately kept
  // out of the versioned registration surface because it carries a raw point cloud
  // payload rather than a bounded interface message.
  static_assert(!tu::has_type<PointCloudMap, Specs>::value);
  SUCCEED();
}

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
