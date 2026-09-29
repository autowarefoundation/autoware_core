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

namespace specs = autoware::component_interface_specs;
namespace tu = autoware::component_interface_specs::test_utils;

// PointCloudMap is intentionally excluded from the versioned surface, so version
// resolution must be ill-formed for it and the detection trait must report it as
// unversioned.
static_assert(
  !tu::has_domain_version<specs::map::PointCloudMap>::value,
  "PointCloudMap must not resolve a domain version (raw point cloud, excluded from versioning)");
// Positive control: a genuinely registered spec is still detected as versioned.
static_assert(
  tu::has_domain_version<specs::map::VectorMap>::value, "VectorMap must resolve a domain version");

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
    using specs::map::MapProjectorInfo;
    tu::expect_topic_qos<MapProjectorInfo>(
      "/map/map_projector_info", 1, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }

  {
    using specs::map::PointCloudMap;
    tu::expect_topic_qos<PointCloudMap>(
      "/map/point_cloud_map", 1, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }

  {
    using specs::map::VectorMap;
    tu::expect_topic_qos<VectorMap>(
      "/map/vector_map", 1, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }
}
