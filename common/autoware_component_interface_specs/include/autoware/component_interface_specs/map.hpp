// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS__MAP_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS__MAP_HPP_

#include <autoware/component_interface_specs/utils.hpp>
#include <autoware/component_interface_specs/version.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>
#include <autoware_map_msgs/srv/get_partial_point_cloud_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rmw/qos_profiles.h>

namespace autoware::component_interface_specs::map
{

struct MapProjectorInfo
{
  using Message = autoware_map_msgs::msg::MapProjectorInfo;
  static constexpr char name[] = "/map/map_projector_info";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

// interface-spec-lint: not-versioned (raw point cloud; read by autoware_interface_spec_lint)
struct PointCloudMap
{
  using Message = sensor_msgs::msg::PointCloud2;
  static constexpr char name[] = "/map/point_cloud_map";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

struct VectorMap
{
  using Message = autoware_map_msgs::msg::LaneletMapBin;
  static constexpr char name[] = "/map/vector_map";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

struct GetDifferentialPointCloudMap
{
  using Service = autoware_map_msgs::srv::GetDifferentialPointCloudMap;
  static constexpr char name[] = "/map/get_differential_pointcloud_map";
};

struct GetPartialPointCloudMap  // new - companion PCD-map delivery path
{
  using Service = autoware_map_msgs::srv::GetPartialPointCloudMap;
  static constexpr char name[] = "/map/get_partial_pointcloud_map";
};

// PointCloudMap is intentionally excluded from the versioned registration surface:
// it carries the full raw point cloud payload rather than a bounded interface
// message, so it is left out of the Specs tuple while the struct itself stays
// available for existing consumers.
AUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN(
  0, 1, 0, VectorMap, MapProjectorInfo, GetDifferentialPointCloudMap, GetPartialPointCloudMap)

// Type-enforce PointCloudMap's exclusion from the versioned surface: it carries a
// raw point cloud payload rather than a bounded interface message, so version
// resolution for it is made ill-formed outright rather than left as a tuple
// omission. This non-template exact match wins overload resolution over the
// template above, so version resolution is ill-formed for PointCloudMap: a
// downstream detection trait (e.g. universe's HasDomainVersion) then sees it as
// unversioned, and a direct spec_version<PointCloudMap>() is a hard compile error.
// Without this the exclusion would be a tuple omission only, and a consumer could
// still register PointCloudMap and emit a manifest record for an interface absent
// from the authority manifest.
constexpr Version resolve_domain_version(const PointCloudMap &) = delete;

}  // namespace autoware::component_interface_specs::map

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS__MAP_HPP_
