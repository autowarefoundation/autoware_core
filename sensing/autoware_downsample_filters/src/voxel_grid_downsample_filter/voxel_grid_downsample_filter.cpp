// Copyright 2026 TIER IV, Inc.
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

#include "voxel_grid_downsample_filter.hpp"

#include "memory.hpp"
#include "transform_info.hpp"

#include <sstream>
#include <string>

namespace autoware::downsample_filters
{
VoxelGridDownsampleFilterCore::VoxelGridDownsampleFilterCore(const Parameters & parameters)
: parameters_(parameters)
{
  faster_voxel_filter_.set_voxel_size(
    parameters_.voxel_size_x, parameters_.voxel_size_y, parameters_.voxel_size_z);
}

ValidationResult VoxelGridDownsampleFilterCore::validate_input(const PointCloud2 & cloud)
{
  if (
    static_cast<std::size_t>(cloud.width) * cloud.height * cloud.point_step != cloud.data.size()) {
    std::ostringstream oss;
    oss << "Invalid PointCloud (data = " << cloud.data.size() << ", width = " << cloud.width
        << ", height = " << cloud.height << ", step = " << cloud.point_step << ")";
    return {false, oss.str()};
  }

  if (
    !utils::is_data_layout_compatible_with_point_xyzircaedt(cloud) &&
    !utils::is_data_layout_compatible_with_point_xyzirc(cloud)) {
    std::string error_message =
      "The pointcloud layout is not compatible with PointXYZIRCAEDT or PointXYZIRC.";

    if (utils::is_data_layout_compatible_with_point_xyziradrt(cloud)) {
      error_message +=
        " Layout is compatible with PointXYZIRADRT. You may be using legacy "
        "code/data.";
    }

    if (utils::is_data_layout_compatible_with_point_xyzi(cloud)) {
      error_message += " Layout is compatible with PointXYZI. You may be using legacy code/data.";
    }

    return {false, error_message};
  }

  if (
    pcl::getFieldIndex(cloud, "x") < 0 || pcl::getFieldIndex(cloud, "y") < 0 ||
    pcl::getFieldIndex(cloud, "z") < 0) {
    return {false, "The input point cloud does not have required x, y, z fields."};
  }

  const int intensity_index = pcl::getFieldIndex(cloud, "intensity");
  if (intensity_index < 0) {
    return {false, "There is no intensity field in the input point cloud."};
  }
  if (cloud.fields[intensity_index].datatype != sensor_msgs::msg::PointField::UINT8) {
    return {false, "The intensity field in the input point cloud is not of type UINT8."};
  }

  return {true, ""};
}

ValidationResult VoxelGridDownsampleFilterCore::filter(
  const PointCloud2ConstPtr & input, PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  faster_voxel_filter_.set_voxel_size(
    parameters_.voxel_size_x, parameters_.voxel_size_y, parameters_.voxel_size_z);
  return faster_voxel_filter_.filter(input, output, TransformInfo{});
}

}  // namespace autoware::downsample_filters
