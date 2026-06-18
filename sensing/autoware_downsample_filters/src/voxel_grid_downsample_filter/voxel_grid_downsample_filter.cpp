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

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

namespace autoware::downsample_filters
{
VoxelGridDownsampleFilter::VoxelGridDownsampleFilter(const Parameters & parameters)
: parameters_(parameters)
{
  faster_voxel_filter_.set_voxel_size(
    parameters_.voxel_size_x, parameters_.voxel_size_y, parameters_.voxel_size_z);
}

tl::expected<PointCloud2, std::string> VoxelGridDownsampleFilter::filter(
  const PointCloud2ConstPtr & input)
{
  // Validate input
  if (
    static_cast<std::size_t>(input->width) * input->height * input->point_step !=
    input->data.size()) {
    std::ostringstream oss;
    oss << "Invalid PointCloud (data = " << input->data.size() << ", width = " << input->width
        << ", height = " << input->height << ", step = " << input->point_step << ")";
    return tl::unexpected(oss.str());
  }

  if (
    !utils::is_data_layout_compatible_with_point_xyzircaedt(*input) &&
    !utils::is_data_layout_compatible_with_point_xyzirc(*input)) {
    std::string error_message =
      "The pointcloud layout is not compatible with PointXYZIRCAEDT or PointXYZIRC.";

    if (utils::is_data_layout_compatible_with_point_xyziradrt(*input)) {
      error_message +=
        " Layout is compatible with PointXYZIRADRT. You may be using legacy "
        "code/data.";
    }

    if (utils::is_data_layout_compatible_with_point_xyzi(*input)) {
      error_message += " Layout is compatible with PointXYZI. You may be using legacy code/data.";
    }

    return tl::unexpected(error_message);
  }

  if (
    pcl::getFieldIndex(*input, "x") < 0 || pcl::getFieldIndex(*input, "y") < 0 ||
    pcl::getFieldIndex(*input, "z") < 0) {
    return tl::unexpected(std::string("The input point cloud does not have required x, y, z fields."));
  }

  const int intensity_index = pcl::getFieldIndex(*input, "intensity");
  if (intensity_index < 0) {
    return tl::unexpected(std::string("There is no intensity field in the input point cloud."));
  }
  if (input->fields[intensity_index].datatype != sensor_msgs::msg::PointField::UINT8) {
    return tl::unexpected(std::string("The intensity field in the input point cloud is not of type UINT8."));
  }

  // Apply filter
  std::scoped_lock lock(mutex_);
  PointCloud2 output;
  faster_voxel_filter_.set_voxel_size(
    parameters_.voxel_size_x, parameters_.voxel_size_y, parameters_.voxel_size_z);
  const auto filter_result = faster_voxel_filter_.filter(input, output, TransformInfo{});

  if (!filter_result.is_valid) {
    return tl::unexpected(filter_result.reason);
  }

  output.header.stamp = input->header.stamp;

  return output;
}

}  // namespace autoware::downsample_filters
