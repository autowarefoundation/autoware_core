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

#include <optional>
#include <sstream>
#include <string>

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

namespace autoware::downsample_filters
{

namespace
{
std::optional<int> find_field_index(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & field_name)
{
  for (size_t i = 0; i < cloud.fields.size(); ++i) {
    if (cloud.fields[i].name == field_name) {
      return static_cast<int>(i);
    }
  }
  return std::nullopt;
}
}  // namespace

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

  const auto x_index = find_field_index(*input, "x");
  const auto y_index = find_field_index(*input, "y");
  const auto z_index = find_field_index(*input, "z");
  const auto intensity_index = find_field_index(*input, "intensity");

  if (!x_index.has_value() || !y_index.has_value() || !z_index.has_value()) {
    return tl::unexpected("The input point cloud does not have required x, y, z fields.");
  }
  if (!intensity_index.has_value()) {
    return tl::unexpected("There is no intensity field in the input point cloud.");
  }
  if (
    input->fields[static_cast<size_t>(intensity_index.value())].datatype !=
    sensor_msgs::msg::PointField::UINT8) {
    return tl::unexpected("The intensity field in the input point cloud is not of type UINT8.");
  }

  // Apply filter
  std::scoped_lock lock(mutex_);
  PointCloud2 output;
  faster_voxel_filter_.set_voxel_size(
    parameters_.voxel_size_x, parameters_.voxel_size_y, parameters_.voxel_size_z);
  const auto filter_result = faster_voxel_filter_.filter(input, output);

  if (!filter_result.is_valid) {
    return tl::unexpected(filter_result.reason);
  }

  output.header.stamp = input->header.stamp;

  return output;
}

}  // namespace autoware::downsample_filters
