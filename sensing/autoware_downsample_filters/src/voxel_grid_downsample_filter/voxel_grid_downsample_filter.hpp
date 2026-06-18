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

#ifndef VOXEL_GRID_DOWNSAMPLE_FILTER__VOXEL_GRID_DOWNSAMPLE_FILTER_HPP_
#define VOXEL_GRID_DOWNSAMPLE_FILTER__VOXEL_GRID_DOWNSAMPLE_FILTER_HPP_

#include "faster_voxel_grid_downsample_filter.hpp"

#include <tl/expected.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mutex>
#include <string>

namespace autoware::downsample_filters
{
class VoxelGridDownsampleFilterCore
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

  struct Parameters
  {
    float voxel_size_x;
    float voxel_size_y;
    float voxel_size_z;
  };

  explicit VoxelGridDownsampleFilterCore(const Parameters & parameters);

  tl::expected<PointCloud2, std::string> filter(const PointCloud2ConstPtr & input);

private:
  FasterVoxelGridDownsampleFilter faster_voxel_filter_;
  Parameters parameters_;
  std::mutex mutex_;
};
}  // namespace autoware::downsample_filters

#endif  // VOXEL_GRID_DOWNSAMPLE_FILTER__VOXEL_GRID_DOWNSAMPLE_FILTER_HPP_
