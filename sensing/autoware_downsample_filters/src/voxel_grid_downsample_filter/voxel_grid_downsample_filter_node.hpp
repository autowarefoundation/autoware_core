// Copyright 2024 TIER IV, Inc.
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

#ifndef VOXEL_GRID_DOWNSAMPLE_FILTER__VOXEL_GRID_DOWNSAMPLE_FILTER_NODE_HPP_  // NOLINT
#define VOXEL_GRID_DOWNSAMPLE_FILTER__VOXEL_GRID_DOWNSAMPLE_FILTER_NODE_HPP_  // NOLINT

#include "voxel_grid_downsample_filter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

// PCL includes
#include <sensor_msgs/msg/point_cloud2.hpp>

// Include tier4 autoware utils
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/published_time_publisher.hpp>
#include <autoware_utils_system/stop_watch.hpp>

namespace autoware::downsample_filters
{
class VoxelGridDownsampleFilterNode : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit VoxelGridDownsampleFilterNode(const rclcpp::NodeOptions & options);

private:
  /** \brief The input PointCloud2 subscriber. */
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_input_;

  /** \brief The output PointCloud2 publisher. */
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;

  /** \brief processing time publisher. **/
  std::unique_ptr<autoware_utils_system::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils_debug::DebugPublisher> debug_publisher_;
  std::unique_ptr<autoware_utils_debug::PublishedTimePublisher> published_time_publisher_;

  /** \brief PointCloud2 data callback. */
  void input_callback(const PointCloud2ConstPtr cloud);

  VoxelGridDownsampleFilter filter_core_;

  /** \brief The maximum queue size (default: 3). */
  size_t max_queue_size_ = 3;
};
}  // namespace autoware::downsample_filters

// clang-format off
#endif  // VOXEL_GRID_DOWNSAMPLE_FILTER__VOXEL_GRID_DOWNSAMPLE_FILTER_NODE_HPP_  // NOLINT
// clang-format on
