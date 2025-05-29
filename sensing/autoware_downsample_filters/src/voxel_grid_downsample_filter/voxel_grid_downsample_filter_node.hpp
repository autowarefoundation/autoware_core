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

#include "transform_info.hpp"

#include <boost/thread/mutex.hpp>

#include <memory>
#include <string>

// PCL includes
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>

// Include tier4 autoware utils
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/published_time_publisher.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_utils_tf/transform_listener.hpp>

namespace autoware::downsample_filters
{
class VoxelGridDownsampleFilter : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit VoxelGridDownsampleFilter(const rclcpp::NodeOptions & options);

private:
  /** \brief The input PointCloud2 subscriber. */
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_input_;

  /** \brief The output PointCloud2 publisher. */
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;

  /** \brief transform listener */
  std::unique_ptr<autoware_utils_tf::TransformListener> transform_listener_{nullptr};

  /** \brief processing time publisher. **/
  std::unique_ptr<autoware_utils_system::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils_debug::DebugPublisher> debug_publisher_;
  std::unique_ptr<autoware_utils_debug::PublishedTimePublisher> published_time_publisher_;

  /** \brief PointCloud2 data callback. */
  void input_callback(const PointCloud2ConstPtr cloud);

  /** \brief Convert output to PointCloud2. */
  bool convert_output_costly(std::unique_ptr<PointCloud2> & output);

  /** \brief apply voxel grid downsample filter and transform point cloud */
  /** \param input input point cloud */
  /** \param output output point cloud */
  /** \param transform_info transform info */
  void filter(
    const PointCloud2ConstPtr & input, PointCloud2 & output, const TransformInfo & transform_info);

  /** \brief voxel size x */
  float voxel_size_x_;
  /** \brief voxel size y */
  float voxel_size_y_;
  /** \brief voxel size z */
  float voxel_size_z_;
  /** \brief The input TF frame the data should be transformed into,
   * if input.header.frame_id is different. */
  std::string tf_input_frame_;
  /** \brief The original data input TF frame. */
  std::string tf_input_orig_frame_;
  /** \brief The output TF frame the data should be transformed into,
   * if input.header.frame_id is different. */
  std::string tf_output_frame_;
  /** \brief Internal mutex. */
  std::mutex mutex_;
  /** \brief The maximum queue size (default: 3). */
  size_t max_queue_size_ = 3;

  /** \brief check if point cloud is valid */
  /** \param cloud point cloud */
  /** \return true if point cloud is valid, false otherwise */
  bool is_valid(const PointCloud2ConstPtr & cloud);

  /** \brief calculate transform matrix */
  /** \param target_frame target frame */
  /** \param from point cloud with original frame id and timestamp */
  /** \param transform_info transform info */
  /** \return true if transform matrix is calculated, false otherwise */
  bool calculate_transform_matrix(
    const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & from,
    TransformInfo & transform_info /*output*/);
};
}  // namespace autoware::downsample_filters

// clang-format off
#endif  // VOXEL_GRID_DOWNSAMPLE_FILTER__VOXEL_GRID_DOWNSAMPLE_FILTER_NODE_HPP_  // NOLINT
// clang-format on
