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

#ifndef RANDOM_DOWNSAMPLE_FILTER__RANDOM_DOWNSAMPLE_FILTER_NODE_HPP_
#define RANDOM_DOWNSAMPLE_FILTER__RANDOM_DOWNSAMPLE_FILTER_NODE_HPP_

#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/published_time_publisher.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_utils_tf/transform_listener.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/thread/mutex.hpp>

#include <pcl/filters/random_sample.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::downsample_filters
{
class RandomDownsampleFilter : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit RandomDownsampleFilter(const rclcpp::NodeOptions & options);

private:
  void filter(const PointCloud2ConstPtr & input, PointCloud2 & output);

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

  /** \brief The input PointCloud2 subscriber. */
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_input_;

  /** \brief The output PointCloud2 publisher. */
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;

  /** \brief processing time publisher. **/
  std::unique_ptr<autoware_utils_system::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils_debug::DebugPublisher> debug_publisher_;
  std::unique_ptr<autoware_utils_debug::PublishedTimePublisher> published_time_publisher_;

  /** \brief The transform listener. */
  std::unique_ptr<autoware_utils_tf::TransformListener> transform_listener_{nullptr};

  void input_callback(const PointCloud2ConstPtr cloud);

  bool is_valid(const PointCloud2ConstPtr & cloud);

  void compute_publish(const PointCloud2ConstPtr & input);

  bool convert_output_costly(std::unique_ptr<PointCloud2> & output);

  size_t sample_num_;

  /** \brief The maximum queue size (default: 3). */
  size_t max_queue_size_ = 3;
};
}  // namespace autoware::downsample_filters

// clang-format off
#endif  // RANDOM_DOWNSAMPLE_FILTER__RANDOM_DOWNSAMPLE_FILTER_NODE_HPP_  // NOLINT
// clang-format on
