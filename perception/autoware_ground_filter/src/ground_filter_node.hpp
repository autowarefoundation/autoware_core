// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef GROUND_FILTER_NODE_HPP_
#define GROUND_FILTER_NODE_HPP_

#include "ground_filter.hpp"

#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/published_time_publisher.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl_msgs/msg/point_indices.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/thread/mutex.hpp>

#if __has_include(<message_filters/subscriber.hpp>)
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#else
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#endif

#include <memory>
#include <mutex>
#include <string>
#include <vector>

class GroundFilterTest;

namespace autoware::ground_filter
{

class GroundFilterComponent : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit GroundFilterComponent(const rclcpp::NodeOptions & options);

  // for test
  friend class ::GroundFilterTest;

private:
  void subscribe();

  void faster_input_indices_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud,
    const pcl_msgs::msg::PointIndices::ConstSharedPtr indices);

  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & param);

  // parameters
  bool elevation_grid_mode_;
  float radial_divider_angle_rad_;
  size_t radial_dividers_num_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  bool use_virtual_ground_point_;
  float split_height_distance_;

  float global_slope_max_angle_rad_;
  float local_slope_max_angle_rad_;
  float global_slope_max_ratio_;
  float local_slope_max_ratio_;
  float split_points_distance_tolerance_;

  bool use_recheck_ground_cluster_;
  bool use_lowest_point_;
  float detection_range_z_max_;

  float grid_size_m_;
  float grid_mode_switch_radius_;
  uint16_t ground_grid_buffer_size_;
  float virtual_lidar_z_;
  float low_priority_region_x_;
  float center_pcl_shift_;
  float non_ground_height_threshold_;

  std::size_t max_queue_size_;
  bool use_indices_;
  bool latched_indices_;
  bool approximate_sync_;

  // grid ground filter processor
  std::unique_ptr<GroundFilter> ground_filter_ptr_;

  // time keeper related
  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    detailed_processing_time_publisher_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  std::unique_ptr<autoware_utils_system::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{
    nullptr};
  std::unique_ptr<autoware_utils_debug::DebugPublisher> debug_publisher_ptr_{nullptr};

  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>
    sync_input_indices_a_;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>
    sync_input_indices_e_;

  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_input_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_output_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_input_filter_;
  message_filters::Subscriber<pcl_msgs::msg::PointIndices> sub_indices_filter_;
  std::unique_ptr<autoware_utils_debug::PublishedTimePublisher> published_time_publisher_;
};

}  // namespace autoware::ground_filter

#endif  // GROUND_FILTER_NODE_HPP_
