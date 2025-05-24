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

#include "voxel_grid_downsample_filter_node.hpp"

#include "faster_voxel_grid_downsample_filter.hpp"
#include "memory.hpp"
#include "transform_info.hpp"

#include <pcl_ros/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/io/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::downsample_filters
{
VoxelGridDownsampleFilter::VoxelGridDownsampleFilter(const rclcpp::NodeOptions & options)
: rclcpp::Node("VoxelGridDownsampleFilter", options),
  voxel_size_x_(declare_parameter<float>("voxel_size_x")),
  voxel_size_y_(declare_parameter<float>("voxel_size_y")),
  voxel_size_z_(declare_parameter<float>("voxel_size_z")),
  tf_input_frame_(declare_parameter<std::string>("input_frame")),
  tf_output_frame_(declare_parameter<std::string>("output_frame")),
  max_queue_size_(static_cast<std::size_t>(declare_parameter<int64_t>("max_queue_size")))
{
  // Set publishers
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    pub_output_ = this->create_publisher<PointCloud2>(
      "output", rclcpp::SensorDataQoS().keep_last(max_queue_size_), pub_options);

    published_time_publisher_ =
      std::make_unique<autoware_utils_debug::PublishedTimePublisher>(this);
  }

  // Set subscribers
  {
    sub_input_ = create_subscription<PointCloud2>(
      "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_),
      std::bind(&VoxelGridDownsampleFilter::input_callback, this, std::placeholders::_1));
    transform_listener_ = std::make_unique<autoware_utils_tf::TransformListener>(this);
  }

  RCLCPP_DEBUG(this->get_logger(), "[Filter Constructor] successfully created.");
}

void VoxelGridDownsampleFilter::input_callback(const PointCloud2ConstPtr cloud)
{
  if (!is_valid(cloud)) {
    RCLCPP_ERROR(this->get_logger(), "[input_callback] Invalid input!");
    return;
  }

  RCLCPP_DEBUG(
    this->get_logger(),
    "[input_callback] PointCloud with %d data points and frame %s on input topic "
    "received.",
    cloud->width * cloud->height, cloud->header.frame_id.c_str());

  tf_input_orig_frame_ = cloud->header.frame_id;

  // For performance reason, defer the transform computation.
  // Do not use pcl_ros::transformPointCloud(). It's too slow due to the unnecessary copy.
  TransformInfo transform_info;
  if (!calculate_transform_matrix(tf_input_frame_, *cloud, transform_info)) return;

  auto output = std::make_unique<PointCloud2>();

  filter(cloud, *output, transform_info);

  if (!convert_output_costly(output)) return;

  output->header.stamp = cloud->header.stamp;
  pub_output_->publish(std::move(output));
  published_time_publisher_->publish_if_subscribed(pub_output_, cloud->header.stamp);
}

bool VoxelGridDownsampleFilter::is_valid(const PointCloud2ConstPtr & cloud)
{
  if (cloud->width * cloud->height * cloud->point_step != cloud->data.size()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid PointCloud (data = %zu, width = %d, height = %d, step = %d) with stamp %f, "
      "and frame %s received!",
      cloud->data.size(), cloud->width, cloud->height, cloud->point_step,
      rclcpp::Time(cloud->header.stamp).seconds(), cloud->header.frame_id.c_str());
    return false;
  }

  if (
    !utils::is_data_layout_compatible_with_point_xyzircaedt(*cloud) &&
    !utils::is_data_layout_compatible_with_point_xyzirc(*cloud)) {
    RCLCPP_ERROR(
      get_logger(),
      "The pointcloud layout is not compatible with PointXYZIRCAEDT or PointXYZIRC. Aborting");

    if (utils::is_data_layout_compatible_with_point_xyziradrt(*cloud)) {
      RCLCPP_ERROR(
        get_logger(),
        "The pointcloud layout is compatible with PointXYZIRADRT. You may be using legacy "
        "code/data");
    }

    if (utils::is_data_layout_compatible_with_point_xyzi(*cloud)) {
      RCLCPP_ERROR(
        get_logger(),
        "The pointcloud layout is compatible with PointXYZI. You may be using legacy "
        "code/data");
    }

    return false;
  }

  return true;
}

bool VoxelGridDownsampleFilter::calculate_transform_matrix(
  const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & from,
  TransformInfo & transform_info)
{
  transform_info.need_transform = false;

  if (target_frame.empty() || from.header.frame_id == target_frame) return true;

  RCLCPP_DEBUG(
    this->get_logger(), "[get_transform_matrix] Transforming input dataset from %s to %s.",
    from.header.frame_id.c_str(), target_frame.c_str());

  auto tf_ptr = transform_listener_->get_transform(
    target_frame, from.header.frame_id, from.header.stamp, rclcpp::Duration::from_seconds(1.0));

  if (!tf_ptr) {
    return false;
  }

  auto eigen_tf = tf2::transformToEigen(*tf_ptr);
  transform_info.eigen_transform = eigen_tf.matrix().cast<float>();
  transform_info.need_transform = true;
  return true;
}

bool VoxelGridDownsampleFilter::convert_output_costly(std::unique_ptr<PointCloud2> & output)
{
  if (!tf_output_frame_.empty() && output->header.frame_id != tf_output_frame_) {
    RCLCPP_DEBUG(
      this->get_logger(), "[convert_output_costly] Transforming output dataset from %s to %s.",
      output->header.frame_id.c_str(), tf_output_frame_.c_str());

    // Convert the cloud into the different frame
    auto cloud_transformed = std::make_unique<PointCloud2>();

    auto tf_ptr = transform_listener_->get_transform(
      tf_output_frame_, output->header.frame_id, output->header.stamp,
      rclcpp::Duration::from_seconds(1.0));
    if (!tf_ptr) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[convert_output_costly] Error converting output dataset from %s to %s.",
        output->header.frame_id.c_str(), tf_output_frame_.c_str());
      return false;
    }

    auto eigen_tf = tf2::transformToEigen(*tf_ptr);
    pcl_ros::transformPointCloud(eigen_tf.matrix().cast<float>(), *output, *cloud_transformed);
    output = std::move(cloud_transformed);
  }

  if (tf_output_frame_.empty() && output->header.frame_id != tf_input_orig_frame_) {
    // No tf_output_frame given, transform the dataset to its original frame
    RCLCPP_DEBUG(
      this->get_logger(), "[convert_output_costly] Transforming output dataset from %s back to %s.",
      output->header.frame_id.c_str(), tf_input_orig_frame_.c_str());

    auto cloud_transformed = std::make_unique<sensor_msgs::msg::PointCloud2>();

    auto tf_ptr = transform_listener_->get_transform(
      tf_input_orig_frame_, output->header.frame_id, output->header.stamp,
      rclcpp::Duration::from_seconds(1.0));

    if (!tf_ptr) {
      return false;
    }

    auto eigen_tf = tf2::transformToEigen(*tf_ptr);
    pcl_ros::transformPointCloud(eigen_tf.matrix().cast<float>(), *output, *cloud_transformed);
    output = std::move(cloud_transformed);
  }

  return true;
}

void VoxelGridDownsampleFilter::filter(
  const PointCloud2ConstPtr & input, PointCloud2 & output, const TransformInfo & transform_info)
{
  std::scoped_lock lock(mutex_);
  FasterVoxelGridDownsampleFilter faster_voxel_filter;
  faster_voxel_filter.set_voxel_size(voxel_size_x_, voxel_size_y_, voxel_size_z_);
  faster_voxel_filter.set_field_offsets(input, this->get_logger());
  faster_voxel_filter.filter(input, output, transform_info, this->get_logger());
}
}  // namespace autoware::downsample_filters

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::downsample_filters::VoxelGridDownsampleFilter)
