// Copyright 2020 Tier IV, Inc.
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

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: filter.cpp 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#include "autoware/downsample_filters/filter.hpp"

#include "autoware/downsample_filters/memory.hpp"

#include <autoware_utils_tf/transform_listener.hpp>
#include <pcl_ros/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/io/io.h>

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
autoware::downsample_filters::Filter::Filter(
  const std::string & filter_name, const rclcpp::NodeOptions & options)
: Node(filter_name, options)
{
  // Set parameters (moved from NodeletLazy onInit)
  {
    tf_input_frame_ = static_cast<std::string>(declare_parameter("input_frame", ""));
    tf_output_frame_ = static_cast<std::string>(declare_parameter("output_frame", ""));
    max_queue_size_ = static_cast<std::size_t>(declare_parameter("max_queue_size", 5));

    RCLCPP_DEBUG_STREAM(
      this->get_logger(),
      "Filter (as Component) successfully created with the following parameters:"
        << std::endl
        << " - max_queue_size   : " << max_queue_size_);
  }

  // Set publisher
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    pub_output_ = this->create_publisher<PointCloud2>(
      "output", rclcpp::SensorDataQoS().keep_last(max_queue_size_), pub_options);
  }

  // TODO(sykwer): Change the corresponding node to subscribe to `faster_input_callback`
  // each time a child class supports the faster version.
  // When all the child classes support the faster version, this workaround is deleted.
  std::set<std::string> supported_nodes = {"VoxelGridDownsampleFilter"};
  auto callback = supported_nodes.find(filter_name) != supported_nodes.end()
                    ? &Filter::faster_input_callback
                    : &Filter::input_callback;

  // Subscribe in an old fashion to input only (no filters)
  // CAN'T use auto-type here.
  std::function<void(const PointCloud2ConstPtr msg)> cb =
    std::bind(callback, this, std::placeholders::_1);
  sub_input_ = create_subscription<PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_), cb);

  // Set tf_listener, tf_buffer.
  setupTF();

  published_time_publisher_ = std::make_unique<autoware_utils_debug::PublishedTimePublisher>(this);
  RCLCPP_DEBUG(this->get_logger(), "[Filter Constructor] successfully created.");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void autoware::downsample_filters::Filter::setupTF()
{
  transform_listener_ = std::make_unique<autoware_utils_tf::TransformListener>(this);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO(sykwer): Temporary Implementation: Delete this function definition when all the filter nodes
// conform to new API.
void autoware::downsample_filters::Filter::compute_publish(const PointCloud2ConstPtr & input)
{
  auto output = std::make_unique<PointCloud2>();

  // Call the virtual method in the child
  filter(input, *output);

  if (!convert_output_costly(output)) return;

  // Copy timestamp to keep it
  output->header.stamp = input->header.stamp;

  // Publish a boost shared ptr
  pub_output_->publish(std::move(output));
  published_time_publisher_->publish_if_subscribed(pub_output_, input->header.stamp);
}

//////////////////////////////////////////////////////////////////////////////////////////////
// TODO(sykwer): Temporary Implementation: Delete this function definition when all the filter nodes
// conform to new API.
void autoware::downsample_filters::Filter::input_callback(const PointCloud2ConstPtr cloud)
{
  // If cloud is given, check if it's valid
  if (!isValid(cloud)) {
    RCLCPP_ERROR(this->get_logger(), "[input_callback] Invalid input!");
    return;
  }

  /// DEBUG
  RCLCPP_DEBUG(
    this->get_logger(),
    "[input_callback] PointCloud with %d data points and frame %s on input topic "
    "received.",
    cloud->width * cloud->height, cloud->header.frame_id.c_str());
  ///

  // Check whether the user has given a different input TF frame
  tf_input_orig_frame_ = cloud->header.frame_id;
  PointCloud2ConstPtr cloud_tf;
  if (!tf_input_frame_.empty() && cloud->header.frame_id != tf_input_frame_) {
    RCLCPP_DEBUG(
      this->get_logger(), "[input_callback] Transforming input dataset from %s to %s.",
      cloud->header.frame_id.c_str(), tf_input_frame_.c_str());

    // Save the original frame ID
    // Convert the cloud into the different frame
    auto cloud_transformed = std::make_unique<PointCloud2>();

    auto tf_ptr = transform_listener_->get_transform(
      tf_output_frame_, cloud_tf->header.frame_id, cloud_tf->header.stamp,
      rclcpp::Duration::from_seconds(1.0));
    if (!tf_ptr) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[convert_output_costly] Error converting output dataset from %s to %s.",
        cloud_tf->header.frame_id.c_str(), tf_output_frame_.c_str());
      return;
    }

    auto eigen_tf = tf2::transformToEigen(*tf_ptr);
    pcl_ros::transformPointCloud(eigen_tf.matrix().cast<float>(), *cloud_tf, *cloud_transformed);
    cloud_tf = std::move(cloud_transformed);

  } else {
    cloud_tf = cloud;
  }

  compute_publish(cloud_tf);
}

// Returns false in error cases
bool autoware::downsample_filters::Filter::calculate_transform_matrix(
  const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & from,
  TransformInfo & transform_info /*output*/)
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

// Returns false in error cases
bool autoware::downsample_filters::Filter::convert_output_costly(
  std::unique_ptr<PointCloud2> & output)
{
  // In terms of performance, we should avoid using pcl_ros library function,
  // but this code path isn't reached in the main use case of Autoware, so it's left as is for now.
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

  // Same as the comment above
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

// TODO(sykwer): Temporary Implementation: Rename this function to `input_callback()` when
// all the filter nodes conform to new API. Then delete the old `input_callback()` defined
// above.
void autoware::downsample_filters::Filter::faster_input_callback(const PointCloud2ConstPtr cloud)
{
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

    return;
  }

  if (!isValid(cloud)) {
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

  // TODO(sykwer): Change to `filter()` call after when the filter nodes conform to new API.
  faster_filter(cloud, *output, transform_info);

  if (!convert_output_costly(output)) return;

  output->header.stamp = cloud->header.stamp;
  pub_output_->publish(std::move(output));
  published_time_publisher_->publish_if_subscribed(pub_output_, cloud->header.stamp);
}

// TODO(sykwer): Temporary Implementation: Remove this interface when all the filter nodes conform
// to new API. It's not a pure virtual function so that a child class does not have to implement
// this function.
void autoware::downsample_filters::Filter::faster_filter(
  const PointCloud2ConstPtr & input, PointCloud2 & output, const TransformInfo & transform_info)
{
  (void)input;
  (void)output;
  (void)transform_info;
}
