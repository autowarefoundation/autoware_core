// Copyright(c) 2025 AutoCore Technology (Nanjing) Co., Ltd. All rights reserved.
//
// Copyright 2025 TIER IV, Inc.
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

#include "crop_box_filter_node.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::crop_box_filter
{
CropBoxFilter::CropBoxFilter(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("crop_box_filter", node_options),
  stop_watch_ptr_(std::make_unique<autoware_utils_system::StopWatch<std::chrono::milliseconds>>()),
  debug_publisher_(std::make_unique<autoware_utils_debug::DebugPublisher>(this, this->get_name())),
  published_time_publisher_(std::make_unique<autoware_utils_debug::PublishedTimePublisher>(this))
{
  // initialize debug tool
  {
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  max_queue_size_ = static_cast<int64_t>(declare_parameter("max_queue_size", 5));

  // get transform info for pointcloud
  {
    tf_input_orig_frame_ =
      static_cast<std::string>(declare_parameter("input_pointcloud_frame", "base_link"));
    tf_input_frame_ = static_cast<std::string>(declare_parameter("input_frame", "base_link"));
    tf_output_frame_ = static_cast<std::string>(declare_parameter("output_frame", "base_link"));

    transform_listener_ = std::make_unique<autoware_utils_tf::TransformListener>(this);

    if (tf_input_orig_frame_ == tf_input_frame_) {
      need_preprocess_transform_ = false;
      eigen_transform_preprocess_ = Eigen::Matrix4f::Identity(4, 4);
    } else {
      auto tf_ptr = transform_listener_->get_transform(
        tf_input_frame_, tf_input_orig_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      if (!tf_ptr) {
        RCLCPP_ERROR(
          this->get_logger(), "Cannot get transform from %s to %s. Please check your TF tree.",
          tf_input_orig_frame_.c_str(), tf_input_frame_.c_str());
        // Set identity transform and disable preprocessing when TF acquisition fails
        need_preprocess_transform_ = false;
        eigen_transform_preprocess_ = Eigen::Matrix4f::Identity(4, 4);
      } else {
        auto eigen_tf = tf2::transformToEigen(*tf_ptr);
        eigen_transform_preprocess_ = eigen_tf.matrix().cast<float>();
        need_preprocess_transform_ = true;
      }
    }

    if (tf_input_frame_ == tf_output_frame_) {
      need_postprocess_transform_ = false;
      eigen_transform_postprocess_ = Eigen::Matrix4f::Identity(4, 4);
    } else {
      auto tf_ptr = transform_listener_->get_transform(
        tf_output_frame_, tf_input_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      if (!tf_ptr) {
        RCLCPP_ERROR(
          this->get_logger(), "Cannot get transform from %s to %s. Please check your TF tree.",
          tf_input_frame_.c_str(), tf_output_frame_.c_str());
        // Set identity transform and disable postprocessing when TF acquisition fails
        need_postprocess_transform_ = false;
        eigen_transform_postprocess_ = Eigen::Matrix4f::Identity(4, 4);
      } else {
        auto eigen_tf = tf2::transformToEigen(*tf_ptr);
        eigen_transform_postprocess_ = eigen_tf.matrix().cast<float>();
        need_postprocess_transform_ = true;
      }
    }
  }

  // get polygon parameters
  {
    auto & p = param_;
    p.min_x = declare_parameter<double>("min_x");
    p.min_y = declare_parameter<double>("min_y");
    p.min_z = declare_parameter<double>("min_z");
    p.max_x = declare_parameter<double>("max_x");
    p.max_y = declare_parameter<double>("max_y");
    p.max_z = declare_parameter<double>("max_z");
    negative_ = declare_parameter<bool>("negative");
    if (tf_input_frame_.empty()) {
      throw std::invalid_argument("Crop box requires non-empty input_frame");
    }
  }
  // set output pointcloud publisher
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    pub_output_ = this->create_publisher<PointCloud2>(
      "output", rclcpp::SensorDataQoS().keep_last(max_queue_size_), pub_options);
  }

  // set additional publishers
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    crop_box_polygon_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "~/crop_box_polygon", 10, pub_options);
  }

  // set parameter service callback
  {
    using std::placeholders::_1;
    set_param_res_ =
      this->add_on_set_parameters_callback(std::bind(&CropBoxFilter::param_callback, this, _1));
  }

  // set input pointcloud callback
  {
    sub_input_ = this->create_subscription<PointCloud2>(
      "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_),
      std::bind(&CropBoxFilter::pointcloud_callback, this, std::placeholders::_1));
  }

  RCLCPP_DEBUG(this->get_logger(), "[Filter Constructor] successfully created.");
}

void CropBoxFilter::filter_pointcloud(const PointCloud2ConstPtr & cloud, PointCloud2 & output)
{
  // set up minimum output metadata required for creating iterators
  output.fields = cloud->fields;
  output.point_step = cloud->point_step;
  output.data.resize(cloud->data.size());

  // create output iterators for writing transformed coordinates
  sensor_msgs::PointCloud2Iterator<float> output_x(output, "x");
  sensor_msgs::PointCloud2Iterator<float> output_y(output, "y");
  sensor_msgs::PointCloud2Iterator<float> output_z(output, "z");

  size_t output_size = 0;
  int skipped_count = 0;

  // create input iterators for reading coordinates
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

  for (size_t point_index = 0; iter_x != iter_x.end();
       ++iter_x, ++iter_y, ++iter_z, ++point_index) {
    // extract point data using iterators
    Eigen::Vector4f point;
    point[0] = *iter_x;
    point[1] = *iter_y;
    point[2] = *iter_z;
    point[3] = 1;

    if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) {
      skipped_count++;
      continue;
    }

    // preprocess point for filtering
    Eigen::Vector4f point_preprocessed = point;

    // apply pre-transform if needed
    if (need_preprocess_transform_) {
      point_preprocessed = eigen_transform_preprocess_ * point;
    }

    bool point_is_inside =
      point_preprocessed[2] > param_.min_z && point_preprocessed[2] < param_.max_z &&
      point_preprocessed[1] > param_.min_y && point_preprocessed[1] < param_.max_y &&
      point_preprocessed[0] > param_.min_x && point_preprocessed[0] < param_.max_x;
    if ((!negative_ && point_is_inside) || (negative_ && !point_is_inside)) {
      const size_t global_offset = point_index * cloud->point_step;

      // copy all fields from input to output
      memcpy(&output.data[output_size], &cloud->data[global_offset], cloud->point_step);

      // overwrite x, y, z with transformed coordinates using output iterators
      if (need_postprocess_transform_) {
        Eigen::Vector4f point_postprocessed = eigen_transform_postprocess_ * point_preprocessed;
        *output_x = point_postprocessed[0];
        *output_y = point_postprocessed[1];
        *output_z = point_postprocessed[2];
      } else if (need_preprocess_transform_) {
        *output_x = point_preprocessed[0];
        *output_y = point_preprocessed[1];
        *output_z = point_preprocessed[2];
      }

      ++output_x;
      ++output_y;
      ++output_z;
      output_size += cloud->point_step;
    }
  }

  if (skipped_count > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "%d points contained NaN values and have been ignored",
      skipped_count);
  }

  // construct output cloud
  output.data.resize(output_size);

  output.header.frame_id = tf_output_frame_;

  output.header.stamp = cloud->header.stamp;

  output.height = 1;
  output.is_bigendian = cloud->is_bigendian;
  output.is_dense = cloud->is_dense;
  output.width = static_cast<uint32_t>(output.data.size() / output.height / output.point_step);
  output.row_step = static_cast<uint32_t>(output.data.size() / output.height);
}

void CropBoxFilter::pointcloud_callback(const PointCloud2ConstPtr cloud)
{
  // check whether the pointcloud is valid
  const ValidationResult result = validate_pointcloud2(*cloud);
  if (!result.is_valid) {
    RCLCPP_ERROR(this->get_logger(), "[input_pointcloud_callback] %s", result.reason.c_str());
    return;
  }

  RCLCPP_DEBUG(
    this->get_logger(),
    "[input_pointcloud_callback] PointCloud with %d data points and frame %s on input topic "
    "received.",
    cloud->width * cloud->height, cloud->header.frame_id.c_str());
  // pointcloud check finished

  // pointcloud processing
  auto output = PointCloud2();

  std::scoped_lock lock(mutex_);
  stop_watch_ptr_->toc("processing_time", true);

  // filtering
  filter_pointcloud(cloud, output);

  // publish polygon if subscribers exist
  if (crop_box_polygon_pub_->get_subscription_count() > 0) {
    crop_box_polygon_pub_->publish(
      generate_crop_box_polygon(param_, tf_input_frame_, get_clock()->now()));
  }

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);

    auto pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds((this->get_clock()->now() - cloud->header.stamp).nanoseconds()))
        .count();

    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }

  // publish result pointcloud
  pub_output_->publish(std::move(output));
  published_time_publisher_->publish_if_subscribed(pub_output_, cloud->header.stamp);
}

// update parameters dynamicly
rcl_interfaces::msg::SetParametersResult CropBoxFilter::param_callback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  CropBoxParam new_param{};

  new_param.min_x = get_param(p, "min_x", new_param.min_x) ? new_param.min_x : param_.min_x;
  new_param.min_y = get_param(p, "min_y", new_param.min_y) ? new_param.min_y : param_.min_y;
  new_param.min_z = get_param(p, "min_z", new_param.min_z) ? new_param.min_z : param_.min_z;
  new_param.max_x = get_param(p, "max_x", new_param.max_x) ? new_param.max_x : param_.max_x;
  new_param.max_y = get_param(p, "max_y", new_param.max_y) ? new_param.max_y : param_.max_y;
  new_param.max_z = get_param(p, "max_z", new_param.max_z) ? new_param.max_z : param_.max_z;
  bool new_negative = false;
  negative_ = get_param(p, "negative", new_negative) ? new_negative : negative_;

  param_ = new_param;

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace autoware::crop_box_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::crop_box_filter::CropBoxFilter)
