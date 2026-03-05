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
    config_.output_frame = static_cast<std::string>(declare_parameter("output_frame", "base_link"));

    transform_listener_ = std::make_unique<autoware_utils_tf::TransformListener>(this);

    if (tf_input_orig_frame_ != tf_input_frame_) {
      auto tf_ptr = transform_listener_->get_transform(
        tf_input_frame_, tf_input_orig_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      if (!tf_ptr) {
        RCLCPP_ERROR(
          this->get_logger(), "Cannot get transform from %s to %s. Please check your TF tree.",
          tf_input_orig_frame_.c_str(), tf_input_frame_.c_str());
      } else {
        auto eigen_tf = tf2::transformToEigen(*tf_ptr);
        config_.eigen_transform_preprocess = eigen_tf.matrix().cast<float>();
        config_.need_preprocess_transform = true;
      }
    }

    if (tf_input_frame_ != config_.output_frame) {
      auto tf_ptr = transform_listener_->get_transform(
        config_.output_frame, tf_input_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      if (!tf_ptr) {
        RCLCPP_ERROR(
          this->get_logger(), "Cannot get transform from %s to %s. Please check your TF tree.",
          tf_input_frame_.c_str(), config_.output_frame.c_str());
      } else {
        auto eigen_tf = tf2::transformToEigen(*tf_ptr);
        config_.eigen_transform_postprocess = eigen_tf.matrix().cast<float>();
        config_.need_postprocess_transform = true;
      }
    }
  }

  // get polygon parameters
  {
    auto & p = config_.param;
    p.min_x = declare_parameter<double>("min_x");
    p.min_y = declare_parameter<double>("min_y");
    p.min_z = declare_parameter<double>("min_z");
    p.max_x = declare_parameter<double>("max_x");
    p.max_y = declare_parameter<double>("max_y");
    p.max_z = declare_parameter<double>("max_z");
    config_.keep_outside_box = declare_parameter<bool>("negative");
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
  std::scoped_lock lock(mutex_);
  stop_watch_ptr_->toc("processing_time", true);

  // filtering
  auto filter_result = filter_pointcloud(*cloud, config_);
  auto & output = filter_result.pointcloud;

  if (filter_result.skipped_nan_count > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "%d points contained NaN values and have been ignored",
      filter_result.skipped_nan_count);
  }

  // publish polygon if subscribers exist
  if (crop_box_polygon_pub_->get_subscription_count() > 0) {
    crop_box_polygon_pub_->publish(
      generate_crop_box_polygon(config_.param, tf_input_frame_, get_clock()->now()));
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

  new_param.min_x = get_param(p, "min_x", new_param.min_x) ? new_param.min_x : config_.param.min_x;
  new_param.min_y = get_param(p, "min_y", new_param.min_y) ? new_param.min_y : config_.param.min_y;
  new_param.min_z = get_param(p, "min_z", new_param.min_z) ? new_param.min_z : config_.param.min_z;
  new_param.max_x = get_param(p, "max_x", new_param.max_x) ? new_param.max_x : config_.param.max_x;
  new_param.max_y = get_param(p, "max_y", new_param.max_y) ? new_param.max_y : config_.param.max_y;
  new_param.max_z = get_param(p, "max_z", new_param.max_z) ? new_param.max_z : config_.param.max_z;
  bool new_keep_outside_box = false;
  config_.keep_outside_box = get_param(p, "negative", new_keep_outside_box)
                               ? new_keep_outside_box
                               : config_.keep_outside_box;

  config_.param = new_param;

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace autoware::crop_box_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::crop_box_filter::CropBoxFilter)
