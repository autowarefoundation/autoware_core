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

#include "autoware/crop_box_filter/crop_box_filter_node.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::crop_box_filter
{
CropBoxFilter::CropBoxFilter(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("crop_box_filter", node_options)
{
  // initialize debug tool
  {
    using autoware_utils_debug::DebugPublisher;
    using autoware_utils_system::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, this->get_name());
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");

    published_time_publisher_ =
      std::make_unique<autoware_utils_debug::PublishedTimePublisher>(this);
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
    p.negative = declare_parameter<bool>("negative");
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
  int x_offset = cloud->fields[pcl::getFieldIndex(*cloud, "x")].offset;
  int y_offset = cloud->fields[pcl::getFieldIndex(*cloud, "y")].offset;
  int z_offset = cloud->fields[pcl::getFieldIndex(*cloud, "z")].offset;

  output.data.resize(cloud->data.size());
  size_t output_size = 0;

  int skipped_count = 0;

  // pointcloud processing loop
  for (size_t global_offset = 0; global_offset + cloud->point_step <= cloud->data.size();
       global_offset += cloud->point_step) {
    // extract point data from point cloud data buffer
    Eigen::Vector4f point;

    std::memcpy(&point[0], &cloud->data[global_offset + x_offset], sizeof(float));
    std::memcpy(&point[1], &cloud->data[global_offset + y_offset], sizeof(float));
    std::memcpy(&point[2], &cloud->data[global_offset + z_offset], sizeof(float));
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
    if ((!param_.negative && point_is_inside) || (param_.negative && !point_is_inside)) {
      // apply post-transform if needed
      if (need_postprocess_transform_) {
        Eigen::Vector4f point_postprocessed = eigen_transform_postprocess_ * point_preprocessed;

        memcpy(&output.data[output_size], &cloud->data[global_offset], cloud->point_step);

        std::memcpy(&output.data[output_size + x_offset], &point_postprocessed[0], sizeof(float));
        std::memcpy(&output.data[output_size + y_offset], &point_postprocessed[1], sizeof(float));
        std::memcpy(&output.data[output_size + z_offset], &point_postprocessed[2], sizeof(float));
      } else {
        memcpy(&output.data[output_size], &cloud->data[global_offset], cloud->point_step);

        if (need_preprocess_transform_) {
          std::memcpy(&output.data[output_size + x_offset], &point_preprocessed[0], sizeof(float));
          std::memcpy(&output.data[output_size + y_offset], &point_preprocessed[1], sizeof(float));
          std::memcpy(&output.data[output_size + z_offset], &point_preprocessed[2], sizeof(float));
        }
      }
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
  output.fields = cloud->fields;
  output.is_bigendian = cloud->is_bigendian;
  output.point_step = cloud->point_step;
  output.is_dense = cloud->is_dense;
  output.width = static_cast<uint32_t>(output.data.size() / output.height / output.point_step);
  output.row_step = static_cast<uint32_t>(output.data.size() / output.height);
}

void CropBoxFilter::pointcloud_callback(const PointCloud2ConstPtr cloud)
{
  // check whether the pointcloud is valid
  if (!is_valid(cloud)) {
    RCLCPP_ERROR(this->get_logger(), "[input_pointcloud_callback] Invalid input pointcloud!");
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
    publish_crop_box_polygon();
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

void CropBoxFilter::publish_crop_box_polygon()
{
  auto generatePoint = [](double x, double y, double z) {
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  };

  const double x1 = param_.max_x;
  const double x2 = param_.min_x;
  const double x3 = param_.min_x;
  const double x4 = param_.max_x;

  const double y1 = param_.max_y;
  const double y2 = param_.max_y;
  const double y3 = param_.min_y;
  const double y4 = param_.min_y;

  const double z1 = param_.min_z;
  const double z2 = param_.max_z;

  geometry_msgs::msg::PolygonStamped polygon_msg;
  polygon_msg.header.frame_id = tf_input_frame_;
  polygon_msg.header.stamp = get_clock()->now();
  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z1));

  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z2));

  crop_box_polygon_pub_->publish(polygon_msg);
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
  new_param.negative =
    get_param(p, "negative", new_param.negative) ? new_param.negative : param_.negative;

  param_ = new_param;

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

bool CropBoxFilter::is_data_layout_compatible_with_point_xyzi(const PointCloud2 & input)
{
  using PointIndex = autoware::point_types::PointXYZIIndex;
  using autoware::point_types::PointXYZI;
  if (input.fields.size() < 4) {
    return false;
  }
  bool same_layout = true;
  const auto & field_x = input.fields.at(static_cast<size_t>(PointIndex::X));
  same_layout &= field_x.name == "x";
  same_layout &= field_x.offset == offsetof(PointXYZI, x);
  same_layout &= field_x.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_x.count == 1;
  const auto & field_y = input.fields.at(static_cast<size_t>(PointIndex::Y));
  same_layout &= field_y.name == "y";
  same_layout &= field_y.offset == offsetof(PointXYZI, y);
  same_layout &= field_y.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_y.count == 1;
  const auto & field_z = input.fields.at(static_cast<size_t>(PointIndex::Z));
  same_layout &= field_z.name == "z";
  same_layout &= field_z.offset == offsetof(PointXYZI, z);
  same_layout &= field_z.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_z.count == 1;
  const auto & field_intensity = input.fields.at(static_cast<size_t>(PointIndex::Intensity));
  same_layout &= field_intensity.name == "intensity";
  same_layout &= field_intensity.offset == offsetof(PointXYZI, intensity);
  same_layout &= field_intensity.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_intensity.count == 1;
  return same_layout;
}

bool CropBoxFilter::is_data_layout_compatible_with_point_xyzirc(const PointCloud2 & input)
{
  using PointIndex = autoware::point_types::PointXYZIRCIndex;
  using autoware::point_types::PointXYZIRC;
  if (input.fields.size() < 6) {
    return false;
  }
  bool same_layout = true;
  const auto & field_x = input.fields.at(static_cast<size_t>(PointIndex::X));
  same_layout &= field_x.name == "x";
  same_layout &= field_x.offset == offsetof(PointXYZIRC, x);
  same_layout &= field_x.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_x.count == 1;
  const auto & field_y = input.fields.at(static_cast<size_t>(PointIndex::Y));
  same_layout &= field_y.name == "y";
  same_layout &= field_y.offset == offsetof(PointXYZIRC, y);
  same_layout &= field_y.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_y.count == 1;
  const auto & field_z = input.fields.at(static_cast<size_t>(PointIndex::Z));
  same_layout &= field_z.name == "z";
  same_layout &= field_z.offset == offsetof(PointXYZIRC, z);
  same_layout &= field_z.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_z.count == 1;
  const auto & field_intensity = input.fields.at(static_cast<size_t>(PointIndex::Intensity));
  same_layout &= field_intensity.name == "intensity";
  same_layout &= field_intensity.offset == offsetof(PointXYZIRC, intensity);
  same_layout &= field_intensity.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_intensity.count == 1;
  const auto & field_return_type = input.fields.at(static_cast<size_t>(PointIndex::ReturnType));
  same_layout &= field_return_type.name == "return_type";
  same_layout &= field_return_type.offset == offsetof(PointXYZIRC, return_type);
  same_layout &= field_return_type.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_return_type.count == 1;
  const auto & field_ring = input.fields.at(static_cast<size_t>(PointIndex::Channel));
  same_layout &= field_ring.name == "channel";
  same_layout &= field_ring.offset == offsetof(PointXYZIRC, channel);
  same_layout &= field_ring.datatype == sensor_msgs::msg::PointField::UINT16;
  same_layout &= field_ring.count == 1;

  return same_layout;
}

bool CropBoxFilter::is_data_layout_compatible_with_point_xyziradrt(const PointCloud2 & input)
{
  using PointIndex = autoware::point_types::PointXYZIRADRTIndex;
  using autoware::point_types::PointXYZIRADRT;
  if (input.fields.size() < 9) {
    return false;
  }
  bool same_layout = true;
  const auto & field_x = input.fields.at(static_cast<size_t>(PointIndex::X));
  same_layout &= field_x.name == "x";
  same_layout &= field_x.offset == offsetof(PointXYZIRADRT, x);
  same_layout &= field_x.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_x.count == 1;
  const auto & field_y = input.fields.at(static_cast<size_t>(PointIndex::Y));
  same_layout &= field_y.name == "y";
  same_layout &= field_y.offset == offsetof(PointXYZIRADRT, y);
  same_layout &= field_y.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_y.count == 1;
  const auto & field_z = input.fields.at(static_cast<size_t>(PointIndex::Z));
  same_layout &= field_z.name == "z";
  same_layout &= field_z.offset == offsetof(PointXYZIRADRT, z);
  same_layout &= field_z.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_z.count == 1;
  const auto & field_intensity = input.fields.at(static_cast<size_t>(PointIndex::Intensity));
  same_layout &= field_intensity.name == "intensity";
  same_layout &= field_intensity.offset == offsetof(PointXYZIRADRT, intensity);
  same_layout &= field_intensity.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_intensity.count == 1;
  const auto & field_ring = input.fields.at(static_cast<size_t>(PointIndex::Ring));
  same_layout &= field_ring.name == "ring";
  same_layout &= field_ring.offset == offsetof(PointXYZIRADRT, ring);
  same_layout &= field_ring.datatype == sensor_msgs::msg::PointField::UINT16;
  same_layout &= field_ring.count == 1;
  const auto & field_azimuth = input.fields.at(static_cast<size_t>(PointIndex::Azimuth));
  same_layout &= field_azimuth.name == "azimuth";
  same_layout &= field_azimuth.offset == offsetof(PointXYZIRADRT, azimuth);
  same_layout &= field_azimuth.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_azimuth.count == 1;
  const auto & field_distance = input.fields.at(static_cast<size_t>(PointIndex::Distance));
  same_layout &= field_distance.name == "distance";
  same_layout &= field_distance.offset == offsetof(PointXYZIRADRT, distance);
  same_layout &= field_distance.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_distance.count == 1;
  const auto & field_return_type = input.fields.at(static_cast<size_t>(PointIndex::ReturnType));
  same_layout &= field_return_type.name == "return_type";
  same_layout &= field_return_type.offset == offsetof(PointXYZIRADRT, return_type);
  same_layout &= field_return_type.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_return_type.count == 1;
  const auto & field_time_stamp = input.fields.at(static_cast<size_t>(PointIndex::TimeStamp));
  same_layout &= field_time_stamp.name == "time_stamp";
  same_layout &= field_time_stamp.offset == offsetof(PointXYZIRADRT, time_stamp);
  same_layout &= field_time_stamp.datatype == sensor_msgs::msg::PointField::FLOAT64;
  same_layout &= field_time_stamp.count == 1;
  return same_layout;
}

bool CropBoxFilter::is_data_layout_compatible_with_point_xyzircaedt(const PointCloud2 & input)
{
  using PointIndex = autoware::point_types::PointXYZIRCAEDTIndex;
  using autoware::point_types::PointXYZIRCAEDT;
  if (input.fields.size() != 10) {
    return false;
  }
  bool same_layout = true;
  const auto & field_x = input.fields.at(static_cast<size_t>(PointIndex::X));
  same_layout &= field_x.name == "x";
  same_layout &= field_x.offset == offsetof(PointXYZIRCAEDT, x);
  same_layout &= field_x.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_x.count == 1;
  const auto & field_y = input.fields.at(static_cast<size_t>(PointIndex::Y));
  same_layout &= field_y.name == "y";
  same_layout &= field_y.offset == offsetof(PointXYZIRCAEDT, y);
  same_layout &= field_y.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_y.count == 1;
  const auto & field_z = input.fields.at(static_cast<size_t>(PointIndex::Z));
  same_layout &= field_z.name == "z";
  same_layout &= field_z.offset == offsetof(PointXYZIRCAEDT, z);
  same_layout &= field_z.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_z.count == 1;
  const auto & field_intensity = input.fields.at(static_cast<size_t>(PointIndex::Intensity));
  same_layout &= field_intensity.name == "intensity";
  same_layout &= field_intensity.offset == offsetof(PointXYZIRCAEDT, intensity);
  same_layout &= field_intensity.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_intensity.count == 1;
  const auto & field_return_type = input.fields.at(static_cast<size_t>(PointIndex::ReturnType));
  same_layout &= field_return_type.name == "return_type";
  same_layout &= field_return_type.offset == offsetof(PointXYZIRCAEDT, return_type);
  same_layout &= field_return_type.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_return_type.count == 1;
  const auto & field_ring = input.fields.at(static_cast<size_t>(PointIndex::Channel));
  same_layout &= field_ring.name == "channel";
  same_layout &= field_ring.offset == offsetof(PointXYZIRCAEDT, channel);
  same_layout &= field_ring.datatype == sensor_msgs::msg::PointField::UINT16;
  same_layout &= field_ring.count == 1;
  const auto & field_azimuth = input.fields.at(static_cast<size_t>(PointIndex::Azimuth));
  same_layout &= field_azimuth.name == "azimuth";
  same_layout &= field_azimuth.offset == offsetof(PointXYZIRCAEDT, azimuth);
  same_layout &= field_azimuth.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_azimuth.count == 1;
  const auto & field_elevation = input.fields.at(static_cast<size_t>(PointIndex::Elevation));
  same_layout &= field_elevation.name == "elevation";
  same_layout &= field_elevation.offset == offsetof(PointXYZIRCAEDT, elevation);
  same_layout &= field_elevation.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_elevation.count == 1;
  const auto & field_distance = input.fields.at(static_cast<size_t>(PointIndex::Distance));
  same_layout &= field_distance.name == "distance";
  same_layout &= field_distance.offset == offsetof(PointXYZIRCAEDT, distance);
  same_layout &= field_distance.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_distance.count == 1;
  const auto & field_time_stamp = input.fields.at(static_cast<size_t>(PointIndex::TimeStamp));
  same_layout &= field_time_stamp.name == "time_stamp";
  same_layout &= field_time_stamp.offset == offsetof(PointXYZIRCAEDT, time_stamp);
  same_layout &= field_time_stamp.datatype == sensor_msgs::msg::PointField::UINT32;
  same_layout &= field_time_stamp.count == 1;
  return same_layout;
}

bool CropBoxFilter::is_valid(const PointCloud2ConstPtr & cloud)
{
  // firstly check the fields of the point cloud
  if (
    !is_data_layout_compatible_with_point_xyzircaedt(*cloud) &&
    !is_data_layout_compatible_with_point_xyzirc(*cloud)) {
    RCLCPP_ERROR(
      get_logger(),
      "The pointcloud layout is not compatible with PointXYZIRCAEDT or PointXYZIRC. Aborting");

    if (is_data_layout_compatible_with_point_xyziradrt(*cloud)) {
      RCLCPP_ERROR(
        get_logger(),
        "The pointcloud layout is compatible with PointXYZIRADRT. You may be using legacy "
        "code/data");
    }

    if (is_data_layout_compatible_with_point_xyzi(*cloud)) {
      RCLCPP_ERROR(
        get_logger(),
        "The pointcloud layout is compatible with PointXYZI. You may be using legacy "
        "code/data");
    }

    return false;
  }

  // secondly, verify the total size of the point cloud
  if (cloud->width * cloud->height * cloud->point_step != cloud->data.size()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid PointCloud (data = %zu, width = %d, height = %d, step = %d) with stamp %f, "
      "and frame %s received!",
      cloud->data.size(), cloud->width, cloud->height, cloud->point_step,
      rclcpp::Time(cloud->header.stamp).seconds(), cloud->header.frame_id.c_str());
    return false;
  }
  return true;
}

}  // namespace autoware::crop_box_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::crop_box_filter::CropBoxFilter)
