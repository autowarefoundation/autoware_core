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

#include "crop_box_filter.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <sstream>
#include <string>

namespace autoware::crop_box_filter
{

static ValidationResult validate_xyz_fields(const PointCloud2 & cloud)
{
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;

  for (const auto & field : cloud.fields) {
    if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
      continue;
    }

    if (field.name == "x") {
      has_x = true;
    } else if (field.name == "y") {
      has_y = true;
    } else if (field.name == "z") {
      has_z = true;
    }

    if (has_x && has_y && has_z) break;
  }

  if (!has_x || !has_y || !has_z) {
    return {false, "The pointcloud does not have the required x, y, z FLOAT32 fields."};
  }
  return {true, ""};
}

static ValidationResult validate_data_size(const PointCloud2 & cloud)
{
  if (cloud.width * cloud.height * cloud.point_step > cloud.data.size()) {
    std::ostringstream oss;
    oss << "Invalid PointCloud (data.size = " << cloud.data.size() << ", width = " << cloud.width
        << ", height = " << cloud.height << ", step = " << cloud.point_step << ") with stamp "
        << cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9 << ", and frame "
        << cloud.header.frame_id << " received!";
    return {false, oss.str()};
  }
  return {true, ""};
}

ValidationResult validate_pointcloud2(const PointCloud2 & cloud)
{
  const ValidationResult xyz_validation_result = validate_xyz_fields(cloud);
  if (!xyz_validation_result.is_valid) {
    return xyz_validation_result;
  }
  const ValidationResult data_size_validation_result = validate_data_size(cloud);
  if (!data_size_validation_result.is_valid) {
    return data_size_validation_result;
  }
  return {true, ""};
}

geometry_msgs::msg::PolygonStamped generate_crop_box_polygon(
  const CropBoxParam & param, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  auto generate_point = [](double x, double y, double z) {
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  };

  const double x1 = param.max_x;
  const double x2 = param.min_x;
  const double x3 = param.min_x;
  const double x4 = param.max_x;

  const double y1 = param.max_y;
  const double y2 = param.max_y;
  const double y3 = param.min_y;
  const double y4 = param.min_y;

  const double z1 = param.min_z;
  const double z2 = param.max_z;

  geometry_msgs::msg::PolygonStamped polygon_msg;
  polygon_msg.header.frame_id = frame_id;
  polygon_msg.header.stamp = stamp;
  polygon_msg.polygon.points.push_back(generate_point(x1, y1, z1));
  polygon_msg.polygon.points.push_back(generate_point(x2, y2, z1));
  polygon_msg.polygon.points.push_back(generate_point(x3, y3, z1));
  polygon_msg.polygon.points.push_back(generate_point(x4, y4, z1));
  polygon_msg.polygon.points.push_back(generate_point(x1, y1, z1));

  polygon_msg.polygon.points.push_back(generate_point(x1, y1, z2));

  polygon_msg.polygon.points.push_back(generate_point(x2, y2, z2));
  polygon_msg.polygon.points.push_back(generate_point(x2, y2, z1));
  polygon_msg.polygon.points.push_back(generate_point(x2, y2, z2));

  polygon_msg.polygon.points.push_back(generate_point(x3, y3, z2));
  polygon_msg.polygon.points.push_back(generate_point(x3, y3, z1));
  polygon_msg.polygon.points.push_back(generate_point(x3, y3, z2));

  polygon_msg.polygon.points.push_back(generate_point(x4, y4, z2));
  polygon_msg.polygon.points.push_back(generate_point(x4, y4, z1));
  polygon_msg.polygon.points.push_back(generate_point(x4, y4, z2));

  polygon_msg.polygon.points.push_back(generate_point(x1, y1, z2));

  return polygon_msg;
}

CropBoxFilter::CropBoxFilter(const CropBoxFilterConfig & config) : config_(config)
{
  if (config_.preprocess_transform) {
    const auto eigen_transform = tf2::transformToEigen(config_.preprocess_transform.value());
    eigen_transform_preprocess_ = eigen_transform.matrix().cast<float>();
    output_frame_id_ = config_.preprocess_transform->header.frame_id;
  }
  if (config_.postprocess_transform) {
    const auto eigen_transform = tf2::transformToEigen(config_.postprocess_transform.value());
    eigen_transform_postprocess_ = eigen_transform.matrix().cast<float>();
    output_frame_id_ = config_.postprocess_transform->header.frame_id;
  }
}

CropBoxFilterResult CropBoxFilter::filter(const PointCloud2 & cloud) const
{
  CropBoxFilterResult result;
  auto & output = result.pointcloud;

  // set up minimum output metadata required for creating iterators
  output.fields = cloud.fields;
  output.point_step = cloud.point_step;
  output.data.resize(cloud.data.size());

  // create output iterators for writing transformed coordinates
  sensor_msgs::PointCloud2Iterator<float> output_x(output, "x");
  sensor_msgs::PointCloud2Iterator<float> output_y(output, "y");
  sensor_msgs::PointCloud2Iterator<float> output_z(output, "z");

  size_t output_size = 0;

  // create input iterators for reading coordinates
  sensor_msgs::PointCloud2ConstIterator<float> input_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> input_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> input_z(cloud, "z");

  for (size_t point_index = 0; input_x != input_x.end();
       ++input_x, ++input_y, ++input_z, ++point_index) {
    Eigen::Vector4f point;
    point[0] = *input_x;
    point[1] = *input_y;
    point[2] = *input_z;
    point[3] = 1;

    if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) {
      result.skipped_nan_count++;
      continue;
    }

    const Eigen::Vector4f point_preprocessed = eigen_transform_preprocess_ * point;

    bool point_is_inside =
      point_preprocessed[2] > config_.param.min_z && point_preprocessed[2] < config_.param.max_z &&
      point_preprocessed[1] > config_.param.min_y && point_preprocessed[1] < config_.param.max_y &&
      point_preprocessed[0] > config_.param.min_x && point_preprocessed[0] < config_.param.max_x;

    if (
      (!config_.keep_outside_box && point_is_inside) ||
      (config_.keep_outside_box && !point_is_inside)) {
      const size_t global_offset = point_index * cloud.point_step;

      memcpy(&output.data[output_size], &cloud.data[global_offset], cloud.point_step);

      const Eigen::Vector4f point_output = eigen_transform_postprocess_ * point_preprocessed;
      *output_x = point_output[0];
      *output_y = point_output[1];
      *output_z = point_output[2];

      ++output_x;
      ++output_y;
      ++output_z;
      output_size += cloud.point_step;
    }
  }

  output.data.resize(output_size);
  output.header.frame_id = output_frame_id_.value_or(cloud.header.frame_id);
  output.header.stamp = cloud.header.stamp;
  output.height = 1;
  output.is_bigendian = cloud.is_bigendian;
  output.is_dense = cloud.is_dense;
  output.width = static_cast<uint32_t>(output.data.size() / output.height / output.point_step);
  output.row_step = static_cast<uint32_t>(output.data.size() / output.height);

  return result;
}

}  // namespace autoware::crop_box_filter
