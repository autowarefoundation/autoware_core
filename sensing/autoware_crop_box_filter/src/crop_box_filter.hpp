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

#ifndef CROP_BOX_FILTER_HPP_
#define CROP_BOX_FILTER_HPP_

#include <Eigen/Eigen>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <optional>
#include <string>

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

namespace autoware::crop_box_filter
{

struct ValidationResult
{
  bool is_valid;
  std::string reason;
};

ValidationResult validate_pointcloud2(const PointCloud2 & cloud);

struct CropBoxParam
{
  float min_x;
  float max_x;
  float min_y;
  float max_y;
  float min_z;
  float max_z;
};

struct CropBoxFilterConfig
{
  CropBoxParam param;
  bool keep_outside_box{false};
  std::string output_frame;
  std::optional<geometry_msgs::msg::TransformStamped> preprocess_transform{std::nullopt};
  std::optional<geometry_msgs::msg::TransformStamped> postprocess_transform{std::nullopt};
};

struct CropBoxFilterResult
{
  PointCloud2 pointcloud;
  int skipped_nan_count{0};
};

class CropBoxFilter
{
public:
  explicit CropBoxFilter(const CropBoxFilterConfig & config);

  CropBoxFilterResult filter(const PointCloud2 & cloud) const;

private:
  CropBoxFilterConfig config_;
  Eigen::Matrix4f eigen_transform_preprocess_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f eigen_transform_postprocess_{Eigen::Matrix4f::Identity()};
};

geometry_msgs::msg::PolygonStamped generate_crop_box_polygon(
  const CropBoxParam & param, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp);

}  // namespace autoware::crop_box_filter

#endif  // CROP_BOX_FILTER_HPP_
