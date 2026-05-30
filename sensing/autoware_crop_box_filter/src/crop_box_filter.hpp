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
#include <rclcpp/parameter.hpp>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <optional>
#include <string>
#include <vector>

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
  bool has_preprocess_{false};
  bool has_postprocess_{false};
  std::optional<std::string> output_frame_id_;
};

geometry_msgs::msg::PolygonStamped generate_crop_box_polygon(
  const CropBoxParam & param, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp);

/// \brief Merge a SetParameters request into an existing crop-box configuration.
///
/// Each crop-box bound and the keep_outside_box flag is taken from \p params when present,
/// otherwise it falls back to the corresponding value in \p current. The pre/post-process
/// transforms in \p current are preserved unchanged.
CropBoxFilterConfig merge_crop_box_params(
  const CropBoxFilterConfig & current, const std::vector<rclcpp::Parameter> & params);

}  // namespace autoware::crop_box_filter

#endif  // CROP_BOX_FILTER_HPP_
