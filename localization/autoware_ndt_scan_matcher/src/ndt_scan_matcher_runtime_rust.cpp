// Copyright 2015-2019 Autoware Foundation
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

#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/ndt_scan_matcher_core.hpp>

#include <cstdint>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{
using autoware::localization_util::exchange_color_crc;

std::vector<float> cloud_to_flat(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  std::vector<float> f;
  f.reserve(cloud.size() * 3);
  for (const auto & p : cloud) {
    f.push_back(p.x);
    f.push_back(p.y);
    f.push_back(p.z);
  }
  return f;
}
}  // namespace

pcl::PointCloud<pcl::PointXYZRGB>::Ptr NDTScanMatcher::visualize_point_score(
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr,
  const float & lower_nvs, const float & upper_nvs,
  NormalDistributionsTransform * /*legacy_ndt_ref*/)
{
  pcl::PointCloud<pcl::PointXYZI> nvs_points_in_map_ptr_i;
  const std::vector<float> flat = cloud_to_flat(*sensor_points_in_map_ptr);
  std::vector<float> scores(sensor_points_in_map_ptr->size(), 0.0F);
  autoware_ndt_scan_matcher_rs_ndt_engine_calc_nearest_voxel_score_each_point(
    rs_.engine_raw(), flat.data(), sensor_points_in_map_ptr->size(), scores.data());
  for (std::size_t i = 0; i < sensor_points_in_map_ptr->size(); ++i) {
    if (scores[i] > 0.0F) {
      pcl::PointXYZI p;
      p.x = sensor_points_in_map_ptr->points[i].x;  // NOLINT
      p.y = sensor_points_in_map_ptr->points[i].y;  // NOLINT
      p.z = sensor_points_in_map_ptr->points[i].z;  // NOLINT
      p.intensity = scores[i];
      nvs_points_in_map_ptr_i.points.push_back(p);
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nvs_points_in_map_ptr_rgb{
    new pcl::PointCloud<pcl::PointXYZRGB>};

  const float range = upper_nvs - lower_nvs;
  for (std::size_t i = 0; i < nvs_points_in_map_ptr_i.size(); i++) {
    pcl::PointXYZRGB point;
    point.x = nvs_points_in_map_ptr_i.points[i].x;
    point.y = nvs_points_in_map_ptr_i.points[i].y;
    point.z = nvs_points_in_map_ptr_i.points[i].z;
    std_msgs::msg::ColorRGBA color =
      exchange_color_crc((nvs_points_in_map_ptr_i.points[i].intensity - lower_nvs) / range);
    point.r = static_cast<std::uint8_t>(color.r * 255);
    point.g = static_cast<std::uint8_t>(color.g * 255);
    point.b = static_cast<std::uint8_t>(color.b * 255);
    nvs_points_in_map_ptr_rgb->points.push_back(point);
  }
  return nvs_points_in_map_ptr_rgb;
}

void NDTScanMatcher::add_regularization_pose(
  const rclcpp::Time & sensor_ros_time, NormalDistributionsTransform * /*legacy_ndt_ref*/)
{
  autoware_ndt_scan_matcher_rs_ndt_engine_set_regularization(
    rs_.engine_raw(), 0.0F, 0.0F, 0.0F);
  AwInterpolatedPose interpolated{};
  const int64_t stamp_ns = static_cast<rclcpp::Time>(sensor_ros_time).nanoseconds();
  if (!autoware_ndt_scan_matcher_rs_regularization_interpolate(
        rs_.raw(), stamp_ns, &interpolated)) {
    return;
  }
  autoware_ndt_scan_matcher_rs_ndt_engine_set_regularization(
    rs_.engine_raw(), static_cast<float>(interpolated.position[0]),
    static_cast<float>(interpolated.position[1]), param_.ndt.regularization_scale_factor);
}

}  // namespace autoware::ndt_scan_matcher
