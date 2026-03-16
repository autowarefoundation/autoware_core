// Copyright 2022 The Autoware Contributors
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

#ifndef POINTCLOUD_MAP_LOADER__DIFFERENTIAL_MAP_LOADER_MODULE_HPP_
#define POINTCLOUD_MAP_LOADER__DIFFERENTIAL_MAP_LOADER_MODULE_HPP_

#include "utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_map_msgs/srv/get_differential_point_cloud_map.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::map_loader
{
class DifferentialMapLoaderModule
{
  using GetDifferentialPointCloudMap = autoware_map_msgs::srv::GetDifferentialPointCloudMap;

public:
  explicit DifferentialMapLoaderModule(
    rclcpp::Node * node, std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict);

private:
  rclcpp::Logger logger_;

  std::map<std::string, PCDFileMetadata> all_pcd_file_metadata_dict_;
  rclcpp::Service<GetDifferentialPointCloudMap>::SharedPtr get_differential_pcd_maps_service_;
  rclcpp::Node * node_{nullptr};

  bool enable_internal_differential_visualization_{false};
  bool visualization_use_pose_{false};
  double visualization_update_interval_sec_{1.0};
  double visualization_center_x_{0.0};
  double visualization_center_y_{0.0};
  double visualization_radius_{150.0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr internal_visualization_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kinematic_state_sub_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  std::optional<geometry_msgs::msg::Point> latest_pose_;
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2> loaded_cells_for_visualization_;
  sensor_msgs::msg::PointCloud2 reusable_merged_cloud_;
  std::vector<uint8_t> reusable_merge_buffer_;
  bool has_merged_cloud_for_visualization_{false};
  mutable std::mutex visualization_mutex_;

  [[nodiscard]] bool on_service_get_differential_point_cloud_map(
    GetDifferentialPointCloudMap::Request::SharedPtr req,
    GetDifferentialPointCloudMap::Response::SharedPtr res) const;
  void differential_area_load(
    const autoware_map_msgs::msg::AreaInfo & area_info, const std::vector<std::string> & cached_ids,
    const GetDifferentialPointCloudMap::Response::SharedPtr & response) const;
  [[nodiscard]] autoware_map_msgs::msg::PointCloudMapCellWithID load_point_cloud_map_cell_with_id(
    const std::string & path, const std::string & map_id) const;
  void on_visualization_timer();
  void on_kinematic_state(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void rebuild_merged_cloud_for_visualization();
  [[nodiscard]] static bool is_compatible_pointcloud_layout(
    const sensor_msgs::msg::PointCloud2 & lhs, const sensor_msgs::msg::PointCloud2 & rhs);
};
}  // namespace autoware::map_loader

#endif  // POINTCLOUD_MAP_LOADER__DIFFERENTIAL_MAP_LOADER_MODULE_HPP_
