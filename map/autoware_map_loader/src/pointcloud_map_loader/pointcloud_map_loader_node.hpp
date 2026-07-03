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

#ifndef POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_HPP_
#define POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_HPP_

#include "differential_map_loader.hpp"
#include "partial_map_loader_module.hpp"
#include "pointcloud_map_loader.hpp"
#include "selected_map_loader_module.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/point_cloud_map_meta_data.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

namespace autoware::map_loader
{
class PointCloudMapLoaderNode : public rclcpp::Node
{
  using GetPartialPointCloudMap = autoware_map_msgs::srv::GetPartialPointCloudMap;
  using GetDifferentialPointCloudMap = autoware_map_msgs::srv::GetDifferentialPointCloudMap;
  using GetSelectedPointCloudMap = autoware_map_msgs::srv::GetSelectedPointCloudMap;

public:
  explicit PointCloudMapLoaderNode(const rclcpp::NodeOptions & options);

private:
  std::unique_ptr<PointcloudMapLoaderModule> pcd_map_loader_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_map_;
  std::unique_ptr<PointcloudMapLoaderModule> downsampled_pcd_map_loader_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_downsampled_pointcloud_map_;
  std::unique_ptr<PartialMapLoaderModule> partial_map_loader_;
  rclcpp::Service<GetPartialPointCloudMap>::SharedPtr get_partial_pcd_maps_service_;
  std::unique_ptr<DifferentialMapLoaderModule> differential_map_loader_;
  rclcpp::Service<GetDifferentialPointCloudMap>::SharedPtr get_differential_pcd_maps_service_;
  std::unique_ptr<SelectedMapLoaderModule> selected_map_loader_;
  rclcpp::Service<GetSelectedPointCloudMap>::SharedPtr get_selected_pcd_maps_service_;
  rclcpp::Publisher<autoware_map_msgs::msg::PointCloudMapMetaData>::SharedPtr pub_metadata_;
};
}  // namespace autoware::map_loader

#endif  // POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_HPP_
