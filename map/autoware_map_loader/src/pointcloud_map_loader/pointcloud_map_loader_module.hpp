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

#ifndef POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_MODULE_HPP_
#define POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_MODULE_HPP_

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/optional.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <vector>

namespace autoware::map_loader
{
class PointcloudMapLoaderModule
{
public:
  explicit PointcloudMapLoaderModule(
    autoware::agnocast_wrapper::Node * node, const std::vector<std::string> & pcd_paths,
    const std::string & publisher_name, const bool use_downsample);

private:
  rclcpp::Logger logger_;
  AUTOWARE_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) pub_pointcloud_map_;

  [[nodiscard]] sensor_msgs::msg::PointCloud2 load_pcd_files(
    const std::vector<std::string> & pcd_paths, const boost::optional<float> leaf_size) const;
};
}  // namespace autoware::map_loader

#endif  // POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_MODULE_HPP_
