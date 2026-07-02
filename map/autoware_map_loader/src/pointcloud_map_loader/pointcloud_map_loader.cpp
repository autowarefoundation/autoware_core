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

#include "pointcloud_map_loader.hpp"

#include <fmt/format.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <vector>

namespace autoware::map_loader
{
sensor_msgs::msg::PointCloud2 downsample_pointcloud(
  const sensor_msgs::msg::PointCloud2 & msg_input, const float leaf_size)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(msg_input, *pcl_input);
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(pcl_input);
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  filter.filter(*pcl_output);

  sensor_msgs::msg::PointCloud2 msg_output;
  pcl::toROSMsg(*pcl_output, msg_output);
  msg_output.header = msg_input.header;
  return msg_output;
}

sensor_msgs::msg::PointCloud2 load_pointcloud_map(
  const std::vector<std::string> & pcd_paths, const boost::optional<float> leaf_size,
  const PointcloudLoaderLogFunction & debug_log, const PointcloudLoaderLogFunction & error_log)
{
  sensor_msgs::msg::PointCloud2 whole_pcd;
  sensor_msgs::msg::PointCloud2 partial_pcd;

  for (size_t i = 0; i < pcd_paths.size(); ++i) {
    const auto & path = pcd_paths[i];
    if (i % 50 == 0) {
      debug_log(fmt::format("Load {} ({} out of {})", path, i + 1, pcd_paths.size()));
    }

    if (pcl::io::loadPCDFile(path, partial_pcd) == -1) {
      error_log("PCD load failed: " + path);
    }

    if (leaf_size) {
      partial_pcd = downsample_pointcloud(partial_pcd, leaf_size.get());
    }

    if (whole_pcd.width == 0) {
      whole_pcd = partial_pcd;
    } else {
      whole_pcd.width += partial_pcd.width;
      whole_pcd.row_step += partial_pcd.row_step;
      whole_pcd.data.insert(whole_pcd.data.end(), partial_pcd.data.begin(), partial_pcd.data.end());
    }
  }

  whole_pcd.header.frame_id = "map";
  return whole_pcd;
}
}  // namespace autoware::map_loader
