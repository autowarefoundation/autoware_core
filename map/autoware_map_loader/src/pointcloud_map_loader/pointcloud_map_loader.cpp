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

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::map_loader
{
namespace fs = std::filesystem;

namespace
{
bool is_pcd_file(const std::string & p)
{
  if (fs::is_directory(p)) {
    return false;
  }

  const std::string ext = fs::path(p).extension();
  return !(ext != ".pcd" && ext != ".PCD");
}
}  // namespace

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

std::vector<std::string> resolve_pcd_paths(
  const std::vector<std::string> & pcd_paths_or_directory,
  const PointcloudLoaderLogFunction & error_log)
{
  std::vector<std::string> pcd_paths;
  for (const auto & p : pcd_paths_or_directory) {
    if (!fs::exists(p)) {
      error_log("invalid path: " + p);
      continue;
    }

    if (is_pcd_file(p)) {
      pcd_paths.push_back(p);
    }

    if (fs::is_directory(p)) {
      for (const auto & file : fs::directory_iterator(p)) {
        const auto filename = file.path().string();
        if (is_pcd_file(filename)) {
          pcd_paths.push_back(filename);
        }
      }
    }
  }

  return pcd_paths;
}

std::map<std::string, PCDFileMetadata> build_pcd_metadata_dict(
  const std::string & pcd_metadata_path, const std::vector<std::string> & pcd_paths)
{
  if (fs::exists(pcd_metadata_path)) {
    std::set<std::string> missing_pcd_names;
    auto pcd_metadata_dict = load_pcd_metadata(pcd_metadata_path);

    pcd_metadata_dict = replace_with_absolute_path(pcd_metadata_dict, pcd_paths, missing_pcd_names);

    // Fail fast when metadata and input PCD set are inconsistent.
    // This keeps downstream loaders from silently operating on incomplete map tiles.
    if (!missing_pcd_names.empty()) {
      std::ostringstream oss;
      oss << "The following segment(s) are missing from the input PCDs: ";
      for (const auto & fname : missing_pcd_names) {
        oss << std::endl << fname;
      }
      throw std::runtime_error(oss.str());
    }

    return pcd_metadata_dict;
  }

  if (pcd_paths.size() == 1) {
    // Compatibility exception: allow single-file maps without an external metadata YAML.
    // In this mode, metadata bounds are inferred directly from the PCD content.
    // Note: metadata-file driven operation is still the preferred long-term path.
    pcl::PointCloud<pcl::PointXYZ> single_pcd;
    const auto & pcd_path = pcd_paths.front();
    if (pcl::io::loadPCDFile(pcd_path, single_pcd) == -1) {
      throw std::runtime_error("PCD load failed: " + pcd_path);
    }

    PCDFileMetadata metadata = {};
    pcl::getMinMax3D(single_pcd, metadata.min, metadata.max);
    return std::map<std::string, PCDFileMetadata>{{pcd_path, metadata}};
  }

  throw std::runtime_error("PCD metadata file not found: " + pcd_metadata_path);
}
}  // namespace autoware::map_loader
