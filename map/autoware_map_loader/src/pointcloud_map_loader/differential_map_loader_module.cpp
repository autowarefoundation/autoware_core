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

#include "differential_map_loader_module.hpp"

#include <algorithm>
#include <chrono>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace autoware::map_loader
{
DifferentialMapLoaderModule::DifferentialMapLoaderModule(
  rclcpp::Node * node, std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict)
: logger_(node->get_logger()),
  all_pcd_file_metadata_dict_(std::move(pcd_file_metadata_dict)),
  node_(node)
{
  get_differential_pcd_maps_service_ = node->create_service<GetDifferentialPointCloudMap>(
    "service/get_differential_pcd_map",
    std::bind(
      &DifferentialMapLoaderModule::on_service_get_differential_point_cloud_map, this,
      std::placeholders::_1, std::placeholders::_2));

  enable_internal_differential_visualization_ =
    node->declare_parameter<bool>("enable_internal_differential_visualization", false);
  visualization_use_pose_ = node->declare_parameter<bool>("internal_visualization_use_pose", false);
  visualization_update_interval_sec_ =
    node->declare_parameter<double>("internal_visualization_update_interval_sec", 1.0);
  visualization_center_x_ = node->declare_parameter<double>("internal_visualization_center_x", 0.0);
  visualization_center_y_ = node->declare_parameter<double>("internal_visualization_center_y", 0.0);
  visualization_radius_ = node->declare_parameter<double>("internal_visualization_radius", 150.0);

  if (!enable_internal_differential_visualization_) {
    return;
  }

  internal_visualization_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "output/differential_pointcloud_map_internal", rclcpp::QoS{1}.transient_local());

  if (visualization_use_pose_) {
    kinematic_state_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", rclcpp::QoS{1},
      std::bind(&DifferentialMapLoaderModule::on_kinematic_state, this, std::placeholders::_1));
  }

  visualization_timer_ = node->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(
      std::max(visualization_update_interval_sec_, 0.1))),
    std::bind(&DifferentialMapLoaderModule::on_visualization_timer, this));
}

void DifferentialMapLoaderModule::differential_area_load(
  const autoware_map_msgs::msg::AreaInfo & area_info, const std::vector<std::string> & cached_ids,
  const GetDifferentialPointCloudMap::Response::SharedPtr & response) const
{
  // iterate over all the available pcd map grids
  std::vector<bool> should_remove(static_cast<int>(cached_ids.size()), true);
  for (const auto & ele : all_pcd_file_metadata_dict_) {
    std::string path = ele.first;
    PCDFileMetadata metadata = ele.second;

    // assume that the map ID = map path (for now)
    const std::string & map_id = path;

    // skip if the pcd file is not within the queried area
    if (!is_grid_within_queried_area(area_info, metadata)) continue;

    auto id_in_cached_list = std::find(cached_ids.begin(), cached_ids.end(), map_id);
    if (id_in_cached_list != cached_ids.end()) {
      int index = static_cast<int>(id_in_cached_list - cached_ids.begin());
      should_remove[index] = false;
    } else {
      autoware_map_msgs::msg::PointCloudMapCellWithID pointcloud_map_cell_with_id =
        load_point_cloud_map_cell_with_id(path, map_id);
      pointcloud_map_cell_with_id.metadata.min_x = metadata.min.x;
      pointcloud_map_cell_with_id.metadata.min_y = metadata.min.y;
      pointcloud_map_cell_with_id.metadata.max_x = metadata.max.x;
      pointcloud_map_cell_with_id.metadata.max_y = metadata.max.y;
      response->new_pointcloud_with_ids.push_back(pointcloud_map_cell_with_id);
    }
  }

  for (size_t i = 0; i < cached_ids.size(); ++i) {
    if (should_remove[i]) {
      response->ids_to_remove.push_back(cached_ids[i]);
    }
  }
}

bool DifferentialMapLoaderModule::on_service_get_differential_point_cloud_map(
  GetDifferentialPointCloudMap::Request::SharedPtr req,
  GetDifferentialPointCloudMap::Response::SharedPtr res) const
{
  auto area = req->area;
  std::vector<std::string> cached_ids = req->cached_ids;
  differential_area_load(area, cached_ids, res);
  res->header.frame_id = "map";
  return true;
}

autoware_map_msgs::msg::PointCloudMapCellWithID
DifferentialMapLoaderModule::load_point_cloud_map_cell_with_id(
  const std::string & path, const std::string & map_id) const
{
  sensor_msgs::msg::PointCloud2 pcd;
  if (pcl::io::loadPCDFile(path, pcd) == -1) {
    RCLCPP_ERROR_STREAM(logger_, "PCD load failed: " << path);
  }
  autoware_map_msgs::msg::PointCloudMapCellWithID pointcloud_map_cell_with_id;
  pointcloud_map_cell_with_id.pointcloud = pcd;
  pointcloud_map_cell_with_id.cell_id = map_id;
  return pointcloud_map_cell_with_id;
}

void DifferentialMapLoaderModule::on_visualization_timer()
{
  double query_center_x = visualization_center_x_;
  double query_center_y = visualization_center_y_;

  if (visualization_use_pose_) {
    std::lock_guard<std::mutex> lock(visualization_mutex_);
    if (!latest_pose_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 3000,
        "Waiting for /localization/kinematic_state. Internal differential visualization is not queried yet.");
      return;
    }
    query_center_x = latest_pose_->x;
    query_center_y = latest_pose_->y;
  }

  auto response = std::make_shared<GetDifferentialPointCloudMap::Response>();
  std::vector<std::string> cached_ids;
  {
    std::lock_guard<std::mutex> lock(visualization_mutex_);
    cached_ids.reserve(loaded_cells_for_visualization_.size());
    for (const auto & [cell_id, cloud] : loaded_cells_for_visualization_) {
      static_cast<void>(cloud);
      cached_ids.push_back(cell_id);
    }
  }

  autoware_map_msgs::msg::AreaInfo area;
  area.center_x = static_cast<float>(query_center_x);
  area.center_y = static_cast<float>(query_center_y);
  area.radius = static_cast<float>(visualization_radius_);
  differential_area_load(area, cached_ids, response);

  bool cache_changed = false;
  {
    std::lock_guard<std::mutex> lock(visualization_mutex_);
    for (const auto & id_to_remove : response->ids_to_remove) {
      cache_changed |= loaded_cells_for_visualization_.erase(id_to_remove) > 0U;
    }
    for (const auto & pcd_with_id : response->new_pointcloud_with_ids) {
      loaded_cells_for_visualization_[pcd_with_id.cell_id] = pcd_with_id.pointcloud;
      cache_changed = true;
    }
  }

  if (cache_changed || !has_merged_cloud_for_visualization_) {
    rebuild_merged_cloud_for_visualization();
  }

  const auto stamp = node_->now();
  reusable_merged_cloud_.header.stamp = stamp;
  reusable_merged_cloud_.header.frame_id = "map";
  internal_visualization_pub_->publish(reusable_merged_cloud_);
}

void DifferentialMapLoaderModule::on_kinematic_state(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(visualization_mutex_);
  latest_pose_ = msg->pose.pose.position;
}

void DifferentialMapLoaderModule::rebuild_merged_cloud_for_visualization()
{
  {
    std::lock_guard<std::mutex> lock(visualization_mutex_);
    if (loaded_cells_for_visualization_.empty()) {
      reusable_merged_cloud_ = sensor_msgs::msg::PointCloud2{};
      has_merged_cloud_for_visualization_ = true;
      return;
    }

    const auto first_itr = loaded_cells_for_visualization_.begin();
    reusable_merged_cloud_ = first_itr->second;

    size_t total_data_size = 0U;
    for (const auto & [cell_id, cloud] : loaded_cells_for_visualization_) {
      static_cast<void>(cell_id);
      if (!is_compatible_pointcloud_layout(reusable_merged_cloud_, cloud)) {
        continue;
      }
      total_data_size += cloud.data.size();
    }

    reusable_merge_buffer_.clear();
    reusable_merge_buffer_.reserve(total_data_size);

    for (const auto & [cell_id, cloud] : loaded_cells_for_visualization_) {
      if (!is_compatible_pointcloud_layout(reusable_merged_cloud_, cloud)) {
        RCLCPP_WARN_STREAM(
          logger_,
          "Skipped internal visualization cell due to incompatible layout. cell_id: " << cell_id);
        continue;
      }
      reusable_merge_buffer_.insert(
        reusable_merge_buffer_.end(), cloud.data.begin(), cloud.data.end());
    }

    reusable_merged_cloud_.data.swap(reusable_merge_buffer_);
    has_merged_cloud_for_visualization_ = true;
  }

  reusable_merged_cloud_.height = 1;
  if (reusable_merged_cloud_.point_step == 0U) {
    reusable_merged_cloud_.width = 0U;
    reusable_merged_cloud_.row_step = 0U;
    return;
  }
  reusable_merged_cloud_.width = static_cast<uint32_t>(
    reusable_merged_cloud_.data.size() / reusable_merged_cloud_.point_step);
  reusable_merged_cloud_.row_step =
    reusable_merged_cloud_.point_step * reusable_merged_cloud_.width;
  reusable_merged_cloud_.is_dense = false;
}

bool DifferentialMapLoaderModule::is_compatible_pointcloud_layout(
  const sensor_msgs::msg::PointCloud2 & lhs, const sensor_msgs::msg::PointCloud2 & rhs)
{
  return lhs.fields == rhs.fields && lhs.is_bigendian == rhs.is_bigendian &&
         lhs.point_step == rhs.point_step;
}
}  // namespace autoware::map_loader
