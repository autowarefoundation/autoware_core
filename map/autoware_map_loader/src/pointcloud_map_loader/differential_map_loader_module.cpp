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

#include <malloc.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>
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

  enable_differential_pcd_map_visualization_ =
    node->declare_parameter<bool>("enable_differential_pcd_map_visualization", false);
  differential_pcd_map_visualization_update_interval_sec_ =
    node->declare_parameter<double>("differential_pcd_map_visualization_update_interval_sec", 1.0);
  differential_pcd_map_visualization_radius_ =
    node->declare_parameter<double>("differential_pcd_map_visualization_radius", 50.0);

  if (!enable_differential_pcd_map_visualization_) {
    return;
  }

  if (differential_pcd_map_visualization_radius_ < 0.0) {
    RCLCPP_WARN(
      logger_, "differential_pcd_map_visualization_radius is negative (%f). Clamping to 0.0.",
      differential_pcd_map_visualization_radius_);
    differential_pcd_map_visualization_radius_ = 0.0;
  }

  internal_visualization_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "output/differential_pointcloud_map_internal", rclcpp::QoS{1});

  kinematic_state_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", rclcpp::QoS{1},
    std::bind(&DifferentialMapLoaderModule::on_kinematic_state, this, std::placeholders::_1));

  const double map_update_interval =
    std::isfinite(differential_pcd_map_visualization_update_interval_sec_)
      ? std::max(differential_pcd_map_visualization_update_interval_sec_, 0.1)
      : 1.0;
  if (!std::isfinite(differential_pcd_map_visualization_update_interval_sec_)) {
    RCLCPP_WARN(
      logger_,
      "differential_pcd_map_visualization_update_interval_sec is non-finite (%f). Falling back to "
      "1.0 s.",
      differential_pcd_map_visualization_update_interval_sec_);
  }
  visualization_timer_ = node->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(safe_interval)),
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
      response->new_pointcloud_with_ids.push_back(
        load_point_cloud_map_cell_with_id(logger_, path, map_id, metadata));
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

void DifferentialMapLoaderModule::on_visualization_timer()
{
  geometry_msgs::msg::Point latest_pose;
  bool has_merged_cloud_for_visualization = false;
  std::unordered_set<std::string> current_active_ids;
  {
    std::lock_guard<std::mutex> lock(visualization_mutex_);
    if (!latest_pose_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 3000,
        "Waiting for /localization/kinematic_state. Internal differential visualization is not "
        "queried yet.");
      return;
    }
    latest_pose = *latest_pose_;
    has_merged_cloud_for_visualization = has_merged_cloud_for_visualization_;
    current_active_ids = active_cell_ids_for_visualization_;
  }

  autoware_map_msgs::msg::AreaInfo area;
  area.center_x = static_cast<float>(latest_pose.x);
  area.center_y = static_cast<float>(latest_pose.y);
  area.radius = static_cast<float>(differential_pcd_map_visualization_radius_);

  std::unordered_set<std::string> desired_active_ids;
  desired_active_ids.reserve(all_pcd_file_metadata_dict_.size());
  for (const auto & [path, metadata] : all_pcd_file_metadata_dict_) {
    if (is_grid_within_queried_area(area, metadata)) {
      desired_active_ids.insert(path);
    }
  }

  std::vector<std::string> ids_to_load;
  ids_to_load.reserve(desired_active_ids.size());
  for (const auto & cell_id : desired_active_ids) {
    if (current_active_ids.count(cell_id) != 0U) {
      continue;
    }

    std::lock_guard<std::mutex> lock(visualization_mutex_);
    if (loaded_cells_for_visualization_.count(cell_id) == 0U) {
      ids_to_load.push_back(cell_id);
    }
  }

  for (const auto & cell_id : ids_to_load) {
    auto pointcloud_map_cell_with_id = load_point_cloud_map_cell_with_id(cell_id, cell_id);
    auto cloud_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(
      std::move(pointcloud_map_cell_with_id.pointcloud));

    std::lock_guard<std::mutex> lock(visualization_mutex_);
    loaded_cells_for_visualization_.try_emplace(cell_id, std::move(cloud_ptr));
  }

  bool active_cells_changed = desired_active_ids != current_active_ids;
  {
    std::lock_guard<std::mutex> lock(visualization_mutex_);
    for (auto it = loaded_cells_for_visualization_.begin();
         it != loaded_cells_for_visualization_.end();) {
      if (desired_active_ids.count(it->first) == 0U) {
        it = loaded_cells_for_visualization_.erase(it);
        active_cells_changed = true;
      } else {
        ++it;
      }
    }
    active_cell_ids_for_visualization_ = std::move(desired_active_ids);
  }

  if (active_cells_changed || !has_merged_cloud_for_visualization) {
    rebuild_merged_cloud_for_visualization();
    reusable_merged_cloud_.header.stamp = node_->now();
    reusable_merged_cloud_.header.frame_id = "map";
    internal_visualization_pub_->publish(reusable_merged_cloud_);
    malloc_trim(0);
  }
}

void DifferentialMapLoaderModule::on_kinematic_state(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(visualization_mutex_);
  latest_pose_ = msg->pose.pose.position;
}

void DifferentialMapLoaderModule::rebuild_merged_cloud_for_visualization()
{
  std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> snapshot;
  std::vector<uint8_t> reusable_data;
  {
    std::lock_guard<std::mutex> lock(visualization_mutex_);
    snapshot.reserve(active_cell_ids_for_visualization_.size());
    for (const auto & cell_id : active_cell_ids_for_visualization_) {
      const auto it = loaded_cells_for_visualization_.find(cell_id);
      if (it != loaded_cells_for_visualization_.end() && it->second) {
        snapshot.push_back(it->second);
      }
    }
    reusable_data = std::move(reusable_merged_cloud_.data);
  }

  sensor_msgs::msg::PointCloud2 merged;
  bool has_any = false;

  for (const auto & c : snapshot) {
    if (c && !c->fields.empty() && c->point_step != 0U) {
      merged = *c;
      has_any = true;
      break;
    }
  }
  if (!has_any) {
    std::lock_guard<std::mutex> lock(visualization_mutex_);
    reusable_merged_cloud_ = sensor_msgs::msg::PointCloud2{};
    reusable_merged_cloud_.data = std::move(reusable_data);
    reusable_merged_cloud_.data.clear();
    has_merged_cloud_for_visualization_ = true;
    return;
  }

  merged.data = std::move(reusable_data);

  size_t total = 0;
  for (const auto & c : snapshot) {
    if (!c) continue;
    if (!is_compatible_pointcloud_layout(merged, *c)) continue;
    total += c->data.size();
  }

  merged.data.resize(total);

  size_t offset = 0;
  for (const auto & c : snapshot) {
    if (!c) continue;
    if (!is_compatible_pointcloud_layout(merged, *c)) continue;
    std::memcpy(merged.data.data() + offset, c->data.data(), c->data.size());
    offset += c->data.size();
  }

  merged.height = 1;
  merged.width =
    merged.point_step ? static_cast<uint32_t>(merged.data.size() / merged.point_step) : 0U;
  merged.row_step = merged.point_step * merged.width;
  merged.is_dense = false;

  {
    std::lock_guard<std::mutex> lock(visualization_mutex_);
    reusable_merged_cloud_ = std::move(merged);
    has_merged_cloud_for_visualization_ = true;
  }
}

bool DifferentialMapLoaderModule::is_compatible_pointcloud_layout(
  const sensor_msgs::msg::PointCloud2 & lhs, const sensor_msgs::msg::PointCloud2 & rhs)
{
  return lhs.fields == rhs.fields && lhs.is_bigendian == rhs.is_bigendian &&
         lhs.point_step == rhs.point_step;
}
}  // namespace autoware::map_loader
