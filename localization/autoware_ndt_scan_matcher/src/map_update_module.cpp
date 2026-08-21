// Copyright 2022 Autoware Foundation
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

#include <autoware/ndt_scan_matcher/map_update_module.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

namespace autoware::ndt_scan_matcher
{

MapUpdateModule::MapUpdateModule(
  Guarded<NdtPtrType> & ndt_ptr, HyperParameters::DynamicMapLoading param,
  PcdLoaderFunction pcd_loader)
: pcd_loader_(std::move(pcd_loader)), ndt_ptr_(ndt_ptr), param_(param)
{
  auto copied = secondary_ndt_ptr_.with([&](auto & secondary_ndt_ptr) {
    // Initially, a direct map update on ndt_ptr_ is needed.
    // ndt_ptr_'s mutex is locked until it is fully rebuilt.
    // From the second update, the update is done on secondary_ndt_ptr_,
    // and ndt_ptr_ is only locked when swapping its pointer with
    // secondary_ndt_ptr_.
    secondary_ndt_ptr.reset(new NdtType);

    return ndt_ptr_.with([&](const auto & ptr) {
      if (ptr) {
        *secondary_ndt_ptr = *ptr;
        return true;
      } else {
        return false;
      }
    });
  });

  if (!copied) {
    std::stringstream message;
    message << "Error at MapUpdateModule::MapUpdateModule."
            << "`ndt_ptr_` is a null NDT pointer.";
    throw std::runtime_error(message.str());
  }
}

MapUpdateModule::UpdateResult MapUpdateModule::callback_timer(
  const geometry_msgs::msg::PointStamped & position)
{
  UpdateResult result;
  DiagnosticsReport & diagnostics = result.diagnostics;

  result.map_updated = secondary_ndt_ptr_.with([&](auto & secondary_ndt_ptr) {
    const auto moved_distance = distance_from_last_update(position.point);
    if (moved_distance) {
      diagnostics.add_key_value(
        {"distance_last_update_position_to_current_position", *moved_distance});
    }
    if (moved_distance && *moved_distance + param_.lidar_radius > param_.map_radius) {
      diagnostics.update_level_and_message(
        DiagnosticLevel::ERROR, "Dynamic map loading is not keeping up.");
    }

    // Skip unless we have moved far enough since the last successful update.
    if (moved_distance && *moved_distance <= param_.update_distance) {
      return false;
    }

    const bool updated = update_map_internal(secondary_ndt_ptr, position.point, diagnostics);
    if (updated && param_.publish_loaded_map) {
      result.loaded_pcd_map = merge_loaded_pcd_map(position.header.stamp);
    }
    return updated;
  });
  return result;
}

std::optional<double> MapUpdateModule::distance_from_last_update(
  const geometry_msgs::msg::Point & position)
{
  const auto last_update_position =
    last_update_position_.with([](const auto & pos) { return pos; });

  if (last_update_position == std::nullopt) {
    return std::nullopt;
  }

  return std::hypot(position.x - last_update_position->x, position.y - last_update_position->y);
}

bool MapUpdateModule::out_of_map_range(const geometry_msgs::msg::Point & position)
{
  const auto moved_distance = distance_from_last_update(position);
  return !moved_distance || *moved_distance + param_.lidar_radius > param_.map_radius;
}

bool MapUpdateModule::update_map_internal(
  NdtPtrType & secondary_ndt_ptr, const geometry_msgs::msg::Point & position,
  DiagnosticsReport & diagnostics)
{
  // When the current position is out of the loaded map range (or nothing has been loaded yet),
  // lock and rebuild ndt_ptr_ from scratch; otherwise update incrementally on secondary_ndt_ptr.
  const bool need_rebuild = out_of_map_range(position);
  diagnostics.add_key_value({"is_need_rebuild", need_rebuild});

  if (need_rebuild) {
    bool updated = false;
    ndt_ptr_.with([&](auto & ndt_ptr) {
      auto param = ndt_ptr->getParams();

      ndt_ptr.reset(new NdtType);
      // The map is rebuilt from scratch, so reset the accumulated debug map as well.
      loaded_pcd_map_.clear();

      ndt_ptr->setParams(param);

      updated = update_ndt(position, *ndt_ptr, diagnostics);
    });

    // check is_updated_map
    diagnostics.add_key_value({"is_updated_map", updated});
    if (!updated) {
      diagnostics.update_level_and_message(
        DiagnosticLevel::ERROR,
        "update_ndt failed. If this happens with initial position estimation, make sure that"
        "(1) the initial position matches the pcd map and (2) the map_loader is working "
        "properly.");
      // Do not advance last_update_position_: the rebuild failed, so the map is not loaded at
      // `position`. Keeping it unchanged leaves out_of_map_range() true so the next update retries.
      return false;
    }

    secondary_ndt_ptr.reset(new NdtType);
    ndt_ptr_.with([&](const auto & ndt_ptr) { *secondary_ndt_ptr = *ndt_ptr; });
  } else {
    // Load map to the secondary_ndt_ptr, which does not require a mutex lock
    // Since the update of the secondary ndt ptr and the NDT align (done on
    // the main ndt_ptr_) overlap, the latency of updating/alignment reduces partly.
    // If the updating is done the main ndt_ptr_, either the update or the NDT
    // align will be blocked by the other.
    const bool updated = update_ndt(position, *secondary_ndt_ptr, diagnostics);

    // check is_updated_map
    diagnostics.add_key_value({"is_updated_map", updated});
    if (!updated) {
      last_update_position_.with([&](auto & pos) { pos = position; });

      return false;
    }

    // Update the NDT map pointer with minimal lock duration to prevent latency spikes.
    // Heavy memory operations (cloning and destruction) are executed outside the ndt_ptr_'slock,
    // while only the fast pointer swap is performed inside the lock scope.

    // 1. Clone the contents of secondary_ndt_ptr to create new_ndt_ptr.
    auto new_ndt_ptr = std::make_shared<NdtType>(*secondary_ndt_ptr);

    // 2. Swap the pointers inside the ndt_ptr_'s lock.
    // - During the swap, the reference count does not decrease to zero,
    //   so the heavy destructor is not called here.
    // - This prevents the align process of NDTScanMatcher from being
    //   blocked for a long time.
    ndt_ptr_.with([&](auto & ndt_ptr) { std::swap(ndt_ptr, new_ndt_ptr); });

    // 3. Handle potential destruction outside the lock.
    // - new_ndt_ptr now holds the old NDT. Even if its heavy destructor
    //   is triggered when this block ends, it happens safely outside the lock.
    new_ndt_ptr.reset();
  }

  // Memorize the position of the last update
  last_update_position_.with([&](auto & pos) { pos = position; });

  return true;
}

MapUpdateModule::UpdateResult MapUpdateModule::update_map(
  const geometry_msgs::msg::PointStamped & position)
{
  UpdateResult result;
  result.map_updated = secondary_ndt_ptr_.with([&](auto & secondary_ndt_ptr) {
    const bool updated = update_map_internal(secondary_ndt_ptr, position.point, result.diagnostics);
    if (updated && param_.publish_loaded_map) {
      result.loaded_pcd_map = merge_loaded_pcd_map(position.header.stamp);
    }
    return updated;
  });
  return result;
}

bool MapUpdateModule::update_ndt(
  const geometry_msgs::msg::Point & position, NdtType & ndt, DiagnosticsReport & diagnostics)
{
  diagnostics.add_key_value(
    {"maps_size_before", static_cast<int64_t>(ndt.getCurrentMapIDs().size())});

  auto request = std::make_shared<GetDifferentialPointCloudMap::Request>();

  request->area.center_x = static_cast<float>(position.x);
  request->area.center_y = static_cast<float>(position.y);
  request->area.radius = static_cast<float>(param_.map_radius);
  request->cached_ids = ndt.getCurrentMapIDs();

  // Fetch the differential point cloud map. The ROS node performs the actual service call.
  const auto response = pcd_loader_(request);

  // check is_succeed_call_pcd_loader
  const bool is_succeed_call_pcd_loader = (response != nullptr);
  diagnostics.add_key_value({"is_succeed_call_pcd_loader", is_succeed_call_pcd_loader});
  if (!is_succeed_call_pcd_loader) {
    diagnostics.update_level_and_message(
      DiagnosticLevel::WARN, "pcd_loader service is not working.");
    return false;  // No update
  }

  auto & maps_to_add = response->new_pointcloud_with_ids;
  auto & map_ids_to_remove = response->ids_to_remove;

  diagnostics.add_key_value({"maps_to_add_size", static_cast<int64_t>(maps_to_add.size())});
  diagnostics.add_key_value(
    {"maps_to_remove_size", static_cast<int64_t>(map_ids_to_remove.size())});

  if (maps_to_add.empty() && map_ids_to_remove.empty()) {
    return false;  // No update
  }

  const auto exe_start_time = std::chrono::system_clock::now();
  // Perform heavy processing outside of the lock scope

  // Add pcd
  for (auto & map : maps_to_add) {
    auto cloud = pcl::make_shared<pcl::PointCloud<PointTarget>>();

    pcl::fromROSMsg(map.pointcloud, *cloud);
    ndt.addTarget(cloud, map.cell_id);

    // Keep the loaded cloud for the debug publish. Stored as the received PointCloud2 to avoid a
    // PCL round-trip, keyed by cell id so it can be erased when the cell is dropped.
    if (param_.publish_loaded_map) {
      loaded_pcd_map_[map.cell_id] = map.pointcloud;
    }
  }

  // Remove pcd
  for (const std::string & map_id_to_remove : map_ids_to_remove) {
    ndt.removeTarget(map_id_to_remove);
    if (param_.publish_loaded_map) {
      loaded_pcd_map_.erase(map_id_to_remove);
    }
  }

  ndt.createVoxelKdtree();

  const auto exe_end_time = std::chrono::system_clock::now();
  const auto duration_micro_sec =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
  const auto exe_time = static_cast<double>(duration_micro_sec) / 1000.0;
  diagnostics.add_key_value({"map_update_execution_time", exe_time});
  diagnostics.add_key_value(
    {"maps_size_after", static_cast<int64_t>(ndt.getCurrentMapIDs().size())});

  return true;  // Updated
}

sensor_msgs::msg::PointCloud2 MapUpdateModule::merge_loaded_pcd_map(
  const builtin_interfaces::msg::Time & stamp) const
{
  sensor_msgs::msg::PointCloud2 merged;

  // Pre-allocate the data buffer to the final size so the cell-by-cell concatenation below fills
  // it in place. Without this, each concatenatePointCloud reallocates and recopies the growing
  // buffer, degrading toward O(n^2) as the number of cells grows.
  // Related PR comment:
  //   - https://github.com/autowarefoundation/autoware_core/pull/1322#discussion_r3819841133
  std::size_t total_data_size = 0;
  for (const auto & [cell_id, pointcloud] : loaded_pcd_map_) {
    total_data_size += pointcloud.data.size();
  }
  merged.data.reserve(total_data_size);

  for (const auto & [cell_id, pointcloud] : loaded_pcd_map_) {
    pcl::concatenatePointCloud(merged, pointcloud, merged);
  }
  merged.header.frame_id = "map";
  merged.header.stamp = stamp;
  return merged;
}

}  // namespace autoware::ndt_scan_matcher
