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
  auto copied = builder_state_.with([&](auto & builder_state) {
    // Initially, a direct map update on ndt_ptr_ is needed.
    // ndt_ptr_'s mutex is locked until it is fully rebuilt.
    // From the second update, the update is done on secondary_ndt_ptr_,
    // and ndt_ptr_ is only locked when swapping its pointer with
    // secondary_ndt_ptr_.
    builder_state.need_rebuild = true;
    builder_state.secondary_ndt_ptr.reset(new NdtType);

    return ndt_ptr_.with([&](const auto & ptr) {
      if (ptr) {
        *builder_state.secondary_ndt_ptr = *ptr;
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

bool MapUpdateModule::callback_timer(
  const bool is_activated, const std::optional<geometry_msgs::msg::Point> & position,
  const DiagnosticsHandlingFunction & diagnostics)
{
  // check is_activated
  diagnostics(DiagnosticsUpdate{{{"is_activated", is_activated}}, std::nullopt, ""});
  if (!is_activated) {
    diagnostics(DiagnosticsUpdate{{}, DiagnosticLevel::WARN, "Node is not activated."});
    return false;
  }

  // check is_set_last_update_position
  const bool is_set_last_update_position = (position != std::nullopt);
  diagnostics(
    DiagnosticsUpdate{
      {{"is_set_last_update_position", is_set_last_update_position}}, std::nullopt, ""});
  if (!is_set_last_update_position) {
    diagnostics(
      DiagnosticsUpdate{
        {},
        DiagnosticLevel::WARN,
        "Cannot find the reference position for map update."
        "Please check if the EKF odometry is provided to NDT."});
    return false;
  }

  return builder_state_.with([&](auto & builder_state) {
    if (should_update_map(builder_state, position.value(), diagnostics)) {
      return update_map_internal(builder_state, position.value(), diagnostics);
    }
    return false;
  });
}

bool MapUpdateModule::should_update_map(
  BuilderState & builder_state, const geometry_msgs::msg::Point & position,
  const DiagnosticsHandlingFunction & diagnostics)
{
  const auto last_update_position =
    last_update_position_.with([](const auto & pos) { return pos; });

  if (last_update_position == std::nullopt) {
    builder_state.need_rebuild = true;
    return true;
  }

  const double dx = position.x - last_update_position->x;
  const double dy = position.y - last_update_position->y;
  const double distance = std::hypot(dx, dy);

  // check distance_last_update_position_to_current_position
  diagnostics(
    DiagnosticsUpdate{
      {{"distance_last_update_position_to_current_position", distance}}, std::nullopt, ""});
  if (distance + param_.lidar_radius > param_.map_radius) {
    diagnostics(
      DiagnosticsUpdate{{}, DiagnosticLevel::ERROR, "Dynamic map loading is not keeping up."});

    // If the map does not keep up with the current position,
    // lock ndt_ptr_ entirely until it is fully rebuilt.
    builder_state.need_rebuild = true;
  }

  return distance > param_.update_distance;
}

bool MapUpdateModule::out_of_map_range(const geometry_msgs::msg::Point & position)
{
  const auto last_update_position =
    last_update_position_.with([](const auto & pos) { return pos; });

  if (last_update_position == std::nullopt) {
    return true;
  }

  const double dx = position.x - last_update_position->x;
  const double dy = position.y - last_update_position->y;
  const double distance = std::hypot(dx, dy);

  // check distance_last_update_position_to_current_position
  return (distance + param_.lidar_radius > param_.map_radius);
}

bool MapUpdateModule::update_map_internal(
  BuilderState & builder_state, const geometry_msgs::msg::Point & position,
  const DiagnosticsHandlingFunction & diagnostics)
{
  diagnostics(
    DiagnosticsUpdate{{{"is_need_rebuild", builder_state.need_rebuild}}, std::nullopt, ""});

  // If the current position is super far from the previous loading position,
  // lock and rebuild ndt_ptr_
  if (builder_state.need_rebuild) {
    bool updated = false;
    ndt_ptr_.with([&](auto & ndt_ptr) {
      auto param = ndt_ptr->getParams();

      ndt_ptr.reset(new NdtType);

      ndt_ptr->setParams(param);

      updated = update_ndt(position, *ndt_ptr, diagnostics);
    });

    // check is_updated_map
    diagnostics(DiagnosticsUpdate{{{"is_updated_map", updated}}, std::nullopt, ""});
    if (!updated) {
      diagnostics(
        DiagnosticsUpdate{
          {},
          DiagnosticLevel::ERROR,
          "update_ndt failed. If this happens with initial position estimation, make sure that"
          "(1) the initial position matches the pcd map and (2) the map_loader is working "
          "properly."});
      last_update_position_.with([&](auto & pos) { pos = position; });
      return false;
    }

    builder_state.need_rebuild = false;

    builder_state.secondary_ndt_ptr.reset(new NdtType);
    ndt_ptr_.with([&](const auto & ndt_ptr) { *builder_state.secondary_ndt_ptr = *ndt_ptr; });
  } else {
    // Load map to the secondary_ndt_ptr, which does not require a mutex lock
    // Since the update of the secondary ndt ptr and the NDT align (done on
    // the main ndt_ptr_) overlap, the latency of updating/alignment reduces partly.
    // If the updating is done the main ndt_ptr_, either the update or the NDT
    // align will be blocked by the other.
    const bool updated = update_ndt(position, *builder_state.secondary_ndt_ptr, diagnostics);

    // check is_updated_map
    diagnostics(DiagnosticsUpdate{{{"is_updated_map", updated}}, std::nullopt, ""});
    if (!updated) {
      last_update_position_.with([&](auto & pos) { pos = position; });

      return false;
    }

    // Update the NDT map pointer with minimal lock duration to prevent latency spikes.
    // Heavy memory operations (cloning and destruction) are executed outside the ndt_ptr_'slock,
    // while only the fast pointer swap is performed inside the lock scope.

    // 1. Clone the contents of secondary_ndt_ptr to create new_ndt_ptr.
    auto new_ndt_ptr = std::make_shared<NdtType>(*builder_state.secondary_ndt_ptr);

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

bool MapUpdateModule::update_map(
  const geometry_msgs::msg::Point & position, const DiagnosticsHandlingFunction & diagnostics)
{
  return builder_state_.with([&](auto & builder_state) {
    return update_map_internal(builder_state, position, diagnostics);
  });
}

bool MapUpdateModule::update_ndt(
  const geometry_msgs::msg::Point & position, NdtType & ndt,
  const DiagnosticsHandlingFunction & diagnostics)
{
  diagnostics(
    DiagnosticsUpdate{
      {{"maps_size_before", static_cast<int64_t>(ndt.getCurrentMapIDs().size())}},
      std::nullopt,
      ""});

  auto request = std::make_shared<GetDifferentialPointCloudMap::Request>();

  request->area.center_x = static_cast<float>(position.x);
  request->area.center_y = static_cast<float>(position.y);
  request->area.radius = static_cast<float>(param_.map_radius);
  request->cached_ids = ndt.getCurrentMapIDs();

  // Fetch the differential point cloud map. The ROS node performs the actual service call.
  const auto response = pcd_loader_(request);

  // check is_succeed_call_pcd_loader
  const bool is_succeed_call_pcd_loader = (response != nullptr);
  diagnostics(
    DiagnosticsUpdate{
      {{"is_succeed_call_pcd_loader", is_succeed_call_pcd_loader}}, std::nullopt, ""});
  if (!is_succeed_call_pcd_loader) {
    diagnostics(DiagnosticsUpdate{{}, DiagnosticLevel::WARN, "pcd_loader service is not working."});
    return false;  // No update
  }

  auto & maps_to_add = response->new_pointcloud_with_ids;
  auto & map_ids_to_remove = response->ids_to_remove;

  diagnostics(
    DiagnosticsUpdate{
      {{"maps_to_add_size", static_cast<int64_t>(maps_to_add.size())},
       {"maps_to_remove_size", static_cast<int64_t>(map_ids_to_remove.size())}},
      std::nullopt,
      ""});

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
  }

  // Remove pcd
  for (const std::string & map_id_to_remove : map_ids_to_remove) {
    ndt.removeTarget(map_id_to_remove);
  }

  ndt.createVoxelKdtree();

  const auto exe_end_time = std::chrono::system_clock::now();
  const auto duration_micro_sec =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
  const auto exe_time = static_cast<double>(duration_micro_sec) / 1000.0;
  diagnostics(
    DiagnosticsUpdate{
      {{"map_update_execution_time", exe_time},
       {"maps_size_after", static_cast<int64_t>(ndt.getCurrentMapIDs().size())}},
      std::nullopt,
      ""});

  return true;  // Updated
}

}  // namespace autoware::ndt_scan_matcher
