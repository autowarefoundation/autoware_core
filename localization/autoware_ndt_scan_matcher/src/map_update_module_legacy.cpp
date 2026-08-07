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

#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace autoware::ndt_scan_matcher
{

struct MapUpdateModule::LegacyState
{
  explicit LegacyState(EngineHolder & ndt) : ndt_ptr(ndt) {}

  EngineHolder & ndt_ptr;
  NdtPtrType secondary_ndt_ptr;
};

MapUpdateModule::MapUpdateModule(
  rclcpp::Node * node, EngineHolder * legacy_ndt_ptr, HyperParameters::DynamicMapLoading param,
  AwNdtScanMatcher * rs_handle)
: rs_handle_(rs_handle), logger_(node->get_logger()), clock_(node->get_clock()), param_(param)
{
  if (legacy_ndt_ptr == nullptr) {
    std::stringstream message;
    message << "Error at MapUpdateModule::MapUpdateModule."
            << "`legacy_ndt_ptr` is null in the legacy build.";
    throw std::runtime_error(message.str());
  }
  legacy_ = std::make_unique<LegacyState>(*legacy_ndt_ptr);

  loaded_pcd_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/loaded_pointcloud_map", rclcpp::QoS{1}.transient_local());

  pcd_loader_client_ =
    node->create_client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>("pcd_loader_service");

  auto copied = builder_state_.with([&](auto & builder_state) {
    // Initially, a direct map update on the legacy engine is needed.
    // Its mutex is locked until the first map is fully rebuilt. From the second update, the update
    // is done on the secondary engine, and the main engine is only locked when swapping pointers.
    builder_state.need_rebuild = true;
    legacy_->secondary_ndt_ptr.reset(new NdtType);

    return legacy_->ndt_ptr.with([&](const auto & ptr) {
      if (ptr) {
        *legacy_->secondary_ndt_ptr = *ptr;
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

MapUpdateModule::~MapUpdateModule() = default;

bool MapUpdateModule::should_update_map(
  BuilderState & builder_state, const geometry_msgs::msg::Point & position,
  std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr)
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
  diagnostics_ptr->add_key_value("distance_last_update_position_to_current_position", distance);
  if (distance + param_.lidar_radius > param_.map_radius) {
    std::stringstream message;
    message << "Dynamic map loading is not keeping up.";
    diagnostics_ptr->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());

    // If the map does not keep up with the current position,
    // lock the legacy engine entirely until it is fully rebuilt.
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

void MapUpdateModule::update_map_internal(
  BuilderState & builder_state, const geometry_msgs::msg::Point & position,
  std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr)
{
  diagnostics_ptr->add_key_value("is_need_rebuild", builder_state.need_rebuild);

  // If the current position is super far from the previous loading position,
  // lock and rebuild the legacy engine
  if (builder_state.need_rebuild) {
    bool updated = false;
    legacy_->ndt_ptr.with([&](auto & ndt_ptr) {
      auto param = ndt_ptr->getParams();

      ndt_ptr.reset(new NdtType);
      loaded_map_.clear();

      ndt_ptr->setParams(param);

      updated = update_ndt(position, *ndt_ptr, diagnostics_ptr);
    });

    // check is_updated_map
    diagnostics_ptr->add_key_value("is_updated_map", updated);
    if (!updated) {
      std::stringstream message;
      message
        << "update_ndt failed. If this happens with initial position estimation, make sure that"
        << "(1) the initial position matches the pcd map and (2) the map_loader is working "
           "properly.";
      diagnostics_ptr->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
      RCLCPP_ERROR_STREAM_THROTTLE(logger_, *clock_, 1000, message.str());

      last_update_position_.with([&](auto & pos) { pos = position; });
      return;
    }

    builder_state.need_rebuild = false;

    legacy_->secondary_ndt_ptr.reset(new NdtType);
    legacy_->ndt_ptr.with([&](const auto & ndt_ptr) { *legacy_->secondary_ndt_ptr = *ndt_ptr; });
  } else {
    // Load map to the secondary engine, which does not require the main engine mutex.
    // Since the secondary update and the NDT align on the main engine can overlap, update/alignment
    // latency is reduced. If the update runs on the main engine, one operation blocks the other.
    const bool updated = update_ndt(position, *legacy_->secondary_ndt_ptr, diagnostics_ptr);

    // check is_updated_map
    diagnostics_ptr->add_key_value("is_updated_map", updated);
    if (!updated) {
      last_update_position_.with([&](auto & pos) { pos = position; });

      return;
    }

    // Update the NDT map pointer with minimal lock duration to prevent latency spikes.
    // Heavy memory operations (cloning and destruction) are executed outside the legacy engine
    // lock, while only the fast pointer swap is performed inside the lock scope.

    // 1. Clone the contents of the secondary engine to create new_ndt_ptr.
    auto new_ndt_ptr = std::make_shared<NdtType>(*legacy_->secondary_ndt_ptr);

    // 2. Swap the pointers inside the legacy engine lock.
    // - During the swap, the reference count does not decrease to zero,
    //   so the heavy destructor is not called here.
    // - This prevents the align process of NDTScanMatcher from being
    //   blocked for a long time.
    legacy_->ndt_ptr.with([&](auto & ndt_ptr) { std::swap(ndt_ptr, new_ndt_ptr); });

    // 3. Handle potential destruction outside the lock.
    // - new_ndt_ptr now holds the old NDT. Even if its heavy destructor
    //   is triggered when this block ends, it happens safely outside the lock.
    new_ndt_ptr.reset();
  }

  // Memorize the position of the last update.
  last_update_position_.with([&](auto & pos) { pos = position; });

  // Publish the new ndt maps
  if (param_.publish_loaded_map) {
    publish_partial_pcd_map();
  }
}

}  // namespace autoware::ndt_scan_matcher
