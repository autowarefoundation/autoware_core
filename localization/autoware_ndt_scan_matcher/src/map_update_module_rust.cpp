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

#include "autoware_ndt_scan_matcher_rs.h"

#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{

struct MapUpdateModule::LegacyState
{
};

namespace
{
std::vector<std::string> current_map_ids(const AwNdtEngine * engine)
{
  std::uint32_t count = 0;
  std::uint32_t total = 0;
  autoware_ndt_scan_matcher_rs_ndt_engine_get_current_map_ids(
    engine, nullptr, 0, nullptr, 0, &count, &total);
  std::vector<std::uint32_t> lengths(count);
  std::vector<std::uint8_t> bytes(total);
  autoware_ndt_scan_matcher_rs_ndt_engine_get_current_map_ids(
    engine, lengths.data(), count, bytes.data(), total, &count, &total);
  std::vector<std::string> ids;
  ids.reserve(count);
  std::size_t off = 0;
  for (std::uint32_t i = 0; i < count; ++i) {
    ids.emplace_back(reinterpret_cast<const char *>(bytes.data()) + off, lengths[i]);
    off += lengths[i];
  }
  return ids;
}
}  // namespace

MapUpdateModule::MapUpdateModule(
  rclcpp::Node * node, EngineHolder * /*legacy_ndt_ptr*/, HyperParameters::DynamicMapLoading param,
  AwNdtScanMatcher * rs_handle)
: legacy_(std::make_unique<LegacyState>()), rs_handle_(rs_handle), logger_(node->get_logger()),
  clock_(node->get_clock()), param_(param)
{
  loaded_pcd_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/loaded_pointcloud_map", rclcpp::QoS{1}.transient_local());

  pcd_loader_client_ =
    node->create_client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>("pcd_loader_service");

  builder_state_.with([](auto & builder_state) { builder_state.need_rebuild = true; });
  if (autoware_ndt_scan_matcher_rs_engine(rs_handle_) == nullptr) {
    std::stringstream message;
    message << "Error at MapUpdateModule::MapUpdateModule."
            << "`rs_handle_` does not expose an NDT engine.";
    throw std::runtime_error(message.str());
  }
}

MapUpdateModule::~MapUpdateModule() = default;

bool MapUpdateModule::should_update_map(
  BuilderState & builder_state, const geometry_msgs::msg::Point & position,
  std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr)
{
  // The whole decision — last-update-position check, distance math, and the need_rebuild
  // policy — is Rust-owned on the handle. The C++ side keeps only the diagnostics (emitted from the
  // returned verdict, preserving the original keys/levels). `is_first_update` is the no-prior-update
  // case (force rebuild + update; no distance diagnostic, matching the old early return).
  (void)builder_state;
  AwMapUpdateDecision decision{};
  autoware_ndt_scan_matcher_rs_map_update_evaluate(
    rs_handle_, position.x, position.y, param_.lidar_radius, param_.map_radius,
    param_.update_distance, &decision);
  if (decision.is_first_update) {
    return true;
  }
  // check distance_last_update_position_to_current_position
  diagnostics_ptr->add_key_value(
    "distance_last_update_position_to_current_position", decision.distance);
  if (decision.out_of_keep_up) {
    std::stringstream message;
    message << "Dynamic map loading is not keeping up.";
    diagnostics_ptr->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
  }
  return decision.should_update;
}

bool MapUpdateModule::out_of_map_range(const geometry_msgs::msg::Point & position)
{
  // The keep-up check (incl. the "no update yet → out of range" case) is Rust-owned on the
  // handle's map-update state.
  return autoware_ndt_scan_matcher_rs_map_update_out_of_range(
    rs_handle_, position.x, position.y, param_.lidar_radius, param_.map_radius);
}

void MapUpdateModule::update_map_internal(
  BuilderState & builder_state, const geometry_msgs::msg::Point & position,
  std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr)
{
  // Rust engine: the portable `apply_map_update` (the `MapSource` Host port) owns the staging engine +
  // atomic `commit_from` double-buffer. We only supply the tiles: `map_source_fill` runs the pcd-loader
  // and pushes the add/remove delta into the Rust-owned builder, which Rust applies to a private
  // staging engine (a clone of the live map, or empty when `need_rebuild`) and publishes in one atomic
  // store — a concurrent align always sees a complete map (the engine's ArcSwap is the lock-free
  // buffer). An empty delta is a no-op on the Rust side (no republish). `need_rebuild` + the
  // last-update position are Rust-owned.
  (void)builder_state;
  const bool need_rebuild = autoware_ndt_scan_matcher_rs_map_update_need_rebuild(rs_handle_);
  diagnostics_ptr->add_key_value("is_need_rebuild", need_rebuild);
  if (need_rebuild) {
    loaded_map_.clear();  // staging starts empty in Rust (rebuild); keep the publish map in sync.
  }

  MapSourceContext source_ctx{this, need_rebuild, diagnostics_ptr.get(), false};
  const AwMapSource source{&source_ctx, &MapUpdateModule::map_source_fill};
  autoware_ndt_scan_matcher_rs_ndt_engine_update_map(
    autoware_ndt_scan_matcher_rs_engine(rs_handle_), &source, position.x, position.y,
    param_.map_radius, need_rebuild);
  const bool updated = source_ctx.updated;

  // check is_updated_map
  diagnostics_ptr->add_key_value("is_updated_map", updated);
  if (!updated) {
    if (need_rebuild) {
      std::stringstream message;
      message
        << "update_ndt failed. If this happens with initial position estimation, make sure that"
        << "(1) the initial position matches the pcd map and (2) the map_loader is working "
           "properly.";
      diagnostics_ptr->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
      RCLCPP_ERROR_STREAM_THROTTLE(logger_, *clock_, 1000, message.str());
    }
    // Failed update: advance the last position, leave need_rebuild latched.
    autoware_ndt_scan_matcher_rs_map_update_record(rs_handle_, position.x, position.y, false);
    return;
  }

  // Successful update: advance the last position + clear need_rebuild.
  autoware_ndt_scan_matcher_rs_map_update_record(rs_handle_, position.x, position.y, true);

  // Publish the new ndt maps
  if (param_.publish_loaded_map) {
    publish_partial_pcd_map();
  }
}

void MapUpdateModule::map_source_fill(
  void * ctx, double cx, double cy, double radius, void * builder)
{
  auto * c = static_cast<MapSourceContext *>(ctx);
  c->updated = c->self->build_map_delta(builder, cx, cy, radius, c->rebuild, *c->diagnostics);
}

bool MapUpdateModule::build_map_delta(
  void * builder, double cx, double cy, double radius, bool rebuild,
  DiagnosticsInterface & diagnostics)
{
  // The Rust staging engine starts empty on rebuild (cached_ids empty → the loader returns every tile)
  // and otherwise clones the live map (cached_ids = its current tiles → the loader returns the delta).
  const std::vector<std::string> cached_ids =
    rebuild ? std::vector<std::string>{}
            : current_map_ids(autoware_ndt_scan_matcher_rs_engine(rs_handle_));
  diagnostics.add_key_value("maps_size_before", cached_ids.size());

  auto request = std::make_shared<autoware_map_msgs::srv::GetDifferentialPointCloudMap::Request>();
  request->area.center_x = static_cast<float>(cx);
  request->area.center_y = static_cast<float>(cy);
  request->area.radius = static_cast<float>(radius);
  request->cached_ids = cached_ids;

  while (!pcd_loader_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
    diagnostics.add_key_value("is_succeed_call_pcd_loader", false);
    std::stringstream message;
    message << "Waiting for pcd loader service. Check the pointcloud_map_loader.";
    diagnostics.update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return false;
  }

  auto result{pcd_loader_client_->async_send_request(
    request,
    [](rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedFuture) {})};

  std::future_status status = result.wait_for(std::chrono::seconds(0));
  while (status != std::future_status::ready) {
    if (!rclcpp::ok()) {
      diagnostics.add_key_value("is_succeed_call_pcd_loader", false);
      std::stringstream message;
      message << "pcd_loader service is not working.";
      diagnostics.update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
      return false;
    }
    status = result.wait_for(std::chrono::seconds(1));
  }
  diagnostics.add_key_value("is_succeed_call_pcd_loader", true);

  auto & maps_to_add = result.get()->new_pointcloud_with_ids;
  auto & map_ids_to_remove = result.get()->ids_to_remove;
  diagnostics.add_key_value("maps_to_add_size", maps_to_add.size());
  diagnostics.add_key_value("maps_to_remove_size", map_ids_to_remove.size());

  if (maps_to_add.empty() && map_ids_to_remove.empty()) {
    return false;  // No update (Rust skips the commit for an empty delta).
  }

  const auto exe_start_time = std::chrono::system_clock::now();
  for (auto & map : maps_to_add) {
    auto cloud = pcl::make_shared<pcl::PointCloud<PointTarget>>();
    pcl::fromROSMsg(map.pointcloud, *cloud);
    std::vector<float> flat;
    flat.reserve(cloud->size() * 3);
    for (const auto & p : *cloud) {
      flat.push_back(p.x);
      flat.push_back(p.y);
      flat.push_back(p.z);
    }
    autoware_ndt_scan_matcher_rs_map_delta_add(
      builder, reinterpret_cast<const std::uint8_t *>(map.cell_id.data()), map.cell_id.size(),
      flat.data(), cloud->size());
    if (param_.publish_loaded_map) {
      loaded_map_[map.cell_id] = cloud;
    }
  }
  for (const std::string & map_id_to_remove : map_ids_to_remove) {
    autoware_ndt_scan_matcher_rs_map_delta_remove(
      builder, reinterpret_cast<const std::uint8_t *>(map_id_to_remove.data()),
      map_id_to_remove.size());
    if (param_.publish_loaded_map) {
      loaded_map_.erase(map_id_to_remove);
    }
  }

  const auto exe_end_time = std::chrono::system_clock::now();
  const auto duration_micro_sec =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
  const auto exe_time = static_cast<double>(duration_micro_sec) / 1000.0;
  diagnostics.add_key_value("map_update_execution_time", exe_time);
  diagnostics.add_key_value(
    "maps_size_after", cached_ids.size() + maps_to_add.size() - map_ids_to_remove.size());
  diagnostics.add_key_value("is_succeed_call_pcd_loader", true);
  return true;  // Updated
}

}  // namespace autoware::ndt_scan_matcher
