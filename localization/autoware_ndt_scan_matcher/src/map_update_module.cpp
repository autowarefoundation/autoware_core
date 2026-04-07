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

#include <memory>
#include <string>

namespace autoware::ndt_scan_matcher
{

MapUpdateModule::MapUpdateModule(
  rclcpp::Node * node, const NdtPtrType & initial_ndt,
  HyperParameters::DynamicMapLoading param)
: logger_(node->get_logger()),
  clock_(node->get_clock()),
  param_(param)
{
  loaded_pcd_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/loaded_pointcloud_map", rclcpp::QoS{1}.transient_local());

  pcd_loader_client_ =
    node->create_client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>("pcd_loader_service");

  if (!initial_ndt) {
    std::stringstream message;
    message << "Error at MapUpdateModule::MapUpdateModule."
            << "`initial_ndt` is a null NDT pointer.";
    throw std::runtime_error(message.str());
  }

  ndt_params_ = initial_ndt->getParams();

  builder_state_.with([&](auto & state) {
    state.secondary_ndt_ptr = std::make_shared<NdtType>(*initial_ndt);
  });
}

bool MapUpdateModule::should_update_map(
  const geometry_msgs::msg::Point & position,
  std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr,
  BuilderState & state)
{
  // Caller holds builder_state_ lock.

  const auto last_pos = last_update_position_.with([](const auto & pos) { return pos; });

  if (last_pos == std::nullopt) {
    state.need_rebuild = true;
    return true;
  }

  const double dx = position.x - last_pos.value().x;
  const double dy = position.y - last_pos.value().y;
  const double distance = std::hypot(dx, dy);

  // check distance_last_update_position_to_current_position
  diagnostics_ptr->add_key_value("distance_last_update_position_to_current_position", distance);
  if (distance + param_.lidar_radius > param_.map_radius) {
    std::stringstream message;
    message << "Dynamic map loading is not keeping up.";
    diagnostics_ptr->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());

    // If the map does not keep up with the current position,
    // the NDT must be fully rebuilt.
    state.need_rebuild = true;
  }

  return distance > param_.update_distance;
}

bool MapUpdateModule::out_of_map_range(const geometry_msgs::msg::Point & position)
{
  // Only locks last_update_position_ (not mtx_), so this never blocks on PCD loading.
  const auto last_pos = last_update_position_.with([](const auto & pos) { return pos; });

  if (last_pos == std::nullopt) {
    return true;
  }

  const double dx = position.x - last_pos.value().x;
  const double dy = position.y - last_pos.value().y;
  const double distance = std::hypot(dx, dy);

  return (distance + param_.lidar_radius > param_.map_radius);
}

MapUpdateModule::NdtPtrType MapUpdateModule::build_if_needed(
  const geometry_msgs::msg::Point & position,
  std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr)
{
  return builder_state_.with([&](auto & state) -> NdtPtrType {
  if (!should_update_map(position, diagnostics_ptr, state)) {
    return nullptr;
  }

  diagnostics_ptr->add_key_value("is_need_rebuild", state.need_rebuild);

  if (state.need_rebuild) {
    // Full rebuild: create a new NDT from scratch.
    // ndt_params_ is immutable after construction — no race.
    // input_source is NOT set here; install_new_ndt() transfers it from old to new.
    auto new_ndt = std::make_shared<NdtType>();
    new_ndt->setParams(ndt_params_);

    const bool updated = update_ndt(position, *new_ndt, diagnostics_ptr);
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
      return nullptr;
    }

    state.need_rebuild = false;
    last_update_position_.with([&](auto & pos) { pos = position; });
    state.secondary_ndt_ptr = std::make_shared<NdtType>(*new_ndt);
    return new_ndt;

  } else {
    // Swap path: build on the secondary (background), then return it for installation.
    const bool updated = update_ndt(position, *state.secondary_ndt_ptr, diagnostics_ptr);
    diagnostics_ptr->add_key_value("is_updated_map", updated);
    if (!updated) {
      last_update_position_.with([&](auto & pos) { pos = position; });
      return nullptr;
    }

    auto result = state.secondary_ndt_ptr;
    last_update_position_.with([&](auto & pos) { pos = position; });
    state.secondary_ndt_ptr = std::make_shared<NdtType>(*result);
    return result;
  }

  });  // builder_state_.with
}

bool MapUpdateModule::update_ndt(
  const geometry_msgs::msg::Point & position, NdtType & ndt,
  std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr)
{
  diagnostics_ptr->add_key_value("maps_size_before", ndt.getCurrentMapIDs().size());

  auto request = std::make_shared<autoware_map_msgs::srv::GetDifferentialPointCloudMap::Request>();

  request->area.center_x = static_cast<float>(position.x);
  request->area.center_y = static_cast<float>(position.y);
  request->area.radius = static_cast<float>(param_.map_radius);
  request->cached_ids = ndt.getCurrentMapIDs();

  while (!pcd_loader_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
    diagnostics_ptr->add_key_value("is_succeed_call_pcd_loader", false);

    std::stringstream message;
    message << "Waiting for pcd loader service. Check the pointcloud_map_loader.";
    diagnostics_ptr->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return false;
  }

  // send a request to map_loader
  auto result{pcd_loader_client_->async_send_request(
    request,
    [](rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedFuture) {})};

  std::future_status status = result.wait_for(std::chrono::seconds(0));
  while (status != std::future_status::ready) {
    // check is_succeed_call_pcd_loader
    if (!rclcpp::ok()) {
      diagnostics_ptr->add_key_value("is_succeed_call_pcd_loader", false);

      std::stringstream message;
      message << "pcd_loader service is not working.";
      diagnostics_ptr->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
      return false;  // No update
    }
    status = result.wait_for(std::chrono::seconds(1));
  }
  diagnostics_ptr->add_key_value("is_succeed_call_pcd_loader", true);

  auto & maps_to_add = result.get()->new_pointcloud_with_ids;
  auto & map_ids_to_remove = result.get()->ids_to_remove;

  diagnostics_ptr->add_key_value("maps_to_add_size", maps_to_add.size());
  diagnostics_ptr->add_key_value("maps_to_remove_size", map_ids_to_remove.size());

  if (maps_to_add.empty() && map_ids_to_remove.empty()) {
    return false;  // No update
  }

  const auto exe_start_time = std::chrono::system_clock::now();

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
  diagnostics_ptr->add_key_value("map_update_execution_time", exe_time);
  diagnostics_ptr->add_key_value("maps_size_after", ndt.getCurrentMapIDs().size());
  diagnostics_ptr->add_key_value("is_succeed_call_pcd_loader", true);
  return true;  // Updated
}

void MapUpdateModule::publish_partial_pcd_map(const NdtType & ndt)
{
  pcl::PointCloud<PointTarget> map_pcl = ndt.getVoxelPCD();
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(map_pcl, map_msg);
  map_msg.header.frame_id = "map";

  loaded_pcd_pub_->publish(map_msg);
}

}  // namespace autoware::ndt_scan_matcher
