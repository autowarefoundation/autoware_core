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

#include <chrono>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ndt_scan_matcher
{

void MapUpdateModule::callback_timer(
  const bool is_activated, const std::optional<geometry_msgs::msg::Point> & position,
  std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr)
{
  // check is_activated
  diagnostics_ptr->add_key_value("is_activated", is_activated);
  if (!is_activated) {
    std::stringstream message;
    message << "Node is not activated.";
    diagnostics_ptr->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return;
  }

  // check is_set_last_update_position
  const bool is_set_last_update_position = (position != std::nullopt);
  diagnostics_ptr->add_key_value("is_set_last_update_position", is_set_last_update_position);
  if (!is_set_last_update_position) {
    std::stringstream message;
    message << "Cannot find the reference position for map update."
            << "Please check if the EKF odometry is provided to NDT.";
    diagnostics_ptr->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return;
  }

  builder_state_.with([&](auto & builder_state) {
    if (should_update_map(builder_state, position.value(), diagnostics_ptr)) {
      update_map_internal(builder_state, position.value(), diagnostics_ptr);
    }
  });
}

void MapUpdateModule::update_map(
  const geometry_msgs::msg::Point & position,
  std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr)
{
  builder_state_.with(
    [&](auto & builder_state) { update_map_internal(builder_state, position, diagnostics_ptr); });
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
  // Perform heavy processing outside of the lock scope

  // Add pcd
  for (auto & map : maps_to_add) {
    auto cloud = pcl::make_shared<pcl::PointCloud<PointTarget>>();

    pcl::fromROSMsg(map.pointcloud, *cloud);
    ndt.addTarget(cloud, map.cell_id);
    if (param_.publish_loaded_map) {
      loaded_map_[map.cell_id] = cloud;
    }
  }

  // Remove pcd
  for (const std::string & map_id_to_remove : map_ids_to_remove) {
    ndt.removeTarget(map_id_to_remove);
    if (param_.publish_loaded_map) {
      loaded_map_.erase(map_id_to_remove);
    }
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

void MapUpdateModule::publish_partial_pcd_map()
{
  pcl::PointCloud<PointTarget> map_pcl;
  sensor_msgs::msg::PointCloud2 map_msg;
  size_t total_points = 0;
  for (const auto & map : loaded_map_) {
    total_points += map.second->size();
  }
  map_pcl.points.reserve(total_points);
  for (const auto & map : loaded_map_) {
    map_pcl += *(map.second);
  }
  pcl::toROSMsg(map_pcl, map_msg);
  map_msg.header.frame_id = "map";
  map_msg.header.stamp = clock_->now();
  loaded_pcd_pub_->publish(map_msg);
}

}  // namespace autoware::ndt_scan_matcher
