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
  rclcpp::Node * node, const std::shared_ptr<NdtResource> & ndt_resource,
  HyperParameters::DynamicMapLoading param)
: ndt_resource_(ndt_resource),
  ndt_ptr_(ndt_resource_->ndt_ptr),
  ndt_ptr_mutex_(ndt_resource_->mutex),
  logger_(node->get_logger()),
  clock_(node->get_clock()),
  param_(param)
{
  loaded_pcd_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/loaded_pointcloud_map", rclcpp::QoS{1}.transient_local());

  pcd_loader_client_ =
    node->create_client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>("pcd_loader_service");

  secondary_ndt_ptr_.reset(new NdtType);

  if (ndt_ptr_) {
    *secondary_ndt_ptr_ = *ndt_ptr_;
  } else {
    std::stringstream message;
    message << "Error at MapUpdateModule::MapUpdateModule."
            << "`ndt_ptr_` is a null NDT pointer.";
    throw std::runtime_error(message.str());
  }

  // Initially, a direct map update on ndt_ptr_ is needed.
  // ndt_ptr_'s mutex is locked until it is fully rebuilt.
  // From the second update, the update is done on secondary_ndt_ptr_,
  // and ndt_ptr_ is only locked when swapping its pointer with
  // secondary_ndt_ptr_.
  need_rebuild_ = true;
}

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

  if (should_update_map(position.value(), diagnostics_ptr)) {
    update_map(position.value(), diagnostics_ptr);
  }
}

bool MapUpdateModule::should_update_map(
  const geometry_msgs::msg::Point & position,
  std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr)
{
  double last_x = 0.0;
  double last_y = 0.0;
  {
    // Guard last_update_position_ while reading it for distance computation.
    std::lock_guard<std::mutex> lock(last_update_position_mtx_);

    if (last_update_position_ == std::nullopt) {
      need_rebuild_ = true;
      return true;
    }

    last_x = last_update_position_.value().x;
    last_y = last_update_position_.value().y;
  }

  const double dx = position.x - last_x;
  const double dy = position.y - last_y;
  const double distance = std::hypot(dx, dy);

  // check distance_last_update_position_to_current_position
  diagnostics_ptr->add_key_value("distance_last_update_position_to_current_position", distance);
  if (distance + param_.lidar_radius > param_.map_radius) {
    std::stringstream message;
    message << "Dynamic map loading is not keeping up.";
    diagnostics_ptr->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());

    // If the map does not keep up with the current position,
    // lock ndt_ptr_ entirely until it is fully rebuilt.
    need_rebuild_ = true;
  }

  return distance > param_.update_distance;
}

bool MapUpdateModule::out_of_map_range(const geometry_msgs::msg::Point & position)
{
  double last_x = 0.0;
  double last_y = 0.0;
  {
    // Guard last_update_position_ while checking map range.
    std::lock_guard<std::mutex> lock(last_update_position_mtx_);

    if (last_update_position_ == std::nullopt) {
      return true;
    }

    last_x = last_update_position_.value().x;
    last_y = last_update_position_.value().y;
  }

  const double dx = position.x - last_x;
  const double dy = position.y - last_y;

  const double distance = std::hypot(dx, dy);

  // check distance_last_update_position_to_current_position
  return (distance + param_.lidar_radius > param_.map_radius);
}

void MapUpdateModule::update_map(
  const geometry_msgs::msg::Point & position,
  std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr)
{
  diagnostics_ptr->add_key_value("is_need_rebuild", need_rebuild_);

  // If the current position is super far from the previous loading position,
  // lock and rebuild ndt_ptr_
  if (need_rebuild_) {
    {
      // Exclusive rebuild of the primary NDT instance.
      std::unique_lock<std::mutex> lock(ndt_ptr_mutex_);

      auto param = ndt_ptr_->getParams();
      auto input_source = ndt_ptr_->getInputSource();

      ndt_ptr_.reset(new NdtType);

      ndt_ptr_->setParams(param);
      if (input_source != nullptr) {
        ndt_ptr_->setInputSource(input_source);
      }

      const bool updated = update_ndt(position, *ndt_ptr_, diagnostics_ptr);

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

        {
          std::lock_guard<std::mutex> pos_lock(last_update_position_mtx_);
          last_update_position_ = position;
        }

        return;
      }
    }

    need_rebuild_ = false;

  } else {
    // Load map to the secondary_ndt_ptr, which does not require a mutex lock
    // Since the update of the secondary ndt ptr and the NDT align (done on
    // the main ndt_ptr_) overlap, the latency of updating/alignment reduces partly.
    // If the updating is done the main ndt_ptr_, either the update or the NDT
    // align will be blocked by the other.
    const bool updated = update_ndt(position, *secondary_ndt_ptr_, diagnostics_ptr);

    // check is_updated_map
    diagnostics_ptr->add_key_value("is_updated_map", updated);
    if (!updated) {
      {
        // Record the attempt position while safely owning the position mutex.
        std::lock_guard<std::mutex> lock(last_update_position_mtx_);
        last_update_position_ = position;
      }

      return;
    }

    std::shared_ptr<NdtType> dummy_ptr;
    {
      // Swap primary/secondary NDT under lock to keep shared state consistent.
      std::lock_guard<std::mutex> lock(ndt_ptr_mutex_);
      dummy_ptr = ndt_ptr_;
      auto input_source = ndt_ptr_->getInputSource();
      ndt_ptr_ = secondary_ndt_ptr_;
      if (input_source != nullptr) {
        ndt_ptr_->setInputSource(input_source);
      }
    }

    // Release the old pointer outside the lock scope.
    dummy_ptr.reset();
  }

  {
    // Refresh the secondary copy while holding the shared NDT mutex.
    std::lock_guard<std::mutex> lock(ndt_ptr_mutex_);
    secondary_ndt_ptr_.reset(new NdtType(*ndt_ptr_));
  }

  // Memorize the position of the last update
  {
    std::lock_guard<std::mutex> lock(last_update_position_mtx_);
    last_update_position_ = position;
  }

  // Publish the new ndt maps
  publish_partial_pcd_map();
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
  auto future_result{pcd_loader_client_->async_send_request(
    request,
    [](rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedFuture) {})};

  std::future_status status = future_result.wait_for(std::chrono::seconds(0));
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
    status = future_result.wait_for(std::chrono::seconds(1));
  }
  diagnostics_ptr->add_key_value("is_succeed_call_pcd_loader", true);

  auto & maps_to_add = future_result.get()->new_pointcloud_with_ids;
  auto & map_ids_to_remove = future_result.get()->ids_to_remove;

  diagnostics_ptr->add_key_value("maps_to_add_size", maps_to_add.size());
  diagnostics_ptr->add_key_value("maps_to_remove_size", map_ids_to_remove.size());

  if (maps_to_add.empty() && map_ids_to_remove.empty()) {
    return false;  // No update
  }

  MapUpdateModule::MapUpdateDiff diff;

  // Add pcd
  for (auto & map : maps_to_add) {
    auto cloud = pcl::make_shared<pcl::PointCloud<PointTarget>>();
    pcl::fromROSMsg(map.pointcloud, *cloud);
    diff.additions.push_back({cloud, map.cell_id});
  }

  // Remove pcd
  for (const std::string & map_id_to_remove : map_ids_to_remove) {
    diff.removals.push_back(map_id_to_remove);
  }

  const auto map_update_result = apply_map_update(ndt, diff);

  if (!map_update_result.updated) {
    return false;  // No update
  }

  diagnostics_ptr->add_key_value("map_update_execution_time", map_update_result.execution_time_ms);
  diagnostics_ptr->add_key_value("maps_size_after", ndt.getCurrentMapIDs().size());
  diagnostics_ptr->add_key_value("is_succeed_call_pcd_loader", true);
  return true;  // Updated
}

void MapUpdateModule::publish_partial_pcd_map()
{
  pcl::PointCloud<PointTarget> map_pcl = ndt_ptr_->getVoxelPCD();
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(map_pcl, map_msg);
  map_msg.header.frame_id = "map";

  loaded_pcd_pub_->publish(map_msg);
}

}  // namespace autoware::ndt_scan_matcher
