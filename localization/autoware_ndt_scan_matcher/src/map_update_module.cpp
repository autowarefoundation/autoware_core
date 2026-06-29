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

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ndt_scan_matcher
{

MapUpdateModule::MapUpdateModule(
  rclcpp::Node * node, EngineHolder & ndt_ptr, HyperParameters::DynamicMapLoading param)
: ndt_ptr_(ndt_ptr), logger_(node->get_logger()), clock_(node->get_clock()), param_(param)
{
  loaded_pcd_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/loaded_pointcloud_map", rclcpp::QoS{1}.transient_local());

  pcd_loader_client_ =
    node->create_client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>("pcd_loader_service");

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

#ifdef NDT_USE_RUST
  // Migrated to Rust (Phase N3): the distance math + threshold decisions. The C++ side keeps the
  // diagnostics and the need_rebuild mutation.
  const AwMapUpdateInput input{position.x,        position.y,         last_update_position->x,
                               last_update_position->y, param_.lidar_radius, param_.map_radius,
                               param_.update_distance};
  AwMapUpdateVerdict verdict{};
  autoware_ndt_scan_matcher_rs_node_evaluate_map_update(&input, &verdict);

  // check distance_last_update_position_to_current_position
  diagnostics_ptr->add_key_value(
    "distance_last_update_position_to_current_position", verdict.distance);
  if (verdict.out_of_keep_up) {
    std::stringstream message;
    message << "Dynamic map loading is not keeping up.";
    diagnostics_ptr->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());

    // If the map does not keep up with the current position,
    // lock ndt_ptr_ entirely until it is fully rebuilt.
    builder_state.need_rebuild = true;
  }

  return verdict.should_update;
#else
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
    // lock ndt_ptr_ entirely until it is fully rebuilt.
    builder_state.need_rebuild = true;
  }

  return distance > param_.update_distance;
#endif
}

bool MapUpdateModule::out_of_map_range(const geometry_msgs::msg::Point & position)
{
  const auto last_update_position =
    last_update_position_.with([](const auto & pos) { return pos; });

  if (last_update_position == std::nullopt) {
    return true;
  }

#ifdef NDT_USE_RUST
  // Migrated to Rust (Phase N3): the keep-up distance check (update_distance is unused here).
  const AwMapUpdateInput input{position.x,        position.y,         last_update_position->x,
                               last_update_position->y, param_.lidar_radius, param_.map_radius,
                               param_.update_distance};
  AwMapUpdateVerdict verdict{};
  autoware_ndt_scan_matcher_rs_node_evaluate_map_update(&input, &verdict);
  return verdict.out_of_keep_up;
#else
  const double dx = position.x - last_update_position->x;
  const double dy = position.y - last_update_position->y;
  const double distance = std::hypot(dx, dy);

  // check distance_last_update_position_to_current_position
  return (distance + param_.lidar_radius > param_.map_radius);
#endif
}

void MapUpdateModule::update_map_internal(
  BuilderState & builder_state, const geometry_msgs::msg::Point & position,
  std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr)
{
  diagnostics_ptr->add_key_value("is_need_rebuild", builder_state.need_rebuild);

#ifdef NDT_USE_RUST
  // Rust engine: the portable `apply_map_update` (the `MapSource` Host port) owns the staging engine +
  // atomic `commit_from` double-buffer. We only supply the tiles: `map_source_fill` runs the pcd-loader
  // and pushes the add/remove delta into the Rust-owned builder, which Rust applies to a private
  // staging engine (a clone of the live map, or empty when `need_rebuild`) and publishes in one atomic
  // store — a concurrent align always sees a complete map (the engine's ArcSwap is the lock-free
  // buffer). An empty delta is a no-op on the Rust side (no republish).
  if (builder_state.need_rebuild) {
    loaded_map_.clear();  // staging starts empty in Rust (rebuild); keep the publish map in sync.
  }

  MapSourceContext source_ctx{this, builder_state.need_rebuild, diagnostics_ptr.get(), false};
  const AwMapSource source{&source_ctx, &MapUpdateModule::map_source_fill};
  ndt_ptr_.with([&](const auto & ndt_ptr) {
    autoware_ndt_scan_matcher_rs_ndt_engine_update_map(
      ndt_ptr->raw_handle(), &source, position.x, position.y, param_.map_radius,
      builder_state.need_rebuild);
  });
  const bool updated = source_ctx.updated;

  // check is_updated_map
  diagnostics_ptr->add_key_value("is_updated_map", updated);
  if (!updated) {
    if (builder_state.need_rebuild) {
      std::stringstream message;
      message
        << "update_ndt failed. If this happens with initial position estimation, make sure that"
        << "(1) the initial position matches the pcd map and (2) the map_loader is working "
           "properly.";
      diagnostics_ptr->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
      RCLCPP_ERROR_STREAM_THROTTLE(logger_, *clock_, 1000, message.str());
    }
    last_update_position_.with([&](auto & pos) { pos = position; });
    return;
  }

  builder_state.need_rebuild = false;
#else
  // If the current position is super far from the previous loading position,
  // lock and rebuild ndt_ptr_
  if (builder_state.need_rebuild) {
    bool updated = false;
    ndt_ptr_.with([&](auto & ndt_ptr) {
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

    builder_state.secondary_ndt_ptr.reset(new NdtType);
    ndt_ptr_.with([&](const auto & ndt_ptr) { *builder_state.secondary_ndt_ptr = *ndt_ptr; });
  } else {
    // Load map to the secondary_ndt_ptr, which does not require a mutex lock
    // Since the update of the secondary ndt ptr and the NDT align (done on
    // the main ndt_ptr_) overlap, the latency of updating/alignment reduces partly.
    // If the updating is done the main ndt_ptr_, either the update or the NDT
    // align will be blocked by the other.
    const bool updated = update_ndt(position, *builder_state.secondary_ndt_ptr, diagnostics_ptr);

    // check is_updated_map
    diagnostics_ptr->add_key_value("is_updated_map", updated);
    if (!updated) {
      last_update_position_.with([&](auto & pos) { pos = position; });

      return;
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
#endif

  // Memorize the position of the last update
  last_update_position_.with([&](auto & pos) { pos = position; });

  // Publish the new ndt maps
  if (param_.publish_loaded_map) {
    publish_partial_pcd_map();
  }
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

#ifdef NDT_USE_RUST
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
            : ndt_ptr_.with([](const auto & ndt_ptr) { return ndt_ptr->getCurrentMapIDs(); });
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
#endif

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
