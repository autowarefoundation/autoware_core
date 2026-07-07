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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_

#include "guarded.hpp"
#include "hyper_parameters.hpp"
#include "ndt_backend.hpp"
#include "ndt_omp/multigrid_ndt_omp.h"
#include "particle.hpp"

#include <autoware/localization_util/util_func.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <autoware_utils_pcl/transforms.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fmt/format.h>
#include <pcl_conversions/pcl_conversions.h>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

struct AwNdtScanMatcher;

namespace autoware::ndt_scan_matcher
{
using DiagnosticsInterface = autoware_utils_diagnostics::DiagnosticsInterface;

class MapUpdateModule
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  // The engine type is selected in one place (ndt_backend.hpp); see plan/ndt_in_rust.md (案B).
  using NdtType = NdtBackend;
  using NdtPtrType = std::shared_ptr<NdtType>;

  struct BuilderState
  {
    bool need_rebuild{true};
#ifndef NDT_USE_RUST
    NdtPtrType secondary_ndt_ptr;
#endif
  };

public:
#ifdef NDT_USE_RUST
  MapUpdateModule(
    rclcpp::Node * node, HyperParameters::DynamicMapLoading param, AwNdtScanMatcher * rs_handle);
#else
  MapUpdateModule(
    rclcpp::Node * node, EngineHolder & ndt_ptr, HyperParameters::DynamicMapLoading param,
    AwNdtScanMatcher * rs_handle = nullptr);
#endif

  bool out_of_map_range(const geometry_msgs::msg::Point & position);

private:
  friend class NDTScanMatcher;

  void callback_timer(
    const bool is_activated, const std::optional<geometry_msgs::msg::Point> & position,
    std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr);

  [[nodiscard]] bool should_update_map(
    BuilderState & builder_state, const geometry_msgs::msg::Point & position,
    std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr);

  void update_map_internal(
    BuilderState & builder_state, const geometry_msgs::msg::Point & position,
    std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr);

  // Do not call this function while holding the lock for ndt_ptr_.
  void update_map(
    const geometry_msgs::msg::Point & position,
    std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr);
  // Update the specified NDT
  bool update_ndt(
    const geometry_msgs::msg::Point & position, NdtType & ndt,
    std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr);

  // Map-update via the portable Rust `apply_map_update` (the `MapSource` Host port). The Rust FFI owns
  // the staging + atomic commit double-buffer; we only supply the tiles. `build_map_delta` runs the
  // pcd-loader fetch and pushes the add/remove delta into the Rust-owned `builder` (returns whether
  // anything changed); `map_source_fill` is the C-ABI trampoline the FFI calls back (ctx ==
  // MapSourceContext*), mirroring the make_host/make_diagnostics vtable pattern.
  struct MapSourceContext
  {
    MapUpdateModule * self;
    bool rebuild;
    DiagnosticsInterface * diagnostics;
    bool updated;
  };
  static void map_source_fill(void * ctx, double cx, double cy, double radius, void * builder);
  bool build_map_delta(
    void * builder, double cx, double cy, double radius, bool rebuild,
    DiagnosticsInterface & diagnostics);

  void publish_partial_pcd_map();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loaded_pcd_pub_;

  rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedPtr
    pcd_loader_client_;

  // Lock ordering (OFF, where these are real mutexes): builder_state_ -> ndt_ptr_ and
  // builder_state_ -> last_update_position_. Under NDT_USE_RUST the live engine is owned by the Rust
  // node handle, so only builder_state_/last_update_position_ are real locks here.
#ifndef NDT_USE_RUST
  EngineHolder & ndt_ptr_;
#endif
  Guarded<BuilderState> builder_state_;
  // Phase 6: the map-update decision state (last-update position + need-rebuild) lives Rust-side on
  // the node handle; this module reads/updates it through the `..._map_update_*` FFIs.
  AwNdtScanMatcher * rs_handle_;
  Guarded<std::optional<geometry_msgs::msg::Point>> last_update_position_{std::nullopt};

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  HyperParameters::DynamicMapLoading param_;

  // All accesses must occur while builder_state_'s lock is held
  std::map<std::string, pcl::PointCloud<PointTarget>::Ptr> loaded_map_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_
