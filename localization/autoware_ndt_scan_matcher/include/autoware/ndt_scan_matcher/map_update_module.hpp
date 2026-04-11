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

namespace autoware::ndt_scan_matcher
{
using DiagnosticsInterface = autoware_utils_diagnostics::DiagnosticsInterface;

class MapUpdateModule
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NdtType = pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;
  using NdtPtrType = std::shared_ptr<NdtType>;

public:
  MapUpdateModule(
    rclcpp::Node * node, const NdtPtrType & initial_ndt, HyperParameters::DynamicMapLoading param);

  // Build a new NDT map if the vehicle has moved far enough from the last update position.
  // Returns a new NDT to install, or nullptr if no update was performed.
  // Thread-safe: can be called from any callback group.
  NdtPtrType build_if_needed(
    const geometry_msgs::msg::Point & position,
    std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr);

  // Check if the given position is outside the loaded map range.
  // Thread-safe.
  bool out_of_map_range(const geometry_msgs::msg::Point & position);

  // Publish the voxelized PCD of the given NDT for debugging visualization.
  void publish_partial_pcd_map(const NdtType & ndt);

private:
  // Heavy map-building state. build_if_needed() holds this for the duration of PCD loading.
  struct BuilderState
  {
    NdtPtrType secondary_ndt_ptr;
    bool need_rebuild = true;
  };

  [[nodiscard]] bool should_update_map(
    const geometry_msgs::msg::Point & position,
    std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr, BuilderState & state);

  // Load/remove PCD tiles into the given NDT object.
  bool update_ndt(
    const geometry_msgs::msg::Point & position, NdtType & ndt,
    std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loaded_pcd_pub_;

  rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedPtr
    pcd_loader_client_;

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  HyperParameters::DynamicMapLoading param_;
  pclomp::NdtParams ndt_params_;  // Immutable after construction; used to init new NDT objects.

  Guarded<BuilderState> builder_state_;

  // Separate from mtx_ so that out_of_map_range() (called from sensor callback
  // while holding ndt_.lock()) does not block on PCD loading.
  Guarded<std::optional<geometry_msgs::msg::Point>> last_update_position_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_
