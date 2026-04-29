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

  struct BuilderState
  {
    bool need_rebuild{true};
    NdtPtrType secondary_ndt_ptr;
  };

public:
  MapUpdateModule(
    rclcpp::Node * node, Guarded<NdtPtrType> & ndt_ptr, HyperParameters::DynamicMapLoading param);

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
  void publish_partial_pcd_map();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loaded_pcd_pub_;

  rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedPtr
    pcd_loader_client_;

  // To prevent deadlocks, acquire locks in the following order:
  // 1. builder_state_ -> ndt_ptr_
  // 2. builder_state_ -> last_update_position_
  Guarded<NdtPtrType> & ndt_ptr_;
  Guarded<BuilderState> builder_state_;
  Guarded<std::optional<geometry_msgs::msg::Point>> last_update_position_{std::nullopt};

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  HyperParameters::DynamicMapLoading param_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_
