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

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace autoware::ndt_scan_matcher
{
using DiagnosticsInterface = autoware_utils_diagnostics::DiagnosticsInterface;

template <typename CloudPtrT>
struct MapUpdateDiffTemplate
{
  struct Addition
  {
    CloudPtrT cloud;
    std::string id;
  };

  std::vector<Addition> additions;
  std::vector<std::string> removals;
};

struct MapUpdateResult
{
  std::size_t added{0};
  std::size_t removed{0};
  bool updated{false};
  double execution_time_ms{0.0};
};

// Shared NDT resource so callers and the map updater coordinate on a single mutex/instance.
template <typename NdtT>
struct SharedNdtResource
{
  std::shared_ptr<NdtT> ndt_ptr;
  std::mutex mutex;
};

class MapUpdateModule
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NdtType = pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;
  using NdtPtrType = std::shared_ptr<NdtType>;
  using TargetCloudPtr = typename pcl::PointCloud<PointTarget>::Ptr;
  using NdtResource = SharedNdtResource<NdtType>;

public:
  using MapUpdateDiff = MapUpdateDiffTemplate<TargetCloudPtr>;
  template <typename NdtT, typename DiffT>
  static MapUpdateResult apply_map_update(NdtT & ndt, const DiffT & diff)
  {
    const auto start = std::chrono::steady_clock::now();

    std::size_t added = 0;
    for (const auto & addition : diff.additions) {
      ndt.addTarget(addition.cloud, addition.id);
      ++added;
    }

    std::size_t removed = 0;
    for (const auto & id : diff.removals) {
      ndt.removeTarget(id);
      ++removed;
    }

    const bool updated = (added + removed) > 0;
    if (updated) {
      ndt.createVoxelKdtree();
    }

    const auto end = std::chrono::steady_clock::now();
    const double execution_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start).count();

    return MapUpdateResult{added, removed, updated, execution_ms};
  }

  MapUpdateModule(
    rclcpp::Node * node, const std::shared_ptr<NdtResource> & ndt_resource,
    HyperParameters::DynamicMapLoading param);

  bool out_of_map_range(const geometry_msgs::msg::Point & position);

private:
  friend class NDTScanMatcher;

  void callback_timer(
    const bool is_activated, const std::optional<geometry_msgs::msg::Point> & position,
    std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr);

  [[nodiscard]] bool should_update_map(
    const geometry_msgs::msg::Point & position,
    std::unique_ptr<DiagnosticsInterface> & diagnostics_ptr);

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

  std::shared_ptr<NdtResource> ndt_resource_;
  NdtPtrType & ndt_ptr_;
  std::mutex & ndt_ptr_mutex_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  std::optional<geometry_msgs::msg::Point> last_update_position_ = std::nullopt;

  HyperParameters::DynamicMapLoading param_;

  // Indicate if there is a prefetch thread waiting for being collected
  NdtPtrType secondary_ndt_ptr_;
  bool need_rebuild_;
  // Keep the last_update_position_ unchanged while checking map range
  std::mutex last_update_position_mtx_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_
