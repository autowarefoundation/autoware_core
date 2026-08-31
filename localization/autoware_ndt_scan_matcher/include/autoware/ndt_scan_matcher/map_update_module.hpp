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

#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace autoware::ndt_scan_matcher
{

// Declared a friend of MapUpdateModule so unit tests can drive its otherwise-private map-update
// entry points (see test/test_map_update_module.cpp).
class MapUpdateModuleTest;

class MapUpdateModule
{
public:
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NdtType = pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;
  using NdtPtrType = std::shared_ptr<NdtType>;

  using GetDifferentialPointCloudMap = autoware_map_msgs::srv::GetDifferentialPointCloudMap;

  // Injected by the ROS node: given a differential map request, returns the response,
  // or nullptr if the map could not be fetched (e.g. the service is unavailable).
  using PcdLoaderFunction = std::function<GetDifferentialPointCloudMap::Response::SharedPtr(
    const GetDifferentialPointCloudMap::Request::SharedPtr &)>;

  // Severity of a diagnostics update. Mirrors diagnostic_msgs::msg::DiagnosticStatus levels so
  // this module needs no ROS diagnostics dependency.
  enum class DiagnosticLevel : int8_t { OK = 0, WARN = 1, ERROR = 2, STALE = 3 };

  // A single diagnostic key/value. The value keeps its type so the ROS node can format it exactly
  // the way DiagnosticsInterface would (e.g. bool as "True"/"False").
  struct DiagnosticKeyValue
  {
    std::string key;
    std::variant<bool, int64_t, double, std::string> value;
  };

  // Diagnostics accumulated while updating the map: key/values plus an overall status (level +
  // message). Returned to the ROS node, which forwards it to a DiagnosticsInterface. Plain data,
  // kept ROS-free on purpose.
  struct DiagnosticsReport
  {
    DiagnosticLevel level{DiagnosticLevel::OK};
    std::string message;
    std::vector<DiagnosticKeyValue> key_values;

    void add_key_value(DiagnosticKeyValue key_value) { key_values.push_back(std::move(key_value)); }

    // Accumulates like DiagnosticsInterface: raises the level and appends the message.
    void update_level_and_message(DiagnosticLevel new_level, const std::string & new_message)
    {
      if (static_cast<int8_t>(new_level) > static_cast<int8_t>(DiagnosticLevel::OK)) {
        if (!message.empty()) {
          message += "; ";
        }
        message += new_message;
      }
      if (static_cast<int8_t>(new_level) > static_cast<int8_t>(level)) {
        level = new_level;
      }
    }
  };

  // Result of a map update entry point: whether the NDT map changed, plus the diagnostics to
  // report for this call.
  struct UpdateResult
  {
    bool map_updated{false};
    DiagnosticsReport diagnostics;
    // The merged loaded point cloud map for debugging. Set only when publish_loaded_map is enabled
    // and the map was updated; the ROS node publishes it as-is.
    std::optional<sensor_msgs::msg::PointCloud2> loaded_pcd_map;
  };

private:
  struct BuilderState
  {
    bool need_rebuild{true};
    NdtPtrType secondary_ndt_ptr;
  };

public:
  MapUpdateModule(
    Guarded<NdtPtrType> & ndt_ptr, HyperParameters::DynamicMapLoading param,
    PcdLoaderFunction pcd_loader);

  bool out_of_map_range(const geometry_msgs::msg::Point & position);

private:
  friend class NDTScanMatcher;
  friend class MapUpdateModuleTest;

  UpdateResult callback_timer(const geometry_msgs::msg::Point & position);

  [[nodiscard]] bool should_update_map(
    BuilderState & builder_state, const geometry_msgs::msg::Point & position,
    DiagnosticsReport & diagnostics);

  // Returns true if the NDT map was actually updated.
  bool update_map_internal(
    BuilderState & builder_state, const geometry_msgs::msg::Point & position,
    DiagnosticsReport & diagnostics);

  // Do not call this function while holding the lock for ndt_ptr_.
  UpdateResult update_map(const geometry_msgs::msg::Point & position);
  // Update the specified NDT
  bool update_ndt(
    const geometry_msgs::msg::Point & position, NdtType & ndt, DiagnosticsReport & diagnostics);

  // Concatenates the cells kept in loaded_pcd_map_ into a single cloud for the debug publish.
  // A cell that cannot be concatenated (mismatching field layout) is skipped and reported as a
  // WARN in the given diagnostics.
  [[nodiscard]] sensor_msgs::msg::PointCloud2 merge_loaded_pcd_map(
    DiagnosticsReport & diagnostics) const;

  PcdLoaderFunction pcd_loader_;

  // To prevent deadlocks, acquire locks in the following order:
  // 1. builder_state_ -> ndt_ptr_
  // 2. builder_state_ -> last_update_position_
  Guarded<NdtPtrType> & ndt_ptr_;
  Guarded<BuilderState> builder_state_;
  Guarded<std::optional<geometry_msgs::msg::Point>> last_update_position_{std::nullopt};

  HyperParameters::DynamicMapLoading param_;

  // Loaded point cloud map cells for the debug publish, keyed by cell id so that cells dropped by
  // a differential update can be erased. Only populated when param_.publish_loaded_map is enabled.
  // Accessed only while builder_state_'s lock is held.
  std::map<std::string, sensor_msgs::msg::PointCloud2> loaded_pcd_map_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_
