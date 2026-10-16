// Copyright 2026 Autoware Foundation
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

#ifndef HARNESS__STUB_MAP_LOADER_HPP_
#define HARNESS__STUB_MAP_LOADER_HPP_

#include "stimulus.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace ndt_test
{

/// @brief One call to `pcd_loader_service`, as the stub saw it.
///
/// The request half is what the node asked for; the answer half is what it was given. Both are
/// otherwise invisible to this binary: the node reports what it did with a map, never what it
/// asked for, so without this record a case can only infer the conversation from its result.
struct LoaderCall
{
  /// @brief Centre and radius of the requested circle, as `MapUpdateModule` filled them in.
  double center_x{0.0};
  double center_y{0.0};
  double radius{0.0};
  /// @brief Cell ids the node said it already holds. The loader must not serve these again.
  std::vector<std::string> cached_ids;
  /// @brief Cell ids served back in `new_pointcloud_with_ids`.
  std::vector<std::string> served_ids;
  /// @brief Cell ids taken back in `ids_to_remove`.
  std::vector<std::string> removed_ids;
  /// @brief Points across every served cell, so a case can tell a real map from an empty answer.
  size_t served_points{0};
};

/// @brief The only map in this world: `make_corner_cloud` in two cells, "0" anchored at the map
/// center and "1" at `second_cell_x`.
///
/// Differential, like the loader it stands in for: a cell is returned when the requested circle
/// covers its anchor and `cached_ids` does not list it, and a cached cell the circle no longer
/// covers comes back in `ids_to_remove`. Re-querying from inside the cells yields an empty
/// response, which `update_ndt` reports as `is_updated_map: False`; so does asking away from both.
///
/// Cell "1" must not move closer than x = 300: every other case queries from at most x = 125 with
/// a radius of 150, so an anchor at x <= 275 would enter their responses and change their maps.
///
/// A spy as well as a stub: every call is recorded, in order, and read back through `calls()`.
///
/// @note `test/stub_pcd_loader.hpp` also answers `pcd_loader_service`, for the three pre-existing
/// node tests. Merging the two was left out of scope, so a change to how the map is served has to
/// be made in both places.
class StubMapLoader : public rclcpp::Node
{
  using GetDifferentialPointCloudMap = autoware_map_msgs::srv::GetDifferentialPointCloudMap;

public:
  StubMapLoader() : Node("stub_map_loader")
  {
    service_ = create_service<GetDifferentialPointCloudMap>(
      "pcd_loader_service",
      std::bind(&StubMapLoader::on_get_map, this, std::placeholders::_1, std::placeholders::_2));
  }

  /// @brief Every call so far, oldest first.
  ///
  /// A copy: the service runs on the loader's own thread, so a reference would race the next call.
  [[nodiscard]] std::vector<LoaderCall> calls() const
  {
    const std::lock_guard<std::mutex> lock(calls_mutex_);
    return calls_;
  }

  /// @brief How many times the node has called the service.
  [[nodiscard]] size_t call_count() const
  {
    const std::lock_guard<std::mutex> lock(calls_mutex_);
    return calls_.size();
  }

private:
  struct Cell
  {
    const char * id;
    double x;
    double y;
  };
  static constexpr std::array<Cell, 2> cells{
    {{"0", map_center_x, map_center_y}, {"1", second_cell_x, map_center_y}}};

  rclcpp::Service<GetDifferentialPointCloudMap>::SharedPtr service_;

  mutable std::mutex calls_mutex_;
  std::vector<LoaderCall> calls_;

  static bool covers(const autoware_map_msgs::msg::AreaInfo & area, const Cell & cell)
  {
    const auto x = static_cast<float>(cell.x);
    const auto y = static_cast<float>(cell.y);
    return area.center_x - area.radius <= x && area.center_x + area.radius >= x &&
           area.center_y - area.radius <= y && area.center_y + area.radius >= y;
  }

  static autoware_map_msgs::msg::PointCloudMapCellWithID make_cell(const Cell & cell)
  {
    const pcl::PointCloud<pcl::PointXYZ> cloud = make_corner_cloud(map_spacing, cell.x, cell.y);

    autoware_map_msgs::msg::PointCloudMapCellWithID msg;
    msg.cell_id = cell.id;
    msg.metadata.min_x = std::numeric_limits<float>::max();
    msg.metadata.min_y = std::numeric_limits<float>::max();
    msg.metadata.max_x = std::numeric_limits<float>::lowest();
    msg.metadata.max_y = std::numeric_limits<float>::lowest();
    for (const auto & point : cloud.points) {
      msg.metadata.min_x = std::min(msg.metadata.min_x, point.x);
      msg.metadata.min_y = std::min(msg.metadata.min_y, point.y);
      msg.metadata.max_x = std::max(msg.metadata.max_x, point.x);
      msg.metadata.max_y = std::max(msg.metadata.max_y, point.y);
    }
    pcl::toROSMsg(cloud, msg.pointcloud);
    return msg;
  }

  void on_get_map(
    GetDifferentialPointCloudMap::Request::SharedPtr req,
    GetDifferentialPointCloudMap::Response::SharedPtr res)
  {
    res->header.frame_id = map_frame;

    LoaderCall call;
    call.center_x = req->area.center_x;
    call.center_y = req->area.center_y;
    call.radius = req->area.radius;
    call.cached_ids = req->cached_ids;

    for (const auto & cell : cells) {
      const bool covered = covers(req->area, cell);
      const bool cached =
        std::find(req->cached_ids.begin(), req->cached_ids.end(), cell.id) != req->cached_ids.end();
      if (cached && !covered) {
        res->ids_to_remove.emplace_back(cell.id);
        call.removed_ids.emplace_back(cell.id);
      }
      if (covered && !cached) {
        auto served = make_cell(cell);
        call.served_ids.emplace_back(cell.id);
        call.served_points +=
          static_cast<size_t>(served.pointcloud.width) * served.pointcloud.height;
        res->new_pointcloud_with_ids.push_back(std::move(served));
      }
    }

    const std::lock_guard<std::mutex> lock(calls_mutex_);
    calls_.push_back(std::move(call));
  }
};

}  // namespace ndt_test

#endif  // HARNESS__STUB_MAP_LOADER_HPP_
