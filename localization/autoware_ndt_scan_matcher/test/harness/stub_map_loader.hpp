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
#include <limits>
#include <string>

namespace ndt_test
{

/// @brief The only map in this world: `make_corner_cloud` in two cells, "0" anchored at the map
/// center and "1" at `second_cell_x`.
///
/// Differential, like the loader it stands in for: a cell is returned when the requested circle
/// covers its anchor and `cached_ids` does not list it, and a cached cell whose anchor the circle
/// no longer covers comes back in `ids_to_remove`. A node re-querying from inside its cells gets an
/// empty response, which `update_ndt` reports as `is_updated_map: False`.
///
/// Cell "1" sits where only the cell-boundary walk reaches it. Every other case queries from at
/// most x = 125 with a radius of 150, so an anchor at x <= 275 would enter their responses and
/// change their maps.
///
/// Asking away from both anchors yields nothing. `MissingMapAbortsBeforeAlignment` asks at
/// (-100, -100): the load fails once, and since a failed load still records the position
/// (`map_update_module.cpp:176`), the timer does not try again until the vehicle moves
/// `update_distance`.
///
/// @note `test/stub_pcd_loader.hpp` also answers `pcd_loader_service`, for the three pre-existing
/// node tests. The two are deliberately not merged *yet*: this one takes its geometry from
/// `make_corner_cloud`, so the scan and the map are provably the same surfaces at different
/// spacings, while the older stub carries its own hand-written copy. Unifying them means touching
/// those three tests, which is out of scope here. Until that happens, a change to how the map is
/// served has to be made in both places.
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
    GetDifferentialPointCloudMap::Response::SharedPtr res) const
  {
    res->header.frame_id = map_frame;
    for (const auto & cell : cells) {
      const bool covered = covers(req->area, cell);
      const bool cached =
        std::find(req->cached_ids.begin(), req->cached_ids.end(), cell.id) != req->cached_ids.end();
      if (cached && !covered) {
        res->ids_to_remove.push_back(cell.id);
      }
      if (covered && !cached) {
        res->new_pointcloud_with_ids.push_back(make_cell(cell));
      }
    }
  }
};

}  // namespace ndt_test

#endif  // HARNESS__STUB_MAP_LOADER_HPP_
