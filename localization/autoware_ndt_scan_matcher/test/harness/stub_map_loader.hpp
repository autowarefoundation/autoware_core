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
#include <limits>
#include <string>

namespace ndt_test
{

/// @brief The only map in this world: `make_corner_cloud` placed at the map center, in one cell.
///
/// Differential, like the loader it stands in for: the cell is returned when the requested circle
/// covers the map center and `cached_ids` does not already list it, and a cached cell the circle no
/// longer covers comes back in `ids_to_remove`. A node that keeps querying from inside the cell
/// therefore gets an empty response, which `update_ndt` reports as `is_updated_map: False`.
///
/// Asking away from the center yields nothing at all. `MissingMapAbortsBeforeAlignment` asks at
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
  static constexpr const char * cell_id = "0";
  rclcpp::Service<GetDifferentialPointCloudMap>::SharedPtr service_;

  void on_get_map(
    GetDifferentialPointCloudMap::Request::SharedPtr req,
    GetDifferentialPointCloudMap::Response::SharedPtr res) const
  {
    res->header.frame_id = map_frame;

    const auto center_x = static_cast<float>(map_center_x);
    const auto center_y = static_cast<float>(map_center_y);
    const bool covers_center = req->area.center_x - req->area.radius <= center_x &&
                               req->area.center_x + req->area.radius >= center_x &&
                               req->area.center_y - req->area.radius <= center_y &&
                               req->area.center_y + req->area.radius >= center_y;
    const bool cached =
      std::find(req->cached_ids.begin(), req->cached_ids.end(), cell_id) != req->cached_ids.end();

    if (cached && !covers_center) {
      res->ids_to_remove.push_back(cell_id);
    }
    if (!covers_center || cached) {
      return;
    }

    const pcl::PointCloud<pcl::PointXYZ> cloud =
      make_corner_cloud(map_spacing, map_center_x, map_center_y);

    autoware_map_msgs::msg::PointCloudMapCellWithID cell;
    cell.cell_id = cell_id;
    cell.metadata.min_x = std::numeric_limits<float>::max();
    cell.metadata.min_y = std::numeric_limits<float>::max();
    cell.metadata.max_x = std::numeric_limits<float>::lowest();
    cell.metadata.max_y = std::numeric_limits<float>::lowest();
    for (const auto & point : cloud.points) {
      cell.metadata.min_x = std::min(cell.metadata.min_x, point.x);
      cell.metadata.min_y = std::min(cell.metadata.min_y, point.y);
      cell.metadata.max_x = std::max(cell.metadata.max_x, point.x);
      cell.metadata.max_y = std::max(cell.metadata.max_y, point.y);
    }
    pcl::toROSMsg(cloud, cell.pointcloud);

    res->new_pointcloud_with_ids.push_back(cell);
  }
};

}  // namespace ndt_test

#endif  // HARNESS__STUB_MAP_LOADER_HPP_
