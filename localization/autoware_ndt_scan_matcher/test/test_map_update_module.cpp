// Copyright 2025 Autoware Foundation
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

#include "test_util.hpp"

#include <autoware/ndt_scan_matcher/map_update_module.hpp>

#include <autoware_map_msgs/msg/point_cloud_map_cell_with_id.hpp>
#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

// These tests exercise the map-update logic that the refactoring extracted out of the ROS node.
// The ROS-dependent pcd load is injected as a plain std::function (PcdLoaderFunction), so the
// whole flow runs in-process with a fake loader and no ROS service / executor.

namespace autoware::ndt_scan_matcher
{
namespace
{
using GetDifferentialPointCloudMap = autoware_map_msgs::srv::GetDifferentialPointCloudMap;

geometry_msgs::msg::Point make_point(const double x, const double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0.0;
  return point;
}

// A fake loader that always returns a single map cell built from the sample cubic pcd.
MapUpdateModule::PcdLoaderFunction make_loader_returning_cell()
{
  return [](const GetDifferentialPointCloudMap::Request::SharedPtr & /*request*/)
           -> GetDifferentialPointCloudMap::Response::SharedPtr {
    auto response = std::make_shared<GetDifferentialPointCloudMap::Response>();
    autoware_map_msgs::msg::PointCloudMapCellWithID cell;
    cell.cell_id = "0";
    pcl::toROSMsg(make_sample_half_cubic_pcd(), cell.pointcloud);
    response->new_pointcloud_with_ids.push_back(cell);
    response->header.frame_id = "map";
    return response;
  };
}

// A fake loader that returns `num_cells` map cells, each the sample cubic pcd shifted along x so
// the cells do not overlap. Exercises the multi-cell merge in merge_loaded_pcd_map().
MapUpdateModule::PcdLoaderFunction make_loader_returning_cells(const std::size_t num_cells)
{
  return [num_cells](const GetDifferentialPointCloudMap::Request::SharedPtr & /*request*/)
           -> GetDifferentialPointCloudMap::Response::SharedPtr {
    auto response = std::make_shared<GetDifferentialPointCloudMap::Response>();
    for (std::size_t i = 0; i < num_cells; ++i) {
      autoware_map_msgs::msg::PointCloudMapCellWithID cell;
      cell.cell_id = std::to_string(i);
      auto cloud = make_sample_half_cubic_pcd();
      for (auto & point : cloud.points) {
        point.x += static_cast<float>(i) * 20.0F;
      }
      pcl::toROSMsg(cloud, cell.pointcloud);
      response->new_pointcloud_with_ids.push_back(cell);
    }
    response->header.frame_id = "map";
    return response;
  };
}

// A fake loader that fails, as if the pcd_loader service were unavailable.
MapUpdateModule::PcdLoaderFunction make_failing_loader()
{
  return [](const GetDifferentialPointCloudMap::Request::SharedPtr & /*request*/)
           -> GetDifferentialPointCloudMap::Response::SharedPtr { return nullptr; };
}
}  // namespace

class MapUpdateModuleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    pclomp::NdtParams ndt_params{};
    ndt_params.trans_epsilon = 0.01;
    ndt_params.step_size = 0.1;
    ndt_params.resolution = 2.0F;
    ndt_params.max_iterations = 30;
    ndt_params.num_threads = 1;
    ndt_params.regularization_scale_factor = 0.0F;
    ndt_ptr_.with([&](auto & ndt) { ndt->setParams(ndt_params); });

    param_.update_distance = 5.0;
    param_.map_radius = 30.0;
    param_.lidar_radius = 5.0;
    param_.publish_loaded_map = false;
  }

  // Accessors for the private entry points. This fixture is a friend of MapUpdateModule, so these
  // members may reach its private methods; the TEST_F bodies call them through the fixture.
  static MapUpdateModule::UpdateResult update_map(
    MapUpdateModule & module, const geometry_msgs::msg::Point & position)
  {
    return module.update_map(position);
  }
  static MapUpdateModule::UpdateResult callback_timer(
    MapUpdateModule & module, const geometry_msgs::msg::Point & position)
  {
    return module.callback_timer(position);
  }
  static bool ndt_has_target(Guarded<MapUpdateModule::NdtPtrType> & ndt_ptr)
  {
    return ndt_ptr.with([](const auto & ndt) { return ndt->hasTarget(); });
  }

  Guarded<MapUpdateModule::NdtPtrType> ndt_ptr_{std::make_shared<MapUpdateModule::NdtType>()};
  HyperParameters::DynamicMapLoading param_{};
};

// The first update has no previous position, so it rebuilds the NDT from scratch and loads a map.
TEST_F(MapUpdateModuleTest, FirstUpdateRebuildsAndLoadsMap)  // NOLINT
{
  MapUpdateModule module(ndt_ptr_, param_, make_loader_returning_cell());

  const auto result = update_map(module, make_point(0.0, 0.0));

  EXPECT_TRUE(result.map_updated);
  EXPECT_TRUE(ndt_has_target(ndt_ptr_));
}

// out_of_map_range is true before any update, false at the last update position, and true again
// once the query moves beyond (map_radius - lidar_radius) from it.
TEST_F(MapUpdateModuleTest, OutOfMapRangeTracksLastUpdate)  // NOLINT
{
  MapUpdateModule module(ndt_ptr_, param_, make_loader_returning_cell());

  EXPECT_TRUE(module.out_of_map_range(make_point(0.0, 0.0)));  // nothing loaded yet

  ASSERT_TRUE(update_map(module, make_point(0.0, 0.0)).map_updated);

  EXPECT_FALSE(module.out_of_map_range(make_point(0.0, 0.0)));   // at the last update position
  EXPECT_TRUE(module.out_of_map_range(make_point(100.0, 0.0)));  // 100 + 5 > 30
}

// callback_timer skips the update when the vehicle has not moved farther than update_distance.
TEST_F(MapUpdateModuleTest, CallbackTimerSkipsWhenNotMovedFarEnough)  // NOLINT
{
  MapUpdateModule module(ndt_ptr_, param_, make_loader_returning_cell());
  ASSERT_TRUE(update_map(module, make_point(0.0, 0.0)).map_updated);

  // Moved 1 m < update_distance (5 m) -> no update.
  const auto result = callback_timer(module, make_point(1.0, 0.0));
  EXPECT_FALSE(result.map_updated);
}

// A failing loader yields no update and an ERROR diagnostic mentioning the loader failure.
TEST_F(MapUpdateModuleTest, LoaderFailureReportsError)  // NOLINT
{
  MapUpdateModule module(ndt_ptr_, param_, make_failing_loader());

  const auto result = update_map(module, make_point(0.0, 0.0));

  EXPECT_FALSE(result.map_updated);
  EXPECT_EQ(result.diagnostics.level, MapUpdateModule::DiagnosticLevel::ERROR);
  EXPECT_NE(
    result.diagnostics.message.find("pcd_loader service is not working."), std::string::npos);
}

// With publish_loaded_map enabled, a successful update returns the merged debug cloud in the "map"
// frame, stamped with the query position's timestamp (the stamp is set inside the module).
TEST_F(MapUpdateModuleTest, PublishesStampedLoadedMapWhenEnabled)  // NOLINT
{
  param_.publish_loaded_map = true;
  MapUpdateModule module(ndt_ptr_, param_, make_loader_returning_cell());

  const auto result = update_map(module, make_point(0.0, 0.0));

  ASSERT_TRUE(result.map_updated);
  ASSERT_TRUE(result.loaded_pcd_map.has_value());
  EXPECT_EQ(result.loaded_pcd_map->header.frame_id, "map");
  EXPECT_GT(
    static_cast<std::size_t>(result.loaded_pcd_map->width) * result.loaded_pcd_map->height, 0U);
}

// The debug cloud is the concatenation of every loaded cell: its point count and its buffer size
// are the totals over the cells. Pins the in-place merge in merge_loaded_pcd_map().
TEST_F(MapUpdateModuleTest, MergesAllLoadedCellsIntoOneCloud)  // NOLINT
{
  constexpr std::size_t num_cells = 3;
  param_.publish_loaded_map = true;
  MapUpdateModule module(ndt_ptr_, param_, make_loader_returning_cells(num_cells));

  const auto result = update_map(module, make_point(0.0, 0.0));

  ASSERT_TRUE(result.map_updated);
  ASSERT_TRUE(result.loaded_pcd_map.has_value());

  // The per-cell totals to compare against: every cell holds one sample cubic pcd.
  sensor_msgs::msg::PointCloud2 one_cell;
  pcl::toROSMsg(make_sample_half_cubic_pcd(), one_cell);

  EXPECT_EQ(result.loaded_pcd_map->height, 1U);  // merging makes the cloud unorganized
  EXPECT_EQ(
    static_cast<std::size_t>(result.loaded_pcd_map->width),
    num_cells * static_cast<std::size_t>(one_cell.width) * one_cell.height);
  EXPECT_EQ(result.loaded_pcd_map->data.size(), num_cells * one_cell.data.size());
  EXPECT_EQ(result.diagnostics.level, MapUpdateModule::DiagnosticLevel::OK);
}

// Without publish_loaded_map, no debug cloud is produced even on a successful update.
TEST_F(MapUpdateModuleTest, DoesNotProduceLoadedMapWhenDisabled)  // NOLINT
{
  param_.publish_loaded_map = false;
  MapUpdateModule module(ndt_ptr_, param_, make_loader_returning_cell());

  const auto result = update_map(module, make_point(0.0, 0.0));

  ASSERT_TRUE(result.map_updated);
  EXPECT_FALSE(result.loaded_pcd_map.has_value());
}

// DiagnosticsReport accumulates like DiagnosticsInterface: the level only rises and non-OK
// messages are joined with "; ". This is pure logic and needs no module instance.
TEST(MapUpdateModuleDiagnosticsReport, RaisesLevelAndJoinsMessages)  // NOLINT
{
  MapUpdateModule::DiagnosticsReport report;
  EXPECT_EQ(report.level, MapUpdateModule::DiagnosticLevel::OK);
  EXPECT_TRUE(report.message.empty());

  // OK-level updates neither raise the level nor append a message.
  report.update_level_and_message(MapUpdateModule::DiagnosticLevel::OK, "ignored");
  EXPECT_EQ(report.level, MapUpdateModule::DiagnosticLevel::OK);
  EXPECT_TRUE(report.message.empty());

  report.update_level_and_message(MapUpdateModule::DiagnosticLevel::WARN, "first");
  report.update_level_and_message(MapUpdateModule::DiagnosticLevel::ERROR, "second");
  // A later, lower level must not lower the accumulated level.
  report.update_level_and_message(MapUpdateModule::DiagnosticLevel::WARN, "third");

  EXPECT_EQ(report.level, MapUpdateModule::DiagnosticLevel::ERROR);
  EXPECT_EQ(report.message, "first; second; third");
}

}  // namespace autoware::ndt_scan_matcher
