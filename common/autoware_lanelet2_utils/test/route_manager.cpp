// Copyright 2025 TIER IV, Inc.
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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/route_manager.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using autoware_planning_msgs::msg::LaneletRoute;

static autoware_planning_msgs::msg::LaneletRoute create_route_msg(
  const std::vector<std::pair<std::vector<lanelet::Id>, lanelet::Id>> & route_ids)
{
  using autoware_planning_msgs::msg::LaneletPrimitive;
  using autoware_planning_msgs::msg::LaneletSegment;
  LaneletRoute route_msg;
  for (const auto & route_id : route_ids) {
    const auto & [ids, preferred_id] = route_id;
    LaneletSegment segment;
    segment.preferred_primitive.id = preferred_id;
    for (const auto & id : ids) {
      auto primitive =
        autoware_planning_msgs::build<LaneletPrimitive>().id(id).primitive_type("road");
      segment.primitives.push_back(primitive);
    }
    route_msg.segments.push_back(segment);
  }
  return route_msg;
}

namespace autoware::experimental
{
class TestWithIntersectionCrossingMap : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map";
    const auto intersection_crossing_map_path =
      fs::read_symlink(sample_map_dir / "intersection" / "crossing.osm");

    lanelet::LaneletMapConstPtr lanelet_map =
      lanelet2_utils::load_mgrs_coordinate_map(intersection_crossing_map_path.string());
    map_msg_ = lanelet2_utils::to_autoware_map_msgs(lanelet_map);

    route_msg_ = create_route_msg(
      {{{2239, 2240, 2241, 2242}, 2241},
       {{2301, 2302, 2300, 2299}, 2300},
       {{2244, 2245, 2246, 2247}, 2246},
       {{2265, 2261, 2262, 2263}, 2262},
       {{2249, 2250, 2251, 2252}, 2251}});

    initial_pose_.position =
      geometry_msgs::build<geometry_msgs::msg::Point>().x(123.0).y(250.0).z(100.0);
    initial_pose_.orientation =
      geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0);
  }

  autoware_map_msgs::msg::LaneletMapBin map_msg_;
  autoware_planning_msgs::msg::LaneletRoute route_msg_;
  geometry_msgs::msg::Pose initial_pose_;
};

TEST_F(TestWithIntersectionCrossingMap, create)
{
  const auto route_manager_opt =
    lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);
  ASSERT_TRUE(route_manager_opt.has_value());
}
}  // namespace autoware::experimental

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
