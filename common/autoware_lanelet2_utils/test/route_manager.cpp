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
#include <utility>
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

geometry_msgs::msg::Pose build_pose(
  const double x, const double y, const double z, const double qx, const double qy, const double qz,
  const double qw)
{
  const auto position = geometry_msgs::msg::Point().set__x(x).set__y(y).set__z(z);
  const auto quat = geometry_msgs::msg::Quaternion().set__x(qx).set__y(qy).set__z(qz).set__w(qw);
  return geometry_msgs::msg::Pose().set__position(position).set__orientation(quat);
}

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
      {{{2239, 2240, 2241, 2242}, 2240},
       {{2301, 2302, 2300, 2299}, 2302},
       {{2244, 2245, 2246, 2247}, 2245},
       {{2265, 2261, 2262, 2263}, 2261},
       {{2249, 2250, 2251, 2252}, 2250}});

    initial_pose_ = build_pose(
      123.0, 250.0, 100.0, 0.0, 0.0, -0.6970803953747329, 0.7169929723394826);  // on lane 2302
  }

  autoware_map_msgs::msg::LaneletMapBin map_msg_;
  autoware_planning_msgs::msg::LaneletRoute route_msg_;
  geometry_msgs::msg::Pose initial_pose_;

  static constexpr double ego_nearest_dist_threshold = 3.0;
  static constexpr double ego_nearest_yaw_threshold = 1.046;
};

TEST_F(TestWithIntersectionCrossingMap, create)
{
  const auto route_manager_opt =
    lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);
  ASSERT_TRUE(route_manager_opt.has_value());
}

TEST_F(TestWithIntersectionCrossingMap, validate_initial_pose)
{
  const auto route_manager_opt =
    lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);

  const auto & route_manager = route_manager_opt.value();
  const auto initial_lanelet = route_manager.current_lanelet();
  ASSERT_EQ(initial_lanelet.id(), 2302);
}

TEST_F(TestWithIntersectionCrossingMap, current_pose_along_non_lane_changing_route)
{
  auto route_manager_opt =
    lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);

  const auto initial_lanelet = route_manager_opt->current_lanelet();
  ASSERT_EQ(initial_lanelet.id(), 2302);

  const auto pose_t1 = build_pose(
    122.93376376185245, 247.71254791887776, 100.0, 0.0, 0.0, -0.7071067811865475,
    0.7071067811865476);  // on lane 2302
  const auto pose_t2 = build_pose(
    122.93376376185245, 236.257378343399, 100.0, 0.0, 0.0, -0.7071067811865475,
    0.7071067811865476);  // on lane 2245
  const auto pose_t3 = build_pose(
    123.16286715336201, 211.62876375611964, 100.0, 0.0, 0.0, -0.720073489482108,
    0.6938978092954757);  // on lane 2245

  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(pose_t1, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2302);
  }

  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(pose_t2, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2245);
  }

  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(pose_t3, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2245);
  }
}

class TestWithIntersectionCrossingMapWithLaneChange : public ::testing::Test
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
      {{{2239, 2240, 2241, 2242}, 2240},
       {{2301, 2302, 2300, 2299}, 2302},
       {{2244, 2245, 2246, 2247}, 2246},  // lane changed
       {{2265, 2261, 2262, 2263}, 2262},
       {{2249, 2250, 2251, 2252}, 2251}});

    initial_pose_ = build_pose(
      123.0, 250.0, 100.0, 0.0, 0.0, -0.6970803953747329, 0.7169929723394826);  // on lane 2302
  }

  autoware_map_msgs::msg::LaneletMapBin map_msg_;
  autoware_planning_msgs::msg::LaneletRoute route_msg_;
  geometry_msgs::msg::Pose initial_pose_;

  static constexpr double ego_nearest_dist_threshold = 3.0;
  static constexpr double ego_nearest_yaw_threshold = 1.046;
};

TEST_F(TestWithIntersectionCrossingMapWithLaneChange, current_pose_along_lane_changing_route)
{
  const auto pose_t1 = build_pose(
    123.00714360821374, 236.44983190720905, 100.0, 0.0, 0.0, -0.7071067811865475,
    0.7071067811865476);  // on lane 2245
  const auto pose_t2 = build_pose(
    122.93376376185245, 236.257378343399, 100.0, 0.0, 0.0, -0.6945983947098338,
    0.7193977134148551);  // slightly on lane 2245
  const auto pose_t3 = build_pose(
    121.3903032556334, 226.77711813780806, 100.0, 0.0, 0.0, -0.7071067811865475,
    0.7071067811865476);  // slightly on lane 2246
  const auto pose_t4 = build_pose(
    120.24536593485449, 219.99908919879695, 100.0, 0.0, 0.0, -0.7071067811865475,
    0.7071067811865476);  // on lane 2246

  auto route_manager_opt =
    lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);

  const auto initial_lanelet = route_manager_opt->current_lanelet();
  ASSERT_EQ(initial_lanelet.id(), 2302);

  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(pose_t1, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2245);
  }

  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(pose_t2, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2245);
  }

  // current position transit to lane 2246, but current_lanelet is along previous value
  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(pose_t3, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2245);
  }

  // commit lane change, current_route_lanelet should laterally change
  {
    route_manager_opt = std::move(route_manager_opt.value()).commit_lane_change_success(pose_t3);
    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2246);
  }

  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(pose_t4, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2246);
  }
}

}  // namespace autoware::experimental

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
