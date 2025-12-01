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

#include "test_case.hpp"

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
class TestRouteManager001 : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto test_case_path =
      std::filesystem::path(
        ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "test_data" / "test_route_manager_001.yaml";
    const auto test_case_data = autoware::test_utils::load_test_case(test_case_path.string());

    lanelet::LaneletMapConstPtr lanelet_map =
      lanelet2_utils::load_mgrs_coordinate_map(test_case_data.map_abs_path);
    map_msg_ = lanelet2_utils::to_autoware_map_msgs(lanelet_map);

    route_msg_ = create_route_msg(
      {{{2239, 2240, 2241, 2242}, 2240},
       {{2301, 2302, 2300, 2299}, 2302},
       {{2244, 2245, 2246, 2247}, 2245},
       {{2265, 2261, 2262, 2263}, 2261}});

    initial_pose_ = test_case_data.manual_poses.at("P0");
    P1 = test_case_data.manual_poses.at("P1");
    P2 = test_case_data.manual_poses.at("P2");
    P3 = test_case_data.manual_poses.at("P3");
    P4 = test_case_data.manual_poses.at("P4");
    P5 = test_case_data.manual_poses.at("P5");
    P6 = test_case_data.manual_poses.at("P6");
  }

  autoware_map_msgs::msg::LaneletMapBin map_msg_;
  autoware_planning_msgs::msg::LaneletRoute route_msg_;

  geometry_msgs::msg::Pose initial_pose_;
  geometry_msgs::msg::Pose P1;
  geometry_msgs::msg::Pose P2;
  geometry_msgs::msg::Pose P3;
  geometry_msgs::msg::Pose P4;
  geometry_msgs::msg::Pose P5;
  geometry_msgs::msg::Pose P6;

  static constexpr double ego_nearest_dist_threshold = 3.0;
  static constexpr double ego_nearest_yaw_threshold = 1.046;
};

TEST_F(TestRouteManager001, create)
{
  const auto route_manager_opt =
    lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);
  ASSERT_TRUE(route_manager_opt.has_value());
}

TEST_F(TestRouteManager001, validate_initial_pose)
{
  const auto route_manager_opt =
    lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);

  const auto & route_manager = route_manager_opt.value();
  const auto initial_lanelet = route_manager.current_lanelet();
  ASSERT_EQ(initial_lanelet.id(), 2302);
}

TEST_F(TestRouteManager001, current_pose_along_non_lane_changing_route)
{
  auto route_manager_opt =
    lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);

  const auto initial_lanelet = route_manager_opt->current_lanelet();
  ASSERT_EQ(initial_lanelet.id(), 2302);

  // goto P1
  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P1, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2302);
  }

  // goto P2
  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P2, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2245);
  }

  // goto P3
  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P3, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2245);
  }
}

TEST_F(TestRouteManager001, current_pose_along_lane_changing_route)
{
  auto route_manager_opt =
    lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);

  const auto initial_lanelet = route_manager_opt->current_lanelet();
  ASSERT_EQ(initial_lanelet.id(), 2302);

  // goto P1
  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P1, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2302);
  }

  // goto P2
  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P2, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2245);
  }

  // goto P4
  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P4, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2245);
  }

  // current position transit to lane 2246, but current_lanelet is along previous value because
  // `commit_lane_change_success` is not called yet
  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P5, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2245);
  }

  // commit lane change, current_route_lanelet should laterally change
  {
    route_manager_opt = std::move(route_manager_opt.value()).commit_lane_change_success(P5);
    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2246);
  }

  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P6, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2246);
  }
}

TEST_F(TestRouteManager001, get_lanelet_sequence_on_route_with_outward_forward)
{
  auto route_manager_opt =
    lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);
  ASSERT_TRUE(route_manager_opt.has_value());

  const auto & route_manager = route_manager_opt.value();

  {
    const auto seq = route_manager.get_lanelet_sequence_on_route(0.0, 0.0);
    ASSERT_EQ(seq.as_lanelets().size(), 1)
      << "if forward/backward length is 0, only current_route_lanelet is returned";
    const auto lane = seq.as_lanelets().front();
    ASSERT_EQ(lane.id(), 2302);
  }

  {
    {
      const auto seq = route_manager.get_lanelet_sequence_on_route(0.0, 9.0);
      ASSERT_EQ(seq.as_lanelets().size(), 1);
      const auto lane = seq.as_lanelets().front();
      ASSERT_EQ(lane.id(), 2302);
    }
    // outward route, should be same
    {
      const auto seq = route_manager.get_lanelet_sequence_outward_route(0.0, 9.0);
      ASSERT_EQ(seq.as_lanelets().size(), 1);
      const auto lane = seq.as_lanelets().front();
      ASSERT_EQ(lane.id(), 2302);
    }
  }

  {
    {
      const auto seq = route_manager.get_lanelet_sequence_on_route(0.0, 15.0);
      ASSERT_EQ(seq.as_lanelets().size(), 2);
      const auto lane1 = seq.as_lanelets().at(0);
      ASSERT_EQ(lane1.id(), 2240);
      const auto lane2 = seq.as_lanelets().at(1);
      ASSERT_EQ(lane2.id(), 2302);
    }
    // outward route, should be same
    {
      const auto seq = route_manager.get_lanelet_sequence_outward_route(0.0, 15.0);
      ASSERT_EQ(seq.as_lanelets().size(), 2);
      const auto lane1 = seq.as_lanelets().at(0);
      ASSERT_EQ(lane1.id(), 2240);
      const auto lane2 = seq.as_lanelets().at(1);
      ASSERT_EQ(lane2.id(), 2302);
    }
  }

  {
    {
      const auto seq = route_manager.get_lanelet_sequence_on_route(0.0, 1000.0);
      ASSERT_EQ(seq.as_lanelets().size(), 2);
      const auto lane1 = seq.as_lanelets().at(0);
      ASSERT_EQ(lane1.id(), 2240);
      const auto lane2 = seq.as_lanelets().at(1);
      ASSERT_EQ(lane2.id(), 2302);
    }
    // outward route, should be same
    {
      const auto seq = route_manager.get_lanelet_sequence_outward_route(0.0, 1000.0);
      ASSERT_EQ(seq.as_lanelets().size(), 2);
      const auto lane1 = seq.as_lanelets().at(0);
      ASSERT_EQ(lane1.id(), 2240);
      const auto lane2 = seq.as_lanelets().at(1);
      ASSERT_EQ(lane2.id(), 2302);
    }
  }

  {
    {
      const auto seq = route_manager.get_lanelet_sequence_on_route(20.0, 1000.0);
      ASSERT_EQ(seq.as_lanelets().size(), 2);
      const auto lane1 = seq.as_lanelets().at(0);
      ASSERT_EQ(lane1.id(), 2240);
      const auto lane2 = seq.as_lanelets().at(1);
      ASSERT_EQ(lane2.id(), 2302);
    }
    // outward route, should be same
    {
      const auto seq = route_manager.get_lanelet_sequence_outward_route(20.0, 1000.0);
      ASSERT_EQ(seq.as_lanelets().size(), 2);
      const auto lane1 = seq.as_lanelets().at(0);
      ASSERT_EQ(lane1.id(), 2240);
      const auto lane2 = seq.as_lanelets().at(1);
      ASSERT_EQ(lane2.id(), 2302);
    }
  }

  {
    {
      const auto seq = route_manager.get_lanelet_sequence_on_route(30.0, 1000.0);
      ASSERT_EQ(seq.as_lanelets().size(), 3);
      const auto lane1 = seq.as_lanelets().at(0);
      ASSERT_EQ(lane1.id(), 2240);
      const auto lane2 = seq.as_lanelets().at(1);
      ASSERT_EQ(lane2.id(), 2302);
      const auto lane3 = seq.as_lanelets().at(2);
      ASSERT_EQ(lane3.id(), 2245);
    }
    // outward route, should be same
    {
      const auto seq = route_manager.get_lanelet_sequence_outward_route(30.0, 1000.0);
      ASSERT_EQ(seq.as_lanelets().size(), 3);
      const auto lane1 = seq.as_lanelets().at(0);
      ASSERT_EQ(lane1.id(), 2240);
      const auto lane2 = seq.as_lanelets().at(1);
      ASSERT_EQ(lane2.id(), 2302);
      const auto lane3 = seq.as_lanelets().at(2);
      ASSERT_EQ(lane3.id(), 2245);
    }
  }

  {
    {
      const auto seq = route_manager.get_lanelet_sequence_on_route(1000.0, 1000.0);
      ASSERT_EQ(seq.as_lanelets().size(), 4);
      const auto lane1 = seq.as_lanelets().at(0);
      ASSERT_EQ(lane1.id(), 2240);
      const auto lane2 = seq.as_lanelets().at(1);
      ASSERT_EQ(lane2.id(), 2302);
      const auto lane3 = seq.as_lanelets().at(2);
      ASSERT_EQ(lane3.id(), 2245);
      const auto lane4 = seq.as_lanelets().at(3);
      ASSERT_EQ(lane4.id(), 2261);
    }
    // outward route
    {
      const auto seq = route_manager.get_lanelet_sequence_outward_route(1000.0, 1000.0);
      ASSERT_EQ(seq.as_lanelets().size(), 5);
      const auto lane1 = seq.as_lanelets().at(0);
      ASSERT_EQ(lane1.id(), 2240);
      const auto lane2 = seq.as_lanelets().at(1);
      ASSERT_EQ(lane2.id(), 2302);
      const auto lane3 = seq.as_lanelets().at(2);
      ASSERT_EQ(lane3.id(), 2245);
      const auto lane4 = seq.as_lanelets().at(3);
      ASSERT_EQ(lane4.id(), 2261);
      const auto lane5 = seq.as_lanelets().at(4);
      ASSERT_EQ(lane5.id(), 2250);
    }
  }
}

TEST_F(TestRouteManager001, get_closest_preferred_route_lanelet)
{
  auto route_manager_opt =
    lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);
  const auto & route_manager = route_manager_opt.value();

  {
    const auto lane_opt = route_manager.get_closest_preferred_route_lanelet(initial_pose_);
    ASSERT_TRUE(lane_opt.has_value());
    EXPECT_EQ(lane_opt.value().id(), 2302);
  }

  {
    const auto lane_opt = route_manager.get_closest_preferred_route_lanelet(P1);
    ASSERT_TRUE(lane_opt.has_value());
    EXPECT_EQ(lane_opt.value().id(), 2302);
  }

  {
    const auto lane_opt = route_manager.get_closest_preferred_route_lanelet(P2);
    ASSERT_TRUE(lane_opt.has_value());
    EXPECT_EQ(lane_opt.value().id(), 2245);
  }

  {
    const auto lane_opt = route_manager.get_closest_preferred_route_lanelet(P3);
    ASSERT_TRUE(lane_opt.has_value());
    EXPECT_EQ(lane_opt.value().id(), 2245);
  }
}

TEST_F(TestRouteManager001, get_closest_route_lanelet_within_constraints)
{
  auto route_manager_opt =
    lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);
  const auto & route_manager = route_manager_opt.value();

  {
    const auto lane_opt = route_manager.get_closest_route_lanelet_within_constraints(
      initial_pose_, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(lane_opt.has_value());
    EXPECT_EQ(lane_opt.value().id(), 2302);
  }

  {
    const auto lane_opt = route_manager.get_closest_route_lanelet_within_constraints(
      P1, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(lane_opt.has_value());
    EXPECT_EQ(lane_opt.value().id(), 2302);
  }

  {
    const auto lane_opt = route_manager.get_closest_route_lanelet_within_constraints(
      P2, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(lane_opt.has_value());
    EXPECT_EQ(lane_opt.value().id(), 2245);
  }

  {
    const auto lane_opt = route_manager.get_closest_route_lanelet_within_constraints(
      P3, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(lane_opt.has_value());
    EXPECT_EQ(lane_opt.value().id(), 2245);
  }

  {
    const auto lane_opt = route_manager.get_closest_route_lanelet_within_constraints(
      P4, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(lane_opt.has_value());
    EXPECT_EQ(lane_opt.value().id(), 2245);
  }

  {
    const auto lane_opt = route_manager.get_closest_route_lanelet_within_constraints(
      P5, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(lane_opt.has_value());
    EXPECT_EQ(lane_opt.value().id(), 2246);
  }

  {
    const auto lane_opt = route_manager.get_closest_route_lanelet_within_constraints(
      P6, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    ASSERT_TRUE(lane_opt.has_value());
    EXPECT_EQ(lane_opt.value().id(), 2246);
  }
}

class TestRouteManager002 : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto test_case_path =
      std::filesystem::path(
        ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "test_data" / "test_route_manager_002.yaml";
    const auto test_case_data = autoware::test_utils::load_test_case(test_case_path.string());

    lanelet::LaneletMapConstPtr lanelet_map =
      lanelet2_utils::load_mgrs_coordinate_map(test_case_data.map_abs_path);
    map_msg_ = lanelet2_utils::to_autoware_map_msgs(lanelet_map);

    route_msg_ =
      create_route_msg({{{2239, 2240, 2241, 2242}, 2239}, {{2301, 2302, 2300, 2299}, 2301}});

    initial_pose_ = test_case_data.manual_poses.at("P0");
    P1 = test_case_data.manual_poses.at("P1");
    P2 = test_case_data.manual_poses.at("P2");
    P3 = test_case_data.manual_poses.at("P3");
    P4 = test_case_data.manual_poses.at("P4");
    P5 = test_case_data.manual_poses.at("P5");
    P6 = test_case_data.manual_poses.at("P6");
    P7 = test_case_data.manual_poses.at("P7");
    P8 = test_case_data.manual_poses.at("P8");
  }

  autoware_map_msgs::msg::LaneletMapBin map_msg_;
  autoware_planning_msgs::msg::LaneletRoute route_msg_;

  geometry_msgs::msg::Pose initial_pose_;
  geometry_msgs::msg::Pose P1;
  geometry_msgs::msg::Pose P2;
  geometry_msgs::msg::Pose P3;
  geometry_msgs::msg::Pose P4;
  geometry_msgs::msg::Pose P5;
  geometry_msgs::msg::Pose P6;
  geometry_msgs::msg::Pose P7;
  geometry_msgs::msg::Pose P8;

  static constexpr double ego_nearest_dist_threshold = 3.0;
  static constexpr double ego_nearest_dist_threshold_strict = 0.0;
  static constexpr double ego_nearest_yaw_threshold = 1.046;
};

TEST_F(TestRouteManager002, current_pose_along_swerving)
{
  auto route_manager_opt =
    lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);
  ASSERT_TRUE(route_manager_opt.has_value()) << "initialization of route_manager failed";

  const auto initial_lanelet = route_manager_opt->current_lanelet();
  ASSERT_EQ(initial_lanelet.id(), 2239);

  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P1, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);

    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2239);
  }

  // begin swerving
  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P2, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);

    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2239);
  }

  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P3, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);

    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2239);
  }

  // swerved to adjacent lane
  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P4, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);

    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2239);
  }

  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P5, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);

    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2301);
  }

  // begin return
  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P6, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);

    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2301);
  }

  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P7, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);

    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2301);
  }

  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P8, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);

    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2301);
  }
}

TEST_F(TestRouteManager002, current_pose_along_swerving_strict_dist_threshold)
{
  auto route_manager_opt =
    lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);
  ASSERT_TRUE(route_manager_opt.has_value()) << "initialization of route_manager failed";

  const auto initial_lanelet = route_manager_opt->current_lanelet();
  ASSERT_EQ(initial_lanelet.id(), 2239);

  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P1, ego_nearest_dist_threshold_strict, ego_nearest_yaw_threshold);

    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2239);
  }

  // begin swerving
  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P2, ego_nearest_dist_threshold_strict, ego_nearest_yaw_threshold);

    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2239);
  }

  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P3, ego_nearest_dist_threshold_strict, ego_nearest_yaw_threshold);

    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2239);
  }

  // swerved to adjacent lane
  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P4, ego_nearest_dist_threshold_strict, ego_nearest_yaw_threshold);

    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2239);
  }

  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P5, ego_nearest_dist_threshold_strict, ego_nearest_yaw_threshold);

    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2301);
  }

  // begin return
  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P6, ego_nearest_dist_threshold_strict, ego_nearest_yaw_threshold);

    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2301);
  }

  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P7, ego_nearest_dist_threshold_strict, ego_nearest_yaw_threshold);

    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2301);
  }

  {
    route_manager_opt =
      std::move(route_manager_opt.value())
        .update_current_pose(P8, ego_nearest_dist_threshold_strict, ego_nearest_yaw_threshold);

    ASSERT_TRUE(route_manager_opt.has_value());

    const auto & route_manager = route_manager_opt.value();
    const auto & current_lanelet = route_manager.current_lanelet();
    ASSERT_EQ(current_lanelet.id(), 2301);
  }
}

class TestRouteManager003 : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto test_case_path =
      std::filesystem::path(
        ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "test_data" / "test_route_manager_003.yaml";
    const auto test_case_data = autoware::test_utils::load_test_case(test_case_path.string());

    lanelet::LaneletMapConstPtr lanelet_map =
      lanelet2_utils::load_mgrs_coordinate_map(test_case_data.map_abs_path);
    map_msg_ = lanelet2_utils::to_autoware_map_msgs(lanelet_map);

    route_msg_ =
      create_route_msg({{{2244, 2245, 2246, 2247}, 2245}, {{2265, 2261, 2262, 2263}, 2261}});

    initial_pose_ = test_case_data.manual_poses.at("P0");
    P1 = test_case_data.manual_poses.at("P1");
  }

  autoware_map_msgs::msg::LaneletMapBin map_msg_;
  autoware_planning_msgs::msg::LaneletRoute route_msg_;

  geometry_msgs::msg::Pose initial_pose_;
  geometry_msgs::msg::Pose P1;

  static constexpr double ego_nearest_dist_threshold = 3.0;
  static constexpr double ego_nearest_yaw_threshold = 1.046;
};

TEST_F(TestRouteManager003, get_lanelet_sequence_at_beginning_of_route)
{
  auto route_manager_opt =
    lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);

  ASSERT_TRUE(route_manager_opt.has_value());

  const auto initial_lanelet = route_manager_opt->current_lanelet();
  ASSERT_EQ(initial_lanelet.id(), 2245);

  const auto & route_manager = route_manager_opt.value();

  {
    const auto seq = route_manager.get_lanelet_sequence_on_route(0.0, 0.0);
    ASSERT_EQ(seq.as_lanelets().size(), 1)
      << "if forward/backward length is 0, only current_route_lanelet is returned";
    const auto lane = seq.as_lanelets().front();
    ASSERT_EQ(lane.id(), 2245);
  }

  {
    {
      const auto seq = route_manager.get_lanelet_sequence_on_route(60.0, 10.0);
      ASSERT_EQ(seq.as_lanelets().size(), 2);
      const auto lane1 = seq.as_lanelets().front();
      const auto lane2 = seq.as_lanelets().at(1);
      ASSERT_EQ(lane1.id(), 2245);
      ASSERT_EQ(lane2.id(), 2261);
    }
    // outward route, should be same
    {
      const auto seq = route_manager.get_lanelet_sequence_outward_route(60.0, 10.0);
      ASSERT_EQ(seq.as_lanelets().size(), 2);
      const auto lane1 = seq.as_lanelets().front();
      const auto lane2 = seq.as_lanelets().at(1);
      ASSERT_EQ(lane1.id(), 2245);
      ASSERT_EQ(lane2.id(), 2261);
    }
  }

  {
    {
      const auto seq = route_manager.get_lanelet_sequence_on_route(70.0, 20.0);
      ASSERT_EQ(seq.as_lanelets().size(), 2);
      const auto lane1 = seq.as_lanelets().front();
      const auto lane2 = seq.as_lanelets().at(1);
      ASSERT_EQ(lane1.id(), 2245);
      ASSERT_EQ(lane2.id(), 2261);
    }
    // outward route
    {
      const auto seq = route_manager.get_lanelet_sequence_outward_route(70.0, 20.0);
      ASSERT_EQ(seq.as_lanelets().size(), 4);
      const auto lane1 = seq.as_lanelets().front();
      const auto lane2 = seq.as_lanelets().at(1);
      const auto lane3 = seq.as_lanelets().at(2);
      const auto lane4 = seq.as_lanelets().at(3);
      ASSERT_EQ(lane1.id(), 2302);
      ASSERT_EQ(lane2.id(), 2245);
      ASSERT_EQ(lane3.id(), 2261);
      ASSERT_EQ(lane4.id(), 2250);
    }
  }

  {
    {
      const auto seq = route_manager.get_lanelet_sequence_on_route(70.0, 50.0);
      ASSERT_EQ(seq.as_lanelets().size(), 2);
      const auto lane1 = seq.as_lanelets().front();
      const auto lane2 = seq.as_lanelets().at(1);
      ASSERT_EQ(lane1.id(), 2245);
      ASSERT_EQ(lane2.id(), 2261);
    }
    // outward route
    {
      const auto seq = route_manager.get_lanelet_sequence_outward_route(70.0, 50.0);
      ASSERT_EQ(seq.as_lanelets().size(), 5);
      const auto lane1 = seq.as_lanelets().front();
      const auto lane2 = seq.as_lanelets().at(1);
      const auto lane3 = seq.as_lanelets().at(2);
      const auto lane4 = seq.as_lanelets().at(3);
      const auto lane5 = seq.as_lanelets().at(4);
      ASSERT_EQ(lane1.id(), 2240);
      ASSERT_EQ(lane2.id(), 2302);
      ASSERT_EQ(lane3.id(), 2245);
      ASSERT_EQ(lane4.id(), 2261);
      ASSERT_EQ(lane5.id(), 2250);
    }
  }
}

TEST_F(TestRouteManager003, get_lanelet_sequence_at_end_of_route)
{
  auto route_manager_opt = lanelet2_utils::RouteManager::create(map_msg_, route_msg_, P1);

  ASSERT_TRUE(route_manager_opt.has_value());

  const auto initial_lanelet = route_manager_opt->current_lanelet();
  ASSERT_EQ(initial_lanelet.id(), 2261);

  const auto & route_manager = route_manager_opt.value();

  {
    const auto seq = route_manager.get_lanelet_sequence_on_route(0.0, 0.0);
    ASSERT_EQ(seq.as_lanelets().size(), 1)
      << "if forward/backward length is 0, only current_route_lanelet is returned";
    const auto lane = seq.as_lanelets().front();
    ASSERT_EQ(lane.id(), 2261);
  }

  {
    const auto seq = route_manager.get_lanelet_sequence_on_route(50.0, 50.0);
    ASSERT_EQ(seq.as_lanelets().size(), 2);
    const auto lane1 = seq.as_lanelets().front();
    const auto lane2 = seq.as_lanelets().at(1);
    ASSERT_EQ(lane1.id(), 2245);
    ASSERT_EQ(lane2.id(), 2261);
  }
  // outward route
  {
    const auto seq = route_manager.get_lanelet_sequence_outward_route(50.0, 50.0);
    ASSERT_EQ(seq.as_lanelets().size(), 4);
    const auto lane1 = seq.as_lanelets().front();
    const auto lane2 = seq.as_lanelets().at(1);
    const auto lane3 = seq.as_lanelets().at(2);
    const auto lane4 = seq.as_lanelets().at(3);
    ASSERT_EQ(lane1.id(), 2302);
    ASSERT_EQ(lane2.id(), 2245);
    ASSERT_EQ(lane3.id(), 2261);
    ASSERT_EQ(lane4.id(), 2250);
  }
}

}  // namespace autoware::experimental

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
