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

#include "utils_test.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/Lanelet.h>

namespace autoware::path_generator
{
using Trajectory = experimental::trajectory::Trajectory<PathPointWithLaneId>;

TEST_F(UtilsTest, connectPathToGoalInsideLaneletSequence)
{
  constexpr auto m_epsilon = 1e-3;
  constexpr auto rad_epsilon = 1e-2;
  const auto path = *Trajectory::Builder{}.build(path_.points);

  auto goal_lanelet_for_path = planner_data_.preferred_lanelets.back();
  auto s_goal = 0.;
  for (const auto & lanelet : planner_data_.preferred_lanelets) {
    if (std::any_of(
          planner_data_.goal_lanelets.begin(), planner_data_.goal_lanelets.end(),
          [&](const auto & goal_lanelet) { return lanelet.id() == goal_lanelet.id(); })) {
      goal_lanelet_for_path = lanelet;
      s_goal += lanelet::utils::getArcCoordinates({lanelet}, planner_data_.goal_pose).length;
      break;
    }
    s_goal += lanelet::geometry::length2d(lanelet);
  }

  {  // normal case
    const auto result = utils::connect_path_to_goal_inside_lanelet_sequence(
      path, planner_data_.preferred_lanelets, planner_data_.goal_pose, goal_lanelet_for_path,
      s_goal, planner_data_, 7.5, 1.0);

    ASSERT_TRUE(result.has_value());

    const auto new_goal = result->compute(result->length());
    ASSERT_NEAR(new_goal.point.pose.position.x, planner_data_.goal_pose.position.x, m_epsilon);
    ASSERT_NEAR(new_goal.point.pose.position.y, planner_data_.goal_pose.position.y, m_epsilon);
    ASSERT_NEAR(new_goal.point.pose.position.z, planner_data_.goal_pose.position.z, m_epsilon);
    ASSERT_NEAR(
      new_goal.point.pose.orientation.x, planner_data_.goal_pose.orientation.x, rad_epsilon);
    ASSERT_NEAR(
      new_goal.point.pose.orientation.y, planner_data_.goal_pose.orientation.y, rad_epsilon);
    ASSERT_NEAR(
      new_goal.point.pose.orientation.z, planner_data_.goal_pose.orientation.z, rad_epsilon);
    ASSERT_NEAR(
      new_goal.point.pose.orientation.w, planner_data_.goal_pose.orientation.w, rad_epsilon);
  }

  {  // lanelets are empty
    const auto result = utils::connect_path_to_goal_inside_lanelet_sequence(
      path, {}, planner_data_.goal_pose, goal_lanelet_for_path, s_goal, planner_data_, 7.5, 1.0);

    ASSERT_FALSE(result.has_value());
  }

  {  // connection_section_length is zero
    const auto result = utils::connect_path_to_goal_inside_lanelet_sequence(
      path, planner_data_.preferred_lanelets, planner_data_.goal_pose, goal_lanelet_for_path,
      s_goal, planner_data_, 0.0, 1.0);

    ASSERT_FALSE(result.has_value());
  }
}

TEST_F(UtilsTest, connectPathToGoal)
{
  constexpr auto m_epsilon = 1e-3;
  constexpr auto rad_epsilon = 1e-2;

  const auto path = *Trajectory::Builder{}.build(path_.points);

  auto goal_lanelet_for_path = planner_data_.preferred_lanelets.back();
  auto s_goal = 0.;
  for (const auto & lanelet : planner_data_.preferred_lanelets) {
    if (std::any_of(
          planner_data_.goal_lanelets.begin(), planner_data_.goal_lanelets.end(),
          [&](const auto & goal_lanelet) { return lanelet.id() == goal_lanelet.id(); })) {
      goal_lanelet_for_path = lanelet;
      s_goal += lanelet::utils::getArcCoordinates({lanelet}, planner_data_.goal_pose).length;
      break;
    }
    s_goal += lanelet::geometry::length2d(lanelet);
  }

  {  // normal case
    const auto result = utils::connect_path_to_goal(
      path, planner_data_.preferred_lanelets, planner_data_.goal_pose, goal_lanelet_for_path,
      s_goal, planner_data_, 7.5, 1.0);

    const auto new_goal = result.compute(result.length());
    ASSERT_NEAR(new_goal.point.pose.position.x, planner_data_.goal_pose.position.x, m_epsilon);
    ASSERT_NEAR(new_goal.point.pose.position.y, planner_data_.goal_pose.position.y, m_epsilon);
    ASSERT_NEAR(new_goal.point.pose.position.z, planner_data_.goal_pose.position.z, m_epsilon);
    ASSERT_NEAR(
      new_goal.point.pose.orientation.x, planner_data_.goal_pose.orientation.x, rad_epsilon);
    ASSERT_NEAR(
      new_goal.point.pose.orientation.y, planner_data_.goal_pose.orientation.y, rad_epsilon);
    ASSERT_NEAR(
      new_goal.point.pose.orientation.z, planner_data_.goal_pose.orientation.z, rad_epsilon);
    ASSERT_NEAR(
      new_goal.point.pose.orientation.w, planner_data_.goal_pose.orientation.w, rad_epsilon);
  }

  {  // goal lanelet is invalid
    const auto result = utils::connect_path_to_goal(
      path, planner_data_.preferred_lanelets, planner_data_.goal_pose,
      lanelet::ConstLanelet(lanelet::InvalId), s_goal, planner_data_, 7.5, 1.0);

    ASSERT_NEAR(result.length(), path.length(), m_epsilon);
  }

  {  // connection_section_length is small
    const auto result = utils::connect_path_to_goal(
      path, planner_data_.preferred_lanelets, planner_data_.goal_pose, goal_lanelet_for_path,
      s_goal, planner_data_, 0.1, 1.0);

    const auto new_goal = result.compute(result.length());
    ASSERT_NEAR(new_goal.point.pose.position.x, planner_data_.goal_pose.position.x, m_epsilon);
    ASSERT_NEAR(new_goal.point.pose.position.y, planner_data_.goal_pose.position.y, m_epsilon);
    ASSERT_NEAR(new_goal.point.pose.position.z, planner_data_.goal_pose.position.z, m_epsilon);
    ASSERT_NEAR(
      new_goal.point.pose.orientation.x, planner_data_.goal_pose.orientation.x, rad_epsilon);
    ASSERT_NEAR(
      new_goal.point.pose.orientation.y, planner_data_.goal_pose.orientation.y, rad_epsilon);
    ASSERT_NEAR(
      new_goal.point.pose.orientation.z, planner_data_.goal_pose.orientation.z, rad_epsilon);
    ASSERT_NEAR(
      new_goal.point.pose.orientation.w, planner_data_.goal_pose.orientation.w, rad_epsilon);
  }

  {  // connection_section_length is larger than distance from start to goal
    const auto result = utils::connect_path_to_goal(
      path, planner_data_.preferred_lanelets, planner_data_.goal_pose, goal_lanelet_for_path,
      s_goal, planner_data_, 100.0, 1.0);

    ASSERT_EQ(result.compute(0.0), path_.points.front());

    const auto new_goal = result.compute(result.length());
    ASSERT_NEAR(new_goal.point.pose.position.x, planner_data_.goal_pose.position.x, m_epsilon);
    ASSERT_NEAR(new_goal.point.pose.position.y, planner_data_.goal_pose.position.y, m_epsilon);
    ASSERT_NEAR(new_goal.point.pose.position.z, planner_data_.goal_pose.position.z, m_epsilon);
    ASSERT_NEAR(
      new_goal.point.pose.orientation.x, planner_data_.goal_pose.orientation.x, rad_epsilon);
    ASSERT_NEAR(
      new_goal.point.pose.orientation.y, planner_data_.goal_pose.orientation.y, rad_epsilon);
    ASSERT_NEAR(
      new_goal.point.pose.orientation.z, planner_data_.goal_pose.orientation.z, rad_epsilon);
    ASSERT_NEAR(
      new_goal.point.pose.orientation.w, planner_data_.goal_pose.orientation.w, rad_epsilon);
  }
}

TEST_F(UtilsTest, isPathInsideLanelets)
{
  {  // normal case
    const auto result = utils::is_path_inside_lanelets(
      *Trajectory::Builder{}.build(path_.points), planner_data_.route_lanelets);

    ASSERT_TRUE(result);
  }

  {  // lanelets are empty
    const auto result =
      utils::is_path_inside_lanelets(*Trajectory::Builder{}.build(path_.points), {});

    ASSERT_FALSE(result);
  }
}

TEST_F(UtilsTest, isPoseInsideLanelets)
{
  {  // normal case
    const auto pose = planner_data_.goal_pose;
    const auto lanelets = planner_data_.route_lanelets;

    const auto result = utils::is_pose_inside_lanelets(pose, lanelets);

    ASSERT_TRUE(result);
  }

  {  // pose is not in any lanelet
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;

    const auto lanelets = planner_data_.route_lanelets;

    const auto result = utils::is_pose_inside_lanelets(pose, lanelets);

    ASSERT_FALSE(result);
  }

  {  // lanelets are empty
    const auto result = utils::is_pose_inside_lanelets(geometry_msgs::msg::Pose{}, {});

    ASSERT_FALSE(result);
  }
}
}  // namespace autoware::path_generator
