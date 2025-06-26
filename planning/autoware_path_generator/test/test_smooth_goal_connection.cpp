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

#include <gtest/gtest.h>

namespace autoware::path_generator
{
using Trajectory = experimental::trajectory::Trajectory<PathPointWithLaneId>;

TEST_F(UtilsTest, modifyPathForSmoothGoalConnection)
{
  constexpr auto epsilon = 1e-3;
  const auto path = *Trajectory::Builder{}.build(path_.points);

  {  // normal case
    const auto result =
      utils::modify_path_for_smooth_goal_connection(path, planner_data_, 7.5, 1.0);

    ASSERT_TRUE(result.has_value());

    const auto new_goal = result->compute(result->length());
    EXPECT_NEAR(new_goal.point.pose.position.x, planner_data_.goal_pose.position.x, epsilon);
    EXPECT_NEAR(new_goal.point.pose.position.y, planner_data_.goal_pose.position.y, epsilon);
    EXPECT_NEAR(new_goal.point.pose.position.z, planner_data_.goal_pose.position.z, epsilon);
    EXPECT_NEAR(new_goal.point.pose.orientation.x, planner_data_.goal_pose.orientation.x, epsilon);
    EXPECT_NEAR(new_goal.point.pose.orientation.y, planner_data_.goal_pose.orientation.y, epsilon);
    EXPECT_NEAR(new_goal.point.pose.orientation.z, planner_data_.goal_pose.orientation.z, epsilon);
    EXPECT_NEAR(new_goal.point.pose.orientation.w, planner_data_.goal_pose.orientation.w, epsilon);
  }

  {  // input path is empty
    const auto result = utils::modify_path_for_smooth_goal_connection({}, planner_data_, {}, {});

    ASSERT_FALSE(result.has_value());
  }

  {  // planner data is not set
    const auto result = utils::modify_path_for_smooth_goal_connection(path, {}, {}, {});

    ASSERT_FALSE(result.has_value());
  }

  {  // search_radius_range is zero
    const auto result = utils::modify_path_for_smooth_goal_connection(path, planner_data_, 0.0, {});

    ASSERT_FALSE(result.has_value());
  }
}

TEST_F(UtilsTest, refinePathForGoal)
{
  constexpr auto epsilon = 1e-3;
  const auto path = *Trajectory::Builder{}.build(path_.points);

  {  // normal case
    const auto result = utils::refine_path_for_goal(
      path, planner_data_.goal_pose, planner_data_.preferred_lanelets.back().id(), 7.5, 1.0);

    const auto new_goal = result.compute(result.length());
    EXPECT_NEAR(new_goal.point.pose.position.x, planner_data_.goal_pose.position.x, epsilon);
    EXPECT_NEAR(new_goal.point.pose.position.y, planner_data_.goal_pose.position.y, epsilon);
    EXPECT_NEAR(new_goal.point.pose.position.z, planner_data_.goal_pose.position.z, epsilon);
    EXPECT_NEAR(new_goal.point.pose.orientation.x, planner_data_.goal_pose.orientation.x, epsilon);
    EXPECT_NEAR(new_goal.point.pose.orientation.y, planner_data_.goal_pose.orientation.y, epsilon);
    EXPECT_NEAR(new_goal.point.pose.orientation.z, planner_data_.goal_pose.orientation.z, epsilon);
    EXPECT_NEAR(new_goal.point.pose.orientation.w, planner_data_.goal_pose.orientation.w, epsilon);
  }

  {  // goal lane id is invalid
    const auto result =
      utils::refine_path_for_goal(path, planner_data_.goal_pose, lanelet::InvalId, {}, {});

    EXPECT_NEAR(result.length(), path.length(), epsilon);
  }

  {  // search_radius_range is small
    const auto result = utils::refine_path_for_goal(
      path, planner_data_.goal_pose, planner_data_.preferred_lanelets.back().id(), 0.1, 1.0);

    const auto new_goal = result.compute(result.length());
    EXPECT_NEAR(new_goal.point.pose.position.x, planner_data_.goal_pose.position.x, epsilon);
    EXPECT_NEAR(new_goal.point.pose.position.y, planner_data_.goal_pose.position.y, epsilon);
    EXPECT_NEAR(new_goal.point.pose.position.z, planner_data_.goal_pose.position.z, epsilon);
    EXPECT_NEAR(new_goal.point.pose.orientation.x, planner_data_.goal_pose.orientation.x, epsilon);
    EXPECT_NEAR(new_goal.point.pose.orientation.y, planner_data_.goal_pose.orientation.y, epsilon);
    EXPECT_NEAR(new_goal.point.pose.orientation.z, planner_data_.goal_pose.orientation.z, epsilon);
    EXPECT_NEAR(new_goal.point.pose.orientation.w, planner_data_.goal_pose.orientation.w, epsilon);
  }

  {  // search_radius_range is larger than distance from start to goal
    const auto result = utils::refine_path_for_goal(
      path, planner_data_.goal_pose, planner_data_.preferred_lanelets.back().id(), 100.0, 1.0);

    EXPECT_EQ(result.compute(0.0), path_.points.front());

    const auto new_goal = result.compute(result.length());
    EXPECT_NEAR(new_goal.point.pose.position.x, planner_data_.goal_pose.position.x, epsilon);
    EXPECT_NEAR(new_goal.point.pose.position.y, planner_data_.goal_pose.position.y, epsilon);
    EXPECT_NEAR(new_goal.point.pose.position.z, planner_data_.goal_pose.position.z, epsilon);
    EXPECT_NEAR(new_goal.point.pose.orientation.x, planner_data_.goal_pose.orientation.x, epsilon);
    EXPECT_NEAR(new_goal.point.pose.orientation.y, planner_data_.goal_pose.orientation.y, epsilon);
    EXPECT_NEAR(new_goal.point.pose.orientation.z, planner_data_.goal_pose.orientation.z, epsilon);
    EXPECT_NEAR(new_goal.point.pose.orientation.w, planner_data_.goal_pose.orientation.w, epsilon);
  }
}

TEST_F(UtilsTest, extractLaneletsFromTrajectory)
{
  {  // normal case
    const auto result = utils::extract_lanelets_from_trajectory(
      *Trajectory::Builder{}.build(path_.points), planner_data_);

    ASSERT_EQ(result.size(), 4);
    ASSERT_EQ(result[0].id(), 50);
    ASSERT_EQ(result[1].id(), 122);
    ASSERT_EQ(result[2].id(), 125);
    ASSERT_EQ(result[3].id(), 10323);
  }

  {  // trajectory is empty
    const auto result = utils::extract_lanelets_from_trajectory({}, planner_data_);

    ASSERT_TRUE(result.empty());
  }
}

TEST_F(UtilsTest, isTrajectoryInsideLanelets)
{
  {  // normal case
    const auto result = utils::is_trajectory_inside_lanelets(
      *Trajectory::Builder{}.build(path_.points), planner_data_.route_lanelets);

    ASSERT_TRUE(result);
  }

  {  // lanelets are empty
    const auto result =
      utils::is_trajectory_inside_lanelets(*Trajectory::Builder{}.build(path_.points), {});

    ASSERT_FALSE(result);
  }
}

TEST_F(UtilsTest, isInLanelets)
{
  {  // normal case
    const auto pose = planner_data_.goal_pose;
    const auto lanelets = planner_data_.route_lanelets;

    const auto result = utils::is_in_lanelets(pose, lanelets);

    ASSERT_TRUE(result);
  }

  {  // pose is not in any lanelet
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;

    const auto lanelets = planner_data_.route_lanelets;

    const auto result = utils::is_in_lanelets(pose, lanelets);

    ASSERT_FALSE(result);
  }

  {  // lanelets are empty
    const auto result = utils::is_in_lanelets(geometry_msgs::msg::Pose{}, {});

    ASSERT_FALSE(result);
  }
}
}  // namespace autoware::path_generator
