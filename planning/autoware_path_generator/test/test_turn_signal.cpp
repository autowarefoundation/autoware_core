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

#include <lanelet2_core/geometry/Lanelet.h>

#include <string>
#include <tuple>
#include <vector>

namespace autoware::path_generator
{
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

struct GetTurnSignalTestParam
{
  struct PositionOnCenterline
  {
    std::vector<lanelet::Id> lane_ids;
    double arc_length;
  };

  std::string description;
  double current_vel;
  double search_distance;
  double search_time;
  double angle_threshold_deg;
  double base_link_to_front;
  PositionOnCenterline current_position;
  uint8_t expected_turn_signal;
};

std::ostream & operator<<(std::ostream & os, const GetTurnSignalTestParam & p)
{
  return os << p.description;
}

struct GetTurnSignalTest : public UtilsTest,
                           public ::testing::WithParamInterface<GetTurnSignalTestParam>
{
  void SetUp() override
  {
    UtilsTest::SetUp();

    set_map("autoware_test_utils", "consecutive_turn/lanelet2_map.osm");

    const auto route_lanelets = get_lanelets_from_ids({500, 501, 489, 497, 494});
    planner_data_.route_lanelets = route_lanelets;
    planner_data_.preferred_lanelets = route_lanelets;
    planner_data_.start_lanelets = {route_lanelets.front()};
    planner_data_.goal_lanelets = {route_lanelets.back()};

    path_ = PathWithLaneId{};
    for (auto it = route_lanelets.begin(); it != route_lanelets.end(); ++it) {
      path_.points.insert(
        path_.points.end(), it->centerline().size() - 1,
        PathPointWithLaneId{}.set__lane_ids({it->id()}));
      if (it != std::prev(route_lanelets.end())) {
        path_.points.back().lane_ids.push_back(std::next(it)->id());
      }
    }
  }
};

TEST_P(GetTurnSignalTest, getTurnSignal)
{
  const auto & p = GetParam();

  const auto current_position = lanelet::geometry::interpolatedPointAtDistance(
    lanelet::LaneletSequence(get_lanelets_from_ids(p.current_position.lane_ids)).centerline2d(),
    p.current_position.arc_length);

  geometry_msgs::msg::Pose current_pose;
  current_pose.position.x = current_position.x();
  current_pose.position.y = current_position.y();

  const auto result = utils::get_turn_signal(
    path_, planner_data_, current_pose, p.current_vel, p.search_distance, p.search_time,
    p.angle_threshold_deg, p.base_link_to_front);

  ASSERT_EQ(result.command, p.expected_turn_signal);
}

INSTANTIATE_TEST_SUITE_P(
  , GetTurnSignalTest,
  ::testing::Values(
    GetTurnSignalTestParam{
      "EgoIsStoppingAndBeforeFirstDesiredSection",
      0.0,
      30.0,
      3.0,
      15.0,
      3.79,
      {{500}, -40.0},
      TurnIndicatorsCommand::NO_COMMAND},
    GetTurnSignalTestParam{
      "EgoIsStoppingAndInFirstDesiredSection",
      0.0,
      30.0,
      3.0,
      15.0,
      3.79,
      {{500}, -20.0},
      TurnIndicatorsCommand::ENABLE_RIGHT},
    GetTurnSignalTestParam{
      "EgoIsMovingAndAheadOfFirstDesiredSection",
      3.5,
      30.0,
      3.0,
      15.0,
      3.79,
      {{500}, -40.0},
      TurnIndicatorsCommand::ENABLE_RIGHT},
    GetTurnSignalTestParam{
      "EgoIsInFirstRequiredSection",
      0.0,
      30.0,
      3.0,
      15.0,
      3.79,
      {{501}, 1.0},
      TurnIndicatorsCommand::ENABLE_RIGHT},
    GetTurnSignalTestParam{
      "EgoIsInConflictedDesiredSection",
      0.0,
      30.0,
      3.0,
      15.0,
      3.79,
      {{501}, -1.0},
      TurnIndicatorsCommand::ENABLE_LEFT},
    GetTurnSignalTestParam{
      "EgoIsInSecondRequiredSection",
      0.0,
      30.0,
      3.0,
      15.0,
      3.79,
      {{497}, 1.0},
      TurnIndicatorsCommand::ENABLE_LEFT},
    GetTurnSignalTestParam{
      "EgoIsInSecondDesiredSection",
      0.0,
      30.0,
      3.0,
      15.0,
      3.79,
      {{497}, -1.0},
      TurnIndicatorsCommand::ENABLE_LEFT},
    GetTurnSignalTestParam{
      "EgoIsOutsideSecondDesiredSection",
      0.0,
      30.0,
      3.0,
      15.0,
      3.79,
      {{494}, 1.0},
      TurnIndicatorsCommand::NO_COMMAND}),
  ::testing::PrintToStringParamName{});

TEST_F(UtilsTest, getTurnSignalRequiredEndPoint)
{
  constexpr lanelet::Id lane_id = 50;
  constexpr double angle_threshold_deg = 15.0;

  const auto result = utils::get_turn_signal_required_end_point(
    planner_data_.lanelet_map_ptr->laneletLayer.get(lane_id), angle_threshold_deg);

  ASSERT_TRUE(result);

  constexpr double epsilon = 0.1;
  EXPECT_NEAR(result.value().x(), 3760.894, epsilon);
  EXPECT_NEAR(result.value().y(), 73749.359, epsilon);
}
}  // namespace autoware::path_generator
