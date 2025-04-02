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

namespace autoware::path_generator
{
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

struct GetTurnSignalTestParam
{
  std::string description;
  double current_vel;
  double search_distance;
  double search_time;
  double angle_threshold_deg;
  double base_link_to_front;
  std::tuple<std::vector<lanelet::Id>, double> current_position_locator;
  uint8_t expected_turn_signal;
};

std::ostream & operator<<(std::ostream & os, const GetTurnSignalTestParam & p)
{
  return os << p.description;
}

struct GetTurnSignalTest : public UtilsTest,
                           public ::testing::WithParamInterface<GetTurnSignalTestParam>
{
};

TEST_P(GetTurnSignalTest, getTurnSignal)
{
  const auto & p = GetParam();

  geometry_msgs::msg::Pose current_pose;
  {
    const auto & [lane_ids, arc_length] = p.current_position_locator;

    lanelet::ConstLanelets lanelets;
    for (const auto & lane_id : lane_ids) {
      lanelets.push_back(planner_data_.lanelet_map_ptr->laneletLayer.get(lane_id));
    }

    const auto current_position = lanelet::geometry::interpolatedPointAtDistance(
      lanelet::LaneletSequence(lanelets).centerline2d(), arc_length);

    current_pose.position.x = current_position.x();
    current_pose.position.y = current_position.y();
  }

  const auto result = utils::get_turn_signal(
    path_, planner_data_, current_pose, p.current_vel, p.search_distance, p.search_time,
    p.angle_threshold_deg, p.base_link_to_front);

  ASSERT_EQ(result.command, p.expected_turn_signal);
}

INSTANTIATE_TEST_SUITE_P(
  , GetTurnSignalTest,
  ::testing::Values(
    GetTurnSignalTestParam{
      "EgoIsStoppingAndBeforeDesiredStartPoint",
      0.0,
      30.0,
      3.0,
      15.0,
      3.79,
      {{125}, -40.0},
      TurnIndicatorsCommand::NO_COMMAND},
    GetTurnSignalTestParam{
      "EgoIsStoppingAndAheadOfDesiredStartPoint",
      0.0,
      30.0,
      3.0,
      15.0,
      3.79,
      {{125}, -20.0},
      TurnIndicatorsCommand::ENABLE_RIGHT},
    GetTurnSignalTestParam{
      "EgoIsMovingAndAheadOfDesiredStartPoint",
      3.5,
      30.0,
      3.0,
      15.0,
      3.79,
      {{125}, -40.0},
      TurnIndicatorsCommand::ENABLE_RIGHT},
    GetTurnSignalTestParam{
      "EgoIsInRequiredSection",
      0.0,
      30.0,
      3.0,
      15.0,
      3.79,
      {{50}, -10.0},
      TurnIndicatorsCommand::ENABLE_RIGHT},
    GetTurnSignalTestParam{
      "EgoIsInDesiredSection",
      0.0,
      30.0,
      3.0,
      15.0,
      3.79,
      {{50}, -1.0},
      TurnIndicatorsCommand::ENABLE_RIGHT},
    GetTurnSignalTestParam{
      "EgoIsAheadOfDesiredEndPoint",
      0.0,
      30.0,
      3.0,
      15.0,
      3.79,
      {{122}, 1.0},
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
