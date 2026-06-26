// Copyright 2021 Tier IV, Inc.
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

#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/state_machine.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <rcl/time.h>

#include <chrono>
#include <iostream>
#include <limits>

using StateMachine = autoware::behavior_velocity_planner::StateMachine;
using State = autoware::behavior_velocity_planner::StateMachine::State;

int enumToInt(State s)
{
  return static_cast<int>(s);
}

TEST(state_machine, on_initialized)
{
  StateMachine state_machine = StateMachine();
  EXPECT_EQ(enumToInt(State::GO), enumToInt(state_machine.getState()));
}

TEST(state_machine, set_state_stop)
{
  StateMachine state_machine = StateMachine();
  state_machine.setState(State::STOP);
  EXPECT_EQ(enumToInt(State::STOP), enumToInt(state_machine.getState()));
}

TEST(state_machine, set_state_stop_with_margin_time)
{
  StateMachine state_machine = StateMachine();
  const double margin_time = 1.0;
  state_machine.setMarginTime(margin_time);
  rclcpp::Clock current_time = rclcpp::Clock(RCL_ROS_TIME);
  rclcpp::Logger logger = rclcpp::get_logger("test_set_state_with_margin_time");
  // DO NOT SET GO until margin time past
  EXPECT_EQ(enumToInt(state_machine.getState()), enumToInt(State::GO));
  state_machine.setStateWithMarginTime(State::STOP, logger, current_time);
  // set STOP immediately when stop is set
  EXPECT_EQ(enumToInt(state_machine.getState()), enumToInt(State::STOP));
}

TEST(state_machine, set_state_go_with_margin_time)
{
  StateMachine state_machine = StateMachine();
  const double margin_time = 0.2;
  state_machine.setMarginTime(margin_time);
  rclcpp::Logger logger = rclcpp::get_logger("test_set_state_with_margin_time");
  state_machine.setState(State::STOP);
  size_t loop_counter = 0;
  // loop until state change from STOP -> GO
  while (state_machine.getState() == State::STOP) {
    EXPECT_EQ(enumToInt(state_machine.getState()), enumToInt(State::STOP));
    rclcpp::Clock current_time = rclcpp::Clock(RCL_ROS_TIME);
    if (state_machine.getDuration() > margin_time) {
      std::cerr << "stop duration is larger than margin time" << std::endl;
    }
    EXPECT_FALSE(state_machine.getDuration() > margin_time);
    state_machine.setStateWithMarginTime(State::GO, logger, current_time);
    loop_counter++;
  }
  // time past STOP -> GO
  if (loop_counter > 2) {
    EXPECT_TRUE(state_machine.getDuration() > margin_time);
    EXPECT_EQ(enumToInt(state_machine.getState()), enumToInt(State::GO));
  } else {
    std::cerr << "[Warning] computational resource is not enough" << std::endl;
  }
}

namespace
{
// Advance the ros-time override of a RCL_ROS_TIME clock to an absolute number of seconds.
void setClockSeconds(rclcpp::Clock & clock, const double seconds)
{
  const auto nanoseconds = static_cast<rcl_time_point_value_t>(seconds * 1e9);
  ASSERT_EQ(rcl_set_ros_time_override(clock.get_clock_handle(), nanoseconds), RCL_RET_OK);
}
}  // namespace

// Deterministic counterpart to set_state_go_with_margin_time: instead of busy-looping on
// wall-clock time (which self-disables its assertions when CI is fast), drive a controllable
// RCL_ROS_TIME clock so the STOP -> GO margin-time branch is asserted on every run.
TEST(state_machine, set_state_go_with_margin_time_deterministic)
{
  rclcpp::Clock clock(RCL_ROS_TIME);
  ASSERT_EQ(rcl_enable_ros_time_override(clock.get_clock_handle()), RCL_RET_OK);
  ASSERT_NO_FATAL_FAILURE(setClockSeconds(clock, 100.0));

  StateMachine state_machine = StateMachine();
  const double margin_time = 1.0;
  state_machine.setMarginTime(margin_time);
  rclcpp::Logger logger = rclcpp::get_logger("test_set_state_with_margin_time_deterministic");

  state_machine.setState(State::STOP);
  ASSERT_EQ(enumToInt(state_machine.getState()), enumToInt(State::STOP));

  // First GO request only arms the timer; state stays STOP and duration is still zero.
  state_machine.setStateWithMarginTime(State::GO, logger, clock);
  EXPECT_EQ(enumToInt(state_machine.getState()), enumToInt(State::STOP));
  EXPECT_DOUBLE_EQ(state_machine.getDuration(), 0.0);

  // Re-request GO after less than the margin time: still STOP, duration recorded but below margin.
  ASSERT_NO_FATAL_FAILURE(setClockSeconds(clock, 100.5));
  state_machine.setStateWithMarginTime(State::GO, logger, clock);
  EXPECT_EQ(enumToInt(state_machine.getState()), enumToInt(State::STOP));
  EXPECT_NEAR(state_machine.getDuration(), 0.5, 1e-6);
  EXPECT_LE(state_machine.getDuration(), margin_time);

  // Re-request GO after more than the margin time: the STOP -> GO transition must fire.
  ASSERT_NO_FATAL_FAILURE(setClockSeconds(clock, 101.5));
  state_machine.setStateWithMarginTime(State::GO, logger, clock);
  EXPECT_EQ(enumToInt(state_machine.getState()), enumToInt(State::GO));
  EXPECT_GT(state_machine.getDuration(), margin_time);
}

// A STOP request received while waiting in STOP must reset the margin timer so a subsequent
// GO has to wait the full margin again (start_time_ is cleared on same-state requests).
TEST(state_machine, stop_request_resets_margin_timer)
{
  rclcpp::Clock clock(RCL_ROS_TIME);
  ASSERT_EQ(rcl_enable_ros_time_override(clock.get_clock_handle()), RCL_RET_OK);
  ASSERT_NO_FATAL_FAILURE(setClockSeconds(clock, 0.0));

  StateMachine state_machine = StateMachine();
  const double margin_time = 1.0;
  state_machine.setMarginTime(margin_time);
  rclcpp::Logger logger = rclcpp::get_logger("test_stop_request_resets_margin_timer");

  state_machine.setState(State::STOP);

  // Arm the timer with a first GO request at t = 0.
  state_machine.setStateWithMarginTime(State::GO, logger, clock);

  // A STOP request at t = 0.5 resets the timer (same-state STOP clears start_time_).
  ASSERT_NO_FATAL_FAILURE(setClockSeconds(clock, 0.5));
  state_machine.setStateWithMarginTime(State::STOP, logger, clock);
  EXPECT_EQ(enumToInt(state_machine.getState()), enumToInt(State::STOP));

  // Even at t = 1.2 (> margin since the first GO), the very next GO only re-arms the timer,
  // so the machine is still STOP because the timer was reset.
  ASSERT_NO_FATAL_FAILURE(setClockSeconds(clock, 1.2));
  state_machine.setStateWithMarginTime(State::GO, logger, clock);
  EXPECT_EQ(enumToInt(state_machine.getState()), enumToInt(State::STOP));

  // Only after another full margin elapses does it finally transition to GO.
  ASSERT_NO_FATAL_FAILURE(setClockSeconds(clock, 2.3));
  state_machine.setStateWithMarginTime(State::GO, logger, clock);
  EXPECT_EQ(enumToInt(state_machine.getState()), enumToInt(State::GO));
}
