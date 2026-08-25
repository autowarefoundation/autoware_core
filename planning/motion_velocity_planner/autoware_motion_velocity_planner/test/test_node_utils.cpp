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

#include "../src/node_utils.hpp"

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace
{
using autoware::motion_velocity_planner::TrafficSignalStamped;
using autoware::motion_velocity_planner::utils::process_traffic_signals;
using autoware::motion_velocity_planner::utils::resample_trajectory_by_min_interval;
using autoware::motion_velocity_planner::utils::TrafficLightIdMap;
using autoware::motion_velocity_planner::utils::TrajectoryPoints;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;

builtin_interfaces::msg::Time make_time(const int32_t sec, const uint32_t nanosec)
{
  builtin_interfaces::msg::Time t;
  t.sec = sec;
  t.nanosec = nanosec;
  return t;
}

TrafficLightGroup make_group(const int64_t id, const uint8_t color)
{
  TrafficLightGroup group;
  group.traffic_light_group_id = id;
  TrafficLightElement element;
  element.color = color;
  group.elements.push_back(element);
  return group;
}

autoware_planning_msgs::msg::TrajectoryPoint make_point(const double x, const double y)
{
  autoware_planning_msgs::msg::TrajectoryPoint p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  return p;
}
}  // namespace

// ---------------------------------------------------------------------------
// process_traffic_signals
// ---------------------------------------------------------------------------

TEST(ProcessTrafficSignals, FreshGreenObservationStoresBodyInBothMaps)
{
  TrafficLightGroupArray msg;
  msg.stamp = make_time(10, 0);
  msg.traffic_light_groups.push_back(make_group(1, TrafficLightElement::GREEN));

  const auto result = process_traffic_signals(msg, TrafficLightIdMap{});

  ASSERT_EQ(result.raw.size(), 1U);
  ASSERT_EQ(result.last_observed.size(), 1U);
  const auto & raw = result.raw.at(1);
  const auto & last = result.last_observed.at(1);
  EXPECT_EQ(raw.stamp.sec, 10);
  ASSERT_EQ(raw.signal.elements.size(), 1U);
  EXPECT_EQ(raw.signal.elements.front().color, TrafficLightElement::GREEN);
  // fresh observation: last_observed mirrors the raw observation
  EXPECT_EQ(last.stamp.sec, 10);
  ASSERT_EQ(last.signal.elements.size(), 1U);
  EXPECT_EQ(last.signal.elements.front().color, TrafficLightElement::GREEN);
}

TEST(ProcessTrafficSignals, UnknownWithPriorKeepsBodyAndRefreshesStamp)
{
  // prior observation: GREEN at t=5
  TrafficLightIdMap last_observed_old;
  {
    TrafficSignalStamped prior;
    prior.stamp = make_time(5, 0);
    prior.signal = make_group(1, TrafficLightElement::GREEN);
    last_observed_old[1] = prior;
  }

  // new observation: UNKNOWN at t=10
  TrafficLightGroupArray msg;
  msg.stamp = make_time(10, 0);
  msg.traffic_light_groups.push_back(make_group(1, TrafficLightElement::UNKNOWN));

  const auto result = process_traffic_signals(msg, last_observed_old);

  // raw always reflects the new (UNKNOWN) observation with the new stamp
  ASSERT_EQ(result.raw.size(), 1U);
  EXPECT_EQ(result.raw.at(1).stamp.sec, 10);
  ASSERT_EQ(result.raw.at(1).signal.elements.size(), 1U);
  EXPECT_EQ(result.raw.at(1).signal.elements.front().color, TrafficLightElement::UNKNOWN);

  // last_observed keeps the prior GREEN body but refreshes the stamp to the new time
  ASSERT_EQ(result.last_observed.size(), 1U);
  const auto & last = result.last_observed.at(1);
  EXPECT_EQ(last.stamp.sec, 10);
  ASSERT_EQ(last.signal.elements.size(), 1U);
  EXPECT_EQ(last.signal.elements.front().color, TrafficLightElement::GREEN);
}

TEST(ProcessTrafficSignals, UnknownWithoutPriorStoresUnknownBody)
{
  TrafficLightGroupArray msg;
  msg.stamp = make_time(10, 0);
  msg.traffic_light_groups.push_back(make_group(2, TrafficLightElement::UNKNOWN));

  const auto result = process_traffic_signals(msg, TrafficLightIdMap{});

  ASSERT_EQ(result.last_observed.size(), 1U);
  const auto & last = result.last_observed.at(2);
  EXPECT_EQ(last.stamp.sec, 10);
  ASSERT_EQ(last.signal.elements.size(), 1U);
  // no prior -> the UNKNOWN body is stored as-is
  EXPECT_EQ(last.signal.elements.front().color, TrafficLightElement::UNKNOWN);
}

TEST(ProcessTrafficSignals, EmptyMessageClearsBothMaps)
{
  TrafficLightIdMap last_observed_old;
  {
    TrafficSignalStamped prior;
    prior.stamp = make_time(5, 0);
    prior.signal = make_group(1, TrafficLightElement::GREEN);
    last_observed_old[1] = prior;
  }

  TrafficLightGroupArray msg;
  msg.stamp = make_time(10, 0);

  const auto result = process_traffic_signals(msg, last_observed_old);

  EXPECT_TRUE(result.raw.empty());
  EXPECT_TRUE(result.last_observed.empty());
}

// ---------------------------------------------------------------------------
// resample_trajectory_by_min_interval
// ---------------------------------------------------------------------------

TEST(ResampleTrajectoryByMinInterval, EmptyInputReturnsEmpty)
{
  const auto out = resample_trajectory_by_min_interval(TrajectoryPoints{}, 0.25);
  EXPECT_TRUE(out.empty());
}

TEST(ResampleTrajectoryByMinInterval, SinglePointIsKept)
{
  TrajectoryPoints in{make_point(1.0, 2.0)};
  const auto out = resample_trajectory_by_min_interval(in, 0.25);
  ASSERT_EQ(out.size(), 1U);
  EXPECT_DOUBLE_EQ(out.front().pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(out.front().pose.position.y, 2.0);
}

TEST(ResampleTrajectoryByMinInterval, AllPointsCloserThanThresholdKeepOnlyFirst)
{
  // step of 0.1 m -> squared distance 0.01 < threshold 0.25
  TrajectoryPoints in;
  for (int i = 0; i < 5; ++i) {
    in.push_back(make_point(0.1 * i, 0.0));
  }
  const auto out = resample_trajectory_by_min_interval(in, 0.5 * 0.5);
  ASSERT_EQ(out.size(), 1U);
  EXPECT_DOUBLE_EQ(out.front().pose.position.x, 0.0);
}

TEST(ResampleTrajectoryByMinInterval, KeepsPointsBeyondThresholdRelativeToLastKept)
{
  // points at x = 0.0, 0.4, 0.8, 1.2 with threshold 0.25 (=0.5^2)
  // 0.0 kept (first); 0.4 -> 0.16 < 0.25 dropped; 0.8 -> 0.64 > 0.25 kept;
  // 1.2 -> dist to last kept (0.8) is 0.16 < 0.25 dropped.
  TrajectoryPoints in{
    make_point(0.0, 0.0), make_point(0.4, 0.0), make_point(0.8, 0.0), make_point(1.2, 0.0)};
  const auto out = resample_trajectory_by_min_interval(in, 0.5 * 0.5);
  ASSERT_EQ(out.size(), 2U);
  EXPECT_DOUBLE_EQ(out[0].pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(out[1].pose.position.x, 0.8);
}

TEST(ResampleTrajectoryByMinInterval, ExactlyAtThresholdIsDropped)
{
  // squared distance exactly equal to threshold -> dropped (strict > comparison)
  TrajectoryPoints in{make_point(0.0, 0.0), make_point(0.5, 0.0)};
  const auto out = resample_trajectory_by_min_interval(in, 0.5 * 0.5);
  ASSERT_EQ(out.size(), 1U);
  EXPECT_DOUBLE_EQ(out.front().pose.position.x, 0.0);
}
