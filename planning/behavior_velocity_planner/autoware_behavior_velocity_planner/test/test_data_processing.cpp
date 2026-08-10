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

#include "data_processing.hpp"

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <deque>

namespace autoware::behavior_velocity_planner::data_processing
{
namespace
{
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;

builtin_interfaces::msg::Time make_time(const int32_t sec, const uint32_t nanosec = 0U)
{
  builtin_interfaces::msg::Time t;
  t.sec = sec;
  t.nanosec = nanosec;
  return t;
}

TrafficLightGroupArray make_array(
  const builtin_interfaces::msg::Time & stamp, const lanelet::Id id, const uint8_t color)
{
  TrafficLightGroupArray array;
  array.stamp = stamp;
  TrafficLightGroup group;
  group.traffic_light_group_id = id;
  TrafficLightElement element;
  element.color = color;
  group.elements.push_back(element);
  array.traffic_light_groups.push_back(group);
  return array;
}

geometry_msgs::msg::TwistStamped make_twist(
  const builtin_interfaces::msg::Time & stamp, const double vx)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.stamp = stamp;
  twist.twist.linear.x = vx;
  return twist;
}

// The buffer stamps are message timestamps, which rclcpp interprets as RCL_ROS_TIME when compared
// against the reference time. Build the reference "now" with the same clock source so the
// comparison does not throw "can't compare times with different time sources".
rclcpp::Time make_ros_time(const int32_t sec, const uint32_t nanosec = 0U)
{
  return rclcpp::Time(sec, nanosec, RCL_ROS_TIME);
}
}  // namespace

// A non-UNKNOWN observation is stored verbatim in both maps.
TEST(ApplyTrafficSignals, NonUnknownStoredVerbatim)
{
  const auto msg = make_array(make_time(5), 100, TrafficLightElement::GREEN);

  const auto result = apply_traffic_signals({}, msg);

  ASSERT_EQ(result.raw.size(), 1U);
  ASSERT_EQ(result.last_observed.size(), 1U);
  ASSERT_EQ(result.raw.count(100), 1U);
  ASSERT_EQ(result.last_observed.count(100), 1U);
  EXPECT_EQ(result.raw.at(100).stamp.sec, 5);
  ASSERT_EQ(result.raw.at(100).signal.elements.size(), 1U);
  EXPECT_EQ(result.raw.at(100).signal.elements.front().color, TrafficLightElement::GREEN);
  EXPECT_EQ(result.last_observed.at(100).signal.elements.front().color, TrafficLightElement::GREEN);
  EXPECT_EQ(result.last_observed.at(100).stamp.sec, 5);
}

// The very first observation being UNKNOWN is stored as-is (no prior to keep).
TEST(ApplyTrafficSignals, FirstUnknownStoredAsIs)
{
  const auto msg = make_array(make_time(5), 100, TrafficLightElement::UNKNOWN);

  const auto result = apply_traffic_signals({}, msg);

  ASSERT_EQ(result.last_observed.count(100), 1U);
  ASSERT_EQ(result.last_observed.at(100).signal.elements.size(), 1U);
  EXPECT_EQ(
    result.last_observed.at(100).signal.elements.front().color, TrafficLightElement::UNKNOWN);
  EXPECT_EQ(result.last_observed.at(100).stamp.sec, 5);
}

// An UNKNOWN observation with a prior keeps the prior body but refreshes the timestamp.
TEST(ApplyTrafficSignals, UnknownKeepsLastObservationRefreshesStamp)
{
  // Establish a GREEN observation at t=5.
  const auto first =
    apply_traffic_signals({}, make_array(make_time(5), 100, TrafficLightElement::GREEN));

  // Then observe UNKNOWN at t=8.
  const auto msg = make_array(make_time(8), 100, TrafficLightElement::UNKNOWN);
  const auto result = apply_traffic_signals(first.last_observed, msg);

  // raw mirrors the latest (UNKNOWN) observation verbatim.
  ASSERT_EQ(result.raw.count(100), 1U);
  EXPECT_EQ(result.raw.at(100).signal.elements.front().color, TrafficLightElement::UNKNOWN);
  EXPECT_EQ(result.raw.at(100).stamp.sec, 8);

  // last_observed keeps the GREEN body but refreshes the stamp to t=8.
  ASSERT_EQ(result.last_observed.count(100), 1U);
  EXPECT_EQ(result.last_observed.at(100).signal.elements.front().color, TrafficLightElement::GREEN);
  EXPECT_EQ(result.last_observed.at(100).stamp.sec, 8);
}

// Stale entries (older than velocity_buffer_time_sec) are pruned from the back.
TEST(PruneVelocityBuffer, RemovesStaleEntriesFromBack)
{
  std::deque<geometry_msgs::msg::TwistStamped> buffer;
  // newest at front, oldest at back
  buffer.push_back(make_twist(make_time(100), 1.0));  // age 0 -> kept
  buffer.push_back(make_twist(make_time(80), 2.0));   // age 20 -> stale
  buffer.push_back(make_twist(make_time(50), 3.0));   // age 50 -> stale

  // velocity_buffer_time_sec is 10.0
  prune_velocity_buffer(buffer, make_ros_time(100));

  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_DOUBLE_EQ(buffer.front().twist.linear.x, 1.0);
  EXPECT_EQ(buffer.front().header.stamp.sec, 100);
}

// An entry exactly at the threshold age is retained (boundary <=).
TEST(PruneVelocityBuffer, KeepsEntryAtExactThreshold)
{
  std::deque<geometry_msgs::msg::TwistStamped> buffer;
  buffer.push_back(
    make_twist(make_time(static_cast<int32_t>(110 - PlannerData::velocity_buffer_time_sec)), 7.0));

  prune_velocity_buffer(buffer, make_ros_time(110));

  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_DOUBLE_EQ(buffer.front().twist.linear.x, 7.0);
}

// An entry stamped in the future relative to now is treated as age 0 and retained
// (the negative-time guard prevents an exception).
TEST(PruneVelocityBuffer, KeepsFutureStampedEntryWithoutThrowing)
{
  std::deque<geometry_msgs::msg::TwistStamped> buffer;
  buffer.push_back(make_twist(make_time(200), 9.0));  // future relative to now=100

  EXPECT_NO_THROW(prune_velocity_buffer(buffer, make_ros_time(100)));

  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_DOUBLE_EQ(buffer.front().twist.linear.x, 9.0);
}

// An empty buffer is left untouched.
TEST(PruneVelocityBuffer, EmptyBufferStaysEmpty)
{
  std::deque<geometry_msgs::msg::TwistStamped> buffer;
  prune_velocity_buffer(buffer, make_ros_time(100));
  EXPECT_TRUE(buffer.empty());
}
}  // namespace autoware::behavior_velocity_planner::data_processing
