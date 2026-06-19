// Copyright 2025 TIER IV
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

#include "../src/stop_filter.hpp"

#include <gtest/gtest.h>

#include <memory>

using autoware::stop_filter::StopFilter;

namespace
{
// Build an odometry sample from its six twist components. The stamp and frame are fixed so the
// filter's forwarding of the header can be asserted.
nav_msgs::msg::Odometry::SharedPtr create_odometry_message(
  const double linear_x, const double linear_y, const double linear_z, const double angular_x,
  const double angular_y, const double angular_z)
{
  auto msg = std::make_shared<nav_msgs::msg::Odometry>();
  msg->header.frame_id = "base_link";
  msg->header.stamp.sec = 1;
  msg->header.stamp.nanosec = 500000000;
  msg->twist.twist.linear.x = linear_x;
  msg->twist.twist.linear.y = linear_y;
  msg->twist.twist.linear.z = linear_z;
  msg->twist.twist.angular.x = angular_x;
  msg->twist.twist.angular.y = angular_y;
  msg->twist.twist.angular.z = angular_z;
  return msg;
}
}  // namespace

// When both linear-x and angular-z exceed the thresholds the vehicle is moving, so the stop flag
// is false. The input timestamp is forwarded onto the flag message.
TEST(StopFilterTest, StopFlagIsFalseWhenMoving)
{
  StopFilter filter(0.1, 0.1);
  const auto input = create_odometry_message(0.2, 0.0, 0.0, 0.0, 0.0, 0.2);

  const auto stop_flag_msg = filter.create_stop_flag_msg(input);

  EXPECT_FALSE(stop_flag_msg.data);
  EXPECT_EQ(stop_flag_msg.stamp, input->header.stamp);
}

// When both linear-x and angular-z are below the thresholds the vehicle is stopped, so the stop
// flag is true.
TEST(StopFilterTest, StopFlagIsTrueWhenStopped)
{
  StopFilter filter(0.1, 0.1);
  const auto input = create_odometry_message(0.05, 0.0, 0.0, 0.0, 0.0, 0.05);

  const auto stop_flag_msg = filter.create_stop_flag_msg(input);

  EXPECT_TRUE(stop_flag_msg.data);
  EXPECT_EQ(stop_flag_msg.stamp, input->header.stamp);
}

// A stop requires both axes below threshold: linear-x alone below threshold is still moving, so the
// twist is preserved.
TEST(StopFilterTest, NotStoppedWhenOnlyLinearVelocityBelowThreshold)
{
  StopFilter filter(0.1, 0.1);
  const auto input = create_odometry_message(0.05, 0.0, 0.0, 0.0, 0.0, 0.2);

  EXPECT_FALSE(filter.create_stop_flag_msg(input).data);
}

// Symmetrically, angular-z alone below threshold is still moving, so the twist is preserved.
TEST(StopFilterTest, NotStoppedWhenOnlyAngularVelocityBelowThreshold)
{
  StopFilter filter(0.1, 0.1);
  const auto input = create_odometry_message(0.2, 0.0, 0.0, 0.0, 0.0, 0.05);

  EXPECT_FALSE(filter.create_stop_flag_msg(input).data);
}

// On a stop every twist component is zeroed while the header is preserved.
TEST(StopFilterTest, FilteredMsgZeroesTwistWhenStopped)
{
  StopFilter filter(0.1, 0.1);
  const auto input = create_odometry_message(0.05, 0.02, 0.01, 0.03, 0.04, 0.05);

  const auto filtered_msg = filter.create_filtered_msg(input);

  EXPECT_EQ(filtered_msg.twist.twist.linear.x, 0.0);
  EXPECT_EQ(filtered_msg.twist.twist.linear.y, 0.0);
  EXPECT_EQ(filtered_msg.twist.twist.linear.z, 0.0);
  EXPECT_EQ(filtered_msg.twist.twist.angular.x, 0.0);
  EXPECT_EQ(filtered_msg.twist.twist.angular.y, 0.0);
  EXPECT_EQ(filtered_msg.twist.twist.angular.z, 0.0);

  EXPECT_EQ(filtered_msg.header.frame_id, input->header.frame_id);
  EXPECT_EQ(filtered_msg.header.stamp, input->header.stamp);
}

// When moving the twist passes through unchanged.
TEST(StopFilterTest, FilteredMsgPreservesTwistWhenMoving)
{
  StopFilter filter(0.1, 0.1);
  const auto input = create_odometry_message(0.2, 0.0, 0.0, 0.0, 0.0, 0.2);

  const auto filtered_msg = filter.create_filtered_msg(input);

  EXPECT_EQ(filtered_msg.twist.twist.linear.x, 0.2);
  EXPECT_EQ(filtered_msg.twist.twist.angular.z, 0.2);
}
