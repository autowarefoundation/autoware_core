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

using autoware::stop_filter::FilterResult;
using autoware::stop_filter::StopFilter;
using autoware::stop_filter::Vector3D;

TEST(StopFilterTest, FilterStopWhenStopped)
{
  StopFilter filter(0.1, 0.1);
  Vector3D linear_velocity = {0.05, 0.0, 0.0};
  Vector3D angular_velocity = {0.0, 0.0, 0.05};

  FilterResult result = filter.apply_stop_filter(linear_velocity, angular_velocity);

  EXPECT_TRUE(result.was_stopped);
  EXPECT_EQ(result.linear_velocity.x, 0.0);
  EXPECT_EQ(result.linear_velocity.y, 0.0);
  EXPECT_EQ(result.linear_velocity.z, 0.0);
  EXPECT_EQ(result.angular_velocity.x, 0.0);
  EXPECT_EQ(result.angular_velocity.y, 0.0);
  EXPECT_EQ(result.angular_velocity.z, 0.0);
}

TEST(StopFilterTest, FilterStopWhenNotStopped)
{
  StopFilter filter(0.1, 0.1);
  Vector3D linear_velocity = {0.2, 0.0, 0.0};
  Vector3D angular_velocity = {0.0, 0.0, 0.2};

  FilterResult result = filter.apply_stop_filter(linear_velocity, angular_velocity);

  EXPECT_FALSE(result.was_stopped);
  EXPECT_EQ(result.linear_velocity.x, 0.2);
  EXPECT_EQ(result.linear_velocity.y, 0.0);
  EXPECT_EQ(result.linear_velocity.z, 0.0);
  EXPECT_EQ(result.angular_velocity.x, 0.0);
  EXPECT_EQ(result.angular_velocity.y, 0.0);
  EXPECT_EQ(result.angular_velocity.z, 0.2);
}

TEST(StopFilterTest, FilterStopOnlyLinearVelocityBelowThreshold)
{
  StopFilter filter(0.1, 0.1);
  Vector3D linear_velocity = {0.05, 0.0, 0.0};
  Vector3D angular_velocity = {0.0, 0.0, 0.2};

  FilterResult result = filter.apply_stop_filter(linear_velocity, angular_velocity);

  EXPECT_FALSE(result.was_stopped);
  EXPECT_EQ(result.linear_velocity.x, 0.05);
  EXPECT_EQ(result.angular_velocity.z, 0.2);
}

TEST(StopFilterTest, FilterStopOnlyAngularVelocityBelowThreshold)
{
  StopFilter filter(0.1, 0.1);
  Vector3D linear_velocity = {0.2, 0.0, 0.0};
  Vector3D angular_velocity = {0.0, 0.0, 0.05};

  FilterResult result = filter.apply_stop_filter(linear_velocity, angular_velocity);

  EXPECT_FALSE(result.was_stopped);
  EXPECT_EQ(result.linear_velocity.x, 0.2);
  EXPECT_EQ(result.angular_velocity.z, 0.05);
}
