// Copyright 2021 TierIV
// Copyright 2025 The Autoware Contributors
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

#include "autoware/vehicle_velocity_converter/vehicle_velocity_converter.hpp"

#include <gtest/gtest.h>

using autoware::vehicle_velocity_converter::VehicleVelocityConverter;
using autoware_vehicle_msgs::msg::VelocityReport;

TEST(TestVehicleVelocityConverter, BasicConversion)
{
  VehicleVelocityConverter converter("base_link", 0.2, 0.1, 1.0);

  VelocityReport velocity_report;
  velocity_report.header.stamp.sec = 123;
  velocity_report.header.stamp.nanosec = 456000000;
  velocity_report.header.frame_id = "base_link";
  velocity_report.longitudinal_velocity = 10.0f;
  velocity_report.lateral_velocity = 0.5f;
  velocity_report.heading_rate = 0.1f;

  const auto result = converter.convert(velocity_report);

  // Check frame ID match
  EXPECT_TRUE(result.frame_id_matched);

  // Check header
  EXPECT_EQ(result.twist_with_covariance.header.frame_id, "base_link");

  // Check velocities (use EXPECT_FLOAT_EQ because VelocityReport fields are float)
  EXPECT_FLOAT_EQ(result.twist_with_covariance.twist.twist.linear.x, 10.0f);
  EXPECT_FLOAT_EQ(result.twist_with_covariance.twist.twist.linear.y, 0.5f);
  EXPECT_FLOAT_EQ(result.twist_with_covariance.twist.twist.angular.z, 0.1f);

  // Check covariance diagonal elements
  EXPECT_DOUBLE_EQ(result.twist_with_covariance.twist.covariance[0 + 0 * 6], 0.2 * 0.2);
  EXPECT_DOUBLE_EQ(result.twist_with_covariance.twist.covariance[1 + 1 * 6], 10000.0);
  EXPECT_DOUBLE_EQ(result.twist_with_covariance.twist.covariance[2 + 2 * 6], 10000.0);
  EXPECT_DOUBLE_EQ(result.twist_with_covariance.twist.covariance[3 + 3 * 6], 10000.0);
  EXPECT_DOUBLE_EQ(result.twist_with_covariance.twist.covariance[4 + 4 * 6], 10000.0);
  EXPECT_DOUBLE_EQ(result.twist_with_covariance.twist.covariance[5 + 5 * 6], 0.1 * 0.1);
}

TEST(TestVehicleVelocityConverter, SpeedScaleFactor)
{
  VehicleVelocityConverter converter("base_link", 0.2, 0.1, 1.5);

  VelocityReport velocity_report;
  velocity_report.header.frame_id = "base_link";
  velocity_report.longitudinal_velocity = 2.0f;
  velocity_report.lateral_velocity = 0.1f;
  velocity_report.heading_rate = 0.3f;

  const auto result = converter.convert(velocity_report);

  // Check that longitudinal velocity is scaled
  EXPECT_FLOAT_EQ(result.twist_with_covariance.twist.twist.linear.x, 2.0f * 1.5f);
  // Lateral velocity should not be scaled
  EXPECT_FLOAT_EQ(result.twist_with_covariance.twist.twist.linear.y, 0.1f);
  // Angular velocity should not be scaled
  EXPECT_FLOAT_EQ(result.twist_with_covariance.twist.twist.angular.z, 0.3f);
}

TEST(TestVehicleVelocityConverter, FrameIdMismatch)
{
  VehicleVelocityConverter converter("base_link", 0.2, 0.1, 1.0);

  VelocityReport velocity_report;
  velocity_report.header.frame_id = "different_frame";  // Not base_link
  velocity_report.longitudinal_velocity = 5.0f;
  velocity_report.lateral_velocity = 0.0f;
  velocity_report.heading_rate = 0.0f;

  const auto result = converter.convert(velocity_report);

  // Frame ID should not match
  EXPECT_FALSE(result.frame_id_matched);

  // Conversion should still work
  EXPECT_EQ(result.twist_with_covariance.header.frame_id, "different_frame");
  EXPECT_FLOAT_EQ(result.twist_with_covariance.twist.twist.linear.x, 5.0f);
}

TEST(TestVehicleVelocityConverter, ZeroVelocity)
{
  VehicleVelocityConverter converter("base_link", 0.2, 0.1, 1.0);

  VelocityReport velocity_report;
  velocity_report.header.frame_id = "base_link";
  velocity_report.longitudinal_velocity = 0.0f;
  velocity_report.lateral_velocity = 0.0f;
  velocity_report.heading_rate = 0.0f;

  const auto result = converter.convert(velocity_report);

  EXPECT_FLOAT_EQ(result.twist_with_covariance.twist.twist.linear.x, 0.0f);
  EXPECT_FLOAT_EQ(result.twist_with_covariance.twist.twist.linear.y, 0.0f);
  EXPECT_FLOAT_EQ(result.twist_with_covariance.twist.twist.angular.z, 0.0f);
}

TEST(TestVehicleVelocityConverter, NegativeVelocity)
{
  VehicleVelocityConverter converter("base_link", 0.2, 0.1, 1.0);

  VelocityReport velocity_report;
  velocity_report.header.frame_id = "base_link";
  velocity_report.longitudinal_velocity = -5.0f;  // Reverse
  velocity_report.lateral_velocity = -0.2f;
  velocity_report.heading_rate = -0.5f;

  const auto result = converter.convert(velocity_report);

  EXPECT_FLOAT_EQ(result.twist_with_covariance.twist.twist.linear.x, -5.0f);
  EXPECT_FLOAT_EQ(result.twist_with_covariance.twist.twist.linear.y, -0.2f);
  EXPECT_FLOAT_EQ(result.twist_with_covariance.twist.twist.angular.z, -0.5f);
}

TEST(TestVehicleVelocityConverter, CovarianceValues)
{
  VehicleVelocityConverter converter("base_link", 0.5, 0.3, 1.0);

  VelocityReport velocity_report;
  velocity_report.header.frame_id = "base_link";
  velocity_report.longitudinal_velocity = 1.0f;
  velocity_report.lateral_velocity = 0.0f;
  velocity_report.heading_rate = 0.0f;

  const auto result = converter.convert(velocity_report);

  // Check that covariance values are computed correctly (stddev^2)
  EXPECT_DOUBLE_EQ(result.twist_with_covariance.twist.covariance[0 + 0 * 6], 0.5 * 0.5);
  EXPECT_DOUBLE_EQ(result.twist_with_covariance.twist.covariance[5 + 5 * 6], 0.3 * 0.3);

  // Check that large variance values are set for unmeasured dimensions
  EXPECT_DOUBLE_EQ(result.twist_with_covariance.twist.covariance[1 + 1 * 6], 10000.0);
  EXPECT_DOUBLE_EQ(result.twist_with_covariance.twist.covariance[2 + 2 * 6], 10000.0);
  EXPECT_DOUBLE_EQ(result.twist_with_covariance.twist.covariance[3 + 3 * 6], 10000.0);
  EXPECT_DOUBLE_EQ(result.twist_with_covariance.twist.covariance[4 + 4 * 6], 10000.0);
}

TEST(TestVehicleVelocityConverter, OffDiagonalCovarianceIsZero)
{
  VehicleVelocityConverter converter("base_link", 0.2, 0.1, 1.0);

  VelocityReport velocity_report;
  velocity_report.header.frame_id = "base_link";
  velocity_report.longitudinal_velocity = 1.0f;
  velocity_report.lateral_velocity = 0.0f;
  velocity_report.heading_rate = 0.0f;

  const auto result = converter.convert(velocity_report);

  // Check that off-diagonal elements are zero
  for (size_t i = 0; i < 36; ++i) {
    // Skip diagonal elements (indices: 0, 7, 14, 21, 28, 35)
    if (i == 0 || i == 7 || i == 14 || i == 21 || i == 28 || i == 35) {
      continue;
    }
    EXPECT_DOUBLE_EQ(result.twist_with_covariance.twist.covariance[i], 0.0)
      << "Off-diagonal element at index " << i << " should be zero";
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
