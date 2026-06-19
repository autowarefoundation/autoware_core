// Copyright 2021 TierIV
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

#include "../src/vehicle_velocity_converter.hpp"

#include <autoware_utils_geometry/msg/covariance.hpp>

#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <gtest/gtest.h>

#include <set>
#include <string>

using autoware::vehicle_velocity_converter::VehicleVelocityConverter;
using autoware_vehicle_msgs::msg::VelocityReport;
using geometry_msgs::msg::TwistWithCovarianceStamped;
using COV_IDX = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;

namespace
{
VelocityReport make_report(
  const double longitudinal_velocity, const double lateral_velocity, const double heading_rate,
  const std::string & frame_id = "base_link")
{
  VelocityReport msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp.sec = 123;
  msg.header.stamp.nanosec = 456;
  msg.longitudinal_velocity = static_cast<float>(longitudinal_velocity);
  msg.lateral_velocity = static_cast<float>(lateral_velocity);
  msg.heading_rate = static_cast<float>(heading_rate);
  return msg;
}

// Assert that every covariance entry except the six diagonal indices is exactly zero.
void expect_off_diagonal_zero(const TwistWithCovarianceStamped & twist)
{
  const std::set<size_t> diagonal = {COV_IDX::X_X,       COV_IDX::Y_Y,         COV_IDX::Z_Z,
                                     COV_IDX::ROLL_ROLL, COV_IDX::PITCH_PITCH, COV_IDX::YAW_YAW};
  for (size_t i = 0; i < twist.twist.covariance.size(); ++i) {
    if (diagonal.count(i) == 0U) {
      EXPECT_EQ(twist.twist.covariance[i], 0.0) << "covariance[" << i << "] expected to be zero";
    }
  }
}
}  // namespace

// Pure-conversion tests: no ROS context, deterministic struct in / struct out.

TEST(VehicleVelocityConverterConvert, AxisMappingAndScale)
{
  const auto msg = make_report(2.0, 0.1, 0.3);
  const VehicleVelocityConverter converter(1.5, 0.2, 0.1);

  const auto twist = converter.convert(msg);

  // Longitudinal velocity is multiplied by the scale factor, lateral and yaw mapped directly.
  // The report fields are float, so the converted values are compared at float precision.
  EXPECT_FLOAT_EQ(twist.twist.twist.linear.x, 2.0 * 1.5);
  EXPECT_FLOAT_EQ(twist.twist.twist.linear.y, 0.1);
  EXPECT_FLOAT_EQ(twist.twist.twist.angular.z, 0.3);
  // Unused twist components must stay zero.
  EXPECT_EQ(twist.twist.twist.linear.z, 0.0);
  EXPECT_EQ(twist.twist.twist.angular.x, 0.0);
  EXPECT_EQ(twist.twist.twist.angular.y, 0.0);
}

TEST(VehicleVelocityConverterConvert, FullCovarianceLayout)
{
  const auto msg = make_report(2.0, 0.1, 0.3);
  const VehicleVelocityConverter converter(1.5, 0.2, 0.1);

  const auto twist = converter.convert(msg);

  // Diagonal entries: vx/wz variances from stddev^2, the rest large fixed values.
  EXPECT_DOUBLE_EQ(twist.twist.covariance[COV_IDX::X_X], 0.2 * 0.2);
  EXPECT_DOUBLE_EQ(twist.twist.covariance[COV_IDX::Y_Y], 10000.0);
  EXPECT_DOUBLE_EQ(twist.twist.covariance[COV_IDX::Z_Z], 10000.0);
  EXPECT_DOUBLE_EQ(twist.twist.covariance[COV_IDX::ROLL_ROLL], 10000.0);
  EXPECT_DOUBLE_EQ(twist.twist.covariance[COV_IDX::PITCH_PITCH], 10000.0);
  EXPECT_DOUBLE_EQ(twist.twist.covariance[COV_IDX::YAW_YAW], 0.1 * 0.1);
  // All 30 off-diagonal entries must be zero.
  expect_off_diagonal_zero(twist);
}

TEST(VehicleVelocityConverterConvert, HeaderCopiedVerbatim)
{
  const auto msg = make_report(2.0, 0.1, 0.3, "custom_frame");
  const VehicleVelocityConverter converter(1.0, 0.2, 0.1);

  const auto twist = converter.convert(msg);

  EXPECT_EQ(twist.header.frame_id, "custom_frame");
  EXPECT_EQ(twist.header.stamp.sec, msg.header.stamp.sec);
  EXPECT_EQ(twist.header.stamp.nanosec, msg.header.stamp.nanosec);
}

TEST(VehicleVelocityConverterConvert, ZeroScaleFactor)
{
  const auto msg = make_report(2.0, 0.1, 0.3);
  const VehicleVelocityConverter converter(0.0, 0.2, 0.1);

  const auto twist = converter.convert(msg);

  EXPECT_EQ(twist.twist.twist.linear.x, 0.0);
  // Lateral and yaw are unaffected by the scale factor.
  EXPECT_FLOAT_EQ(twist.twist.twist.linear.y, 0.1);
  EXPECT_FLOAT_EQ(twist.twist.twist.angular.z, 0.3);
}

TEST(VehicleVelocityConverterConvert, NegativeVelocityAndScale)
{
  const auto msg = make_report(-2.0, -0.1, -0.3);
  const VehicleVelocityConverter converter(-1.5, 0.2, 0.1);

  const auto twist = converter.convert(msg);

  EXPECT_FLOAT_EQ(twist.twist.twist.linear.x, -2.0 * -1.5);
  EXPECT_FLOAT_EQ(twist.twist.twist.linear.y, -0.1);
  EXPECT_FLOAT_EQ(twist.twist.twist.angular.z, -0.3);
  // Variances stay non-negative (stddev squared) regardless of velocity sign.
  EXPECT_DOUBLE_EQ(twist.twist.covariance[COV_IDX::X_X], 0.2 * 0.2);
  EXPECT_DOUBLE_EQ(twist.twist.covariance[COV_IDX::YAW_YAW], 0.1 * 0.1);
}
