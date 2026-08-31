// Copyright 2025 Autoware Foundation
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

#include "gyro_odometer.hpp"

#include <autoware_utils_geometry/msg/covariance.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <gtest/gtest.h>

#include <array>
#include <deque>
#include <string>

namespace autoware::gyro_odometer
{
namespace
{

using geometry_msgs::msg::TwistWithCovarianceStamped;
using sensor_msgs::msg::Imu;
using COV_IDX_XYZ = autoware_utils_geometry::xyz_covariance_index::XYZ_COV_IDX;
using COV_IDX_XYZRPY = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;

builtin_interfaces::msg::Time make_stamp(int32_t sec, uint32_t nanosec)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

Imu make_imu(
  const builtin_interfaces::msg::Time & stamp, const std::string & frame_id, double wx, double wy,
  double wz, double cov_xx, double cov_yy, double cov_zz)
{
  Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = frame_id;
  imu.angular_velocity.x = wx;
  imu.angular_velocity.y = wy;
  imu.angular_velocity.z = wz;
  imu.angular_velocity_covariance[COV_IDX_XYZ::X_X] = cov_xx;
  imu.angular_velocity_covariance[COV_IDX_XYZ::Y_Y] = cov_yy;
  imu.angular_velocity_covariance[COV_IDX_XYZ::Z_Z] = cov_zz;
  return imu;
}

TwistWithCovarianceStamped make_vehicle_twist(
  const builtin_interfaces::msg::Time & stamp, double vx, double cov_xx)
{
  TwistWithCovarianceStamped twist;
  twist.header.stamp = stamp;
  twist.header.frame_id = "base_link";
  twist.twist.twist.linear.x = vx;
  twist.twist.covariance[COV_IDX_XYZRPY::X_X] = cov_xx;
  return twist;
}

// Large enough that none of the scenarios below ever trip the message-timeout check.
constexpr double huge_timeout_sec = 1e12;

}  // namespace

// transform_covariance: the maximum diagonal term is written to every diagonal term, off-diagonals
// are zeroed.
TEST(GyroOdometer, TransformCovariancePicksMaxDiagonalAndZerosOffDiagonals)
{
  std::array<double, 9> cov = {};
  cov[COV_IDX_XYZ::X_X] = 1.0;
  cov[COV_IDX_XYZ::Y_Y] = 5.0;  // max
  cov[COV_IDX_XYZ::Z_Z] = 3.0;
  // pollute off-diagonals to make sure they are dropped
  cov[COV_IDX_XYZ::X_Y] = 42.0;
  cov[COV_IDX_XYZ::Z_X] = -7.0;

  const std::array<double, 9> out = transform_covariance(cov);

  EXPECT_DOUBLE_EQ(out[COV_IDX_XYZ::X_X], 5.0);
  EXPECT_DOUBLE_EQ(out[COV_IDX_XYZ::Y_Y], 5.0);
  EXPECT_DOUBLE_EQ(out[COV_IDX_XYZ::Z_Z], 5.0);
  EXPECT_DOUBLE_EQ(out[COV_IDX_XYZ::X_Y], 0.0);
  EXPECT_DOUBLE_EQ(out[COV_IDX_XYZ::X_Z], 0.0);
  EXPECT_DOUBLE_EQ(out[COV_IDX_XYZ::Y_X], 0.0);
  EXPECT_DOUBLE_EQ(out[COV_IDX_XYZ::Y_Z], 0.0);
  EXPECT_DOUBLE_EQ(out[COV_IDX_XYZ::Z_X], 0.0);
  EXPECT_DOUBLE_EQ(out[COV_IDX_XYZ::Z_Y], 0.0);
}

// fuse_twist: means over multiple entries, covariance reduction by queue size, fixed Y_Y/Z_Z, and
// the output stamp being the later of the two latest queue stamps.
TEST(GyroOdometer, FuseTwistComputesMeansCovarianceAndStamp)
{
  std::deque<TwistWithCovarianceStamped> vehicle_twist_queue;
  {
    TwistWithCovarianceStamped t;
    t.header.stamp = make_stamp(10, 0);
    t.twist.twist.linear.x = 2.0;
    t.twist.covariance[COV_IDX_XYZRPY::X_X] = 4.0;
    vehicle_twist_queue.push_back(t);

    t.header.stamp = make_stamp(12, 0);  // latest vehicle twist stamp
    t.twist.twist.linear.x = 4.0;
    t.twist.covariance[COV_IDX_XYZRPY::X_X] = 8.0;
    vehicle_twist_queue.push_back(t);
  }
  // mean vx = 3.0; summed cov / n = 12/2 = 6.0; reduced again / n = 6.0/2 = 3.0

  std::deque<Imu> gyro_queue;
  {
    Imu imu;
    imu.header.frame_id = "base_link";
    imu.header.stamp = make_stamp(11, 0);
    imu.angular_velocity.x = 0.2;
    imu.angular_velocity.y = 0.4;
    imu.angular_velocity.z = 0.6;
    imu.angular_velocity_covariance[COV_IDX_XYZ::X_X] = 1.0;
    imu.angular_velocity_covariance[COV_IDX_XYZ::Y_Y] = 2.0;
    imu.angular_velocity_covariance[COV_IDX_XYZ::Z_Z] = 3.0;
    gyro_queue.push_back(imu);

    imu.header.stamp = make_stamp(13, 0);  // latest imu stamp -> overall latest
    imu.angular_velocity.x = 0.4;
    imu.angular_velocity.y = 0.8;
    imu.angular_velocity.z = 1.2;
    imu.angular_velocity_covariance[COV_IDX_XYZ::X_X] = 3.0;
    imu.angular_velocity_covariance[COV_IDX_XYZ::Y_Y] = 6.0;
    imu.angular_velocity_covariance[COV_IDX_XYZ::Z_Z] = 9.0;
    gyro_queue.push_back(imu);
  }
  // gyro means: x=0.3, y=0.6, z=0.9
  // gyro cov sums/n: x=4/2=2, y=8/2=4, z=12/2=6; reduced again /n: x=1, y=2, z=3

  const TwistWithCovarianceStamped out = fuse_twist(vehicle_twist_queue, gyro_queue);

  EXPECT_DOUBLE_EQ(out.twist.twist.linear.x, 3.0);
  EXPECT_DOUBLE_EQ(out.twist.twist.angular.x, 0.3);
  EXPECT_DOUBLE_EQ(out.twist.twist.angular.y, 0.6);
  EXPECT_DOUBLE_EQ(out.twist.twist.angular.z, 0.9);

  EXPECT_DOUBLE_EQ(out.twist.covariance[COV_IDX_XYZRPY::X_X], 3.0);
  EXPECT_DOUBLE_EQ(out.twist.covariance[COV_IDX_XYZRPY::Y_Y], 100000.0);
  EXPECT_DOUBLE_EQ(out.twist.covariance[COV_IDX_XYZRPY::Z_Z], 100000.0);
  EXPECT_DOUBLE_EQ(out.twist.covariance[COV_IDX_XYZRPY::ROLL_ROLL], 1.0);
  EXPECT_DOUBLE_EQ(out.twist.covariance[COV_IDX_XYZRPY::PITCH_PITCH], 2.0);
  EXPECT_DOUBLE_EQ(out.twist.covariance[COV_IDX_XYZRPY::YAW_YAW], 3.0);

  // output stamp is the later of latest vehicle twist (12s) and latest imu (13s)
  EXPECT_EQ(out.header.stamp.sec, 13);
  EXPECT_EQ(out.header.stamp.nanosec, 0u);
  // frame id is taken from the front of the gyro queue
  EXPECT_EQ(out.header.frame_id, "base_link");
}

// fuse_twist: when the latest vehicle-twist stamp is later than the latest IMU stamp, the output
// stamp follows the vehicle twist.
TEST(GyroOdometer, FuseTwistChoosesLaterVehicleTwistStamp)
{
  std::deque<TwistWithCovarianceStamped> vehicle_twist_queue;
  {
    TwistWithCovarianceStamped t;
    t.header.stamp = make_stamp(20, 500);
    vehicle_twist_queue.push_back(t);
  }
  std::deque<Imu> gyro_queue;
  {
    Imu imu;
    imu.header.frame_id = "imu_link";
    imu.header.stamp = make_stamp(20, 100);
    gyro_queue.push_back(imu);
  }

  const TwistWithCovarianceStamped out = fuse_twist(vehicle_twist_queue, gyro_queue);

  EXPECT_EQ(out.header.stamp.sec, 20);
  EXPECT_EQ(out.header.stamp.nanosec, 500u);
  EXPECT_EQ(out.header.frame_id, "imu_link");
}

// apply_stop_compensation: when both |angular.z| and |linear.x| are below 0.01, all angular
// components are zeroed.
TEST(GyroOdometer, ApplyStopCompensationZeroesAngularWhenStopped)
{
  TwistWithCovarianceStamped twist;
  twist.twist.twist.linear.x = 0.005;
  twist.twist.twist.angular.x = 0.5;
  twist.twist.twist.angular.y = -0.4;
  twist.twist.twist.angular.z = 0.001;

  const TwistWithCovarianceStamped out = apply_stop_compensation(twist);

  EXPECT_DOUBLE_EQ(out.twist.twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(out.twist.twist.angular.y, 0.0);
  EXPECT_DOUBLE_EQ(out.twist.twist.angular.z, 0.0);
  // linear.x is preserved
  EXPECT_DOUBLE_EQ(out.twist.twist.linear.x, 0.005);
}

// apply_stop_compensation: when the vehicle is moving (large linear.x), angular is preserved.
TEST(GyroOdometer, ApplyStopCompensationPreservesAngularWhenMoving)
{
  TwistWithCovarianceStamped twist;
  twist.twist.twist.linear.x = 3.0;  // moving
  twist.twist.twist.angular.x = 0.5;
  twist.twist.twist.angular.y = -0.4;
  twist.twist.twist.angular.z = 0.001;  // small yaw but vehicle is moving

  const TwistWithCovarianceStamped out = apply_stop_compensation(twist);

  EXPECT_DOUBLE_EQ(out.twist.twist.angular.x, 0.5);
  EXPECT_DOUBLE_EQ(out.twist.twist.angular.y, -0.4);
  EXPECT_DOUBLE_EQ(out.twist.twist.angular.z, 0.001);
  EXPECT_DOUBLE_EQ(out.twist.twist.linear.x, 3.0);
}

// apply_stop_compensation: a large yaw rate keeps angular even when linear.x is small.
TEST(GyroOdometer, ApplyStopCompensationPreservesAngularWhenTurning)
{
  TwistWithCovarianceStamped twist;
  twist.twist.twist.linear.x = 0.0;
  twist.twist.twist.angular.z = 0.5;  // turning in place
  twist.twist.twist.angular.x = 0.1;

  const TwistWithCovarianceStamped out = apply_stop_compensation(twist);

  EXPECT_DOUBLE_EQ(out.twist.twist.angular.x, 0.1);
  EXPECT_DOUBLE_EQ(out.twist.twist.angular.z, 0.5);
}

// An IMU covariance that differs per axis is widened to the largest of its diagonal terms before
// it reaches the fusion, so the reported angular covariances are all that largest term.
TEST(GyroOdometer, ImuCovarianceIsWidenedToItsLargestDiagonalTerm)
{
  GyroOdometer gyro_odometer{huge_timeout_sec};
  const auto stamp = make_stamp(100, 0);

  gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));
  gyro_odometer.input_imu(make_imu(stamp, "base_link", 0.1, 0.2, 0.3, 1.0, 5.0, 3.0));
  const auto output = gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));

  ASSERT_TRUE(output.has_value());
  const auto & covariance = output->twist_with_covariance_raw.twist.covariance;
  EXPECT_DOUBLE_EQ(covariance[COV_IDX_XYZRPY::ROLL_ROLL], 5.0);
  EXPECT_DOUBLE_EQ(covariance[COV_IDX_XYZRPY::PITCH_PITCH], 5.0);
  EXPECT_DOUBLE_EQ(covariance[COV_IDX_XYZRPY::YAW_YAW], 5.0);
}

}  // namespace autoware::gyro_odometer
