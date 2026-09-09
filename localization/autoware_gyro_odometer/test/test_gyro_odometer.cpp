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
#include <rclcpp/time.hpp>

#include <gtest/gtest.h>

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

}  // namespace

// IMU samples alone never fuse, however many arrive.
TEST(GyroOdometer, ImuAloneNeverFuses)
{
  GyroOdometer gyro_odometer{10.0};
  const auto stamp = make_stamp(100, 0);

  const Imu imu = make_imu(stamp, "base_link", 0.1, 0.2, 0.3, 0.01, 0.02, 0.03);
  EXPECT_FALSE(gyro_odometer.input_imu(imu).has_value());
  EXPECT_FALSE(gyro_odometer.input_imu(imu).has_value());
  EXPECT_FALSE(gyro_odometer.input_imu(imu).has_value());

  const GyroOdometer::Status status = gyro_odometer.take_status();
  EXPECT_TRUE(status.imu_arrived);
  EXPECT_FALSE(status.vehicle_twist_arrived);
}

// Vehicle twists alone never fuse, however many arrive.
TEST(GyroOdometer, VehicleTwistAloneNeverFuses)
{
  GyroOdometer gyro_odometer{10.0};
  const auto stamp = make_stamp(100, 0);

  const TwistWithCovarianceStamped twist = make_vehicle_twist(stamp, 1.0, 4.0);
  EXPECT_FALSE(gyro_odometer.input_vehicle_twist(twist).has_value());
  EXPECT_FALSE(gyro_odometer.input_vehicle_twist(twist).has_value());
  EXPECT_FALSE(gyro_odometer.input_vehicle_twist(twist).has_value());

  const GyroOdometer::Status status = gyro_odometer.take_status();
  EXPECT_TRUE(status.vehicle_twist_arrived);
  EXPECT_FALSE(status.imu_arrived);
}

// Vehicle twists that arrive while no IMU sample is queued accumulate, and the IMU sample that
// completes the pair fuses against all of them at once: the reported longitudinal velocity is
// their mean and the reported variance is their mean variance divided by how many there were.
TEST(GyroOdometer, AccumulatedVehicleTwistsAreAveragedIntoOneFusion)
{
  GyroOdometer gyro_odometer{10.0};
  const auto stamp = make_stamp(100, 0);

  // The first message of each kind only marks its side as arrived, so the queues have to be primed
  // before a scenario can put a known number of messages in them.
  gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  gyro_odometer.input_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  ASSERT_TRUE(gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0)).has_value())
    << "priming did not reach a first fusion";

  gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));
  EXPECT_FALSE(gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 3.0, 4.0)).has_value())
    << "fused before an IMU sample completed the pair";

  const auto output =
    gyro_odometer.input_imu(make_imu(stamp, "base_link", 0.1, 0.2, 0.3, 0.01, 0.02, 0.03));
  ASSERT_TRUE(output.has_value());
  const auto & fused = output->twist_with_covariance_raw;

  EXPECT_DOUBLE_EQ(fused.twist.twist.linear.x, 2.0);
  EXPECT_DOUBLE_EQ(fused.twist.covariance[COV_IDX_XYZRPY::X_X], 2.0);
  EXPECT_DOUBLE_EQ(fused.twist.twist.angular.x, 0.1);
  EXPECT_DOUBLE_EQ(fused.twist.twist.angular.y, 0.2);
  EXPECT_DOUBLE_EQ(fused.twist.twist.angular.z, 0.3);

  const GyroOdometer::Status status = gyro_odometer.take_status();
  EXPECT_EQ(status.vehicle_twist_queue_size, 2);
  EXPECT_EQ(status.imu_queue_size, 1);
}

// IMU samples that arrive while no vehicle twist is queued accumulate, and the vehicle twist that
// completes the pair fuses against their mean angular velocity.
TEST(GyroOdometer, AccumulatedImuSamplesAreAveragedIntoOneFusion)
{
  GyroOdometer gyro_odometer{10.0};
  const auto stamp = make_stamp(100, 0);

  gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  gyro_odometer.input_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  ASSERT_TRUE(gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0)).has_value())
    << "priming did not reach a first fusion";

  gyro_odometer.input_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.2, 0.01, 0.01, 0.01));
  EXPECT_FALSE(
    gyro_odometer.input_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.4, 0.01, 0.01, 0.01))
      .has_value())
    << "fused before a vehicle twist completed the pair";

  const auto output = gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));
  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->twist_with_covariance_raw.twist.twist.angular.z, 0.3);
  EXPECT_DOUBLE_EQ(
    output->twist_with_covariance_raw.twist.covariance[COV_IDX_XYZRPY::YAW_YAW], 0.005);

  const GyroOdometer::Status status = gyro_odometer.take_status();
  EXPECT_EQ(status.vehicle_twist_queue_size, 1);
  EXPECT_EQ(status.imu_queue_size, 2);
}

// The output carries the later of the two input stamps, whichever side it comes from.
TEST(GyroOdometer, OutputCarriesTheLaterVehicleTwistStamp)
{
  GyroOdometer gyro_odometer{10.0};
  const auto priming_stamp = make_stamp(100, 0);

  gyro_odometer.input_vehicle_twist(make_vehicle_twist(priming_stamp, 0.0, 0.0));
  gyro_odometer.input_imu(make_imu(priming_stamp, "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  ASSERT_TRUE(
    gyro_odometer.input_vehicle_twist(make_vehicle_twist(priming_stamp, 0.0, 0.0)).has_value())
    << "priming did not reach a first fusion";

  gyro_odometer.input_imu(
    make_imu(make_stamp(98, 0), "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));
  const auto output =
    gyro_odometer.input_vehicle_twist(make_vehicle_twist(make_stamp(99, 0), 1.0, 4.0));
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(rclcpp::Time(output->twist_with_covariance_raw.header.stamp).seconds(), 99.0);
}

// Mirror of the above: this time the IMU sample is the later of the two.
TEST(GyroOdometer, OutputCarriesTheLaterImuStamp)
{
  GyroOdometer gyro_odometer{10.0};
  const auto priming_stamp = make_stamp(100, 0);

  gyro_odometer.input_vehicle_twist(make_vehicle_twist(priming_stamp, 0.0, 0.0));
  gyro_odometer.input_imu(make_imu(priming_stamp, "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  ASSERT_TRUE(
    gyro_odometer.input_vehicle_twist(make_vehicle_twist(priming_stamp, 0.0, 0.0)).has_value())
    << "priming did not reach a first fusion";

  gyro_odometer.input_vehicle_twist(make_vehicle_twist(make_stamp(98, 0), 1.0, 4.0));
  const auto output = gyro_odometer.input_imu(
    make_imu(make_stamp(99, 0), "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(rclcpp::Time(output->twist_with_covariance_raw.header.stamp).seconds(), 99.0);
}

// A vehicle twist older than the tolerance drops the pending data instead of fusing it, and says
// so through the diagnostics.
TEST(GyroOdometer, VehicleTwistOlderThanToleranceDropsPendingData)
{
  GyroOdometer gyro_odometer{1.0};
  const auto stamp = make_stamp(100, 0);

  gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  gyro_odometer.input_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  ASSERT_TRUE(gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0)).has_value())
    << "priming did not reach a first fusion";

  gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));
  EXPECT_FALSE(
    gyro_odometer
      .input_imu(make_imu(make_stamp(105, 0), "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01))
      .has_value());

  EXPECT_DOUBLE_EQ(gyro_odometer.take_status().latest_vehicle_twist_dt, 5.0);
}

// Mirror of the above: this time the IMU sample is the one older than the tolerance.
TEST(GyroOdometer, ImuOlderThanToleranceDropsPendingData)
{
  GyroOdometer gyro_odometer{1.0};
  const auto stamp = make_stamp(100, 0);

  gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  gyro_odometer.input_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  ASSERT_TRUE(gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0)).has_value())
    << "priming did not reach a first fusion";

  gyro_odometer.input_imu(make_imu(stamp, "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));
  EXPECT_FALSE(gyro_odometer.input_vehicle_twist(make_vehicle_twist(make_stamp(105, 0), 1.0, 4.0))
                 .has_value());

  EXPECT_DOUBLE_EQ(gyro_odometer.take_status().latest_imu_dt, 5.0);
}

// The staleness judgment depends only on each side's most recent stamp: stamps seen earlier leave
// no residue, so a mutually consistent pair fuses regardless of what was fused before it.
TEST(GyroOdometer, StalenessDependsOnlyOnTheLatestStamps)
{
  GyroOdometer gyro_odometer{1.0};

  gyro_odometer.input_vehicle_twist(make_vehicle_twist(make_stamp(100, 0), 0.0, 0.0));
  gyro_odometer.input_imu(make_imu(make_stamp(100, 0), "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  ASSERT_TRUE(
    gyro_odometer.input_vehicle_twist(make_vehicle_twist(make_stamp(100, 0), 0.0, 0.0)).has_value())
    << "priming did not reach a first fusion";

  gyro_odometer.input_vehicle_twist(make_vehicle_twist(make_stamp(10, 0), 3.0, 4.0));
  gyro_odometer.input_imu(
    make_imu(make_stamp(10, 0), "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));
  const auto output =
    gyro_odometer.input_vehicle_twist(make_vehicle_twist(make_stamp(10, 0), 3.0, 4.0));
  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->twist_with_covariance_raw.twist.twist.linear.x, 3.0);
}

// At a standstill the compensated pair reports no rotation at all, while the raw pair keeps what
// the IMU measured.
TEST(GyroOdometer, StandstillClearsAngularVelocityInTheCompensatedOutput)
{
  GyroOdometer gyro_odometer{10.0};
  const auto stamp = make_stamp(100, 0);

  gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  gyro_odometer.input_imu(make_imu(stamp, "base_link", 0.5, 0.6, 0.0, 0.01, 0.01, 0.01));
  const auto output = gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 0.0, 4.0));
  ASSERT_TRUE(output.has_value());

  EXPECT_DOUBLE_EQ(output->twist_with_covariance_raw.twist.twist.angular.x, 0.5);
  EXPECT_DOUBLE_EQ(output->twist_with_covariance_raw.twist.twist.angular.y, 0.6);
  EXPECT_DOUBLE_EQ(output->twist_raw.twist.angular.x, 0.5);

  EXPECT_DOUBLE_EQ(output->twist_with_covariance.twist.twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(output->twist_with_covariance.twist.twist.angular.y, 0.0);
  EXPECT_DOUBLE_EQ(output->twist_with_covariance.twist.twist.angular.z, 0.0);
  EXPECT_DOUBLE_EQ(output->twist.twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(output->twist.twist.angular.y, 0.0);
}

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

}  // namespace autoware::gyro_odometer
