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

#include <array>
#include <deque>
#include <optional>
#include <string>

namespace autoware::gyro_odometer
{
namespace
{

using geometry_msgs::msg::TwistWithCovarianceStamped;
using sensor_msgs::msg::Imu;
using COV_IDX_XYZ = autoware_utils_geometry::xyz_covariance_index::XYZ_COV_IDX;
using COV_IDX_XYZRPY = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;

// Large enough that none of the scenarios below ever trip the message-timeout check.
constexpr double huge_timeout_sec = 1e12;
// Small enough that a scenario can deliberately trip it by stamping one side ahead of the other.
constexpr double short_timeout_sec = 1.0;
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

// Bring a fresh GyroOdometer to the state every scenario starts from: both sides have been seen
// once, both queues are empty, and one fusion has already gone through. The first message of each
// kind only marks its side as arrived, so this is what it takes before a scenario can put a known
// number of messages in the queues.
void prime(GyroOdometer & gyro_odometer)
{
  const auto stamp = make_stamp(100, 0);
  gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  gyro_odometer.input_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  const auto primed = gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  EXPECT_TRUE(primed.has_value()) << "priming did not reach a first fusion";
}

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

// ---------------------------------------------------------------------------
// The queue-and-fuse sequence, driven message by message.
// ---------------------------------------------------------------------------

// Nothing has been fed in yet, so neither side counts as having arrived.
TEST(GyroOdometer, NoInputLeavesNeitherSideArrived)
{
  GyroOdometer gyro_odometer{huge_timeout_sec};

  const GyroOdometer::Status status = gyro_odometer.take_status();

  EXPECT_FALSE(status.vehicle_twist_arrived);
  EXPECT_FALSE(status.imu_arrived);
  EXPECT_EQ(status.vehicle_twist_queue_size, 0);
  EXPECT_EQ(status.imu_queue_size, 0);
}

// IMU samples alone never fuse, however many arrive.
TEST(GyroOdometer, ImuAloneNeverFuses)
{
  GyroOdometer gyro_odometer{huge_timeout_sec};
  const auto stamp = make_stamp(100, 0);
  const Imu imu = make_imu(stamp, "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01);

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
  GyroOdometer gyro_odometer{huge_timeout_sec};
  const auto stamp = make_stamp(100, 0);
  const TwistWithCovarianceStamped twist = make_vehicle_twist(stamp, 1.0, 4.0);

  EXPECT_FALSE(gyro_odometer.input_vehicle_twist(twist).has_value());
  EXPECT_FALSE(gyro_odometer.input_vehicle_twist(twist).has_value());
  EXPECT_FALSE(gyro_odometer.input_vehicle_twist(twist).has_value());

  const GyroOdometer::Status status = gyro_odometer.take_status();
  EXPECT_TRUE(status.vehicle_twist_arrived);
  EXPECT_FALSE(status.imu_arrived);
}

// A completed pair produces all four messages: the raw pair carries the fused values, and the
// compensated pair carries them too while the vehicle is moving.
TEST(GyroOdometer, CompletedPairProducesAllFourMessages)
{
  GyroOdometer gyro_odometer{huge_timeout_sec};
  const auto stamp = make_stamp(100, 0);

  gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));
  gyro_odometer.input_imu(make_imu(stamp, "base_link", 0.1, 0.2, 0.3, 0.01, 0.02, 0.03));
  const auto output = gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));

  ASSERT_TRUE(output.has_value());
  const auto & [twist_raw, twist_with_cov_raw, twist, twist_with_cov] = *output;

  EXPECT_DOUBLE_EQ(twist_raw.twist.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(twist_raw.twist.angular.z, 0.3);
  EXPECT_DOUBLE_EQ(twist.twist.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(twist.twist.angular.z, 0.3);

  EXPECT_EQ(twist_with_cov_raw.header.frame_id, "base_link");
  EXPECT_DOUBLE_EQ(twist_with_cov_raw.twist.twist.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(twist_with_cov_raw.twist.twist.angular.x, 0.1);
  EXPECT_DOUBLE_EQ(twist_with_cov_raw.twist.twist.angular.y, 0.2);
  EXPECT_DOUBLE_EQ(twist_with_cov_raw.twist.twist.angular.z, 0.3);
  // The lateral and vertical velocities are not estimated, and say so through a large variance.
  EXPECT_DOUBLE_EQ(twist_with_cov_raw.twist.twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(twist_with_cov_raw.twist.twist.linear.z, 0.0);
  EXPECT_DOUBLE_EQ(twist_with_cov_raw.twist.covariance[COV_IDX_XYZRPY::Y_Y], 100000.0);
  EXPECT_DOUBLE_EQ(twist_with_cov_raw.twist.covariance[COV_IDX_XYZRPY::Z_Z], 100000.0);
  EXPECT_EQ(twist_with_cov.twist.covariance, twist_with_cov_raw.twist.covariance);
}

// Vehicle twists that arrive while no IMU sample is queued accumulate, and the IMU sample that
// completes the pair fuses against all of them at once: the reported longitudinal velocity is
// their mean and the reported variance is their mean variance divided by how many there were.
TEST(GyroOdometer, AccumulatedVehicleTwistsAreAveragedIntoOneFusion)
{
  GyroOdometer gyro_odometer{huge_timeout_sec};
  prime(gyro_odometer);
  const auto stamp = make_stamp(100, 0);

  EXPECT_FALSE(gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0)).has_value());
  EXPECT_FALSE(gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 3.0, 4.0)).has_value());

  const auto output =
    gyro_odometer.input_imu(make_imu(stamp, "base_link", 0.1, 0.2, 0.3, 0.01, 0.02, 0.03));

  ASSERT_TRUE(output.has_value());
  const auto & fused = std::get<1>(*output);
  EXPECT_DOUBLE_EQ(fused.twist.twist.linear.x, 2.0);
  EXPECT_DOUBLE_EQ(fused.twist.covariance[COV_IDX_XYZRPY::X_X], 2.0);
  EXPECT_DOUBLE_EQ(fused.twist.twist.angular.z, 0.3);

  const GyroOdometer::Status status = gyro_odometer.take_status();
  EXPECT_EQ(status.vehicle_twist_queue_size, 2);
  EXPECT_EQ(status.imu_queue_size, 1);
}

// IMU samples that arrive while no vehicle twist is queued accumulate, and the vehicle twist that
// completes the pair fuses against their mean angular velocity.
TEST(GyroOdometer, AccumulatedImuSamplesAreAveragedIntoOneFusion)
{
  GyroOdometer gyro_odometer{huge_timeout_sec};
  prime(gyro_odometer);
  const auto stamp = make_stamp(100, 0);

  EXPECT_FALSE(
    gyro_odometer.input_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.2, 0.01, 0.01, 0.01))
      .has_value());
  EXPECT_FALSE(
    gyro_odometer.input_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.4, 0.01, 0.01, 0.01))
      .has_value());

  const auto output = gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));

  ASSERT_TRUE(output.has_value());
  const auto & fused = std::get<1>(*output);
  EXPECT_DOUBLE_EQ(fused.twist.twist.angular.z, 0.3);
  EXPECT_DOUBLE_EQ(fused.twist.covariance[COV_IDX_XYZRPY::YAW_YAW], 0.005);

  const GyroOdometer::Status status = gyro_odometer.take_status();
  EXPECT_EQ(status.vehicle_twist_queue_size, 1);
  EXPECT_EQ(status.imu_queue_size, 2);
}

// The output carries the later of the two input stamps, whichever side it comes from.
TEST(GyroOdometer, OutputCarriesTheLaterVehicleTwistStamp)
{
  GyroOdometer gyro_odometer{huge_timeout_sec};
  prime(gyro_odometer);

  gyro_odometer.input_imu(
    make_imu(make_stamp(98, 0), "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));
  const auto output =
    gyro_odometer.input_vehicle_twist(make_vehicle_twist(make_stamp(99, 0), 1.0, 4.0));

  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(rclcpp::Time(std::get<1>(*output).header.stamp).seconds(), 99.0);
}

// Mirror of the above: this time the IMU sample is the later of the two.
TEST(GyroOdometer, OutputCarriesTheLaterImuStamp)
{
  GyroOdometer gyro_odometer{huge_timeout_sec};
  prime(gyro_odometer);

  gyro_odometer.input_vehicle_twist(make_vehicle_twist(make_stamp(98, 0), 1.0, 4.0));
  const auto output = gyro_odometer.input_imu(
    make_imu(make_stamp(99, 0), "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));

  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(rclcpp::Time(std::get<1>(*output).header.stamp).seconds(), 99.0);
}

// A vehicle twist older than the tolerance drops the pending data instead of fusing it, and the
// age it was judged on is left where the caller can report it.
TEST(GyroOdometer, VehicleTwistOlderThanToleranceDropsPendingData)
{
  GyroOdometer gyro_odometer{short_timeout_sec};
  prime(gyro_odometer);

  gyro_odometer.input_vehicle_twist(make_vehicle_twist(make_stamp(100, 0), 1.0, 4.0));
  const auto output = gyro_odometer.input_imu(
    make_imu(make_stamp(105, 0), "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));

  EXPECT_FALSE(output.has_value());
  const GyroOdometer::Status status = gyro_odometer.take_status();
  EXPECT_DOUBLE_EQ(status.latest_vehicle_twist_dt, 5.0);
  EXPECT_DOUBLE_EQ(status.latest_imu_dt, 0.0);

  // What was pending did not survive: a fresh pair fuses from its own values alone.
  gyro_odometer.input_vehicle_twist(make_vehicle_twist(make_stamp(105, 0), 7.0, 4.0));
  const auto next = gyro_odometer.input_imu(
    make_imu(make_stamp(105, 0), "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));
  ASSERT_TRUE(next.has_value());
  EXPECT_DOUBLE_EQ(std::get<1>(*next).twist.twist.linear.x, 7.0);
}

// Mirror of the above: this time the IMU sample is the one older than the tolerance.
TEST(GyroOdometer, ImuOlderThanToleranceDropsPendingData)
{
  GyroOdometer gyro_odometer{short_timeout_sec};
  prime(gyro_odometer);

  gyro_odometer.input_imu(
    make_imu(make_stamp(100, 0), "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));
  const auto output =
    gyro_odometer.input_vehicle_twist(make_vehicle_twist(make_stamp(105, 0), 1.0, 4.0));

  EXPECT_FALSE(output.has_value());
  const GyroOdometer::Status status = gyro_odometer.take_status();
  EXPECT_DOUBLE_EQ(status.latest_imu_dt, 5.0);
  EXPECT_DOUBLE_EQ(status.latest_vehicle_twist_dt, 0.0);
}

// A sample the caller could not bring into the output frame never fuses, and takes whatever was
// pending on either side with it.
TEST(GyroOdometer, UntransformableImuDropsPendingData)
{
  GyroOdometer gyro_odometer{huge_timeout_sec};
  prime(gyro_odometer);
  const auto stamp = make_stamp(100, 0);

  gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 99.0, 4.0));
  gyro_odometer.input_untransformable_imu(
    make_imu(stamp, "imu_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));

  // The discarded sample still counts as the IMU side having been heard from.
  EXPECT_TRUE(gyro_odometer.take_status().imu_arrived);

  // What was pending did not survive: a fresh pair fuses from its own values alone.
  gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 5.0, 4.0));
  const auto output =
    gyro_odometer.input_imu(make_imu(stamp, "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));

  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(std::get<1>(*output).twist.twist.linear.x, 5.0);
}

// Discarding a sample before any vehicle twist has been heard from leaves the vehicle-twist age
// alone: there is no stamp yet to measure one against.
TEST(GyroOdometer, UntransformableImuBeforeAnyVehicleTwistLeavesItsAgeAlone)
{
  GyroOdometer gyro_odometer{huge_timeout_sec};

  gyro_odometer.input_untransformable_imu(
    make_imu(make_stamp(100, 0), "imu_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));

  const GyroOdometer::Status status = gyro_odometer.take_status();
  EXPECT_TRUE(status.imu_arrived);
  EXPECT_FALSE(status.vehicle_twist_arrived);
  EXPECT_DOUBLE_EQ(status.latest_vehicle_twist_dt, 0.0);
}

// At a standstill the compensated pair reports no rotation at all, while the raw pair keeps what
// the IMU measured.
TEST(GyroOdometer, StandstillClearsAngularVelocityInTheCompensatedOutput)
{
  GyroOdometer gyro_odometer{huge_timeout_sec};
  const auto stamp = make_stamp(100, 0);

  gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  gyro_odometer.input_imu(make_imu(stamp, "base_link", 0.5, 0.6, 0.0, 0.01, 0.01, 0.01));
  const auto output = gyro_odometer.input_vehicle_twist(make_vehicle_twist(stamp, 0.0, 4.0));

  ASSERT_TRUE(output.has_value());
  const auto & [twist_raw, twist_with_cov_raw, twist, twist_with_cov] = *output;

  EXPECT_DOUBLE_EQ(twist_with_cov_raw.twist.twist.angular.x, 0.5);
  EXPECT_DOUBLE_EQ(twist_with_cov_raw.twist.twist.angular.y, 0.6);
  EXPECT_DOUBLE_EQ(twist_raw.twist.angular.x, 0.5);

  EXPECT_DOUBLE_EQ(twist_with_cov.twist.twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(twist_with_cov.twist.twist.angular.y, 0.0);
  EXPECT_DOUBLE_EQ(twist_with_cov.twist.twist.angular.z, 0.0);
  EXPECT_DOUBLE_EQ(twist.twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.twist.angular.y, 0.0);
}

}  // namespace autoware::gyro_odometer
