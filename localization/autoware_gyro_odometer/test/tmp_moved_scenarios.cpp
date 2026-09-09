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

  gyro_odometer.input_imu(make_imu(make_stamp(98, 0), "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));
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
  gyro_odometer.input_imu(make_imu(make_stamp(10, 0), "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));
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
