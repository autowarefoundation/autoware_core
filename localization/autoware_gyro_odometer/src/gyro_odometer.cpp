// Copyright 2015-2019 Autoware Foundation
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
#include <rclcpp/time.hpp>

#include <algorithm>
#include <cmath>
#include <deque>

namespace autoware::gyro_odometer
{

GyroOdometer::GyroOdometer(double message_timeout_sec) : message_timeout_sec_(message_timeout_sec)
{
}

std::optional<GyroOdometer::OutputData> GyroOdometer::input_vehicle_twist(
  const geometry_msgs::msg::TwistWithCovarianceStamped & vehicle_twist_msg)
{
  vehicle_twist_arrived_ = true;
  latest_vehicle_twist_ros_time_ = vehicle_twist_msg.header.stamp;
  vehicle_twist_queue_.push_back(vehicle_twist_msg);

  const auto twist_with_cov =
    concat_gyro_and_odometer(std::max(latest_vehicle_twist_ros_time_, latest_imu_ros_time_));
  if (!twist_with_cov) {
    return std::nullopt;
  }
  return make_output(*twist_with_cov);
}

std::optional<GyroOdometer::OutputData> GyroOdometer::input_imu(
  const sensor_msgs::msg::Imu & imu_msg)
{
  imu_arrived_ = true;
  latest_imu_ros_time_ = imu_msg.header.stamp;
  gyro_queue_.push_back(imu_msg);

  const auto twist_with_cov =
    concat_gyro_and_odometer(std::max(latest_vehicle_twist_ros_time_, latest_imu_ros_time_));
  if (!twist_with_cov) {
    return std::nullopt;
  }
  return make_output(*twist_with_cov);
}

GyroOdometer::Status GyroOdometer::take_status() const
{
  Status status;
  status.vehicle_twist_arrived = vehicle_twist_arrived_;
  status.imu_arrived = imu_arrived_;
  status.latest_vehicle_twist_dt = latest_vehicle_twist_dt_;
  status.latest_imu_dt = latest_imu_dt_;
  status.latest_vehicle_twist_ros_time = latest_vehicle_twist_ros_time_;
  status.latest_imu_ros_time = latest_imu_ros_time_;
  status.vehicle_twist_queue_size = latest_vehicle_twist_queue_size_;
  status.imu_queue_size = latest_imu_queue_size_;
  return status;
}

std::optional<geometry_msgs::msg::TwistWithCovarianceStamped>
GyroOdometer::concat_gyro_and_odometer(rclcpp::Time reference_time)
{
  // check arrive first topic
  if (!vehicle_twist_arrived_) {
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return std::nullopt;
  }
  if (!imu_arrived_) {
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return std::nullopt;
  }

  // check timeout
  latest_vehicle_twist_dt_ = std::abs((reference_time - latest_vehicle_twist_ros_time_).seconds());
  latest_imu_dt_ = std::abs((reference_time - latest_imu_ros_time_).seconds());
  if (latest_vehicle_twist_dt_ > message_timeout_sec_) {
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return std::nullopt;
  }
  if (latest_imu_dt_ > message_timeout_sec_) {
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return std::nullopt;
  }

  // check queue size
  latest_vehicle_twist_queue_size_ = static_cast<int32_t>(vehicle_twist_queue_.size());
  latest_imu_queue_size_ = static_cast<int32_t>(gyro_queue_.size());
  if (vehicle_twist_queue_.empty()) {
    // wait for the vehicle twist side; the queued gyro samples are kept for the next attempt
    return std::nullopt;
  }
  if (gyro_queue_.empty()) {
    // wait for the imu side; the queued vehicle twists are kept for the next attempt
    return std::nullopt;
  }

  // fuse the vehicle twist and the (already transformed) gyro queue
  const geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov =
    fuse_twist(vehicle_twist_queue_, gyro_queue_);

  vehicle_twist_queue_.clear();
  gyro_queue_.clear();
  return twist_with_cov;
}

GyroOdometer::OutputData GyroOdometer::make_output(
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov_raw)
{
  geometry_msgs::msg::TwistStamped twist_raw;
  twist_raw.header = twist_with_cov_raw.header;
  twist_raw.twist = twist_with_cov_raw.twist.twist;

  // clear imu yaw bias if vehicle is stopped
  const geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance =
    apply_stop_compensation(twist_with_cov_raw);

  geometry_msgs::msg::TwistStamped twist;
  twist.header = twist_with_covariance.header;
  twist.twist = twist_with_covariance.twist.twist;

  return std::make_tuple(twist_raw, twist_with_cov_raw, twist, twist_with_covariance);
}

std::array<double, 9> transform_covariance(const std::array<double, 9> & cov)
{
  using COV_IDX = autoware_utils_geometry::xyz_covariance_index::XYZ_COV_IDX;

  double max_cov = std::max({cov[COV_IDX::X_X], cov[COV_IDX::Y_Y], cov[COV_IDX::Z_Z]});

  std::array<double, 9> cov_transformed = {};
  cov_transformed.fill(0.);
  cov_transformed[COV_IDX::X_X] = max_cov;
  cov_transformed[COV_IDX::Y_Y] = max_cov;
  cov_transformed[COV_IDX::Z_Z] = max_cov;
  return cov_transformed;
}

geometry_msgs::msg::TwistWithCovarianceStamped fuse_twist(
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & vehicle_twist_queue,
  const std::deque<sensor_msgs::msg::Imu> & gyro_queue)
{
  using COV_IDX_XYZ = autoware_utils_geometry::xyz_covariance_index::XYZ_COV_IDX;
  using COV_IDX_XYZRPY = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;

  // calc mean, covariance
  double vx_mean = 0;
  geometry_msgs::msg::Vector3 gyro_mean{};
  double vx_covariance_original = 0;
  geometry_msgs::msg::Vector3 gyro_covariance_original{};
  for (const auto & vehicle_twist : vehicle_twist_queue) {
    vx_mean += vehicle_twist.twist.twist.linear.x;
    vx_covariance_original += vehicle_twist.twist.covariance[COV_IDX_XYZRPY::X_X];
  }
  vx_mean /= static_cast<double>(vehicle_twist_queue.size());
  vx_covariance_original /= static_cast<double>(vehicle_twist_queue.size());

  for (const auto & gyro : gyro_queue) {
    gyro_mean.x += gyro.angular_velocity.x;
    gyro_mean.y += gyro.angular_velocity.y;
    gyro_mean.z += gyro.angular_velocity.z;
    gyro_covariance_original.x += gyro.angular_velocity_covariance[COV_IDX_XYZ::X_X];
    gyro_covariance_original.y += gyro.angular_velocity_covariance[COV_IDX_XYZ::Y_Y];
    gyro_covariance_original.z += gyro.angular_velocity_covariance[COV_IDX_XYZ::Z_Z];
  }
  gyro_mean.x /= static_cast<double>(gyro_queue.size());
  gyro_mean.y /= static_cast<double>(gyro_queue.size());
  gyro_mean.z /= static_cast<double>(gyro_queue.size());
  gyro_covariance_original.x /= static_cast<double>(gyro_queue.size());
  gyro_covariance_original.y /= static_cast<double>(gyro_queue.size());
  gyro_covariance_original.z /= static_cast<double>(gyro_queue.size());

  // concat
  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov;
  const auto latest_vehicle_twist_stamp = rclcpp::Time(vehicle_twist_queue.back().header.stamp);
  const auto latest_imu_stamp = rclcpp::Time(gyro_queue.back().header.stamp);
  if (latest_vehicle_twist_stamp < latest_imu_stamp) {
    twist_with_cov.header.stamp = latest_imu_stamp;
  } else {
    twist_with_cov.header.stamp = latest_vehicle_twist_stamp;
  }
  twist_with_cov.header.frame_id = gyro_queue.front().header.frame_id;
  twist_with_cov.twist.twist.linear.x = vx_mean;
  twist_with_cov.twist.twist.angular = gyro_mean;

  // From a statistical point of view, here we reduce the covariances according to the number of
  // observed data
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::X_X] =
    vx_covariance_original / static_cast<double>(vehicle_twist_queue.size());
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::Y_Y] = 100000.0;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::Z_Z] = 100000.0;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::ROLL_ROLL] =
    gyro_covariance_original.x / static_cast<double>(gyro_queue.size());
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::PITCH_PITCH] =
    gyro_covariance_original.y / static_cast<double>(gyro_queue.size());
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::YAW_YAW] =
    gyro_covariance_original.z / static_cast<double>(gyro_queue.size());

  return twist_with_cov;
}

geometry_msgs::msg::TwistWithCovarianceStamped apply_stop_compensation(
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov)
{
  geometry_msgs::msg::TwistWithCovarianceStamped result = twist_with_cov;

  // clear imu yaw bias if vehicle is stopped
  if (
    std::fabs(twist_with_cov.twist.twist.angular.z) < 0.01 &&
    std::fabs(twist_with_cov.twist.twist.linear.x) < 0.01) {
    result.twist.twist.angular.x = 0.0;
    result.twist.twist.angular.y = 0.0;
    result.twist.twist.angular.z = 0.0;
  }

  return result;
}

}  // namespace autoware::gyro_odometer
