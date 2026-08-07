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

#ifndef GYRO_ODOMETER_HPP_
#define GYRO_ODOMETER_HPP_

/*logic depends on diagnostics so that it snapshots only the stuff diag actually needs*/
#include "gyro_odometer_diagnostics.hpp"

#include <autoware_utils_geometry/msg/covariance.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <array>
#include <deque>
#include <optional>
#include <string>
#include <tuple>
namespace autoware::gyro_odometer
{
class GyroOdometer
{
private:
  using COV_IDX = autoware_utils_geometry::xyz_covariance_index::XYZ_COV_IDX;

public:
  explicit GyroOdometer();
  using OutputData = std::tuple<
    geometry_msgs::msg::TwistStamped, geometry_msgs::msg::TwistWithCovarianceStamped,
    geometry_msgs::msg::TwistStamped, geometry_msgs::msg::TwistWithCovarianceStamped>;

  /*latest transform failure of imu is referenced also in twist callback
    to clear both internal queues*/
  std::optional<OutputData> callback_vehicle_twist_internal(
    const geometry_msgs::msg::TwistWithCovarianceStamped & vehicle_twist_msg_ptr,
    rclcpp::Time current_time, double message_timeout_sec, bool is_succeed_transform_imu);

  std::optional<OutputData> callback_imu_internal(
    const sensor_msgs::msg::Imu & imu_msg_ptr, rclcpp::Time current_time,
    double message_timeout_sec, bool is_succeed_transform_imu);

  static OutputData publish_data_internal(
    const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov_raw);

  /// \brief Copy out the fusion-owned part of the diagnostics state. The caller fills the fields it
  /// owns itself (is_succeed_transform_imu, message_timeout_sec, output_frame). See
  /// DiagnosticsState.
  DiagnosticsState take_diagnostics_state() const;

private:
  std::optional<OutputData> try_concat_gyro_and_odometer(
    rclcpp::Time current_time, double message_timeout_sec, bool is_succeed_transform_imu);
  std::optional<geometry_msgs::msg::TwistWithCovarianceStamped> concat_gyro_and_odometer(
    rclcpp::Time current_time, double message_timeout_sec, bool is_succeed_transform_imu);

  bool vehicle_twist_arrived_;
  bool imu_arrived_;
  rclcpp::Time latest_vehicle_twist_ros_time_;
  rclcpp::Time latest_imu_ros_time_;
  double latest_vehicle_twist_dt_;
  double latest_imu_dt_;
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> vehicle_twist_queue_;
  std::deque<sensor_msgs::msg::Imu> gyro_queue_;
};

/// \brief Reduce an angular-velocity covariance (xyz layout) to an isotropic diagonal covariance.
///
/// The maximum of the three diagonal terms (X_X, Y_Y, Z_Z) is written to all three diagonal
/// terms; every off-diagonal term is zeroed. Pure function: output depends only on the input.
std::array<double, 9> transform_covariance(const std::array<double, 9> & cov);

/// \brief Fuse the vehicle-twist queue and the (already gyro-frame-transformed) IMU queue into a
/// single twist with covariance.
///
/// Computes the per-queue means and the statistically reduced covariances, and selects the output
/// header stamp as the later of the latest vehicle-twist and latest IMU stamps. The IMU queue must
/// already be transformed into the output frame (its covariances reduced via transform_covariance).
/// Both queues must be non-empty; emptiness is the caller's responsibility to check.
///
/// Pure function: it reads the queues and produces an output message without touching any node
/// state, the clock, TF, or publishers.
geometry_msgs::msg::TwistWithCovarianceStamped fuse_twist(
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & vehicle_twist_queue,
  const std::deque<sensor_msgs::msg::Imu> & gyro_queue);

/// \brief Clear the IMU-derived angular velocity when the vehicle is judged to be stopped.
///
/// When both |angular.z| and |linear.x| are below 0.01, all three angular components are zeroed;
/// otherwise the input is returned unchanged. Pure function returning a new message.
geometry_msgs::msg::TwistWithCovarianceStamped apply_stop_compensation(
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov);

}  // namespace autoware::gyro_odometer

#endif  // GYRO_ODOMETER_HPP_
