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

#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <tuple>

namespace autoware::gyro_odometer
{

/// \brief Queue-and-fuse logic of the gyro odometer, independent of ROS communication and TF.
///
/// Messages are fed in through input_vehicle_twist() / input_imu(), each of which returns the fused
/// output at the moment a fusion completes. IMU samples are expected to already be expressed in the
/// output frame; bringing them there is the caller's business. Staleness is judged between the
/// two inputs' header stamps, so this class holds no clock or frame state of its own.
class GyroOdometer
{
public:
  /// \brief Construct with the age beyond which a queued message is considered stale.
  ///
  /// \param message_timeout_sec a fusion attempt discards both queues once either side is older
  /// than this, measured against the newer of the two inputs' latest stamps.
  explicit GyroOdometer(double message_timeout_sec);

  /// \brief The four twist messages a successful fusion produces: raw fused twist, raw fused twist
  /// with covariance, stop-compensated twist, and stop-compensated twist with covariance,
  /// respectively.
  using OutputData = std::tuple<
    geometry_msgs::msg::TwistStamped, geometry_msgs::msg::TwistWithCovarianceStamped,
    geometry_msgs::msg::TwistStamped, geometry_msgs::msg::TwistWithCovarianceStamped>;

  /// \brief Snapshot of the internal state that the caller reports as diagnostics.
  ///
  /// The queue sizes are the values observed during the most recent fusion attempt, not a live
  /// read: a successful fusion empties both queues immediately, so a live read would almost always
  /// be 0.
  struct Status
  {
    bool vehicle_twist_arrived{false};
    bool imu_arrived{false};
    double latest_vehicle_twist_dt{0.0};
    double latest_imu_dt{0.0};
    rclcpp::Time latest_vehicle_twist_ros_time;
    rclcpp::Time latest_imu_ros_time;
    int32_t vehicle_twist_queue_size{0};
    int32_t imu_queue_size{0};
  };

  /// \brief Queue \p vehicle_twist_msg and attempt a fusion.
  /// \return the fused output if this call completed a fusion, std::nullopt otherwise.
  std::optional<OutputData> input_vehicle_twist(
    const geometry_msgs::msg::TwistWithCovarianceStamped & vehicle_twist_msg);

  /// \brief Queue \p imu_msg, which must already be expressed in the output frame, and attempt a
  /// fusion.
  /// \return the fused output if this call completed a fusion, std::nullopt otherwise.
  std::optional<OutputData> input_imu(const sensor_msgs::msg::Imu & imu_msg);

  /// \brief Read the current state for diagnostics reporting.
  Status take_status() const;

private:
  std::optional<geometry_msgs::msg::TwistWithCovarianceStamped> concat_gyro_and_odometer(
    rclcpp::Time reference_time);

  static OutputData make_output(
    const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov_raw);

  double message_timeout_sec_;
  bool vehicle_twist_arrived_{false};
  bool imu_arrived_{false};
  rclcpp::Time latest_vehicle_twist_ros_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_imu_ros_time_{0, 0, RCL_ROS_TIME};
  double latest_vehicle_twist_dt_{0.0};
  double latest_imu_dt_{0.0};
  int32_t latest_vehicle_twist_queue_size_{0};
  int32_t latest_imu_queue_size_{0};
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
