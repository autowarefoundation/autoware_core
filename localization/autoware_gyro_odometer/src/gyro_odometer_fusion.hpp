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

#ifndef GYRO_ODOMETER_FUSION_HPP_
#define GYRO_ODOMETER_FUSION_HPP_

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace autoware::gyro_odometer
{

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

/// \brief A single diagnostics finding: a severity level paired with its human-readable message.
struct DiagnosticsEntry
{
  int8_t level{diagnostic_msgs::msg::DiagnosticStatus::OK};
  std::string message;
};

/// \brief Inputs needed to decide the gyro_odometer diagnostics level and messages.
struct DiagnosticsState
{
  bool vehicle_twist_arrived{false};
  bool imu_arrived{false};
  bool is_succeed_transform_imu{false};
  double latest_vehicle_twist_dt{0.0};
  double latest_imu_dt{0.0};
  double message_timeout_sec{0.0};
  std::string output_frame;
};

/// \brief Result of evaluating the diagnostics state.
///
/// \c entries holds one element per triggered condition (level + message), preserving the node's
/// historical check order. \c level is the maximum severity across all entries. \c log_message is
/// every entry message concatenated, each terminated by "; ".
struct DiagnosticsResult
{
  std::vector<DiagnosticsEntry> entries;
  int8_t level{diagnostic_msgs::msg::DiagnosticStatus::OK};
  std::string log_message;
};

/// \brief Evaluate the diagnostics state into per-condition entries, an aggregated level, and a
/// concatenated log message.
///
/// Pure function: no node, clock, or interface dependency. The messages are byte-for-byte identical
/// to the node's original strings (the TF-failure message embeds \c state.output_frame).
DiagnosticsResult determine_diagnostics(const DiagnosticsState & state);

}  // namespace autoware::gyro_odometer

#endif  // GYRO_ODOMETER_FUSION_HPP_
