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

#include "autoware/unified_localization_core/ekf_core.hpp"
#include "autoware/unified_localization_core/covariance_index.hpp"
#include "autoware/unified_localization_core/matrix_types.hpp"
#include "autoware/unified_localization_core/measurement_core.hpp"
#include "autoware/unified_localization_core/state_index.hpp"
#include "autoware/unified_localization_core/state_transition_core.hpp"
#include "autoware/unified_localization_core/types.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace autoware::unified_localization_core
{

static bool has_nan_or_inf(const Eigen::MatrixXd & m)
{
  return m.array().isNaN().any() || m.array().isInf().any();
}

static double squared_mahalanobis(
  const Eigen::VectorXd & x, const Eigen::VectorXd & y, const Eigen::MatrixXd & C)
{
  const Eigen::VectorXd d = x - y;
  return d.dot(C.ldlt().solve(d));
}

void EKFCore::Simple1DFilter::init(double obs, double obs_var)
{
  x = obs;
  var = obs_var;
  initialized = true;
}

void EKFCore::Simple1DFilter::update(double obs, double obs_var, double dt)
{
  if (!initialized) {
    init(obs, obs_var);
    return;
  }
  const double proc_var_d = proc_var_c * dt * dt;
  var = var + proc_var_d;
  const double gain = var / (var + obs_var);
  x = x + gain * (obs - x);
  var = (1.0 - gain) * var;
}

EKFCore::EKFCore(const EKFParams & params)
: params_(params),
  dim_x_(static_cast<int>(STATE_DIM)),
  accumulated_delay_times_(params.extend_state_step, 1e15)
{
  Eigen::MatrixXd x = Eigen::MatrixXd::Zero(dim_x_, 1);
  Eigen::MatrixXd p = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 1e15;
  const int YAW = static_cast<int>(StateIndex::YAW);
  const int YAWB = static_cast<int>(StateIndex::YAWB);
  const int VX = static_cast<int>(StateIndex::VX);
  const int WZ = static_cast<int>(StateIndex::WZ);
  p(YAW, YAW) = 50.0;
  if (params_.enable_yaw_bias_estimation) {
    p(YAWB, YAWB) = 50.0;
  }
  p(VX, VX) = 1000.0;
  p(WZ, WZ) = 50.0;
  kalman_filter_.init(x, p, static_cast<int>(params_.extend_state_step));
  z_filter_.set_proc_var(params_.z_filter_proc_dev * params_.z_filter_proc_dev);
  roll_filter_.set_proc_var(params_.roll_filter_proc_dev * params_.roll_filter_proc_dev);
  pitch_filter_.set_proc_var(params_.pitch_filter_proc_dev * params_.pitch_filter_proc_dev);
}

void EKFCore::initialize(
  const PoseWithCovariance & initial_pose,
  double transform_dx, double transform_dy, double transform_dyaw)
{
  const int X = static_cast<int>(StateIndex::X);
  const int Y = static_cast<int>(StateIndex::Y);
  const int YAW = static_cast<int>(StateIndex::YAW);
  const int YAWB = static_cast<int>(StateIndex::YAWB);
  const int VX = static_cast<int>(StateIndex::VX);
  const int WZ = static_cast<int>(StateIndex::WZ);
  using Idx = CovarianceIndex::Index;

  Eigen::MatrixXd x(dim_x_, 1);
  Eigen::MatrixXd p = Eigen::MatrixXd::Zero(dim_x_, dim_x_);
  const double yaw0 = get_yaw_from_quaternion(
    initial_pose.orientation_x, initial_pose.orientation_y,
    initial_pose.orientation_z, initial_pose.orientation_w);
  x(X) = initial_pose.position_x + transform_dx;
  x(Y) = initial_pose.position_y + transform_dy;
  x(YAW) = normalize_yaw(yaw0 + transform_dyaw);
  x(YAWB) = 0.0;
  x(VX) = 0.0;
  x(WZ) = 0.0;

  p(X, X) = initial_pose.covariance[Idx::X_X];
  p(Y, Y) = initial_pose.covariance[Idx::Y_Y];
  p(YAW, YAW) = initial_pose.covariance[Idx::YAW_YAW];
  if (params_.enable_yaw_bias_estimation) {
    p(YAWB, YAWB) = 0.0001;
  }
  p(VX, VX) = 0.01;
  p(WZ, WZ) = 0.01;

  kalman_filter_.init(x, p, static_cast<int>(params_.extend_state_step));

  z_filter_.init(initial_pose.position_z, initial_pose.covariance[Idx::Z_Z]);
  const double roll = 0.0;
  const double pitch = 0.0;
  roll_filter_.init(roll, initial_pose.covariance[Idx::ROLL_ROLL]);
  pitch_filter_.init(pitch, initial_pose.covariance[Idx::PITCH_PITCH]);
}

void EKFCore::accumulate_delay_time(double dt)
{
  std::copy_backward(
    accumulated_delay_times_.begin(), accumulated_delay_times_.end() - 1,
    accumulated_delay_times_.end());
  accumulated_delay_times_.front() = 0.0;
  for (size_t i = 1; i < accumulated_delay_times_.size(); ++i) {
    accumulated_delay_times_[i] += dt;
  }
}

size_t EKFCore::find_closest_delay_time_index(double target_value) const
{
  if (target_value > accumulated_delay_times_.back()) {
    return accumulated_delay_times_.size();
  }
  auto lower = std::lower_bound(
    accumulated_delay_times_.begin(), accumulated_delay_times_.end(), target_value);
  if (lower == accumulated_delay_times_.begin()) {
    return 0;
  }
  if (lower == accumulated_delay_times_.end()) {
    return accumulated_delay_times_.size() - 1;
  }
  auto prev = lower - 1;
  bool closer_prev = (target_value - *prev) < (*lower - target_value);
  return closer_prev ? static_cast<size_t>(std::distance(accumulated_delay_times_.begin(), prev))
                     : static_cast<size_t>(std::distance(accumulated_delay_times_.begin(), lower));
}

void EKFCore::predict(double dt)
{
  const Eigen::MatrixXd x_curr = kalman_filter_.getLatestX();
  const double proc_cov_vx_d = std::pow(params_.proc_stddev_vx_c * dt, 2.0);
  const double proc_cov_wz_d = std::pow(params_.proc_stddev_wz_c * dt, 2.0);
  const double proc_cov_yaw_d = std::pow(params_.proc_stddev_yaw_c * dt, 2.0);
  Vector6d x_next = predict_next_state(x_curr, dt);
  Matrix6d a = create_state_transition_matrix(x_curr, dt);
  Matrix6d q = process_noise_covariance(proc_cov_yaw_d, proc_cov_vx_d, proc_cov_wz_d);
  kalman_filter_.predictWithDelay(x_next, a, q);
  ekf_dt_ = dt;
  accumulate_delay_time(dt);
}

void EKFCore::update_simple_1d_filters(
  double z, double roll, double pitch,
  double z_var, double roll_var, double pitch_var,
  double dt)
{
  z_filter_.update(z, z_var, dt);
  roll_filter_.update(roll, roll_var, dt);
  pitch_filter_.update(pitch, pitch_var, dt);
}

bool EKFCore::measurement_update_pose(const PoseWithCovariance & pose, double t_curr_sec)
{
  const int X = static_cast<int>(StateIndex::X);
  const int Y = static_cast<int>(StateIndex::Y);
  const int YAW = static_cast<int>(StateIndex::YAW);
  constexpr int dim_y = 3;
  using Idx = CovarianceIndex::Index;

  double delay_time = (t_curr_sec - pose.timestamp_sec) + params_.pose_additional_delay;
  delay_time = std::max(delay_time, 0.0);
  const size_t delay_step = find_closest_delay_time_index(delay_time);
  if (delay_step >= params_.extend_state_step) {
    return false;
  }

  double yaw = get_yaw_from_quaternion(
    pose.orientation_x, pose.orientation_y, pose.orientation_z, pose.orientation_w);
  const double ekf_yaw = kalman_filter_.getLatestX()(delay_step * dim_x_ + YAW);
  const double yaw_error = normalize_yaw(yaw - ekf_yaw);
  yaw = yaw_error + ekf_yaw;

  Eigen::MatrixXd y(dim_y, 1);
  y << pose.position_x, pose.position_y, yaw;
  if (has_nan_or_inf(y)) {
    return false;
  }

  const Eigen::Vector3d y_ekf(
    kalman_filter_.getLatestX()(delay_step * dim_x_ + X),
    kalman_filter_.getLatestX()(delay_step * dim_x_ + Y),
    ekf_yaw);
  const Eigen::MatrixXd p_curr = kalman_filter_.getLatestP();
  const Eigen::MatrixXd p_y = p_curr.block(0, 0, dim_y, dim_y);
  const double dist = std::sqrt(squared_mahalanobis(y_ekf, y, p_y));
  if (dist > params_.pose_gate_dist) {
    return false;
  }

  const Eigen::Matrix<double, 3, 6> c = pose_measurement_matrix();
  const Eigen::Matrix3d r = pose_measurement_covariance(pose.covariance, params_.pose_smoothing_steps);
  kalman_filter_.updateWithDelay(y, c, r, static_cast<int>(delay_step));

  const double z = pose.position_z;
  const double roll = 0.0;
  const double pitch = 0.0;
  const double z_var = pose.covariance[Idx::Z_Z] * static_cast<double>(params_.pose_smoothing_steps);
  const double roll_var = pose.covariance[Idx::ROLL_ROLL] * static_cast<double>(params_.pose_smoothing_steps);
  const double pitch_var = pose.covariance[Idx::PITCH_PITCH] * static_cast<double>(params_.pose_smoothing_steps);
  update_simple_1d_filters(z, roll, pitch, z_var, roll_var, pitch_var, ekf_dt_);
  return true;
}

bool EKFCore::measurement_update_twist(const TwistWithCovariance & twist, double t_curr_sec)
{
  const int VX = static_cast<int>(StateIndex::VX);
  const int WZ = static_cast<int>(StateIndex::WZ);
  constexpr int dim_y = 2;

  last_angular_x_ = twist.angular_x;
  last_angular_y_ = twist.angular_y;
  last_angular_z_ = twist.angular_z;

  double delay_time = (t_curr_sec - twist.timestamp_sec) + params_.twist_additional_delay;
  delay_time = std::max(delay_time, 0.0);
  const size_t delay_step = find_closest_delay_time_index(delay_time);
  if (delay_step >= params_.extend_state_step) {
    return false;
  }

  Eigen::MatrixXd y(dim_y, 1);
  y << twist.linear_x, twist.angular_z;
  if (has_nan_or_inf(y)) {
    return false;
  }

  const Eigen::Vector2d y_ekf(
    kalman_filter_.getLatestX()(delay_step * dim_x_ + VX),
    kalman_filter_.getLatestX()(delay_step * dim_x_ + WZ));
  const Eigen::MatrixXd p_curr = kalman_filter_.getLatestP();
  const Eigen::MatrixXd p_y = p_curr.block(4, 4, dim_y, dim_y);
  const double dist = std::sqrt(squared_mahalanobis(y_ekf, y, p_y));
  if (dist > params_.twist_gate_dist) {
    return false;
  }

  const Eigen::Matrix<double, 2, 6> c = twist_measurement_matrix();
  const Eigen::Matrix2d r = twist_measurement_covariance(twist.covariance, params_.twist_smoothing_steps);
  kalman_filter_.updateWithDelay(y, c, r, static_cast<int>(delay_step));
  return true;
}

void EKFCore::get_current_pose(
  double /* t_sec */, bool biased_yaw,
  double & out_x, double & out_y, double & out_z,
  double & out_qx, double & out_qy, double & out_qz, double & out_qw) const
{
  const int X = static_cast<int>(StateIndex::X);
  const int Y = static_cast<int>(StateIndex::Y);
  const int YAW = static_cast<int>(StateIndex::YAW);
  const int YAWB = static_cast<int>(StateIndex::YAWB);
  const Eigen::MatrixXd x = kalman_filter_.getLatestX();
  out_x = x(X);
  out_y = x(Y);
  out_z = z_filter_.x;
  const double roll = roll_filter_.x;
  const double pitch = pitch_filter_.x;
  const double biased_yaw_val = x(YAW);
  const double yaw_bias = x(YAWB);
  const double yaw = biased_yaw ? biased_yaw_val : (biased_yaw_val + yaw_bias);
  quaternion_from_rpy(roll, pitch, yaw, out_qx, out_qy, out_qz, out_qw);
}

void EKFCore::get_current_twist(double /* t_sec */, double & out_vx, double & out_wz) const
{
  const int VX = static_cast<int>(StateIndex::VX);
  const int WZ = static_cast<int>(StateIndex::WZ);
  const Eigen::MatrixXd x = kalman_filter_.getLatestX();
  out_vx = x(VX);
  out_wz = x(WZ);
}

void EKFCore::get_pose_covariance(std::array<double, 36> & out) const
{
  out.fill(0.0);
  using Idx = CovarianceIndex::Index;
  const Matrix6d p = kalman_filter_.getLatestP();
  const int X = static_cast<int>(StateIndex::X);
  const int Y = static_cast<int>(StateIndex::Y);
  const int YAW = static_cast<int>(StateIndex::YAW);
  out[Idx::X_X] = p(X, X);
  out[Idx::X_Y] = p(X, Y);
  out[Idx::X_YAW] = p(X, YAW);
  out[Idx::Y_X] = p(Y, X);
  out[Idx::Y_Y] = p(Y, Y);
  out[Idx::Y_YAW] = p(Y, YAW);
  out[Idx::YAW_X] = p(YAW, X);
  out[Idx::YAW_Y] = p(YAW, Y);
  out[Idx::YAW_YAW] = p(YAW, YAW);
  out[Idx::Z_Z] = z_filter_.var;
  out[Idx::ROLL_ROLL] = roll_filter_.var;
  out[Idx::PITCH_PITCH] = pitch_filter_.var;
}

void EKFCore::get_twist_covariance(std::array<double, 36> & out) const
{
  out.fill(0.0);
  const Matrix6d p = kalman_filter_.getLatestP();
  const int VX = static_cast<int>(StateIndex::VX);
  const int WZ = static_cast<int>(StateIndex::WZ);
  out[0] = p(VX, VX);
  out[5] = p(VX, WZ);
  out[30] = p(WZ, VX);
  out[35] = p(WZ, WZ);
}

double EKFCore::get_yaw_bias() const
{
  const int YAWB = static_cast<int>(StateIndex::YAWB);
  return kalman_filter_.getLatestX()(YAWB);
}

}  // namespace autoware::unified_localization_core
