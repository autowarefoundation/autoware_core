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

#ifndef AUTOWARE__UNIFIED_LOCALIZATION_CORE__EKF_CORE_HPP_
#define AUTOWARE__UNIFIED_LOCALIZATION_CORE__EKF_CORE_HPP_

#include "autoware/unified_localization_core/ekf_params.hpp"
#include "autoware/unified_localization_core/types.hpp"

#include <autoware/kalman_filter/time_delay_kalman_filter.hpp>

#include <Eigen/Core>
#include <array>
#include <memory>
#include <vector>

namespace autoware::unified_localization_core
{

class EKFCore
{
public:
  explicit EKFCore(const EKFParams & params);

  /** Initialize with initial pose and optional transform (dx, dy, dyaw). */
  void initialize(
    const PoseWithCovariance & initial_pose,
    double transform_dx = 0.0,
    double transform_dy = 0.0,
    double transform_dyaw = 0.0);

  void predict(double dt);

  /** @return true if update was applied. */
  bool measurement_update_pose(const PoseWithCovariance & pose, double t_curr_sec);
  bool measurement_update_twist(const TwistWithCovariance & twist, double t_curr_sec);

  void get_current_pose(
    double t_sec, bool biased_yaw,
    double & out_x, double & out_y, double & out_z,
    double & out_qx, double & out_qy, double & out_qz, double & out_qw) const;
  void get_current_twist(
    double t_sec, double & out_vx, double & out_wz) const;
  void get_pose_covariance(std::array<double, 36> & out) const;
  void get_twist_covariance(std::array<double, 36> & out) const;
  double get_yaw_bias() const;

private:
  using TimeDelayKalmanFilter = autoware::kalman_filter::TimeDelayKalmanFilter;

  size_t find_closest_delay_time_index(double target_value) const;
  void accumulate_delay_time(double dt);
  void update_simple_1d_filters(
    double z, double roll, double pitch,
    double z_var, double roll_var, double pitch_var,
    double dt);

  TimeDelayKalmanFilter kalman_filter_;
  EKFParams params_;
  int dim_x_{6};
  std::vector<double> accumulated_delay_times_;
  double ekf_dt_{0.02};

  struct Simple1DFilter
  {
    bool initialized{false};
    double x{0.0};
    double var{1e9};
    double proc_var_c{0.0};
    void init(double obs, double obs_var);
    void update(double obs, double obs_var, double dt);
    void set_proc_var(double v) { proc_var_c = v; }
  };
  Simple1DFilter z_filter_;
  Simple1DFilter roll_filter_;
  Simple1DFilter pitch_filter_;

  double last_angular_x_{0.0};
  double last_angular_y_{0.0};
  double last_angular_z_{0.0};
};

}  // namespace autoware::unified_localization_core

#endif  // AUTOWARE__UNIFIED_LOCALIZATION_CORE__EKF_CORE_HPP_
