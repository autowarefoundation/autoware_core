// Copyright 2018-2019 Autoware Foundation
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

#ifndef EKF_LOCALIZER_HPP_
#define EKF_LOCALIZER_HPP_

#include "utils/aged_object_queue.hpp"
#include "utils/hyper_parameters.hpp"
#include "utils/state_index.hpp"

#include <autoware/kalman_filter/kalman_filter.hpp>
#include <autoware/kalman_filter/time_delay_kalman_filter.hpp>
#include <tf2/utils.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::ekf_localizer
{
using autoware::kalman_filter::TimeDelayKalmanFilter;

struct EKFDiagnosticInfo
{
  size_t no_update_count{0};
  size_t queue_size{0};
  bool is_passed_delay_gate{true};
  double delay_time{0.0};
  double delay_time_threshold{0.0};
  bool is_passed_mahalanobis_gate{true};
  double mahalanobis_distance{0.0};
};

struct CoreWarning
{
  std::string text;
  uint32_t throttle_ms{0};
};

struct EKFUpdateResult
{
  EKFDiagnosticInfo pose_diag_info;
  EKFDiagnosticInfo twist_diag_info;
  std::vector<CoreWarning> warnings;
};

class Simple1DFilter
{
public:
  Simple1DFilter()
  {
    initialized_ = false;
    x_ = 0;
    var_ = 1e9;
    proc_var_x_c_ = 0.0;
  };

  void init(const double init_obs, const double obs_var)
  {
    x_ = init_obs;
    var_ = obs_var;
    initialized_ = true;
  };

  void update(const double obs, const double obs_var, const double dt)
  {
    if (!initialized_) {
      init(obs, obs_var);
      return;
    }

    // Prediction step (current variance)
    double proc_var_x_d = proc_var_x_c_ * dt * dt;
    var_ = var_ + proc_var_x_d;

    // Update step
    double kalman_gain = var_ / (var_ + obs_var);
    x_ = x_ + kalman_gain * (obs - x_);
    var_ = (1 - kalman_gain) * var_;
  };

  void set_proc_var(const double proc_var) { proc_var_x_c_ = proc_var; }
  [[nodiscard]] double get_x() const { return x_; }
  [[nodiscard]] double get_var() const { return var_; }

private:
  bool initialized_;
  double x_;
  double var_;
  double proc_var_x_c_;
};

class EKFLocalizer
{
public:
  using PoseWithCovariance = geometry_msgs::msg::PoseWithCovarianceStamped;
  using TwistWithCovariance = geometry_msgs::msg::TwistWithCovarianceStamped;
  using Pose = geometry_msgs::msg::PoseStamped;
  using Twist = geometry_msgs::msg::TwistStamped;

  explicit EKFLocalizer(const HyperParameters & params);

  void initialize(
    const PoseWithCovariance & initial_pose,
    const geometry_msgs::msg::TransformStamped & transform);

  [[nodiscard]] geometry_msgs::msg::PoseStamped get_current_pose(bool get_biased_yaw) const;
  [[nodiscard]] geometry_msgs::msg::TwistStamped get_current_twist() const;
  [[nodiscard]] double get_yaw_bias() const;
  [[nodiscard]] std::array<double, 36> get_current_pose_covariance() const;
  [[nodiscard]] std::array<double, 36> get_current_twist_covariance() const;

  void push_pose(const std::shared_ptr<const PoseWithCovariance> & pose);
  void push_twist(const std::shared_ptr<const TwistWithCovariance> & twist);
  EKFUpdateResult update_step(const double t_curr_sec);
  void reset();

private:
  [[nodiscard]] size_t find_closest_delay_time_index(double target_value) const;

  void accumulate_delay_time(const double dt);
  void predict_with_delay(const double dt);

  bool measurement_update_pose(
    const PoseWithCovariance & pose, const double t_curr_sec, EKFDiagnosticInfo & pose_diag_info,
    std::vector<CoreWarning> & warnings_out);

  bool measurement_update_twist(
    const TwistWithCovariance & twist, const double t_curr_sec, EKFDiagnosticInfo & twist_diag_info,
    std::vector<CoreWarning> & warnings_out);

  geometry_msgs::msg::PoseWithCovarianceStamped compensate_rph_with_delay(
    const PoseWithCovariance & pose, tf2::Vector3 last_angular_velocity, const double delay_time);

  void update_simple_1d_filters(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose, const size_t smoothing_step);

  TimeDelayKalmanFilter kalman_filter_;

  const int dim_x_;
  std::vector<double> accumulated_delay_times_;
  const HyperParameters params_;

  Simple1DFilter z_filter_;
  Simple1DFilter roll_filter_;
  Simple1DFilter pitch_filter_;

  /**
   * @brief last angular velocity for compensating rph with delay
   */
  tf2::Vector3 last_angular_velocity_;

  double ekf_dt_;
};

}  // namespace autoware::ekf_localizer

#endif  // EKF_LOCALIZER_HPP_
