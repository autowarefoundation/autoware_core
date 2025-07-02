// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__VELOCITY_SMOOTHER__SMOOTHER__SMOOTHER_BASE_HPP_
#define AUTOWARE__VELOCITY_SMOOTHER__SMOOTHER__SMOOTHER_BASE_HPP_

#include "autoware/velocity_smoother/resample.hpp"
#include "rclcpp/rclcpp.hpp"

#include <autoware_utils_debug/time_keeper.hpp>

#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include <limits>
#include <memory>
#include <vector>

namespace autoware::velocity_smoother
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

class SmootherBase
{
public:
  struct BaseParam
  {
    double max_accel;   // max acceleration in planning [m/s2] > 0
    double min_decel;   // min deceleration in planning [m/s2] < 0
    double stop_decel;  // deceleration at a stop point [m/s2] <= 0
    double max_jerk;
    double min_jerk;
    double min_decel_for_lateral_acc_lim_filter;  // deceleration limit applied in the lateral
                                                  // acceleration filter to avoid sudden braking.
    double min_curve_velocity;                    // min velocity at curve [m/s]
    double decel_distance_before_curve;  // distance before slow down for lateral acc at a curve
    double decel_distance_after_curve;   // distance after slow down for lateral acc at a curve
    // Velocity-dependent steering angle rate parameters
    std::vector<double> lateral_acceleration_limits;  // lateral acceleration limits [m/ss]
    std::vector<double> velocity_thresholds;          // velocity thresholds [m/s]
    std::vector<double> steering_angle_rate_limits;   // steering angle rate limits [degree/s]
    double wheel_base;                                // wheel base [m]
    double sample_ds;                                 // distance between trajectory points [m]
    double curvature_threshold;  // look-up distance of Trajectory point for calculation of steering
                                 // angle limit [m]
    double curvature_calculation_distance;  // threshold steering degree limit to trigger
                                            // steeringRateLimit [degree]
    resampling::ResampleParam resample_param;
  };

  explicit SmootherBase(
    rclcpp::Node & node, const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper);
  virtual ~SmootherBase() = default;
  virtual bool apply(
    const double initial_vel, const double initial_acc, const TrajectoryPoints & input,
    TrajectoryPoints & output, std::vector<TrajectoryPoints> & debug_trajectories,
    const bool publish_debug_trajs) = 0;

  virtual TrajectoryPoints resampleTrajectory(
    const TrajectoryPoints & input, const double v0, const geometry_msgs::msg::Pose & current_pose,
    const double nearest_dist_threshold, const double nearest_yaw_threshold) const = 0;

  virtual TrajectoryPoints applyLateralAccelerationFilter(
    const TrajectoryPoints & input, [[maybe_unused]] const double v0 = 0.0,
    [[maybe_unused]] const double a0 = 0.0, [[maybe_unused]] const bool enable_smooth_limit = false,
    const bool use_resampling = true, const double input_points_interval = 1.0) const;

  TrajectoryPoints applySteeringRateLimit(
    const TrajectoryPoints & input, const bool use_resampling = true,
    const double input_points_interval = 1.0) const;

  double getMaxAccel() const;
  double getMinDecel() const;
  double getMaxJerk() const;
  double getMinJerk() const;

  void setWheelBase(const double wheel_base);
  void setMaxAccel(const double max_acceleration);
  void setMaxJerk(const double max_jerk);

  void setParam(const BaseParam & param);
  BaseParam getBaseParam() const;

  template <typename ThresholdType, typename ComputeRatioFunc>
  std::vector<std::pair<double, double>> computeRatioLimits(
    const std::vector<double> & velocity_thresholds,
    const std::vector<ThresholdType> & threshold_values, ComputeRatioFunc compute_ratio_func) const;

  template <typename RatioType, typename ThresholdType, typename ComputeVelocityFunc>
  double computeVelocityLimit(
    const RatioType local_ratio, const std::vector<std::pair<double, double>> & ratio_limits,
    const std::vector<double> & velocity_thresholds,
    const std::vector<ThresholdType> & threshold_values,
    ComputeVelocityFunc compute_velocity_func) const;

  // Compute acc/v**2 limits at difference velocity threshold
  std::vector<std::pair<double, double>> computeLateralAccelerationVelocitySquareRatioLimits()
    const;
  // Helper function to get steering angle rate limit based on velocity
  std::vector<std::pair<double, double>> computeSteerRateVelocityRatioLimits() const;

  double computeVelocityLimitFromLateralAcc(
    const double local_curvature,
    const std::vector<std::pair<double, double>> lateral_acceleration_velocity_square_ratio_limits)
    const;

  double computeVelocityLimitFromSteerRate(
    const double local_steer_rate_velocity_ratio,
    const std::vector<std::pair<double, double>> steer_rate_velocity_ratio_limits) const;

protected:
  BaseParam base_param_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};
};
}  // namespace autoware::velocity_smoother

#endif  // AUTOWARE__VELOCITY_SMOOTHER__SMOOTHER__SMOOTHER_BASE_HPP_
