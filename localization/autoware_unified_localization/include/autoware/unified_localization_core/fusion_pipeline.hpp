// Copyright 2025 Autoware Foundation
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE__UNIFIED_LOCALIZATION_CORE__FUSION_PIPELINE_HPP_
#define AUTOWARE__UNIFIED_LOCALIZATION_CORE__FUSION_PIPELINE_HPP_

#include "autoware/unified_localization_core/ekf_core.hpp"
#include "autoware/unified_localization_core/ekf_params.hpp"
#include "autoware/unified_localization_core/stop_filter_core.hpp"
#include "autoware/unified_localization_core/twist2accel_core.hpp"
#include "autoware/unified_localization_core/types.hpp"

#include <memory>

namespace autoware::unified_localization_core
{

struct FusionPipelineParams
{
  EKFParams ekf;
  StopFilterParams stop_filter;
  Twist2AccelParams twist2accel;
};

/**
 * MVP Core pipeline: EKF fusion + stop_filter + twist2accel.
 * ROS-free; all I/O uses struct types.
 */
class FusionPipeline
{
public:
  explicit FusionPipeline(const FusionPipelineParams & params);

  void initialize(const PoseWithCovariance & initial_pose);

  /**
   * One step: predict EKF with dt, then apply pose/twist updates if provided.
   * @param t_curr_sec  current time (seconds)
   * @param dt_sec  time step since last predict
   * @param pose  optional pose measurement (nullptr to skip)
   * @param twist  optional twist measurement (nullptr to skip)
   */
  void step(
    double t_curr_sec, double dt_sec,
    const PoseWithCovariance * pose,
    const TwistWithCovariance * twist);

  /** Get current odometry (EKF pose + twist, then stop-filtered). */
  void get_odometry(double t_sec, OdometryOutput & out) const;

  /** Get acceleration from two consecutive twists (e.g. EKF twist at t and t-dt). */
  void get_acceleration(
    double prev_t_sec, double curr_t_sec,
    const Vector3 & prev_linear, const Vector3 & prev_angular,
    const Vector3 & curr_linear, const Vector3 & curr_angular,
    AccelerationOutput & out);

  bool is_initialized() const { return initialized_; }

private:
  FusionPipelineParams params_;
  std::unique_ptr<EKFCore> ekf_;
  std::unique_ptr<StopFilterCore> stop_filter_;
  std::unique_ptr<Twist2AccelCore> twist2accel_;
  bool initialized_{false};
  double last_t_sec_{0.0};
};

}  // namespace autoware::unified_localization_core

#endif  // AUTOWARE__UNIFIED_LOCALIZATION_CORE__FUSION_PIPELINE_HPP_
