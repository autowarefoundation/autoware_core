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

#include "autoware/unified_localization_core/fusion_pipeline.hpp"
#include "autoware/unified_localization_core/types.hpp"

namespace autoware::unified_localization_core
{

FusionPipeline::FusionPipeline(const FusionPipelineParams & params) : params_(params)
{
  ekf_ = std::make_unique<EKFCore>(params_.ekf);
  stop_filter_ = std::make_unique<StopFilterCore>(params_.stop_filter);
  twist2accel_ = std::make_unique<Twist2AccelCore>(params_.twist2accel);
}

void FusionPipeline::initialize(const PoseWithCovariance & initial_pose)
{
  ekf_->initialize(initial_pose, 0.0, 0.0, 0.0);
  initialized_ = true;
  last_t_sec_ = initial_pose.timestamp_sec;
}

void FusionPipeline::step(
  double t_curr_sec, double dt_sec,
  const PoseWithCovariance * pose,
  const TwistWithCovariance * twist)
{
  if (!initialized_) {
    return;
  }
  ekf_->predict(dt_sec);
  if (pose) {
    ekf_->measurement_update_pose(*pose, t_curr_sec);
  }
  if (twist) {
    ekf_->measurement_update_twist(*twist, t_curr_sec);
  }
  last_t_sec_ = t_curr_sec;
}

void FusionPipeline::get_odometry(double t_sec, OdometryOutput & out) const
{
  if (!initialized_ || !ekf_) {
    return;
  }
  out.timestamp_sec = t_sec;
  double qx, qy, qz, qw;
  ekf_->get_current_pose(t_sec, false, out.position_x, out.position_y, out.position_z, qx, qy, qz, qw);
  out.orientation_x = qx;
  out.orientation_y = qy;
  out.orientation_z = qz;
  out.orientation_w = qw;
  double vx, wz;
  ekf_->get_current_twist(t_sec, vx, wz);
  out.linear_x = vx;
  out.linear_y = 0.0;
  out.linear_z = 0.0;
  out.angular_x = 0.0;
  out.angular_y = 0.0;
  out.angular_z = wz;
  ekf_->get_pose_covariance(out.pose_covariance);
  ekf_->get_twist_covariance(out.twist_covariance);

  Vector3 lin{out.linear_x, out.linear_y, out.linear_z};
  Vector3 ang{out.angular_x, out.angular_y, out.angular_z};
  Vector3 out_lin, out_ang;
  bool was_stopped{false};
  stop_filter_->apply(lin, ang, out_lin, out_ang, was_stopped);
  out.linear_x = out_lin.x;
  out.linear_y = out_lin.y;
  out.linear_z = out_lin.z;
  out.angular_x = out_ang.x;
  out.angular_y = out_ang.y;
  out.angular_z = out_ang.z;
}

void FusionPipeline::get_acceleration(
  double prev_t_sec, double curr_t_sec,
  const Vector3 & prev_linear, const Vector3 & prev_angular,
  const Vector3 & curr_linear, const Vector3 & curr_angular,
  AccelerationOutput & out)
{
  if (twist2accel_) {
    twist2accel_->estimate(prev_t_sec, curr_t_sec, prev_linear, prev_angular, curr_linear, curr_angular, out);
  }
}

}  // namespace autoware::unified_localization_core
