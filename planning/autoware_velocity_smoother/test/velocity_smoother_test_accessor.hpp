// Copyright 2025 Autonomous Systems sp. z o.o.
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

#ifndef AUTOWARE__VELOCITY_SMOOTHER__TEST__VELOCITY_SMOOTHER_TEST_ACCESSOR_HPP_
#define AUTOWARE__VELOCITY_SMOOTHER__TEST__VELOCITY_SMOOTHER_TEST_ACCESSOR_HPP_

#include "autoware/velocity_smoother/node.hpp"

#include <cstddef>

namespace autoware::velocity_smoother::test
{

class VelocitySmootherTestAccessor
{
public:
  explicit VelocitySmootherTestAccessor(VelocitySmootherNode & node) : node_(node) {}

  void applyExternalVelocityLimit(TrajectoryPoints & traj) const
  {
    node_.applyExternalVelocityLimit(traj);
  }

  void calcExternalVelocityLimit() { node_.calcExternalVelocityLimit(); }

  double maxVelocityWithDeceleration() const { return node_.max_velocity_with_deceleration_; }

  void setCurrentOdometry(const Odometry::ConstSharedPtr & odom)
  {
    node_.current_odometry_ptr_ = odom;
  }
  void setCurrentAcceleration(const AccelWithCovarianceStamped::ConstSharedPtr & accel)
  {
    node_.current_acceleration_ptr_ = accel;
  }

  void setCurrentClosestPointFromPrevOutput(const TrajectoryPoint & point)
  {
    node_.current_closest_point_from_prev_output_ = point;
  }

  void setExternalVelocityLimitPtr(const VelocityLimit::ConstSharedPtr & external_limit)
  {
    node_.external_velocity_limit_ptr_ = external_limit;
  }

  void setExternalVelocityLimitStateVelocity(const double v)
  {
    node_.external_velocity_limit_.velocity = v;
  }

  void setPrevOutputSize(const std::size_t size) { node_.prev_output_.resize(size); }

  void setMaxVelocityWithDeceleration(const double v) { node_.max_velocity_with_deceleration_ = v; }

  void setExternalVelocityLimitVelocity(const double v)
  {
    node_.external_velocity_limit_.velocity = v;
  }
  void setExternalVelocityLimitDistance(const double dist)
  {
    node_.external_velocity_limit_.dist = dist;
  }

private:
  VelocitySmootherNode & node_;
};

}  // namespace autoware::velocity_smoother::test

#endif  // AUTOWARE__VELOCITY_SMOOTHER__TEST__VELOCITY_SMOOTHER_TEST_ACCESSOR_HPP_
