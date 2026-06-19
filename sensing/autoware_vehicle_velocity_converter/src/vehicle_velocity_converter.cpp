// Copyright 2021 TierIV
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

#include "vehicle_velocity_converter.hpp"

#include <autoware_utils_geometry/msg/covariance.hpp>

namespace autoware::vehicle_velocity_converter
{
geometry_msgs::msg::TwistWithCovarianceStamped convert(
  const autoware_vehicle_msgs::msg::VelocityReport & msg, const double speed_scale_factor,
  const double stddev_vx, const double stddev_wz)
{
  using COV_IDX = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;

  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance_msg;
  twist_with_covariance_msg.header = msg.header;
  twist_with_covariance_msg.twist.twist.linear.x = msg.longitudinal_velocity * speed_scale_factor;
  twist_with_covariance_msg.twist.twist.linear.y = msg.lateral_velocity;
  twist_with_covariance_msg.twist.twist.angular.z = msg.heading_rate;
  twist_with_covariance_msg.twist.covariance[COV_IDX::X_X] = stddev_vx * stddev_vx;
  twist_with_covariance_msg.twist.covariance[COV_IDX::Y_Y] = 10000.0;
  twist_with_covariance_msg.twist.covariance[COV_IDX::Z_Z] = 10000.0;
  twist_with_covariance_msg.twist.covariance[COV_IDX::ROLL_ROLL] = 10000.0;
  twist_with_covariance_msg.twist.covariance[COV_IDX::PITCH_PITCH] = 10000.0;
  twist_with_covariance_msg.twist.covariance[COV_IDX::YAW_YAW] = stddev_wz * stddev_wz;

  return twist_with_covariance_msg;
}
}  // namespace autoware::vehicle_velocity_converter
