// Copyright 2021 TierIV
// Copyright 2025 The Autoware Contributors
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

#include "autoware/vehicle_velocity_converter/vehicle_velocity_converter.hpp"

namespace autoware::vehicle_velocity_converter
{

VehicleVelocityConverter::VehicleVelocityConverter(
  const std::string frame_id, const double velocity_stddev_xx,
  const double angular_velocity_stddev_zz, const double speed_scale_factor)
: frame_id_(frame_id),
  covariance_xx_(velocity_stddev_xx * velocity_stddev_xx),
  covariance_zz_(angular_velocity_stddev_zz * angular_velocity_stddev_zz),
  speed_scale_factor_(speed_scale_factor)
{
}

VehicleVelocityConverter::ConversionResult VehicleVelocityConverter::convert(
  const autoware_vehicle_msgs::msg::VelocityReport & velocity_report) const
{
  ConversionResult result;

  result.frame_id_matched = (velocity_report.header.frame_id == frame_id_);

  result.twist_with_covariance.header = velocity_report.header;

  result.twist_with_covariance.twist.twist.linear.x =
    velocity_report.longitudinal_velocity * speed_scale_factor_;
  result.twist_with_covariance.twist.twist.linear.y = velocity_report.lateral_velocity;
  result.twist_with_covariance.twist.twist.angular.z = velocity_report.heading_rate;

  constexpr double kLargeVariance = 10000.0;
  result.twist_with_covariance.twist.covariance[0 + 0 * 6] = covariance_xx_;
  result.twist_with_covariance.twist.covariance[1 + 1 * 6] = kLargeVariance;
  result.twist_with_covariance.twist.covariance[2 + 2 * 6] = kLargeVariance;
  result.twist_with_covariance.twist.covariance[3 + 3 * 6] = kLargeVariance;
  result.twist_with_covariance.twist.covariance[4 + 4 * 6] = kLargeVariance;
  result.twist_with_covariance.twist.covariance[5 + 5 * 6] = covariance_zz_;

  return result;
}

}  // namespace autoware::vehicle_velocity_converter
