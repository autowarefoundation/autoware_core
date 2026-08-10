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

#ifndef VEHICLE_VELOCITY_CONVERTER_HPP_
#define VEHICLE_VELOCITY_CONVERTER_HPP_

#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

namespace autoware::vehicle_velocity_converter
{
/// \brief Converts a VelocityReport into a TwistWithCovarianceStamped using fixed conversion
/// settings.
///
/// The settings (scale factor and standard deviations) do not change frame to frame, so they are
/// supplied once at construction.
class VehicleVelocityConverter
{
public:
  /// \brief Construct the converter.
  /// \param speed_scale_factor multiplier applied to the longitudinal velocity
  /// \param stddev_vx longitudinal velocity standard deviation (variance = stddev_vx^2)
  /// \param stddev_wz yaw rate standard deviation (variance = stddev_wz^2)
  VehicleVelocityConverter(double speed_scale_factor, double stddev_vx, double stddev_wz);

  /// \brief Convert a VelocityReport into a TwistWithCovarianceStamped.
  ///
  /// Applies the longitudinal speed scale factor, maps the report axes onto the twist, and fills
  /// the fixed diagonal covariance (large values on the unobserved axes). The header is copied
  /// verbatim from the input message.
  ///
  /// \param msg input velocity report
  /// \return converted twist with covariance
  geometry_msgs::msg::TwistWithCovarianceStamped convert(
    const autoware_vehicle_msgs::msg::VelocityReport & msg) const;

private:
  double speed_scale_factor_;
  double stddev_vx_;
  double stddev_wz_;
};
}  // namespace autoware::vehicle_velocity_converter

#endif  // VEHICLE_VELOCITY_CONVERTER_HPP_
