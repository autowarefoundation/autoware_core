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

#ifndef AUTOWARE__VEHICLE_VELOCITY_CONVERTER__VEHICLE_VELOCITY_CONVERTER_HPP_
#define AUTOWARE__VEHICLE_VELOCITY_CONVERTER__VEHICLE_VELOCITY_CONVERTER_HPP_

#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <string>

namespace autoware::vehicle_velocity_converter
{

class VehicleVelocityConverter
{
public:
  struct ConversionResult
  {
    geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance;
    bool frame_id_matched;
  };

  explicit VehicleVelocityConverter(
    const std::string frame_id,
    const double velocity_stddev_xx,
    const double angular_velocity_stddev_zz,
    const double speed_scale_factor);

  ConversionResult convert(const autoware_vehicle_msgs::msg::VelocityReport & velocity_report) const;

private:
  std::string frame_id_;
  double covariance_xx_;
  double covariance_zz_;
  double speed_scale_factor_;
};

}  // namespace autoware::vehicle_velocity_converter

#endif  // AUTOWARE__VEHICLE_VELOCITY_CONVERTER__VEHICLE_VELOCITY_CONVERTER_HPP_
