// Copyright 2015-2021 Autoware Foundation
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

#include "autoware/vehicle_info_utils/vehicle_info.hpp"

#include <rclcpp/rclcpp.hpp>

#include <limits>

namespace autoware::vehicle_info_utils
{
autoware_utils::LinearRing2d VehicleInfo::createFootprint(const double margin) const
{
  return createFootprint(margin, margin);
}

autoware_utils::LinearRing2d VehicleInfo::createFootprint(
  const double lat_margin, const double lon_margin) const
{
  return createFootprint(lat_margin, lat_margin, lat_margin, lon_margin, lon_margin);
}

autoware_utils::LinearRing2d VehicleInfo::createFootprint(
  const double front_lat_margin, const double center_lat_margin, const double rear_lat_margin,
  const double front_lon_margin, const double rear_lon_margin, const bool center_at_base_link) const
{
  using autoware_utils::LinearRing2d;
  using autoware_utils::Point2d;

  // Longitudinal positions
  const double x_front = front_overhang_m + wheel_base_m + front_lon_margin;
  const double x_center = center_at_base_link ? 0.0 : (wheel_base_m / 2.0);
  const double x_rear = -(rear_overhang_m + rear_lon_margin);

  const double y_left_front = wheel_tread_m * 0.5 + left_overhang_m + front_lat_margin;
  const double y_right_front = -(wheel_tread_m * 0.5 + right_overhang_m + front_lat_margin);

  const double y_left_center = wheel_tread_m * 0.5 + left_overhang_m + center_lat_margin;
  const double y_right_center = -(wheel_tread_m * 0.5 + right_overhang_m + center_lat_margin);

  const double y_left_rear = wheel_tread_m * 0.5 + left_overhang_m + rear_lat_margin;
  const double y_right_rear = -(wheel_tread_m * 0.5 + right_overhang_m + rear_lat_margin);

  LinearRing2d footprint;
  footprint.reserve(7);
  footprint.emplace_back(x_front, y_left_front);
  footprint.emplace_back(x_front, y_right_front);
  footprint.emplace_back(x_center, y_right_center);
  footprint.emplace_back(x_rear, y_right_rear);
  footprint.emplace_back(x_rear, y_left_rear);
  footprint.emplace_back(x_center, y_left_center);
  footprint.emplace_back(x_front, y_left_front);

  return footprint;
}

VehicleInfo createVehicleInfo(
  const double wheel_radius_m, const double wheel_width_m, const double wheel_base_m_arg,
  const double wheel_tread_m, const double front_overhang_m, const double rear_overhang_m,
  const double left_overhang_m, const double right_overhang_m, const double vehicle_height_m,
  const double max_steer_angle_rad_arg)
{
  static constexpr double MIN_WHEEL_BASE_M = 1e-6;
  double wheel_base_m = wheel_base_m_arg;
  if (std::abs(wheel_base_m) < MIN_WHEEL_BASE_M) {
    RCLCPP_ERROR(
      rclcpp::get_logger("vehicle_info"), "wheel_base_m %f is almost 0.0, clamping to %f",
      wheel_base_m, MIN_WHEEL_BASE_M);
    wheel_base_m = MIN_WHEEL_BASE_M;
  }

  double max_steer_angle_rad = max_steer_angle_rad_arg;
  static constexpr double MAX_STEER_ANGLE_RAD = 1e-6;
  if (std::abs(max_steer_angle_rad) < MAX_STEER_ANGLE_RAD) {
    RCLCPP_ERROR(
      rclcpp::get_logger("vehicle_info"), "max_steer_angle_rad %f is almost 0.0, clamping to %f",
      max_steer_angle_rad, MAX_STEER_ANGLE_RAD);
    max_steer_angle_rad = MAX_STEER_ANGLE_RAD;
  }

  static constexpr double MIN_POSITIVE_VALUE = 0.0;
  const bool has_non_positive_values =
    wheel_radius_m <= MIN_POSITIVE_VALUE || wheel_width_m <= MIN_POSITIVE_VALUE ||
    wheel_base_m <= MIN_POSITIVE_VALUE || wheel_tread_m <= MIN_POSITIVE_VALUE ||
    front_overhang_m <= MIN_POSITIVE_VALUE || rear_overhang_m <= MIN_POSITIVE_VALUE ||
    left_overhang_m <= MIN_POSITIVE_VALUE || right_overhang_m <= MIN_POSITIVE_VALUE ||
    vehicle_height_m <= MIN_POSITIVE_VALUE || max_steer_angle_rad <= MIN_POSITIVE_VALUE;

  if (has_non_positive_values) {
    RCLCPP_ERROR(
      rclcpp::get_logger("vehicle_info"), "given parameters contain non positive values");
  }

  // Calculate derived parameters
  const double vehicle_length_m_ = front_overhang_m + wheel_base_m + rear_overhang_m;
  const double vehicle_width_m_ = wheel_tread_m + left_overhang_m + right_overhang_m;
  const double min_longitudinal_offset_m_ = -rear_overhang_m;
  const double max_longitudinal_offset_m_ = front_overhang_m + wheel_base_m;
  const double min_lateral_offset_m_ = -(wheel_tread_m / 2.0 + right_overhang_m);
  const double max_lateral_offset_m_ = wheel_tread_m / 2.0 + left_overhang_m;
  const double min_height_offset_m_ = 0.0;
  const double max_height_offset_m_ = vehicle_height_m;

  return VehicleInfo{
    // Base parameters
    wheel_radius_m,
    wheel_width_m,
    wheel_base_m,
    wheel_tread_m,
    front_overhang_m,
    rear_overhang_m,
    left_overhang_m,
    right_overhang_m,
    vehicle_height_m,
    max_steer_angle_rad,
    // Derived parameters
    vehicle_length_m_,
    vehicle_width_m_,
    min_longitudinal_offset_m_,
    max_longitudinal_offset_m_,
    min_lateral_offset_m_,
    max_lateral_offset_m_,
    min_height_offset_m_,
    max_height_offset_m_,
  };
}

double VehicleInfo::calcMaxCurvature() const
{
  const double radius = wheel_base_m / std::tan(max_steer_angle_rad);
  return 1.0 / radius;
}

double VehicleInfo::calcCurvatureFromSteerAngle(const double steer_angle) const
{
  static constexpr double MIN_WHEEL_BASE_M = 1e-6;
  if (wheel_base_m < MIN_WHEEL_BASE_M) {
    RCLCPP_ERROR(
      rclcpp::get_logger("vehicle_info"), "wheel_base_m %f should not be 0 or negative",
      wheel_base_m);
    return std::numeric_limits<double>::quiet_NaN();
  }

  // radius = wheel_base_m / std::tan(steer_angle)
  // curvature = 1.0 / radius;
  // Merge formulas and eliminate variable "radius" to avoid division by zero operations.
  return std::tan(steer_angle) / wheel_base_m;
}

double VehicleInfo::calcSteerAngleFromCurvature(const double curvature) const
{
  static constexpr double MIN_CURVATURE = 1e-6;
  if (std::abs(curvature) < MIN_CURVATURE) {
    return 0.0;
  }

  const double radius = 1.0 / curvature;
  return std::atan2(wheel_base_m, radius);
}
}  // namespace autoware::vehicle_info_utils
