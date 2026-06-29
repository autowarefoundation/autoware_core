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

#ifndef AUTOWARE__VEHICLE_INFO_UTILS__VEHICLE_INFO_UTILS_HPP_
#define AUTOWARE__VEHICLE_INFO_UTILS__VEHICLE_INFO_UTILS_HPP_

#include "autoware/vehicle_info_utils/vehicle_info.hpp"

#include <autoware_utils_rclcpp/parameter.hpp>
#include <rclcpp/node.hpp>

#include <type_traits>

namespace autoware::vehicle_info_utils
{
/// This is a convenience class for saving you from declaring all parameters
/// manually and calculating derived parameters.
/// This class supposes that necessary parameters are set when the node is launched.
///
class VehicleInfoUtils
{
public:
  /// Constructor
  // NOTE(soblin): this throws which should be replaced with a factory
  template <typename NodeT>
  explicit VehicleInfoUtils(NodeT & node) : vehicle_info_(create_from_node(node))
  {
  }

  /// Get vehicle info
  [[nodiscard]] VehicleInfo getVehicleInfo() const;

private:
  template <typename NodeT>
  static VehicleInfo create_from_node(NodeT & node)
  {
    static constexpr const char * WHEEL_RADIUS = "wheel_radius";
    static constexpr const char * WHEEL_WIDTH = "wheel_width";
    static constexpr const char * WHEEL_BASE = "wheel_base";
    static constexpr const char * WHEEL_TREAD = "wheel_tread";
    static constexpr const char * FRONT_OVERHANG = "front_overhang";
    static constexpr const char * REAR_OVERHANG = "rear_overhang";
    static constexpr const char * LEFT_OVERHANG = "left_overhang";
    static constexpr const char * RIGHT_OVERHANG = "right_overhang";
    static constexpr const char * VEHICLE_HEIGHT = "vehicle_height";
    static constexpr const char * MAX_STEER_ANGLE = "max_steer_angle";

    using autoware_utils_rclcpp::get_or_declare_parameter;
    const auto wheel_radius_m = get_or_declare_parameter<double>(node, WHEEL_RADIUS);
    const auto wheel_width_m = get_or_declare_parameter<double>(node, WHEEL_WIDTH);
    const auto wheel_base_m = get_or_declare_parameter<double>(node, WHEEL_BASE);
    const auto wheel_tread_m = get_or_declare_parameter<double>(node, WHEEL_TREAD);
    const auto front_overhang_m = get_or_declare_parameter<double>(node, FRONT_OVERHANG);
    const auto rear_overhang_m = get_or_declare_parameter<double>(node, REAR_OVERHANG);
    const auto left_overhang_m = get_or_declare_parameter<double>(node, LEFT_OVERHANG);
    const auto right_overhang_m = get_or_declare_parameter<double>(node, RIGHT_OVERHANG);
    const auto vehicle_height_m = get_or_declare_parameter<double>(node, VEHICLE_HEIGHT);
    const auto max_steer_angle_rad = get_or_declare_parameter<double>(node, MAX_STEER_ANGLE);

    return createVehicleInfo(
      wheel_radius_m, wheel_width_m, wheel_base_m, wheel_tread_m, front_overhang_m, rear_overhang_m,
      left_overhang_m, right_overhang_m, vehicle_height_m, max_steer_angle_rad);
  }

  /// Buffer for base parameters
  VehicleInfo vehicle_info_;
};

}  // namespace autoware::vehicle_info_utils

#endif  // AUTOWARE__VEHICLE_INFO_UTILS__VEHICLE_INFO_UTILS_HPP_
