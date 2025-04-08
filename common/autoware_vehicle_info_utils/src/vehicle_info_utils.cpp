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

#include "autoware/vehicle_info_utils/vehicle_info_utils.hpp"

#include <string>

namespace
{
template <class T>
T getParameter(rclcpp::Node & node, const std::string & name)
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }

  try {
    return node.declare_parameter<T>(name);
  } catch (const rclcpp::ParameterTypeException & ex) {
    static constexpr const char * ERROR_MESSAGE =
      "Failed to get parameter `%s`, please set it when you launch the node.";

    RCLCPP_ERROR(node.get_logger(), ERROR_MESSAGE, name.c_str());
    throw;
  }
}
}  // namespace

namespace autoware::vehicle_info_utils
{
VehicleInfoUtils::VehicleInfoUtils(rclcpp::Node & node)
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

  const auto wheel_radius_m = getParameter<double>(node, WHEEL_RADIUS);
  const auto wheel_width_m = getParameter<double>(node, WHEEL_WIDTH);
  const auto wheel_base_m = getParameter<double>(node, WHEEL_BASE);
  const auto wheel_tread_m = getParameter<double>(node, WHEEL_TREAD);
  const auto front_overhang_m = getParameter<double>(node, FRONT_OVERHANG);
  const auto rear_overhang_m = getParameter<double>(node, REAR_OVERHANG);
  const auto left_overhang_m = getParameter<double>(node, LEFT_OVERHANG);
  const auto right_overhang_m = getParameter<double>(node, RIGHT_OVERHANG);
  const auto vehicle_height_m = getParameter<double>(node, VEHICLE_HEIGHT);
  const auto max_steer_angle_rad = getParameter<double>(node, MAX_STEER_ANGLE);

  vehicle_info_ = createVehicleInfo(
    wheel_radius_m, wheel_width_m, wheel_base_m, wheel_tread_m, front_overhang_m, rear_overhang_m,
    left_overhang_m, right_overhang_m, vehicle_height_m, max_steer_angle_rad);
}

VehicleInfo VehicleInfoUtils::getVehicleInfo() const
{
  return vehicle_info_;
}
}  // namespace autoware::vehicle_info_utils
