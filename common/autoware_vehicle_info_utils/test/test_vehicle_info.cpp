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

// Unit tests for the pure (no-ROS) helpers in vehicle_info.cpp.
// These exercise createVehicleInfo's clamping/validation branches,
// calcMaxMinDimension's two branches, extendVehicleInfo, the full 5-margin
// createFootprint overload (including center_at_base_link), and the NaN guard
// in calcCurvatureFromSteerAngle. They construct VehicleInfo directly via the
// free functions / factory and never touch rclcpp.

#include <autoware/vehicle_info_utils/vehicle_info.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace
{
using autoware::vehicle_info_utils::createVehicleInfo;
using autoware::vehicle_info_utils::extendVehicleInfo;
using autoware::vehicle_info_utils::VehicleInfo;

// Nominal, all-positive base parameters used as a baseline for several tests.
VehicleInfo makeNominalVehicleInfo()
{
  return createVehicleInfo(
    /*wheel_radius_m=*/0.39, /*wheel_width_m=*/0.42, /*wheel_base_m=*/2.74,
    /*wheel_tread_m=*/1.63, /*front_overhang_m=*/1.0, /*rear_overhang_m=*/1.03,
    /*left_overhang_m=*/0.1, /*right_overhang_m=*/0.1, /*vehicle_height_m=*/2.5,
    /*max_steer_angle_rad=*/0.7);
}
}  // namespace

// createVehicleInfo: nominal inputs produce the documented derived values.
TEST(VehicleInfoPure, create_vehicle_info_derived_values)
{
  const auto info = makeNominalVehicleInfo();

  EXPECT_DOUBLE_EQ(info.wheel_base_m, 2.74);
  EXPECT_DOUBLE_EQ(info.max_steer_angle_rad, 0.7);

  // Derived parameters.
  EXPECT_DOUBLE_EQ(info.vehicle_length_m, 1.0 + 2.74 + 1.03);
  EXPECT_DOUBLE_EQ(info.vehicle_width_m, 1.63 + 0.1 + 0.1);
  EXPECT_DOUBLE_EQ(info.min_longitudinal_offset_m, -1.03);
  EXPECT_DOUBLE_EQ(info.max_longitudinal_offset_m, 1.0 + 2.74);
  EXPECT_DOUBLE_EQ(info.min_lateral_offset_m, -(1.63 / 2.0 + 0.1));
  EXPECT_DOUBLE_EQ(info.max_lateral_offset_m, 1.63 / 2.0 + 0.1);
  EXPECT_DOUBLE_EQ(info.min_height_offset_m, 0.0);
  EXPECT_DOUBLE_EQ(info.max_height_offset_m, 2.5);
}

// createVehicleInfo: wheel_base near 0 is clamped to MIN_WHEEL_BASE_M (1e-6).
TEST(VehicleInfoPure, create_vehicle_info_clamps_zero_wheel_base)
{
  constexpr double min_wheel_base = 1e-6;
  const auto info = createVehicleInfo(
    /*wheel_radius_m=*/0.39, /*wheel_width_m=*/0.42, /*wheel_base_m=*/0.0,
    /*wheel_tread_m=*/1.63, /*front_overhang_m=*/1.0, /*rear_overhang_m=*/1.03,
    /*left_overhang_m=*/0.1, /*right_overhang_m=*/0.1, /*vehicle_height_m=*/2.5,
    /*max_steer_angle_rad=*/0.7);

  EXPECT_DOUBLE_EQ(info.wheel_base_m, min_wheel_base);
  // Derived values reflect the clamped wheel_base.
  EXPECT_DOUBLE_EQ(info.vehicle_length_m, 1.0 + min_wheel_base + 1.03);
  EXPECT_DOUBLE_EQ(info.max_longitudinal_offset_m, 1.0 + min_wheel_base);
}

// createVehicleInfo: tiny negative wheel_base (|value| < 1e-6) is also clamped.
TEST(VehicleInfoPure, create_vehicle_info_clamps_tiny_negative_wheel_base)
{
  constexpr double min_wheel_base = 1e-6;
  const auto info = createVehicleInfo(
    /*wheel_radius_m=*/0.39, /*wheel_width_m=*/0.42, /*wheel_base_m=*/-1e-9,
    /*wheel_tread_m=*/1.63, /*front_overhang_m=*/1.0, /*rear_overhang_m=*/1.03,
    /*left_overhang_m=*/0.1, /*right_overhang_m=*/0.1, /*vehicle_height_m=*/2.5,
    /*max_steer_angle_rad=*/0.7);

  EXPECT_DOUBLE_EQ(info.wheel_base_m, min_wheel_base);
}

// createVehicleInfo: max_steer_angle near 0 is clamped to MAX_STEER_ANGLE_RAD (1e-6).
TEST(VehicleInfoPure, create_vehicle_info_clamps_zero_max_steer_angle)
{
  constexpr double min_steer = 1e-6;
  const auto info = createVehicleInfo(
    /*wheel_radius_m=*/0.39, /*wheel_width_m=*/0.42, /*wheel_base_m=*/2.74,
    /*wheel_tread_m=*/1.63, /*front_overhang_m=*/1.0, /*rear_overhang_m=*/1.03,
    /*left_overhang_m=*/0.1, /*right_overhang_m=*/0.1, /*vehicle_height_m=*/2.5,
    /*max_steer_angle_rad=*/0.0);

  EXPECT_DOUBLE_EQ(info.max_steer_angle_rad, min_steer);
}

// createVehicleInfo: the has_non_positive_values branch only logs; it must not
// alter the stored/derived values. Negative overhangs are passed through verbatim.
TEST(VehicleInfoPure, create_vehicle_info_non_positive_values_pass_through)
{
  const auto info = createVehicleInfo(
    /*wheel_radius_m=*/0.39, /*wheel_width_m=*/0.42, /*wheel_base_m=*/2.74,
    /*wheel_tread_m=*/1.63, /*front_overhang_m=*/-1.0, /*rear_overhang_m=*/1.03,
    /*left_overhang_m=*/0.1, /*right_overhang_m=*/0.1, /*vehicle_height_m=*/2.5,
    /*max_steer_angle_rad=*/0.7);

  // Non-positive front_overhang is preserved (not clamped) and flows into derived values.
  EXPECT_DOUBLE_EQ(info.front_overhang_m, -1.0);
  EXPECT_DOUBLE_EQ(info.vehicle_length_m, -1.0 + 2.74 + 1.03);
  EXPECT_DOUBLE_EQ(info.max_longitudinal_offset_m, -1.0 + 2.74);
}

// calcMaxMinDimension: branch where rear_overhang_m <= base2front.
TEST(VehicleInfoPure, calc_max_min_dimension_rear_overhang_le_base2front)
{
  const auto info = makeNominalVehicleInfo();
  // vehicle_length_m = 4.77, rear_overhang_m = 1.03 -> base2front = 3.74.
  // 1.03 <= 3.74, so the first branch is taken.
  const double base2front = info.vehicle_length_m - info.rear_overhang_m;
  ASSERT_LE(info.rear_overhang_m, base2front);

  const auto [max_dimension, min_dimension] = info.calcMaxMinDimension();
  EXPECT_DOUBLE_EQ(min_dimension, std::min(0.5 * info.vehicle_width_m, info.rear_overhang_m));
  EXPECT_DOUBLE_EQ(max_dimension, std::hypot(base2front, 0.5 * info.vehicle_width_m));
}

// calcMaxMinDimension: branch where rear_overhang_m > base2front.
TEST(VehicleInfoPure, calc_max_min_dimension_rear_overhang_gt_base2front)
{
  // Construct a shape with a large rear overhang so the else branch is taken.
  // vehicle_length_m = 4.0, rear_overhang_m = 3.0 -> base2front = 1.0, 3.0 > 1.0.
  const auto info = VehicleInfo::createVehicleInfoForVehicleShape(
    /*length=*/4.0, /*width=*/2.0, /*base_length=*/2.0, /*max_steering=*/0.5,
    /*base2back=*/3.0);
  const double base2front = info.vehicle_length_m - info.rear_overhang_m;
  ASSERT_GT(info.rear_overhang_m, base2front);

  const auto [max_dimension, min_dimension] = info.calcMaxMinDimension();
  EXPECT_DOUBLE_EQ(min_dimension, std::min(0.5 * info.vehicle_width_m, base2front));
  EXPECT_DOUBLE_EQ(max_dimension, std::hypot(info.rear_overhang_m, 0.5 * info.vehicle_width_m));
  // Concrete values: width 2.0 -> half 1.0; min(1.0, 1.0) = 1.0; hypot(3.0, 1.0).
  EXPECT_DOUBLE_EQ(min_dimension, 1.0);
  EXPECT_DOUBLE_EQ(max_dimension, std::hypot(3.0, 1.0));
}

// extendVehicleInfo: margin is distributed as documented; untouched fields stay zero.
TEST(VehicleInfoPure, extend_vehicle_info_distributes_margin)
{
  const auto base = VehicleInfo::createVehicleInfoForVehicleShape(
    /*length=*/4.0, /*width=*/2.0, /*base_length=*/2.5, /*max_steering=*/0.6,
    /*base2back=*/1.0);

  constexpr double margin = 0.4;
  const auto extended = extendVehicleInfo(base, margin);

  EXPECT_DOUBLE_EQ(extended.vehicle_length_m, 4.0 + margin);
  EXPECT_DOUBLE_EQ(extended.vehicle_width_m, 2.0 + margin);
  EXPECT_DOUBLE_EQ(extended.wheel_base_m, 2.5);
  EXPECT_DOUBLE_EQ(extended.max_steer_angle_rad, 0.6);
  EXPECT_DOUBLE_EQ(extended.rear_overhang_m, 1.0 + margin / 2.0);
  // Fields not set by createVehicleInfoForVehicleShape default to zero.
  EXPECT_DOUBLE_EQ(extended.front_overhang_m, 0.0);
  EXPECT_DOUBLE_EQ(extended.wheel_tread_m, 0.0);
}

// extendVehicleInfo: default margin (0.0) is a pure pass-through of shape fields.
TEST(VehicleInfoPure, extend_vehicle_info_zero_margin)
{
  const auto base = VehicleInfo::createVehicleInfoForVehicleShape(
    /*length=*/4.0, /*width=*/2.0, /*base_length=*/2.5, /*max_steering=*/0.6,
    /*base2back=*/1.0);

  const auto extended = extendVehicleInfo(base);
  EXPECT_DOUBLE_EQ(extended.vehicle_length_m, 4.0);
  EXPECT_DOUBLE_EQ(extended.vehicle_width_m, 2.0);
  EXPECT_DOUBLE_EQ(extended.rear_overhang_m, 1.0);
}

// createFootprint (5-margin overload): asymmetric margins with center_at_base_link=true.
// Verifies all 7 ring points, including the center-right/left points (indices 2 and 5),
// the closing point (index 6), and that the center x is aligned at base_link (x=0).
TEST(VehicleInfoPure, create_footprint_asymmetric_margins_center_at_base_link)
{
  const auto info = makeNominalVehicleInfo();

  constexpr double front_lat = 0.2;
  constexpr double center_lat = 0.3;
  constexpr double rear_lat = 0.4;
  constexpr double front_lon = 0.5;
  constexpr double rear_lon = 0.6;
  constexpr bool center_at_base_link = true;

  const auto footprint = info.createFootprint(
    front_lat, center_lat, rear_lat, front_lon, rear_lon, center_at_base_link, std::nullopt);

  ASSERT_EQ(footprint.size(), 7u);

  const double x_front = info.front_overhang_m + info.wheel_base_m + front_lon;  // 1.0+2.74+0.5
  const double x_center = 0.0;                               // center_at_base_link
  const double x_rear = -(info.rear_overhang_m + rear_lon);  // -(1.03+0.6)

  const double half_tread = info.wheel_tread_m * 0.5;  // 0.815
  const double y_left_front = half_tread + info.left_overhang_m + front_lat;
  const double y_right_front = -(half_tread + info.right_overhang_m + front_lat);
  const double y_left_center = half_tread + info.left_overhang_m + center_lat;
  const double y_right_center = -(half_tread + info.right_overhang_m + center_lat);
  const double y_left_rear = half_tread + info.left_overhang_m + rear_lat;
  const double y_right_rear = -(half_tread + info.right_overhang_m + rear_lat);

  // Index 0: front-left
  EXPECT_DOUBLE_EQ(footprint.at(0).x(), x_front);
  EXPECT_DOUBLE_EQ(footprint.at(0).y(), y_left_front);
  // Index 1: front-right
  EXPECT_DOUBLE_EQ(footprint.at(1).x(), x_front);
  EXPECT_DOUBLE_EQ(footprint.at(1).y(), y_right_front);
  // Index 2: center-right
  EXPECT_DOUBLE_EQ(footprint.at(2).x(), x_center);
  EXPECT_DOUBLE_EQ(footprint.at(2).y(), y_right_center);
  // Index 3: rear-right
  EXPECT_DOUBLE_EQ(footprint.at(3).x(), x_rear);
  EXPECT_DOUBLE_EQ(footprint.at(3).y(), y_right_rear);
  // Index 4: rear-left
  EXPECT_DOUBLE_EQ(footprint.at(4).x(), x_rear);
  EXPECT_DOUBLE_EQ(footprint.at(4).y(), y_left_rear);
  // Index 5: center-left
  EXPECT_DOUBLE_EQ(footprint.at(5).x(), x_center);
  EXPECT_DOUBLE_EQ(footprint.at(5).y(), y_left_center);
  // Index 6: closing point == front-left
  EXPECT_DOUBLE_EQ(footprint.at(6).x(), x_front);
  EXPECT_DOUBLE_EQ(footprint.at(6).y(), y_left_front);
}

// createFootprint (5-margin overload): with center_at_base_link=false the center
// points are placed at wheel_base/2 instead of the base_link origin.
TEST(VehicleInfoPure, create_footprint_center_at_wheelbase_center)
{
  const auto info = makeNominalVehicleInfo();

  const auto footprint = info.createFootprint(
    /*front_lat=*/0.0, /*center_lat=*/0.0, /*rear_lat=*/0.0, /*front_lon=*/0.0,
    /*rear_lon=*/0.0, /*center_at_base_link=*/false, std::nullopt);

  ASSERT_EQ(footprint.size(), 7u);
  // Center points (indices 2 and 5) sit at wheel_base/2 when not aligned to base_link.
  EXPECT_DOUBLE_EQ(footprint.at(2).x(), info.wheel_base_m / 2.0);
  EXPECT_DOUBLE_EQ(footprint.at(5).x(), info.wheel_base_m / 2.0);
}

// calcCurvatureFromSteerAngle: NaN guard when wheel_base_m < 1e-6.
TEST(VehicleInfoPure, calc_curvature_from_steer_angle_nan_guard)
{
  VehicleInfo info{};
  info.wheel_base_m = 0.0;  // below MIN_WHEEL_BASE_M (1e-6)

  EXPECT_TRUE(std::isnan(info.calcCurvatureFromSteerAngle(0.3)));
}

// calcCurvatureFromSteerAngle: valid wheel_base returns tan(steer)/wheel_base.
TEST(VehicleInfoPure, calc_curvature_from_steer_angle_valid)
{
  const auto info = makeNominalVehicleInfo();
  EXPECT_DOUBLE_EQ(info.calcCurvatureFromSteerAngle(0.3), std::tan(0.3) / info.wheel_base_m);
}
