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

  // Derived parameters, hand-computed from the nominal inputs (front_overhang 1.0,
  // wheel_base 2.74, rear_overhang 1.03, wheel_tread 1.63, left/right_overhang 0.1,
  // vehicle_height 2.5). Tolerances absorb binary floating-point rounding of the literals.
  // vehicle_length = front_overhang + wheel_base + rear_overhang = 1.0 + 2.74 + 1.03 = 4.77.
  EXPECT_NEAR(info.vehicle_length_m, 4.77, 1e-9);
  // vehicle_width = wheel_tread + left_overhang + right_overhang = 1.63 + 0.1 + 0.1 = 1.83.
  EXPECT_NEAR(info.vehicle_width_m, 1.83, 1e-9);
  // min_longitudinal_offset = -rear_overhang = -1.03.
  EXPECT_DOUBLE_EQ(info.min_longitudinal_offset_m, -1.03);
  // max_longitudinal_offset = front_overhang + wheel_base = 1.0 + 2.74 = 3.74.
  EXPECT_NEAR(info.max_longitudinal_offset_m, 3.74, 1e-9);
  // min_lateral_offset = -(wheel_tread / 2 + right_overhang) = -(0.815 + 0.1) = -0.915.
  EXPECT_NEAR(info.min_lateral_offset_m, -0.915, 1e-9);
  // max_lateral_offset = wheel_tread / 2 + left_overhang = 0.815 + 0.1 = 0.915.
  EXPECT_NEAR(info.max_lateral_offset_m, 0.915, 1e-9);
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
  // Derived values reflect the clamped wheel_base (1e-6). Hand-computed:
  // vehicle_length = front_overhang + clamped_wheel_base + rear_overhang
  //                = 1.0 + 1e-6 + 1.03 = 2.030001.
  EXPECT_NEAR(info.vehicle_length_m, 2.030001, 1e-12);
  // max_longitudinal_offset = front_overhang + clamped_wheel_base = 1.0 + 1e-6 = 1.000001.
  EXPECT_NEAR(info.max_longitudinal_offset_m, 1.000001, 1e-12);
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
  // Hand-computed from front_overhang -1.0, wheel_base 2.74, rear_overhang 1.03:
  EXPECT_DOUBLE_EQ(info.front_overhang_m, -1.0);
  // vehicle_length = -1.0 + 2.74 + 1.03 = 2.77.
  EXPECT_NEAR(info.vehicle_length_m, 2.77, 1e-9);
  // max_longitudinal_offset = -1.0 + 2.74 = 1.74.
  EXPECT_NEAR(info.max_longitudinal_offset_m, 1.74, 1e-9);
}

// calcMaxMinDimension: branch where rear_overhang_m <= base2front.
TEST(VehicleInfoPure, calc_max_min_dimension_rear_overhang_le_base2front)
{
  const auto info = makeNominalVehicleInfo();
  // The nominal inputs select the rear_overhang_m <= base2front branch. Hand-computed:
  //   vehicle_length = 1.0 + 2.74 + 1.03 = 4.77, rear_overhang = 1.03,
  //   base2front = 4.77 - 1.03 = 3.74; 1.03 <= 3.74 -> first branch.
  //   half_width = vehicle_width / 2 = 1.83 / 2 = 0.915.

  const auto [max_dimension, min_dimension] = info.calcMaxMinDimension();
  // min = min(half_width, rear_overhang) = min(0.915, 1.03) = 0.915.
  EXPECT_NEAR(min_dimension, 0.915, 1e-9);
  // max = hypot(base2front, half_width) = hypot(3.74, 0.915); inputs chosen by the test,
  // only the stdlib is shared with the implementation.
  EXPECT_NEAR(max_dimension, std::hypot(3.74, 0.915), 1e-9);
}

// calcMaxMinDimension: branch where rear_overhang_m > base2front.
TEST(VehicleInfoPure, calc_max_min_dimension_rear_overhang_gt_base2front)
{
  // Construct a shape with a large rear overhang so the else branch is taken.
  // vehicle_length_m = 4.0, rear_overhang_m = 3.0 -> base2front = 1.0, 3.0 > 1.0.
  const auto info = VehicleInfo::createVehicleInfoForVehicleShape(
    /*length=*/4.0, /*width=*/2.0, /*base_length=*/2.0, /*max_steering=*/0.5,
    /*base2back=*/3.0);
  // Independently hand-computed: base2front = length - rear_overhang = 4.0 - 3.0 = 1.0,
  // so rear_overhang (3.0) > base2front (1.0) and the else branch is taken.

  const auto [max_dimension, min_dimension] = info.calcMaxMinDimension();
  // half_width = width / 2 = 2.0 / 2 = 1.0; min = min(half_width, base2front) = min(1.0, 1.0)
  // = 1.0.
  EXPECT_DOUBLE_EQ(min_dimension, 1.0);
  // max = hypot(rear_overhang, half_width) = hypot(3.0, 1.0); inputs chosen by the test.
  EXPECT_DOUBLE_EQ(max_dimension, std::hypot(3.0, 1.0));
}

// extendVehicleInfo: margin is distributed as documented; untouched fields stay zero.
TEST(VehicleInfoPure, extend_vehicle_info_distributes_margin)
{
  const auto base = VehicleInfo::createVehicleInfoForVehicleShape(
    /*length=*/4.0, /*width=*/2.0, /*base_length=*/2.5, /*max_steering=*/0.6,
    /*base2back=*/1.0);

  const auto extended = extendVehicleInfo(base, /*margin=*/0.4);

  // Expected values hand-computed for margin 0.4, independently of the implementation:
  // vehicle_length = length + margin = 4.0 + 0.4 = 4.4.
  EXPECT_DOUBLE_EQ(extended.vehicle_length_m, 4.4);
  // vehicle_width = width + margin = 2.0 + 0.4 = 2.4.
  EXPECT_DOUBLE_EQ(extended.vehicle_width_m, 2.4);
  EXPECT_DOUBLE_EQ(extended.wheel_base_m, 2.5);
  EXPECT_DOUBLE_EQ(extended.max_steer_angle_rad, 0.6);
  // rear_overhang = base2back + margin / 2 = 1.0 + 0.2 = 1.2.
  EXPECT_DOUBLE_EQ(extended.rear_overhang_m, 1.2);
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

  // All expected coordinates are hand-computed from the nominal inputs and the chosen
  // margins, independently of the implementation's expressions:
  //   front_overhang 1.0, wheel_base 2.74, rear_overhang 1.03, half_tread 1.63/2 = 0.815,
  //   left/right_overhang 0.1.
  // x_front  = front_overhang + wheel_base + front_lon = 1.0 + 2.74 + 0.5 = 4.24.
  // x_center = 0.0 (center_at_base_link).
  // x_rear   = -(rear_overhang + rear_lon) = -(1.03 + 0.6) = -1.63.
  constexpr double x_front = 4.24;
  constexpr double x_center = 0.0;
  constexpr double x_rear = -1.63;

  // y_left_*  = half_tread + left_overhang  + lat_margin =  0.815 + 0.1 + lat_margin.
  // y_right_* = -(half_tread + right_overhang + lat_margin) = -(0.815 + 0.1 + lat_margin).
  // front (lat 0.2): 0.815 + 0.1 + 0.2 = 1.115.
  // center (lat 0.3): 0.815 + 0.1 + 0.3 = 1.215.
  // rear (lat 0.4): 0.815 + 0.1 + 0.4 = 1.315.
  constexpr double y_left_front = 1.115;
  constexpr double y_right_front = -1.115;
  constexpr double y_left_center = 1.215;
  constexpr double y_right_center = -1.215;
  constexpr double y_left_rear = 1.315;
  constexpr double y_right_rear = -1.315;

  constexpr double tol = 1e-9;
  // Index 0: front-left
  EXPECT_NEAR(footprint.at(0).x(), x_front, tol);
  EXPECT_NEAR(footprint.at(0).y(), y_left_front, tol);
  // Index 1: front-right
  EXPECT_NEAR(footprint.at(1).x(), x_front, tol);
  EXPECT_NEAR(footprint.at(1).y(), y_right_front, tol);
  // Index 2: center-right
  EXPECT_NEAR(footprint.at(2).x(), x_center, tol);
  EXPECT_NEAR(footprint.at(2).y(), y_right_center, tol);
  // Index 3: rear-right
  EXPECT_NEAR(footprint.at(3).x(), x_rear, tol);
  EXPECT_NEAR(footprint.at(3).y(), y_right_rear, tol);
  // Index 4: rear-left
  EXPECT_NEAR(footprint.at(4).x(), x_rear, tol);
  EXPECT_NEAR(footprint.at(4).y(), y_left_rear, tol);
  // Index 5: center-left
  EXPECT_NEAR(footprint.at(5).x(), x_center, tol);
  EXPECT_NEAR(footprint.at(5).y(), y_left_center, tol);
  // Index 6: closing point == front-left
  EXPECT_NEAR(footprint.at(6).x(), x_front, tol);
  EXPECT_NEAR(footprint.at(6).y(), y_left_front, tol);
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
  // Hand-computed: wheel_base 2.74 / 2 = 1.37.
  EXPECT_NEAR(footprint.at(2).x(), 1.37, 1e-9);
  EXPECT_NEAR(footprint.at(5).x(), 1.37, 1e-9);
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
  // Independently hand-computed: curvature = tan(steer) / wheel_base = tan(0.3) / 2.74.
  // tan(0.3) = 0.30933624960962325 (stdlib), / 2.74 = 0.11289644146336614.
  EXPECT_NEAR(info.calcCurvatureFromSteerAngle(0.3), 0.11289644146336614, 1e-12);
}
