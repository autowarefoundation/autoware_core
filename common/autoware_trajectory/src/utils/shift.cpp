// Copyright 2024 TIER IV, Inc.
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

#include "autoware/trajectory/utils/shift.hpp"

#include "autoware/trajectory/detail/logging.hpp"
#include "autoware/trajectory/interpolator/cubic_spline.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{

ShiftParameters::ShiftParameters(
  const double velocity, const double lateral_acc_limit, const double longitudinal_acc)
: velocity(velocity), lateral_acc_limit(lateral_acc_limit), longitudinal_acc(longitudinal_acc)
{
}

namespace detail
{
// This function calculates base longitudinal and lateral lengths
// when acceleration limit is not considered (simple division approach).
std::pair<std::vector<double>, std::vector<double>> get_base_lengths_without_accel_limit(
  const double arc_length, const double shift_length)
{
  // Alias for clarity
  const double total_arc_length = arc_length;
  const double total_shift_length = shift_length;

  // Prepare base longitudinal positions
  const std::vector<double> base_longitudinal = {
    0.0, 0.25 * total_arc_length, 0.75 * total_arc_length, total_arc_length};

  // Prepare base lateral positions
  const std::vector<double> base_lateral = {
    0.0, 1.0 / 12.0 * total_shift_length, 11.0 / 12.0 * total_shift_length, total_shift_length};

  return {base_longitudinal, base_lateral};
}

// This function calculates base longitudinal and lateral lengths
// when acceleration limit is not considered, but velocity and acceleration are known.
std::pair<std::vector<double>, std::vector<double>> get_base_lengths_without_accel_limit(
  const double arc_length, const double shift_length, const double velocity,
  const double longitudinal_acc, const double total_time)
{
  // Aliases for clarity
  const double total_arc_length = arc_length;
  const double total_shift_length = shift_length;
  const double v0 = velocity;         // initial velocity
  const double a = longitudinal_acc;  // longitudinal acceleration
  const double t = total_time / 4.0;  // quarter of total time

  // Calculate first segment in longitudinal direction
  // s1 = v0 * t + 1/2 * a * t^2 (but capped by total_arc_length)
  const double segment_s1 = std::min(v0 * t + 0.5 * a * t * t, total_arc_length);

  // Velocity at the end of first segment
  const double v1 = v0 + a * t;

  // Calculate second segment in longitudinal direction
  // s2 = s1 + 2 * v1 * t + 2 * a * t^2 (but capped by total_arc_length)
  const double segment_s2 = std::min(segment_s1 + 2.0 * v1 * t + 2.0 * a * t * t, total_arc_length);

  // Prepare base longitudinal positions
  const std::vector<double> base_longitudinal = {0.0, segment_s1, segment_s2, total_arc_length};

  // Prepare base lateral positions (simple division approach as original)
  const std::vector<double> base_lateral = {
    0.0, 1.0 / 12.0 * total_shift_length, 11.0 / 12.0 * total_shift_length, total_shift_length};

  return {base_longitudinal, base_lateral};
}

tl::expected<std::pair<std::vector<double>, std::vector<double>>, ShiftError> calc_base_lengths(
  const double & arc_length, const double & shift_length, const ShiftParameters & shift_parameters)
{
  // Threshold for treating acceleration as zero
  const double acc_threshold = 1.0e-4;

  // Extract initial velocity and clamp negative acceleration to zero
  const double v_0_lon = std::abs(shift_parameters.velocity);
  const double a_lon =
    (shift_parameters.longitudinal_acc > acc_threshold) ? shift_parameters.longitudinal_acc : 0.0;

  // If there is no need to consider acceleration limit
  if (v_0_lon < 1.0e-5 && a_lon < acc_threshold) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), get_clock(), 3000,
      "Velocity is effectively zero. "
      "No lateral acceleration limit will be applied.");
    return get_base_lengths_without_accel_limit(arc_length, shift_length);
  }

  // Prepare main parameters
  const double L_lon = arc_length;          // NOLINT
  const double L = std::abs(shift_length);  // NOLINT

  // Calculate total time (total_time) to travel 'target_arclength'
  // If effective_lon_acc is valid (> 0), use time from kinematic formula. Otherwise, use s/v
  const double T_total = (a_lon > acc_threshold)  // NOLINT
                           ? (-v_0_lon + std::sqrt(v_0_lon * v_0_lon + 2.0 * a_lon * L_lon)) / a_lon
                           : (L_lon / v_0_lon);

  // Calculate the maximum lateral acceleration if we do not add further constraints
  const double max_lateral_acc = 8.0 * L / (T_total * T_total);

  // If the max_lateral_acc is already below the limit, no need to reduce it
  const double a_lim_lat = shift_parameters.lateral_acc_limit;
  if (max_lateral_acc < a_lim_lat) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), get_clock(), 3000, "No need to consider lateral acc limit. max: %f, limit: %f",
      max_lateral_acc, shift_parameters.lateral_acc_limit);
    return get_base_lengths_without_accel_limit(L_lon, shift_length, v_0_lon, a_lon, T_total);
  }

  // Compute intermediate times (jerk_time / accel_time) and lateral jerk
  const double T_j = T_total / 2.0 - 2.0 * L / (a_lim_lat * T_total);  // NOLINT
  const double T_a = 4.0 * L / (a_lim_lat * T_total) - T_total / 2.0;  // NOLINT
  const double jerk =
    (2.0 * a_lim_lat * a_lim_lat * T_total) / (a_lim_lat * T_total * T_total - 4.0 * L);

  // If computed times or jerk are invalid (negative or too small), skip the acc limit
  if (T_j < 0.0 || T_a < 0.0 || jerk < 0.0) {
    std::stringstream ss;
    ss << "computed maximum lateral acc: " << max_lateral_acc << " (hard limit is " << a_lim_lat
       << ")" << std::endl;
    if (T_j < 0.0) {
      ss << "Infeasible: T_j is negative" << std::endl;
    }
    if (T_a < 0.0) {
      ss << "Infeasible: T_a is negative" << std::endl;
    }
    if (jerk < 0.0) {
      ss << "Infeasible: jerk is negative" << std::endl;
    }
    ss << "(1)Increase the lateral_acc_limit or (2)Decrease shift_length if possible or "
          "(3)Re-check velocity and acceleration settings for consistency"
       << std::endl;
    return tl::unexpected(ShiftError{ss.str()});
  }

  // Precompute powers for jerk_time and accel_time
  const double jerk_time3 = std::pow(T_j, 3);
  const double accel_time2_jerk = std::pow(T_a, 2) * T_j;
  const double accel_time_jerk2 = T_a * std::pow(T_j, 2);

  // ------------------------------------------------------
  // Calculate longitudinal base points
  // ------------------------------------------------------
  // Segment s1
  const double s1 = std::min(T_j * v_0_lon + 0.5 * a_lon * T_j * T_j, L_lon);
  const double v1 = v_0_lon + a_lon * T_j;

  // Segment s2
  const double s2 = std::min(s1 + T_a * v1 + 0.5 * a_lon * T_a * T_a, L_lon);
  const double v2 = v1 + a_lon * T_a;

  // Segment s3 = s4
  const double s3 = std::min(s2 + T_j * v2 + 0.5 * a_lon * T_j * T_j, L_lon);
  const double v3 = v2 + a_lon * T_j;

  // Segment s5
  const double s5 = std::min(s3 + T_j * v3 + 0.5 * a_lon * T_j * T_j, L_lon);
  const double v5 = v3 + a_lon * T_j;

  // Segment s6
  const double s6 = std::min(s5 + T_a * v5 + 0.5 * a_lon * std::pow(T_a, 2), L_lon);
  const double v6 = v5 + a_lon * T_a;

  // Segment s7
  const double s7 = std::min(s6 + T_j * v6 + 0.5 * a_lon * std::pow(T_j, 2), L_lon);

  // ------------------------------------------------------
  // Calculate lateral base points
  // ------------------------------------------------------
  // sign determines the direction of shift
  const double shift_sign = (shift_length > 0.0) ? 1.0 : -1.0;

  // Shift amounts at each segment
  const double l1 = shift_sign * (1.0 / 6.0 * jerk * jerk_time3);
  const double l2 = shift_sign * (1.0 / 6.0 * jerk * jerk_time3 + 0.5 * jerk * accel_time_jerk2 +
                                  0.5 * jerk * accel_time2_jerk);
  const double l3 = shift_sign * (jerk * jerk_time3 + 1.5 * jerk * accel_time_jerk2 +
                                  0.5 * jerk * accel_time2_jerk);  // = l4
  const double l5 = shift_sign * (11.0 / 6.0 * jerk * jerk_time3 + 2.5 * jerk * accel_time_jerk2 +
                                  0.5 * jerk * accel_time2_jerk);
  const double l6 = shift_sign * (11.0 / 6.0 * jerk * jerk_time3 + 3.0 * jerk * accel_time_jerk2 +
                                  jerk * accel_time2_jerk);
  const double l7 = shift_sign * (2.0 * jerk * jerk_time3 + 3.0 * jerk * accel_time_jerk2 +
                                  jerk * accel_time2_jerk);

  // Construct the output vectors
  const std::vector<double> base_lon = {0.0, s1, s2, s3, s5, s6, s7};
  const std::vector<double> base_lat = {0.0, l1, l2, l3, l5, l6, l7};

  return std::make_pair(base_lon, base_lat);
}

static std::pair<std::vector<double>, std::vector<double>> sanitize_same_base(
  const std::pair<std::vector<double>, std::vector<double>> & lon_lat_original)
{
  static constexpr double k_base_threshold = 1e-3;
  const auto & [lon_original, lat_original] = lon_lat_original;
  std::vector<double> base_lon;
  std::vector<double> base_lat;
  base_lon.push_back(lon_original.front());
  base_lat.push_back(lat_original.front());
  for (const auto [lon, lat] : ranges::views::zip(lon_original, lat_original)) {
    if (std::fabs(base_lon.back() - lon) < k_base_threshold) {
      continue;
    }
    base_lon.push_back(lon);
    base_lat.push_back(lat);
  }
  return {base_lon, base_lat};
}

tl::expected<ShiftElement, ShiftError> shift_impl(
  ShiftElement shift_element, const ShiftInterval & shift_interval,
  const ShiftParameters & shift_parameters)
{
  if (std::max(shift_interval.start, shift_interval.end) <= shift_element.lon_bases.front()) {
    for (auto & lateral_shift : shift_element.lat_shifts) {
      lateral_shift += shift_interval.lateral_offset;
    }
    return shift_element;
  }

  if (std::min(shift_interval.start, shift_interval.end) >= shift_element.lon_bases.back()) {
    return shift_element;
  }

  auto cubic_spline_original = interpolator::CubicSpline::Builder{}
                                 .set_bases(shift_element.lon_bases)
                                 .set_values(shift_element.lat_shifts)
                                 .build();

  if (!cubic_spline_original) {
    return tl::unexpected(ShiftError{"Failed to build cubic spline of original"});
  }

  const double shift_arc_length = std::abs(shift_interval.end - shift_interval.start);
  // Calculate base lengths
  const auto try_calc_base_length = calc_base_lengths(
    shift_arc_length,               //
    shift_interval.lateral_offset,  //
    shift_parameters);
  if (!try_calc_base_length) {
    return tl::unexpected(try_calc_base_length.error());
  }
  // base_lon starts from 0.0
  // NOTE(soblin): calc_base_length may be like [s0, s1, L_lon, L_lon, L_lon], in which case it
  // causes zero division in interpolator
  const auto [base_lon, base_lat] = sanitize_same_base(try_calc_base_length.value());

  auto cubic_spline_shift =
    interpolator::CubicSpline::Builder{}.set_bases(base_lon).set_values(base_lat).build();

  // for above zero division reason, cubic spline may fail
  if (!cubic_spline_shift) {
    std::stringstream ss;
    ss << "Failed to build cubic spline for shift calculation because interval [s0, ... ,s7] "
          "reached L_lon: " +
            cubic_spline_shift.error().what
       << std::endl;
    ss << "L_lon = " << (shift_interval.end - shift_interval.start)
       << ", L = " << shift_interval.lateral_offset << std::endl;
    ss << "v_lon = " << shift_parameters.velocity
       << ", lat_acc_limit = " << shift_parameters.lateral_acc_limit << std::endl;
    return tl::unexpected{ShiftError{ss.str()}};
  }

  std::set<double> merged_bases{shift_element.lon_bases.begin(), shift_element.lon_bases.end()};
  for (const auto new_base : base_lon) {
    merged_bases.insert(new_base + shift_interval.start);
  }

  const auto shift_start_it = merged_bases.find(base_lon.front() + shift_interval.start);
  if (shift_start_it == merged_bases.end()) {
    return tl::unexpected(ShiftError{"could not find shift start base in shift_impl"});
  }
  const auto shift_end_it = merged_bases.find(base_lon.back() + shift_interval.start);
  if (shift_end_it == merged_bases.end()) {
    return tl::unexpected(ShiftError{"could not find shift end base in shift_impl"});
  }

  // Erase elements from merged_bases that are less than shift_element.lon_bases.front() or greater
  // than shift_element.lon_bases.back().
  for (auto it = merged_bases.begin(); it != merged_bases.end();) {
    if (*it < shift_element.lon_bases.front() || *it > shift_element.lon_bases.back()) {
      it = merged_bases.erase(it);  // erase returns the next iterator
    } else {
      ++it;
    }
  }

  std::vector<double> new_bases;
  std::vector<double> shift_values;
  new_bases.reserve(merged_bases.size());
  shift_values.reserve(merged_bases.size());

  for (const auto s : merged_bases) {
    // Calculate the shift length at the current base
    new_bases.push_back(s);
    double original = cubic_spline_original->compute(s);
    if (s < shift_interval.start) {
      // before shifted
      shift_values.push_back(original);
    } else if (s <= shift_interval.end) {
      // middle
      shift_values.push_back(original + cubic_spline_shift->compute(s - shift_interval.start));
    } else {
      // after shifted
      shift_values.push_back(original + cubic_spline_shift->compute(shift_arc_length));
    }
  }

  shift_element.lon_bases = std::move(new_bases);
  shift_element.lat_shifts = std::move(shift_values);

  return shift_element;
}
}  // namespace detail
}  // namespace autoware::experimental::trajectory
