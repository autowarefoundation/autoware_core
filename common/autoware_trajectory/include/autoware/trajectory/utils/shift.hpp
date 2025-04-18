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

#ifndef AUTOWARE__TRAJECTORY__UTILS__SHIFT_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__SHIFT_HPP_

#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/forward.hpp"

#include <range/v3/view/zip.hpp>
#include <tl_expected/expected.hpp>

#include <string>
#include <vector>

namespace autoware::experimental::trajectory
{

/**
 * @struct ShiftInterval
 * @brief Represents an interval for shifting a trajectory.
 */
struct ShiftInterval
{
  const double start;           ///< Start position of the shift interval.
  const double end;             ///< End position of the shift interval.
  const double lateral_offset;  ///< Length of the shift to be applied.
};

/**
 * @struct ShiftParameters
 * @brief Represents parameters for shifting a trajectory.
 */
struct ShiftParameters
{
  ShiftParameters(
    const double velocity_, const double lateral_acc_limit_, const double longitudinal_acc_ = 0.0)
  : velocity(velocity_), lateral_acc_limit(lateral_acc_limit_), longitudinal_acc(longitudinal_acc_)
  {
  }
  const double velocity;           ///< Velocity parameter for the shift.
  const double lateral_acc_limit;  ///< Lateral acceleration limit for the shift.
  const double longitudinal_acc;   ///< Longitudinal acceleration parameter for the shift.
};

struct ShiftError
{
  const std::string what;
};

namespace detail
{

struct ShiftElementWithInterval
{
  const std::vector<double> lon_bases;
  const std::vector<double> lat_shifts;
  const size_t shift_start_index;
  const size_t shift_end_index;
};

/**
 * @brief Internal implementation to apply a shift to a trajectory.
 * @param reference_bases A vector of double values representing the sequence of bases for the
 * trajectory.
 * @param shift_interval The interval over which the shift is applied.
 * @param shift_parameters The parameters for the shift.
 * @return pair of two vector<double> indicating the new base (original base + additional shift
 * base) and shift value at each base if feasible, otherwise ShiftError type
 */
tl::expected<ShiftElementWithInterval, ShiftError> shift_impl(
  const std::vector<double> & reference_bases, const ShiftInterval & shift_interval,
  const ShiftParameters & shift_parameters);

}  // namespace detail

template <typename PointType>
struct ShiftedTrajectory
{
  trajectory::Trajectory<PointType> trajectory;
  const double shift_start_s;
  const double shift_end_s;
};

/**
 * @brief Shifts a trajectory based on a single shift interval and parameters.
 * @tparam PointType The type of points in the trajectory.
 * @param reference_trajectory The reference trajectory to be shifted.
 * @param shift_interval The interval for shifting.
 * @param shift_parameters The parameters for the shift.
 * @return The shifted trajectory.
 */
template <typename PointType>
tl::expected<ShiftedTrajectory<PointType>, ShiftError> shift(
  const trajectory::Trajectory<PointType> & reference_trajectory,
  const ShiftInterval & shift_interval, const ShiftParameters & shift_parameters)
{
  if (shift_parameters.velocity <= 0.0) {
    return tl::unexpected{ShiftError{"only positive longitudinal velocity is supported"}};
  }
  if (
    reference_trajectory.start() > shift_interval.start ||
    reference_trajectory.end() < shift_interval.end) {
    return tl::unexpected{ShiftError{"given shift interval is out of trajectory range"}};
  }
  const double shift_arc_length = std::abs(shift_interval.end - shift_interval.start);
  if (shift_arc_length <= 0.0) {
    return tl::unexpected{ShiftError{"only forward shift is supported"}};
  }

  const auto try_shift_bases = detail::shift_impl(
    reference_trajectory.get_underlying_bases(), shift_interval, shift_parameters);
  if (!try_shift_bases) {
    return tl::unexpected{try_shift_bases.error()};
  }
  const auto & [shift_bases, shift_values, shift_start_index, shift_end_index] =
    try_shift_bases.value();

  // Apply shift.
  std::vector<PointType> shifted_points;
  shifted_points.reserve(shift_bases.size());
  for (const auto [base, shift_length] : ranges::views::zip(shift_bases, shift_values)) {
    shifted_points.emplace_back(reference_trajectory.compute(base));
    const double azimuth = reference_trajectory.azimuth(base);
    detail::to_point(shifted_points.back()).x += std::sin(azimuth) * shift_length;
    detail::to_point(shifted_points.back()).y -= std::cos(azimuth) * shift_length;
  }
  auto shifted_trajectory = reference_trajectory;
  const auto valid = shifted_trajectory.build(shifted_points);
  if (!valid) {
    // This Exception is never thrown, because `shifted_bases` contains at least 4 or 7 new bases in
    // addition to reference_bases, which is enough for cubic spline
    return tl::unexpected{ShiftError{"Failed to build cubic spline for shift calculation."}};
  }
  const auto shifted_trajectory_bases = shifted_trajectory.get_underlying_bases();
  return ShiftedTrajectory<PointType>{
    shifted_trajectory, shifted_trajectory_bases.at(shift_start_index),
    shifted_trajectory_bases.at(shift_end_index)};
}

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__SHIFT_HPP_
