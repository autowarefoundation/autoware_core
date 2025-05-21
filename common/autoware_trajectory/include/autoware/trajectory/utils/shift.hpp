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
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{

// TODO(yhisaki): Add InverseShiftInterval which represents the shift from backward to forward.
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
    const double velocity, const double lateral_acc_limit, const double longitudinal_acc = 0.0);
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

struct ShiftElement
{
  std::vector<double> lon_bases;
  std::vector<double> lat_shifts;

  // Constructor
  explicit ShiftElement(const std::vector<double> & lon_bases)
  : lon_bases(lon_bases), lat_shifts(lon_bases.size(), 0.0)
  {
  }
  size_t size() const { return lon_bases.size(); }
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
tl::expected<ShiftElement, ShiftError> shift_impl(
  ShiftElement shift_element, const ShiftInterval & shift_interval,
  const ShiftParameters & shift_parameters);

}  // namespace detail

/**
 * @brief Shifts a trajectory based on a single shift interval and parameters.
 * @tparam PointType The type of points in the trajectory.
 * @param reference_trajectory The reference trajectory to be shifted.
 * @param shift_interval The interval for shifting.
 * @param shift_parameters The parameters for the shift.
 * @return The shifted trajectory.
 */
template <typename PointType>
tl::expected<trajectory::Trajectory<PointType>, ShiftError> shift(
  const trajectory::Trajectory<PointType> & reference_trajectory,
  const std::vector<ShiftInterval> & shift_intervals, const ShiftParameters & shift_parameters)
{
  if (shift_parameters.velocity < 0.0) {
    return tl::unexpected{ShiftError{"Longitudinal velocity must be positive."}};
  }

  detail::ShiftElement shift_element{reference_trajectory.get_underlying_bases()};

  for (const ShiftInterval & shift_interval : shift_intervals) {
    auto try_shift = detail::shift_impl(shift_element, shift_interval, shift_parameters);
    if (!try_shift) {
      return tl::unexpected{try_shift.error()};
    }
    shift_element = std::move(try_shift.value());
  }

  // Apply shift.
  std::vector<PointType> shifted_points;
  shifted_points.reserve(shift_element.lon_bases.size());
  for (const auto [base, shift_length] :
       ranges::views::zip(shift_element.lon_bases, shift_element.lat_shifts)) {
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
  return shifted_trajectory;
}

/**
 * @brief Shifts a trajectory based on a single shift interval and parameters.
 * @tparam PointType The type of points in the trajectory.
 * @param reference_trajectory The reference trajectory to be shifted.
 * @param shift_interval The interval for shifting.
 * @param shift_parameters The parameters for the shift.
 * @return The shifted trajectory.
 */
template <typename PointType>
tl::expected<trajectory::Trajectory<PointType>, ShiftError> shift(
  const trajectory::Trajectory<PointType> & reference_trajectory,
  const ShiftInterval & shift_interval, const ShiftParameters & shift_parameters)
{
  return shift(reference_trajectory, std::vector<ShiftInterval>{shift_interval}, shift_parameters);
}

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__SHIFT_HPP_
