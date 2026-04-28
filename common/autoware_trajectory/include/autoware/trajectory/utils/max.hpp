// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY__UTILS__MAX_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__MAX_HPP_

#include "autoware/trajectory/detail/helpers.hpp"
#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/temporal_trajectory.hpp"

#include <algorithm>
#include <cassert>
#include <iterator>
#include <type_traits>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{

/**
 * @enum MaxSearchMethod
 * @brief Search method used by `max`.
 */
enum class MaxSearchMethod {
  UnderlyingBases,  ///< Evaluate only at the trajectory's underlying bases.
  FixedInterval,    ///< Evaluate at a fixed interval over the trajectory range.
};

/**
 * @struct MaxResult
 * @brief Represents the trajectory point where an evaluated value is maximized.
 * @tparam PointType Type of the point that identifies the maximum position.
 * @tparam ValueType Type returned by the evaluator.
 */
template <class PointType, class ValueType>
struct MaxResult
{
  PointType point;  ///< Arc length `s` for spatial trajectories, or time-distance pair.
  ValueType value;  ///< Maximum evaluated value.
};

namespace detail
{

template <class Evaluator, class ComputePoint>
[[nodiscard]] auto max_at_bases(
  Evaluator & evaluator, const std::vector<double> & bases, ComputePoint && compute_point)
{
  using ComputedPoint = std::invoke_result_t<ComputePoint &, double>;
  using PointType = std::decay_t<ComputedPoint>;
  using EvaluatorResult = point_or_parameter_invoke_result_t<Evaluator, PointType>;

  assert(!bases.empty() && "max: trajectory has no search bases");

  double max_base = bases.front();
  EvaluatorResult max_value = invoke_with_point_or_parameter(
    evaluator, max_base, [&compute_point, &max_base]() { return compute_point(max_base); });

  for (auto it = std::next(bases.begin()); it != bases.end(); ++it) {
    const auto base = *it;
    EvaluatorResult value = invoke_with_point_or_parameter(
      evaluator, base, [&compute_point, &base]() { return compute_point(base); });

    if (max_value < value) {
      max_base = base;
      max_value = std::move(value);
    }
  }

  return MaxResult<double, EvaluatorResult>{max_base, std::move(max_value)};
}

}  // namespace detail

/**
 * @brief Find the underlying trajectory point that maximizes an evaluated value.
 * @details The evaluator is applied at the trajectory's underlying bases.
 * @tparam Method Search method used to sample the trajectory.
 * @tparam TrajectoryPointType The type of points in the trajectory.
 * @tparam Evaluator Callable type that evaluates a trajectory point or arc length `s`.
 * @param trajectory The trajectory to evaluate.
 * @param evaluator The function to apply to each underlying trajectory point or arc length `s`.
 * @return The arc length `s` and maximum value.
 * @pre The trajectory has at least one underlying base.
 */
template <
  MaxSearchMethod Method = MaxSearchMethod::UnderlyingBases, class TrajectoryPointType,
  class Evaluator>
[[nodiscard]] auto max(const Trajectory<TrajectoryPointType> & trajectory, Evaluator && evaluator)
  -> MaxResult<double, detail::point_or_parameter_invoke_result_t<Evaluator, TrajectoryPointType>>
{
  static_assert(Method == MaxSearchMethod::UnderlyingBases, "max: interval is required");
  return detail::max_at_bases(
    evaluator, trajectory.get_underlying_bases(),
    [&trajectory](const double s) { return trajectory.compute(s); });
}

/**
 * @brief Find the trajectory point that maximizes an evaluated value using a fixed arc-length
 * interval.
 * @tparam Method Search method used to sample the trajectory.
 * @tparam TrajectoryPointType The type of points in the trajectory.
 * @tparam Evaluator Callable type that evaluates a trajectory point or arc length `s`.
 * @param trajectory The trajectory to evaluate.
 * @param evaluator The function to apply to each sampled trajectory point or arc length `s`.
 * @param interval Fixed arc-length search interval.
 * @return The arc length `s` and maximum value.
 * @pre `interval` is positive.
 */
template <MaxSearchMethod Method, class TrajectoryPointType, class Evaluator>
[[nodiscard]] auto max(
  const Trajectory<TrajectoryPointType> & trajectory, Evaluator && evaluator, const double interval)
  -> MaxResult<double, detail::point_or_parameter_invoke_result_t<Evaluator, TrajectoryPointType>>
{
  static_assert(Method == MaxSearchMethod::FixedInterval, "max: interval is not supported");
  assert(interval > 0.0 && "max: fixed search interval must be positive");

  std::vector<double> bases;
  const double end = trajectory.length();
  for (double s = 0.0; s < end; s += interval) {
    bases.push_back(s);
  }
  bases.push_back(end);
  return detail::max_at_bases(
    evaluator, bases, [&trajectory](const double s) { return trajectory.compute(s); });
}

/**
 * @brief Find the underlying temporal trajectory point that maximizes an evaluated value.
 * @details The evaluator is applied at the temporal trajectory's underlying time bases.
 * @tparam Method Search method used to sample the temporal trajectory.
 * @tparam Evaluator Callable type that evaluates a temporal trajectory point or time `t`.
 * @param trajectory The temporal trajectory to evaluate.
 * @param evaluator The function to apply to each underlying temporal trajectory point or time `t`.
 * @return The time-distance pair and maximum value.
 * @pre The trajectory has at least one underlying time base.
 */
template <MaxSearchMethod Method = MaxSearchMethod::UnderlyingBases, class Evaluator>
[[nodiscard]] auto max(const TemporalTrajectory & trajectory, Evaluator && evaluator) -> MaxResult<
  TimeDistancePair,
  detail::point_or_parameter_invoke_result_t<Evaluator, TemporalTrajectory::PointType>>
{
  static_assert(Method == MaxSearchMethod::UnderlyingBases, "max: interval is required");
  const auto result = detail::max_at_bases(
    evaluator, trajectory.get_underlying_time_bases(),
    [&trajectory](const double time) { return trajectory.compute_from_time(time); });

  return MaxResult<TimeDistancePair, decltype(result.value)>{
    TimeDistancePair{result.point, trajectory.time_to_distance(result.point)},
    std::move(result.value)};
}

/**
 * @brief Find the temporal trajectory point that maximizes an evaluated value using a fixed time
 * interval.
 * @tparam Method Search method used to sample the temporal trajectory.
 * @tparam Evaluator Callable type that evaluates a temporal trajectory point or time `t`.
 * @param trajectory The temporal trajectory to evaluate.
 * @param evaluator The function to apply to each sampled temporal trajectory point or time `t`.
 * @param interval Fixed time search interval.
 * @return The time-distance pair and maximum value.
 * @pre `interval` is positive.
 */
template <MaxSearchMethod Method, class Evaluator>
[[nodiscard]] auto max(
  const TemporalTrajectory & trajectory, Evaluator && evaluator, const double interval)
  -> MaxResult<
    TimeDistancePair,
    detail::point_or_parameter_invoke_result_t<Evaluator, TemporalTrajectory::PointType>>
{
  static_assert(Method == MaxSearchMethod::FixedInterval, "max: interval is not supported");
  assert(interval > 0.0 && "max: fixed search interval must be positive");

  std::vector<double> bases;
  const double start = trajectory.start_time();
  const double end = trajectory.end_time();
  for (double t = start; t < end; t += interval) {
    bases.push_back(t);
  }
  bases.push_back(end);
  const auto result = detail::max_at_bases(evaluator, bases, [&trajectory](const double time) {
    return trajectory.compute_from_time(time);
  });

  return MaxResult<TimeDistancePair, decltype(result.value)>{
    TimeDistancePair{result.point, trajectory.time_to_distance(result.point)},
    std::move(result.value)};
}

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__MAX_HPP_
