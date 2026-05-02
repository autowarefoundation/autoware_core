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

#ifndef AUTOWARE__TRAJECTORY__UTILS__FIND_INTERVALS_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__FIND_INTERVALS_HPP_

#include "autoware/trajectory/detail/helpers.hpp"
#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/temporal_trajectory.hpp"

#include <functional>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{

/**
 * @struct Interval
 * @brief Represents an interval with a start and end value.
 */
struct Interval
{
  const double start;  ///< Start value of the interval.
  const double end;    ///< End value of the interval.
};

/**
 * @struct TemporalInterval
 * @brief Represents an interval on both distance and time axes.
 */
struct TemporalInterval
{
  TimeDistancePair start;  ///< Start point of the interval on both axes.
  TimeDistancePair end;    ///< End point of the interval on both axes.
};

namespace detail::impl
{

/**
 * @brief Internal implementation to find intervals in a sequence of bases that satisfy a
 * constraint.
 * @param bases A vector of double values representing the sequence of bases.
 * @param constraint A function that evaluates whether a given base satisfies the constraint.
 * @param max_iter Maximum number of iterations for the binary search when finding interval
 * boundaries.
 * @return A vector of Interval objects representing the intervals where the constraint is
 * satisfied.
 */
std::vector<Interval> find_intervals_impl(
  const std::vector<double> & bases, const std::function<bool(const double &)> & constraint,
  int max_iter = 0);

}  // namespace detail::impl

/**
 * @brief Finds intervals in a trajectory where the given constraint is satisfied.
 * @tparam TrajectoryPointType The type of points in the trajectory.
 * @tparam Constraint A callable type that evaluates a constraint on a trajectory point or arc
 * length `s`.
 * @param trajectory The trajectory to evaluate.
 * @param constraint The constraint to apply to each point or arc length `s` in the trajectory.
 * @param max_iter Maximum number of iterations for the binary search when finding interval
 * boundaries.
 * @return A vector of Interval objects representing the intervals where the constraint is
 * satisfied.
 */
template <class TrajectoryPointType, class Constraint>
std::vector<Interval> find_intervals(
  const Trajectory<TrajectoryPointType> & trajectory, Constraint && constraint, int max_iter = 0)
{
  return detail::impl::find_intervals_impl(
    trajectory.get_underlying_bases(),
    [constraint = std::forward<Constraint>(constraint), &trajectory](const double & s) {
      return detail::invoke_with_point_or_parameter(
        constraint, s, [&trajectory, &s]() { return trajectory.compute(s); });
    },
    max_iter);
}

/**
 * @brief Finds intervals in a temporal trajectory where the given constraint is satisfied.
 * @tparam Constraint A callable type that evaluates a constraint on a temporal trajectory point or
 * time `t`.
 * @param trajectory The temporal trajectory to evaluate.
 * @param constraint The constraint to apply to each point or time `t` in the temporal trajectory.
 * @param max_iter Maximum number of iterations for the binary search when finding interval
 * boundaries.
 * @return A vector of TemporalInterval objects representing the distance and time intervals where
 * the constraint is satisfied.
 */
template <class Constraint>
std::vector<TemporalInterval> find_intervals(
  const TemporalTrajectory & trajectory, Constraint && constraint, int max_iter = 0)
{
  const auto time_intervals = detail::impl::find_intervals_impl(
    trajectory.get_underlying_time_bases(),
    [constraint = std::forward<Constraint>(constraint), &trajectory](const double & t) {
      return detail::invoke_with_point_or_parameter(
        constraint, t, [&trajectory, &t]() { return trajectory.compute_from_time(t); });
    },
    max_iter);

  std::vector<TemporalInterval> intervals;
  intervals.reserve(time_intervals.size());
  for (const auto & interval : time_intervals) {
    intervals.emplace_back(
      TemporalInterval{
        TimeDistancePair{interval.start, trajectory.time_to_distance(interval.start)},
        TimeDistancePair{interval.end, trajectory.time_to_distance(interval.end)}});
  }

  return intervals;
}

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__FIND_INTERVALS_HPP_
