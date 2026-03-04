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

#ifndef AUTOWARE__TRAJECTORY__UTILS__FIND_IF_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__FIND_IF_HPP_

#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/forward.hpp"

#include <range/v3/all.hpp>

#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{

namespace detail::impl
{

/**
 * @brief Internal implementation to find intervals in a sequence of bases that satisfy a
 * constraint.
 * @param bases A vector of double values representing the sequence of bases.
 * @param constraint A function that evaluates whether a given base satisfies the constraint.
 * @return A vector of Interval objects representing the intervals where the constraint is
 * satisfied.
 */
std::optional<double> find_first_index_if_impl(
  const std::vector<double> & bases, const std::function<bool(const double &)> & constraint,
  const size_t max_iter = 0);

}  // namespace detail::impl

/**
 * @brief Finds the first index in a trajectory where the given constraint is satisfied.
 * @tparam TrajectoryPointType The type of points in the trajectory.
 * @tparam Constraint A callable type that evaluates a constraint on a trajectory point.
 * @param trajectory The trajectory to evaluate.
 * @param constraint The constraint to apply to each point in the trajectory.
 * @param max_iter The maximum number of iterations for the binary search.
 * @return The first index where the constraint is satisfied (std::nullopt if not found).
 */
template <class TrajectoryPointType, class Constraint>
std::optional<double> find_first_index_if(
  const Trajectory<TrajectoryPointType> & trajectory, Constraint && constraint,
  const size_t max_iter = 0)
{
  using autoware::experimental::trajectory::detail::to_point;

  return detail::impl::find_first_index_if_impl(
    trajectory.get_underlying_bases(),
    [constraint = std::forward<Constraint>(constraint), &trajectory](const double & s) {
      return constraint(trajectory.compute(s));
    },
    max_iter);
}

/**
 * @brief Finds the last index in a trajectory where the given constraint is satisfied.
 * @tparam TrajectoryPointType The type of points in the trajectory.
 * @tparam Constraint A callable type that evaluates a constraint on a trajectory point.
 * @param trajectory The trajectory to evaluate.
 * @param constraint The constraint to apply to each point in the trajectory.
 * @param max_iter The maximum number of iterations for the binary search.
 * @return The last index where the constraint is satisfied (std::nullopt if not found).
 */
template <class TrajectoryPointType, class Constraint>
std::optional<double> find_last_index_if(
  const Trajectory<TrajectoryPointType> & trajectory, Constraint && constraint,
  const size_t max_iter = 0)
{
  using autoware::experimental::trajectory::detail::to_point;

  const auto & bases = trajectory.get_underlying_bases();
  return detail::impl::find_first_index_if_impl(
    bases | ranges::views::reverse | ranges::to<std::vector>(),
    [constraint = std::forward<Constraint>(constraint), &trajectory](const double & s) {
      return constraint(trajectory.compute(s));
    },
    max_iter);
}

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__FIND_IF_HPP_
