// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY__UTILS__FRENET_UTILS_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__FRENET_UTILS_HPP_

#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/forward.hpp"

#include <Eigen/Core>

#include <vector>

namespace autoware::experimental::trajectory
{

struct FrenetCoordinate
{
  double longitudinal;
  double lateral;

  FrenetCoordinate(const double longitudinal, const double lateral)
  : longitudinal(longitudinal), lateral(lateral)
  {
  }
};

namespace detail::impl
{

std::vector<FrenetCoordinate> compute_frenet_coordinate_impl(
  const std::function<Eigen::Vector2d(const double & s)> & trajectory_compute,
  const std::vector<double> & bases, const Eigen::Vector2d & point);

}  // namespace detail::impl

template <class PointType, class InputPointType>
[[nodiscard]] std::vector<FrenetCoordinate> compute_frenet_coordinate(
  const Trajectory<PointType> & trajectory,  //
  const InputPointType & point)
{
  return detail::impl::compute_frenet_coordinate_impl(
    [&trajectory](const double & s) {
      auto point = detail::to_point(trajectory.compute(s));
      Eigen::Vector2d p;
      p << point.x, point.y;
      return p;
    },
    trajectory.get_underlying_bases(), {detail::to_point(point).x, detail::to_point(point).y});
}
}  // namespace autoware::experimental::trajectory
#endif  // AUTOWARE__TRAJECTORY__UTILS__FRENET_UTILS_HPP_
