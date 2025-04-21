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
#include "autoware/trajectory/utils/closest.hpp"

#include <tl_expected/expected.hpp>

#include <fmt/format.h>

#include <string>
#include <utility>

namespace autoware::trajectory
{

struct FrenetUtilsUnexpected
{
  const std::string what;
};

template <class PointType, class InputPointType>
[[nodiscard]] tl::expected<std::pair<double, double>, FrenetUtilsUnexpected>
compute_frenet_coordinate(
  const Trajectory<PointType> & trajectory,  //
  const InputPointType & point)
{
  double closest_s = closest(trajectory, point);

  if (closest_s < 0.0) {
    return tl::unexpected<FrenetUtilsUnexpected>{{fmt::format(
      "point (x = {}, y = {}) is before the start of the trajectory", detail::to_point(point).x,
      detail::to_point(point).y)}};
  }

  if (closest_s > trajectory.length()) {
    return tl::unexpected<FrenetUtilsUnexpected>{{fmt::format(
      "point (x = {}, y = {}) is after the end of the trajectory", detail::to_point(point).x,
      detail::to_point(point).y)}};
  }

  const auto point_xy = detail::to_point(point);
  const auto closest_point_xy = detail::to_point(trajectory.compute(closest_s));

  const double azimuth = trajectory.azimuth(closest_s);

  const double w_x = point_xy.x - closest_point_xy.x;
  const double w_y = point_xy.y - closest_point_xy.y;

  // Normal vector perpendicular to azimuth direction
  const double n_x = -std::sin(azimuth);
  const double n_y = std::cos(azimuth);

  return std::make_pair(closest_s, w_x * n_x + w_y * n_y);
}

template <class PointType>
[[nodiscard]] tl::expected<PointType, FrenetUtilsUnexpected> move_point_along_frenet_coordinate(
  const Trajectory<PointType> & trajectory,  //
  const PointType & point,                   //
  const double & longitude,                  //
  const double & lateral)
{
  auto frenet_coordinate = compute_frenet_coordinate(trajectory, point);
  if (!frenet_coordinate) {
    return tl::unexpected<FrenetUtilsUnexpected>{
      {"Failed to compute_frenet_coordinate. " + frenet_coordinate.error().what}};
  }

  double longitudinal_moved_s = frenet_coordinate->first + longitude;
  if (longitudinal_moved_s < 0.0) {
    return tl::unexpected<FrenetUtilsUnexpected>{{fmt::format(
      "Moved point (x = {}, y = {}) is before the start of the trajectory",
      detail::to_point(point).x, detail::to_point(point).y)}};
  }

  if (longitudinal_moved_s > trajectory.length()) {
    return tl::unexpected<FrenetUtilsUnexpected>{{fmt::format(
      "Moved point (x = {}, y = {}) is after the end of the trajectory", detail::to_point(point).x,
      detail::to_point(point).y)}};
  }

  auto moved_point = trajectory.compute(longitudinal_moved_s);
  const double azimuth = trajectory.azimuth(longitudinal_moved_s);
  detail::to_point(moved_point).x -= (frenet_coordinate->second + lateral) * std::sin(azimuth);
  detail::to_point(moved_point).y += (frenet_coordinate->second + lateral) * std::cos(azimuth);
  return moved_point;
}

}  // namespace autoware::trajectory
#endif  // AUTOWARE__TRAJECTORY__UTILS__FRENET_UTILS_HPP_
