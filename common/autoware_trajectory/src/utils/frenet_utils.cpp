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

#include "autoware/trajectory/utils/frenet_utils.hpp"

#include <optional>
#include <vector>

namespace autoware::experimental::trajectory::detail::impl
{

std::vector<FrenetCoordinate> compute_frenet_coordinate_impl(
  const std::function<Eigen::Vector2d(const double & s)> & trajectory_compute,
  const std::vector<double> & bases, const Eigen::Vector2d & point)
{
  std::vector<FrenetCoordinate> frenet_coordinates;

  for (size_t i = 1; i < bases.size(); ++i) {
    const Eigen::Vector2d p0 = trajectory_compute(bases.at(i - 1));
    const Eigen::Vector2d p1 = trajectory_compute(bases.at(i));

    const Eigen::Vector2d v = p1 - p0;
    const Eigen::Vector2d w = point - p0;
    const double c1 = w.dot(v);
    const double c2 = v.dot(v);
    const std::optional<FrenetCoordinate> frenet_coordinate =
      [&]() -> std::optional<FrenetCoordinate> {
      if (c1 <= 0 || c2 <= c1) {
        return std::nullopt;
      }
      return FrenetCoordinate(
        bases.at(i - 1) + c1 / c2 * (p1 - p0).norm(), (point - (p0 + (c1 / c2) * v)).norm());
    }();

    if (frenet_coordinate) {
      frenet_coordinates.emplace_back(frenet_coordinate.value());
    }
  }

  return frenet_coordinates;
}

}  // namespace autoware::experimental::trajectory::detail::impl
