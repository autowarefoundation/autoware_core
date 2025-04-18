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

#include "autoware/trajectory/utils/closest.hpp"

#include <algorithm>
#include <limits>
#include <vector>

namespace autoware::experimental::trajectory::detail::impl
{
std::optional<double> closest_with_constraint_impl(
  const std::function<Eigen::Vector3d(const double & s)> & trajectory_compute,
  const std::vector<double> & bases, const Eigen::Vector3d & point,
  const std::function<bool(const double &)> & constraint)
{
  using trajectory::detail::to_point;
  std::vector<double> distances_from_segments;
  std::vector<double> lengths_from_start_points;

  for (size_t i = 1; i < bases.size(); ++i) {
    const Eigen::Vector3d p0 = trajectory_compute(bases.at(i - 1));
    const Eigen::Vector3d p1 = trajectory_compute(bases.at(i));

    const Eigen::Vector3d v = p1 - p0;
    const Eigen::Vector3d w = point - p0;
    const double c1 = w.dot(v);
    const double c2 = v.dot(v);
    const auto length_from_start_point = [&]() {
      if (c1 <= 0) {
        return bases.at(i - 1);
      }
      if (c2 <= c1) {
        return bases.at(i);
      }
      return bases.at(i - 1) + c1 / c2 * (p1 - p0).norm();
    }();
    const auto distance_from_segment = [&]() {
      if (c1 <= 0) {
        return (point - p0).norm();
      }
      if (c2 <= c1) {
        return (point - p1).norm();
      }
      return (point - (p0 + (c1 / c2) * v)).norm();
    }();
    if (constraint(length_from_start_point)) {
      distances_from_segments.push_back(distance_from_segment);
      lengths_from_start_points.push_back(length_from_start_point);
    }
    if (constraint(length_from_start_point)) {
      distances_from_segments.push_back(distance_from_segment);
      lengths_from_start_points.push_back(length_from_start_point);
    }
  }
  if (distances_from_segments.empty()) {
    return std::nullopt;
  }

  auto min_it = std::min_element(distances_from_segments.begin(), distances_from_segments.end());

  return lengths_from_start_points[std::distance(distances_from_segments.begin(), min_it)];
}
}  // namespace autoware::experimental::trajectory::detail::impl
