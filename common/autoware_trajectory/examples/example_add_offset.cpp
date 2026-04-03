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

#include "autoware/trajectory/trajectory_point.hpp"
#include "autoware/trajectory/utils/add_offset.hpp"

#include <autoware/pyplot/pyplot.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/all.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <string>
#include <vector>

using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::build;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

using ranges::to;
using ranges::views::transform;

namespace
{

void plot_trajectory_with_underlying(
  const autoware::experimental::trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const std::string & color, const std::string & label, autoware::pyplot::PyPlot & plt)
{
  const auto s = trajectory.base_arange(0.05);
  const auto c = trajectory.compute(s);
  const auto x =
    c | transform([](const auto & p) { return p.pose.position.x; }) | to<std::vector>();
  const auto y =
    c | transform([](const auto & p) { return p.pose.position.y; }) | to<std::vector>();
  plt.plot(Args(x, y), Kwargs("label"_a = label, "color"_a = color));

  const auto base = trajectory.get_underlying_bases();
  const auto th =
    base | transform([&](const auto & s) { return trajectory.azimuth(s); }) | to<std::vector>();
  const auto th_cos = th | transform([](const auto s) { return std::cos(s); }) | to<std::vector>();
  const auto th_sin = th | transform([](const auto s) { return std::sin(s); }) | to<std::vector>();
  const auto base_x =
    base | transform([&](const auto & s) { return trajectory.compute(s).pose.position.x; }) |
    to<std::vector>();
  const auto base_y =
    base | transform([&](const auto & s) { return trajectory.compute(s).pose.position.y; }) |
    to<std::vector>();
  plt.quiver(
    Args(base_x, base_y, th_cos, th_sin),
    Kwargs("color"_a = color, "scale"_a = 2, "scale_units"_a = "xy", "angles"_a = "xy"));
}

}  // namespace

int main()
{
  pybind11::scoped_interpreter guard{};
  auto plt = autoware::pyplot::import();

  // Offset parameters
  constexpr double front_offset = 5.0;   // forward offset from base_link [m]
  constexpr double rear_offset = -5.0;   // rear offset from base_link [m]
  constexpr double left_offset = 1.0;    // left offset from base_link [m]
  constexpr double right_offset = -1.0;  // right offset from base_link [m]

  // Create a circular arc trajectory
  std::vector<TrajectoryPoint> points;
  const double radius = 20.0;       // radius of the circle [m]
  const double start_angle = 0.0;   // start angle [rad]
  const double end_angle = M_PI_2;  // 90 degree arc
  const double step_angle = 0.1;    // angle step [rad]

  for (double angle = start_angle; angle <= end_angle; angle += step_angle) {
    // Circle centered at (0, radius) so it starts at origin going in +x direction
    const double x = radius * std::sin(angle);
    const double y = radius * (1.0 - std::cos(angle));

    TrajectoryPoint point;
    point.pose.position = build<Point>().x(x).y(y).z(0.0);
    points.push_back(point);
  }

  auto trajectory = autoware::experimental::trajectory::Trajectory<TrajectoryPoint>::Builder{}
                      .build(points)
                      .value();

  trajectory.align_orientation_with_trajectory_direction();

  // Get offset trajectories
  auto front_trajectory =
    autoware::experimental::trajectory::add_offset(trajectory, front_offset, 0.0);
  auto rear_trajectory =
    autoware::experimental::trajectory::add_offset(trajectory, rear_offset, 0.0);
  auto left_trajectory =
    autoware::experimental::trajectory::add_offset(trajectory, 0.0, left_offset);
  auto right_trajectory =
    autoware::experimental::trajectory::add_offset(trajectory, 0.0, right_offset);

  // Plot 1: Forward and rear offsets
  plot_trajectory_with_underlying(trajectory, "black", "base_link", plt);
  plot_trajectory_with_underlying(
    front_trajectory, "red", "front (+" + std::to_string(front_offset) + "m)", plt);
  plot_trajectory_with_underlying(
    rear_trajectory, "blue", "rear (" + std::to_string(rear_offset) + "m)", plt);

  plt.axis(Args("equal"));
  plt.grid();
  plt.legend();
  plt.title(Args("Forward/Rear Offset Trajectories"));
  plt.show();

  plt.clf();

  // Plot 2: Left and right offsets
  plot_trajectory_with_underlying(trajectory, "black", "base_link", plt);
  plot_trajectory_with_underlying(
    left_trajectory, "red", "left (+" + std::to_string(left_offset) + "m)", plt);
  plot_trajectory_with_underlying(
    right_trajectory, "blue", "right (" + std::to_string(right_offset) + "m)", plt);

  plt.axis(Args("equal"));
  plt.grid();
  plt.legend();
  plt.title(Args("Lateral Offset Trajectories"));
  plt.show();

  return 0;
}
