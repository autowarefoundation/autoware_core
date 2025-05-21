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

#include "autoware/trajectory/point.hpp"
#include "autoware/trajectory/utils/shift.hpp"

#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware/pyplot/pyplot.hpp>
#include <range/v3/all.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

geometry_msgs::msg::Point point(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}

using ranges::to;
using ranges::views::transform;

void plot_trajectory_with_underlying(
  const autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Point> & trajectory,
  const std::string & color, const std::string & label, autoware::pyplot::PyPlot & plt)
{
  const auto s = trajectory.base_arange(0.05);
  const auto c = trajectory.compute(s);
  const auto x = c | transform([](const auto & p) { return p.x; }) | to<std::vector>();
  const auto y = c | transform([](const auto & p) { return p.y; }) | to<std::vector>();
  plt.plot(Args(x, y), Kwargs("label"_a = label, "color"_a = color));

  const auto base = trajectory.get_underlying_bases();
  const auto th =
    base | transform([&](const auto & s) { return trajectory.azimuth(s); }) | to<std::vector>();
  const auto th_cos = th | transform([](const auto s) { return std::cos(s); }) | to<std::vector>();
  const auto th_sin = th | transform([](const auto s) { return std::sin(s); }) | to<std::vector>();
  const auto base_x =
    base | transform([&](const auto s) { return trajectory.compute(s).x; }) | to<std::vector>();
  const auto base_y =
    base | transform([&](const auto s) { return trajectory.compute(s).y; }) | to<std::vector>();
  plt.quiver(
    Args(base_x, base_y, th_cos, th_sin),
    Kwargs("color"_a = color, "scale"_a = 2, "scale_units"_a = "xy", "angles"_a = "xy"));
}

int main()
{
  using autoware::experimental::trajectory::ShiftInterval;
  using autoware::experimental::trajectory::ShiftParameters;

  pybind11::scoped_interpreter guard{};
  auto plt = autoware::pyplot::import();

  std::vector<geometry_msgs::msg::Point> points = {point(0.0, 0.0),  point(1.0, 0.0),
                                                   point(6.0, 0.0),  point(9.0, 0.0),
                                                   point(12.0, 0.0), point(18.0, 0.0)};

  auto trajectory =
    autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Point>::Builder{}.build(
      points);

  if (!trajectory) {
    return 1;
  }

  /*
    four points
    const double longitudinal_velocity = 2.77;
    const double lateral_jerk = 1.0;
    const double lateral_shift = 2.5;
    const double lateral_acc_limit = 5.0;

    const double longitudinal_velocity = 2.77;
    const double lateral_jerk = 1.5;
    const double lateral_shift = 5.0;
    const double lateral_acc_limit = 1.5;
   */
  // six points
  const double longitudinal_velocity = 2.77;
  const double lateral_jerk = 1.0;
  const double lateral_shift = 2.5;
  const double lateral_acc_limit = 5.0;
  const double longitudinal_dist = autoware::motion_utils::calc_longitudinal_dist_from_jerk(
    lateral_shift, lateral_jerk, longitudinal_velocity);

  const auto start_s = 3.0;
  const ShiftInterval shift_interval{start_s, start_s + longitudinal_dist, lateral_shift};
  const ShiftParameters shift_parameter{
    longitudinal_velocity,
    lateral_acc_limit,
  };

  auto shifted_trajectory_info =
    autoware::experimental::trajectory::shift(*trajectory, shift_interval, shift_parameter);
  if (!shifted_trajectory_info) {
    std::cout << shifted_trajectory_info.error().what << std::endl;
    return 1;
  }
  const auto & shifted_trajectory = shifted_trajectory_info.value();

  plot_trajectory_with_underlying(*trajectory, "blue", "original", plt);
  plot_trajectory_with_underlying(shifted_trajectory, "red", "shifted", plt);

  plt.axis(Args("equal"));
  plt.grid();
  plt.legend();
  plt.show();

  plt.clf();

  const ShiftInterval shift_left_interval{start_s, start_s + longitudinal_dist, -lateral_shift};
  auto shifted_trajectory_left_info =
    autoware::experimental::trajectory::shift(*trajectory, shift_left_interval, shift_parameter);
  if (!shifted_trajectory_info) {
    std::cout << shifted_trajectory_info.error().what << std::endl;
    return 1;
  }
  const auto & shifted_trajectory_left = shifted_trajectory_left_info.value();

  plot_trajectory_with_underlying(*trajectory, "black", "original", plt);
  plot_trajectory_with_underlying(shifted_trajectory, "red", "lateral_offset = +2.5", plt);
  plot_trajectory_with_underlying(shifted_trajectory_left, "blue", "lateral_offset = -2.5", plt);
  plt.axis(Args("equal"));
  plt.grid();
  plt.legend();
  plt.show();

  return 0;
}
