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

#include <autoware/pyplot/pyplot.hpp>
#include <range/v3/all.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <iostream>
#include <vector>

geometry_msgs::msg::Point point(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}

int main()
{
  pybind11::scoped_interpreter guard{};
  auto plt = autoware::pyplot::import();
  using ranges::to;
  using ranges::views::transform;

  std::vector<geometry_msgs::msg::Point> points = {
    point(0.49, 0.59), point(0.61, 1.22), point(0.86, 1.93), point(1.20, 2.56), point(1.51, 3.17),
    point(1.85, 3.76), point(2.14, 4.26), point(2.60, 4.56), point(3.07, 4.55), point(3.61, 4.30),
    point(3.95, 4.01), point(4.29, 3.68), point(4.90, 3.25), point(5.54, 3.10), point(6.24, 3.18),
    point(6.88, 3.54), point(7.51, 4.25), point(7.85, 4.93), point(8.03, 5.73), point(8.16, 6.52),
    point(8.31, 7.28), point(8.45, 7.93), point(8.68, 8.45), point(8.96, 8.96), point(9.32, 9.36)};

  auto trajectory =
    autoware::trajectory::Trajectory<geometry_msgs::msg::Point>::Builder{}.build(points);

  if (!trajectory) {
    return 1;
  }

  std::cout << "length: " << trajectory->length() << std::endl;

  {
    {
      const auto s = trajectory->base_arange(0.01);
      const auto c = trajectory->compute(s);
      const auto x = c | transform([](const auto & p) { return p.x; }) | to<std::vector>();
      const auto y = c | transform([](const auto & p) { return p.y; }) | to<std::vector>();
      plt.plot(Args(x, y), Kwargs("label"_a = "original"));
    }
    autoware::trajectory::ShiftInterval shift_interval{
      trajectory->length() / 2.0, trajectory->length() * 8.0 / 10, 0.5};
    autoware::trajectory::ShiftParameters shift_parameter{5.555, 2.0};
    auto shifted_trajectory =
      autoware::trajectory::shift(*trajectory, shift_interval, shift_parameter);
    if (shifted_trajectory) {
      const auto s = shifted_trajectory->base_arange(0.01);
      const auto c = shifted_trajectory->compute(s);
      const auto x = c | transform([](const auto & p) { return p.x; }) | to<std::vector>();
      const auto y = c | transform([](const auto & p) { return p.y; }) | to<std::vector>();

      plt.plot(Args(x, y), Kwargs("label"_a = "shifted"));
      plt.axis(Args("equal"));
      plt.grid();
      plt.legend();
      plt.show();
    } else {
      std::cout << shifted_trajectory.error().what << std::endl;
    }
  }
  return 0;
}
