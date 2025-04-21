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
#include "autoware/trajectory/utils/frenet_utils.hpp"

#include <autoware/pyplot/pyplot.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

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

  std::vector<geometry_msgs::msg::Point> points = {
    point(0.42, 0.36), point(1.56, 0.53), point(3.13, 0.80), point(5.15, 1.28), point(6.85, 2.01),
    point(7.85, 2.83), point(8.18, 4.12), point(8.25, 5.28), point(8.10, 6.33), point(7.37, 7.57),
    point(5.98, 8.15), point(4.86, 8.65), point(3.59, 8.83), point(2.05, 9.04), point(0.52, 9.15)};

  auto trajectory =
    autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Point>::Builder{}.build(
      points);

  if (!trajectory) {
    return 1;
  }

  geometry_msgs::msg::Point base;
  base.x = 5.0;
  base.y = 5.0;
  auto frenet_coordinates =
    autoware::experimental::trajectory::compute_frenet_coordinate(*trajectory, base);

  std::cout << "Frenet coordinates: " << std::endl;
  for (const auto & frenet_coordinate : frenet_coordinates) {
    std::cout << "Longitudinal: " << frenet_coordinate.longitudinal
              << ", Lateral: " << frenet_coordinate.lateral << std::endl;
  }

  plt.scatter(
    Args(std::vector<double>{base.x}, std::vector<double>{base.y}), Kwargs("color"_a = "red"));

  std::vector<double> x_points;
  std::vector<double> y_points;

  for (const auto & point : points) {
    x_points.push_back(point.x);
    y_points.push_back(point.y);
  }
  plt.scatter(Args(x_points, y_points), Kwargs("color"_a = "green"));

  std::vector<double> x_interpolated;
  std::vector<double> y_interpolated;

  for (double s = 0; s < trajectory->length(); s += 0.1) {
    auto p = trajectory->compute(s);
    x_interpolated.push_back(p.x);
    y_interpolated.push_back(p.y);
  }
  plt.plot(Args(x_interpolated, y_interpolated), Kwargs("color"_a = "blue"));

  for (const auto & frenet_coordinate : frenet_coordinates) {
    auto p = trajectory->compute(frenet_coordinate.longitudinal);
    plt.scatter(
      Args(std::vector<double>{p.x}, std::vector<double>{p.y}), Kwargs("color"_a = "blue"));
    const double azimuth = trajectory->azimuth(frenet_coordinate.longitudinal);

    plt.quiver(
      Args(p.x, p.y, std::cos(azimuth), std::sin(azimuth)),
      Kwargs("color"_a = "blue", "label"_a = "Frenet direction"));

    plt.plot(
      Args(std::vector<double>{base.x, p.x}, std::vector<double>{base.y, p.y}),
      Kwargs("color"_a = "blue", "label"_a = "Frenet direction"));
  }

  plt.axis(Args("equal"));
  plt.show();

  return 0;
}
