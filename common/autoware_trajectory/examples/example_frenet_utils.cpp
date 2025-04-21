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
    std::vector<double> x;
    std::vector<double> y;
    for (double s = 0.0; s < trajectory->length(); s += 0.01) {
      auto p = trajectory->compute(s);
      x.push_back(p.x);
      y.push_back(p.y);
    }
    plt.plot(Args(x, y), Kwargs("label"_a = "original"));

    geometry_msgs::msg::Point p1;
    p1.x = 7.2;
    p1.y = 6.2;

    geometry_msgs::msg::Point p2;
    p2.x = 8.9;
    p2.y = 5.9;

    auto frenet_coordinate1 = autoware::trajectory::compute_frenet_coordinate(*trajectory, p1);
    auto frenet_coordinate2 = autoware::trajectory::compute_frenet_coordinate(*trajectory, p2);

    if (!frenet_coordinate1 || !frenet_coordinate2) {
      return 1;
    }

    std::cout << "frenet coordinate1: " << frenet_coordinate1->first << ", "
              << frenet_coordinate1->second << std::endl;

    std::cout << "frenet coordinate2: " << frenet_coordinate2->first << ", "
              << frenet_coordinate2->second << std::endl;

    plt.scatter(Args(p1.x, p1.y), Kwargs("label"_a = "point1"));
    plt.scatter(Args(p2.x, p2.y), Kwargs("label"_a = "point2"));

    auto moved_point1 =
      autoware::trajectory::move_point_along_frenet_coordinate(*trajectory, p1, 1.0, 0.0);
    auto moved_point2 =
      autoware::trajectory::move_point_along_frenet_coordinate(*trajectory, p2, 1.0, 0.0);

    if (!moved_point1 || !moved_point2) {
      return 1;
    }

    plt.scatter(Args(moved_point1->x, moved_point1->y), Kwargs("label"_a = "moved_point1"));
    plt.scatter(Args(moved_point2->x, moved_point2->y), Kwargs("label"_a = "moved_point2"));

    plt.axis(Args("equal"));
    plt.legend();
    plt.show();
  }

  return 0;
}
