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

#include "autoware/trajectory/pose.hpp"
#include "autoware_utils_geometry/geometry.hpp"

#include <autoware/pyplot/pyplot.hpp>
#include <range/v3/all.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <string>
#include <vector>

geometry_msgs::msg::Pose pose(double x, double y)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = 0.0;
  return p;
}

using autoware::experimental::trajectory::Trajectory;
using ranges::to;
using ranges::views::transform;

void plot_trajectory_base_with_orientation(
  const Trajectory<geometry_msgs::msg::Pose> & trajectory, const std::string & label,
  autoware::pyplot::Axes & ax)
{
  const auto s = trajectory.get_underlying_bases();
  const auto c = trajectory.compute(s);
  const auto x =
    c | transform([](const auto & point) { return point.position.x; }) | to<std::vector>();
  const auto y =
    c | transform([](const auto & point) { return point.position.y; }) | to<std::vector>();
  const auto th =
    c | transform([](const auto & quat) { return autoware_utils_geometry::get_rpy(quat).z; }) |
    to<std::vector>();
  const auto cos_th = th | transform([](const auto v) { return std::cos(v); }) | to<std::vector>();
  const auto sin_th = th | transform([](const auto v) { return std::sin(v); }) | to<std::vector>();
  ax.scatter(Args(x, y), Kwargs("color"_a = "red", "marker"_a = "o", "label"_a = "underlying"));
  ax.quiver(Args(x, y, cos_th, sin_th), Kwargs("color"_a = "green", "label"_a = label));
}

int main()
{
  pybind11::scoped_interpreter guard{};

  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 2);
  auto & ax1 = axes[0];
  auto & ax2 = axes[1];

  std::vector<geometry_msgs::msg::Pose> poses = {
    pose(0.49, 0.59), pose(0.61, 1.22), pose(0.86, 1.93), pose(1.20, 2.56), pose(1.51, 3.17),
    pose(1.85, 3.76), pose(2.14, 4.26), pose(2.60, 4.56), pose(3.07, 4.55), pose(3.61, 4.30),
    pose(3.95, 4.01), pose(4.29, 3.68), pose(4.90, 3.25), pose(5.54, 3.10), pose(6.24, 3.18),
    pose(6.88, 3.54), pose(7.51, 4.25), pose(7.85, 4.93), pose(8.03, 5.73), pose(8.16, 6.52),
    pose(8.31, 7.28), pose(8.45, 7.93), pose(8.68, 8.45), pose(8.96, 8.96), pose(9.32, 9.36)};

  using autoware::experimental::trajectory::Trajectory;

  auto trajectory = Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(poses);

  plot_trajectory_base_with_orientation(*trajectory, "before", ax1);
  trajectory->align_orientation_with_trajectory_direction();
  plot_trajectory_base_with_orientation(
    *trajectory, "after align_orientation_with_trajectory_direction()", ax2);

  {
    std::vector<double> x;
    std::vector<double> y;

    for (double i = 0.0; i <= trajectory->length(); i += 0.01) {
      auto p = trajectory->compute(i);
      x.push_back(p.position.x);
      y.push_back(p.position.y);
    }

    ax1.plot(Args(x, y), Kwargs("label"_a = "Trajectory", "color"_a = "blue"));
    ax2.plot(Args(x, y), Kwargs("label"_a = "Trajectory", "color"_a = "blue"));
  }

  fig.tight_layout();
  for (auto & ax : axes) {
    ax.set_aspect(Args("equal"));
    ax.grid();
    ax.legend();
  }
  plt.show();
}
