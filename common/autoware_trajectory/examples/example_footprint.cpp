// Copyright 2025 Tier IV, Inc.
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

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/pose.hpp"
#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/utils/find_nearest.hpp"
#include "autoware/trajectory/utils/footprint.hpp"

#include <autoware/pyplot/pyplot.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/all.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

using autoware::experimental::trajectory::Trajectory;
using autoware_utils_geometry::create_point;
using autoware_utils_geometry::create_quaternion_from_rpy;
using ranges::to;
using ranges::views::transform;

static Trajectory<geometry_msgs::msg::Pose> build_parabolic_trajectory(
  const size_t num_points, const double interval, const bool reverse = false)
{
  std::vector<geometry_msgs::msg::Pose> raw_poses;
  raw_poses.reserve(num_points);

  double half = static_cast<double>(num_points - 1) / 2.0;
  for (size_t i = 0; i < num_points; ++i) {
    double x;
    double y;
    double yaw;
    if (!reverse) {
      x = (static_cast<double>(i) - half) * interval;
      y = x * x;                       // Parabola: y = x^2
      yaw = std::atan2(2.0 * x, 1.0);  // Tangent direction
    } else {
      x = (-1 * static_cast<double>(i) + half) * interval;
      y = x * x;                              // Parabola: y = x^2
      yaw = std::atan2(2.0 * x, 1.0) - M_PI;  // Tangent direction
    }
    geometry_msgs::msg::Pose p;
    p.position = create_point(x, y, 0.0);
    p.orientation = create_quaternion_from_rpy(0.0, 0.0, yaw);
    raw_poses.push_back(p);
  }

  auto traj = Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(raw_poses);
  return traj.value();
}

template <class TrajectoryPointType>
static void plot_trajectory(
  autoware::pyplot::Axes & ax, const Trajectory<TrajectoryPointType> & traj,
  const std::string & traj_name)
{
  std::vector<double> x_all;
  std::vector<double> y_all;

  for (double s = 0.0; s < traj.length(); s += 0.01) {
    auto p = traj.compute(s);
    x_all.push_back(p.position.x);
    y_all.push_back(p.position.y);
  }

  std::string label = traj_name + " Trajectory";

  ax.plot(Args(x_all, y_all), Kwargs("color"_a = "black", "label"_a = label));
}

template <class TrajectoryPointType>
static void plot_traj_with_orientation(
  autoware::pyplot::Axes & ax, const Trajectory<TrajectoryPointType> & traj)
{
  const auto s = traj.get_underlying_bases();
  const auto C = s | transform([&](const double s) { return traj.compute(s); }) | to<std::vector>();
  const auto Cx = C | transform([&](const auto & p) { return p.position.x; }) | to<std::vector>();
  const auto Cy = C | transform([&](const auto & p) { return p.position.y; }) | to<std::vector>();
  const auto yaw = s | transform([&](const double s) {
                     return autoware_utils_geometry::get_rpy(traj.compute(s).orientation).z;
                   }) |
                   to<std::vector>();
  const auto cos_yaw =
    yaw | transform([&](const double s) { return std::cos(s); }) | to<std::vector>();
  const auto sin_yaw =
    yaw | transform([&](const double s) { return std::sin(s); }) | to<std::vector>();
  ax.quiver(
    Args(Cx, Cy, cos_yaw, sin_yaw),
    Kwargs(
      "color"_a = "green", "scale"_a = 1.5, "width"_a = 0.01, "angles"_a = "xy",
      "scale_units"_a = "xy", "label"_a = "orientation", "alpha"_a = 1));
}

static void draw_polygon(
  autoware::pyplot::Axes & ax, const autoware_utils_geometry::Polygon2d & polygon)
{
  std::vector<double> x_list;
  std::vector<double> y_list;
  for (auto point : polygon.outer()) {
    x_list.push_back(point.x());
    y_list.push_back(point.y());
  }
  ax.plot(Args(x_list, y_list), Kwargs("color"_a = "grey", "label"_a = "Path Polygon"));
  ax.fill(
    Args(x_list, y_list),
    Kwargs("color"_a = "skyblue", "alpha"_a = 0.4, "edgecolor"_a = "blue", "linewidth"_a = 2));
}

static void draw_footprints(
  autoware::pyplot::Axes & ax, const std::vector<autoware_utils_geometry::Polygon2d> & footprints)
{
  for (auto fp : footprints) {
    std::vector<double> x_list;
    std::vector<double> y_list;
    for (auto point : fp.outer()) {
      x_list.push_back(point.x());
      y_list.push_back(point.y());
    }

    ax.fill(
      Args(x_list, y_list),
      Kwargs("color"_a = "grey", "alpha"_a = 0.4, "edgecolor"_a = "grey", "linewidth"_a = 2));
    ax.scatter(Args(x_list, y_list), Kwargs("color"_a = "grey"));
  }
}

int main1()
{
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 1);
  auto ax = axes[0];

  auto traj = build_parabolic_trajectory(11, 0.5);
  plot_trajectory(ax, traj, "Parabolic to the right");
  plot_traj_with_orientation(ax, traj);

  const auto polygon = autoware::experimental::trajectory::build_path_polygon(
    traj, 0, traj.get_underlying_bases()[traj.get_underlying_bases().size() - 1], 0.5, 0.5);

  draw_polygon(ax, polygon);
  ax.set_title(Args("build_path_polygon"), Kwargs("fontsize"_a = 16));

  ax.legend();
  ax.grid();
  ax.set_aspect(Args("equal"));
  fig.tight_layout();
  plt.show();
  return 0;
}

int main2()
{
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 1);
  auto ax = axes[0];

  auto traj = build_parabolic_trajectory(11, 0.5);
  plot_trajectory(ax, traj, "Parabolic to the right");
  plot_traj_with_orientation(ax, traj);

  autoware_utils_geometry::Point2d left_front{-0.5, 0.25};
  autoware_utils_geometry::Point2d right_front{0.5, 0.25};
  autoware_utils_geometry::Point2d right_rear{0.5, -0.25};
  autoware_utils_geometry::Point2d left_rear{-0.5, -0.25};
  autoware_utils_geometry::LinearRing2d base_ring{left_front, right_front, right_rear, left_rear};

  const auto footprints = autoware::experimental::trajectory::build_path_footprints(
    traj, 0, traj.get_underlying_bases()[traj.get_underlying_bases().size() - 1], 1.0, base_ring);

  draw_footprints(ax, footprints);
  ax.set_title(Args("build_path_footprints for LinearRing2d"), Kwargs("fontsize"_a = 16));

  ax.legend();
  ax.grid();
  ax.set_aspect(Args("equal"));
  fig.tight_layout();
  plt.show();
  return 0;
}

int main3()
{
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 1);
  auto ax = axes[0];

  auto traj = build_parabolic_trajectory(11, 0.5);
  plot_trajectory(ax, traj, "Parabolic to the right");
  plot_traj_with_orientation(ax, traj);

  autoware_utils_geometry::Point2d left_front{-0.5, 0.25};
  autoware_utils_geometry::Point2d right_front{0.5, 0.25};
  autoware_utils_geometry::Point2d right_rear{0.5, -0.25};
  autoware_utils_geometry::Point2d left_rear{-0.5, -0.25};
  autoware_utils_geometry::Polygon2d base_polygon;
  base_polygon.outer().push_back(left_front);
  base_polygon.outer().push_back(right_front);
  base_polygon.outer().push_back(right_rear);
  base_polygon.outer().push_back(left_rear);

  const auto footprints = autoware::experimental::trajectory::build_path_footprints(
    traj, 0, traj.get_underlying_bases()[traj.get_underlying_bases().size() - 1], 1.0,
    base_polygon);

  draw_footprints(ax, footprints);
  ax.set_title(Args("build_path_footprints for Polygon2d"), Kwargs("fontsize"_a = 16));

  ax.legend();
  ax.grid();
  ax.set_aspect(Args("equal"));
  fig.tight_layout();
  plt.show();
  return 0;
}

int main()
{
  pybind11::scoped_interpreter guard{};
  main1();
  main2();
  main3();
  return 0;
}
