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

#include "autoware/trajectory/utils/crossed.hpp"
#include "autoware/trajectory/utils/pretty_build.hpp"

#include <autoware/pyplot/pyplot.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/all.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <string>
#include <vector>

using autoware::experimental::trajectory::Trajectory;
using ranges::to;
using ranges::views::transform;

using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_utils_geometry::create_quaternion_from_yaw;
using geometry_msgs::build;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Quaternion;

using autoware_utils_geometry::LineString2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

// For plotting
template <class TrajectoryPointType>
static void plot_trajectory(
  autoware::pyplot::Axes & ax, const Trajectory<TrajectoryPointType> & traj,
  const std::string & traj_name)
{
  std::vector<double> x_all;
  std::vector<double> y_all;

  for (double s = 0.0; s < traj.length(); s += 0.01) {
    auto p = traj.compute(s);
    x_all.push_back(autoware_utils_geometry::get_pose(p).position.x);
    y_all.push_back(autoware_utils_geometry::get_pose(p).position.y);
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
  const auto Cx =
    C | transform([&](const auto & p) { return autoware_utils_geometry::get_pose(p).position.x; }) |
    to<std::vector>();
  const auto Cy =
    C | transform([&](const auto & p) { return autoware_utils_geometry::get_pose(p).position.y; }) |
    to<std::vector>();
  const auto yaw = s | transform([&](const double s) {
                     return autoware_utils_geometry::get_rpy(
                              autoware_utils_geometry::get_pose(traj.compute(s)).orientation)
                       .z;
                   }) |
                   to<std::vector>();
  const auto cos_yaw =
    yaw | transform([&](const double s) { return std::cos(s); }) | to<std::vector>();
  const auto sin_yaw =
    yaw | transform([&](const double s) { return std::sin(s); }) | to<std::vector>();
  ax.quiver(
    Args(Cx, Cy, cos_yaw, sin_yaw),
    Kwargs(
      "color"_a = "orange", "scale"_a = 1.5, "width"_a = 0.01, "angles"_a = "xy",
      "scale_units"_a = "xy", "label"_a = "orientation", "alpha"_a = 1));
}

static void plot_linestring(
  autoware::pyplot::Axes & ax, const LineString2d & line, const std::string & line_color,
  const std::string & line_name)
{
  std::vector<double> x_all;
  std::vector<double> y_all;

  for (auto point : line) {
    x_all.push_back(point.x());
    y_all.push_back(point.y());
  }
  ax.plot(Args(x_all, y_all), Kwargs("color"_a = line_color, "label"_a = line_name));
}

template <class TrajectoryPointType>
static void plot_intersection(
  autoware::pyplot::Axes & ax, const Trajectory<TrajectoryPointType> & traj,
  const std::vector<double> & crossed_line, const std::string & point_color,
  const std::string & point_name)
{
  std::vector<double> x_all;
  std::vector<double> y_all;

  for (auto s : crossed_line) {
    auto point = autoware_utils_geometry::get_pose(traj.compute(s)).position;
    x_all.push_back(point.x);
    y_all.push_back(point.y);
  }
  ax.scatter(Args(x_all, y_all), Kwargs("color"_a = point_color, "label"_a = point_name));
}

static void plot_polygon(autoware::pyplot::Axes & ax, const std::vector<Point2d> & polygon)
{
  std::vector<double> x_all;
  std::vector<double> y_all;
  for (auto point : polygon) {
    x_all.push_back(point.x());
    y_all.push_back(point.y());
  }
  ax.plot(Args(x_all, y_all), Kwargs("color"_a = "grey", "label"_a = "Given Polygon"));
  ax.fill(
    Args(x_all, y_all),
    Kwargs("color"_a = "skyblue", "alpha"_a = 0.4, "edgecolor"_a = "blue", "linewidth"_a = 2));
}

static Trajectory<PathPointWithLaneId> build_trajectory()
{
  std::vector<PathPointWithLaneId> points;
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(0.0).y(0.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(0.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(4.0).y(4.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(M_PI / 2.0));
    point.point.longitudinal_velocity_mps = 20.0;
    point.point.lateral_velocity_mps = 1.0;
    point.point.heading_rate_rps = 1.0;
    point.lane_ids = std::vector<std::int64_t>{2};
    points.push_back(point);
  }
  const auto points4_result = autoware::experimental::trajectory::detail::populate4(points);
  const auto & points4 = points4_result.value();

  const auto trajectory_opt = autoware::experimental::trajectory::pretty_build(points4);
  return trajectory_opt.value();
}

void linestring()
{
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 1, Kwargs("figsize"_a = std::make_tuple(8, 8)));
  auto ax = axes[0];

  // Build Trajectory
  auto traj = build_trajectory();
  traj.align_orientation_with_trajectory_direction();

  plot_trajectory(ax, traj, "Line");
  plot_traj_with_orientation(ax, traj);
  {
    const LineString2d line{Point2d{4.0, 0.0}, Point2d{0.0, 4.0}};
    const auto crossed_line = autoware::experimental::trajectory::crossed(traj, line);
    // plot line
    plot_linestring(ax, line, "blue", "cross line 1");
    // using s (arc length) of the traj
    plot_intersection(ax, traj, crossed_line, "blue", "intersection 1");
  }

  {
    const LineString2d line{Point2d{3.0, 0.0}, Point2d{3.0, 4.0}};
    const auto crossed_line = autoware::experimental::trajectory::crossed(traj, line);
    plot_linestring(ax, line, "green", "cross line 2");
    // using s (arc length) of the traj
    plot_intersection(ax, traj, crossed_line, "green", "intersection 2");
  }

  ax.set_title(Args("Cross Linestring"), Kwargs("fontsize"_a = 16));

  ax.legend();
  ax.grid();
  ax.set_aspect(Args("equal"));
  fig.tight_layout();
  plt.savefig(Args("crossed_linestring.svg"));
  plt.show();
}

void polygon()
{
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 1, Kwargs("figsize"_a = std::make_tuple(8, 8)));
  auto ax = axes[0];

  // Build Trajectory
  auto traj = build_trajectory();
  traj.align_orientation_with_trajectory_direction();

  plot_trajectory(ax, traj, "Line");
  plot_traj_with_orientation(ax, traj);

  const std::vector<Point2d> open_polygon{
    Point2d{1.0, 1.0}, Point2d{3.0, 1.0}, Point2d{3.0, 3.0}, Point2d{1.0, 3.0}};
  {
    const auto crossed_line =
      autoware::experimental::trajectory::crossed_with_polygon(traj, open_polygon);
    plot_polygon(ax, open_polygon);
    plot_intersection(
      ax, traj, crossed_line, "red", "Whole line check $ \\rightarrow $ has two Intersections");
  }
  std::vector<double> range = {2.0 * std::sqrt(2.0) - 0.5, 2.0 * std::sqrt(2.0) + 0.5};
  plot_intersection(
    ax, traj, range, "blue",
    "Check from $ s=\\sqrt{2.0} - 0.5 $ to $ s=\\sqrt{2.0} + 0.5 \\rightarrow $ no Intersection");

  ax.set_title(Args("Cross Polygon"), Kwargs("fontsize"_a = 16));

  ax.legend();
  ax.grid();
  ax.set_aspect(Args("equal"));
  fig.tight_layout();
  plt.savefig(Args("crossed_polygon.svg"));
  plt.show();
}

int main()
{
  pybind11::scoped_interpreter guard{};
  linestring();
  polygon();
  return 0;
}
