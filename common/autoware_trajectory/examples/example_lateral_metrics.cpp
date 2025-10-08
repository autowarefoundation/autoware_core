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
#include "autoware/trajectory/utils/lateral_metrics.hpp"

#include <autoware/pyplot/pyplot.hpp>
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

static void plot_point(
  autoware::pyplot::Axes & ax, const geometry_msgs::msg::Point & point, const std::string & color,
  const std::string & label, const bool & with_text = false)
{
  ax.scatter(Args(point.x, point.y), Kwargs("color"_a = color, "label"_a = label));
  if (with_text) {
    ax.text(Args(point.x, point.y, label), Kwargs("va"_a = "bottom", "ha"_a = "center"));
  }
}

static void draw_line(
  autoware::pyplot::Axes & ax, const geometry_msgs::msg::Point & from_point,
  const geometry_msgs::msg::Point & to_point, const std::string & color, const std::string & label,
  const bool & with_text = false)
{
  std::vector<double> x_{from_point.x, to_point.x};
  std::vector<double> y_{from_point.y, to_point.y};
  ax.plot(Args(x_, y_), Kwargs("color"_a = color, "label"_a = label, "linestyle"_a = "--"));
  if (with_text) {
    ax.text(
      Args((from_point.x + to_point.x) / 2, (from_point.y + to_point.y) / 2, label),
      Kwargs("color"_a = "black", "va"_a = "bottom", "ha"_a = "center"));
  }
}

static void draw_square_angle_line(
  autoware::pyplot::Axes & ax, const geometry_msgs::msg::Pose & from_pose,
  const geometry_msgs::msg::Point & to_point, const std::string & color, const std::string & label)
{
  const auto lateral_offset = autoware_utils_geometry::calc_lateral_deviation(from_pose, to_point);
  const auto vertical_point =
    autoware_utils_geometry::calc_offset_pose(from_pose, 0, lateral_offset, 0, 0);

  std::vector<double> x_{from_pose.position.x, vertical_point.position.x, to_point.x};
  std::vector<double> y_{from_pose.position.y, vertical_point.position.y, to_point.y};
  ax.plot(Args(x_, y_), Kwargs("color"_a = color, "label"_a = label, "linestyle"_a = "--"));
}

static void draw_tangent_line(
  autoware::pyplot::Axes & ax, const geometry_msgs::msg::Pose & target_pose)
{
  const auto front_point = autoware_utils_geometry::calc_offset_pose(target_pose, 1.5, 0, 0, 0);
  const auto back_point = autoware_utils_geometry::calc_offset_pose(target_pose, -1.5, 0, 0, 0);
  std::vector<double> x_{front_point.position.x, back_point.position.x};
  std::vector<double> y_{front_point.position.y, back_point.position.y};
  ax.plot(Args(x_, y_), Kwargs("color"_a = "grey", "label"_a = "tangent line"));
}

int main1()
{
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 2, Kwargs("figsize"_a = std::make_tuple(18, 15)));
  auto & ax1 = axes[0];
  auto & ax2 = axes[1];

  geometry_msgs::msg::Point target_point;
  target_point.x = 1.0;
  target_point.y = 0.5;
  target_point.z = 0.0;

  // To the right

  auto traj = build_parabolic_trajectory(11, 0.5);
  plot_trajectory(ax1, traj, "Parabolic to the right");
  plot_traj_with_orientation(ax1, traj);

  const auto nearest_s = autoware::experimental::trajectory::find_nearest_index(traj, target_point);

  const auto target_s = nearest_s - 0.5;
  const auto target_pose = traj.compute(target_s);

  const auto longitudinal_offset =
    autoware_utils_geometry::calc_longitudinal_deviation(target_pose, target_point);
  const auto vertical_pose =
    autoware_utils_geometry::calc_offset_pose(target_pose, longitudinal_offset, 0, 0, 0);

  auto on_left = autoware::experimental::trajectory::is_left_side(traj, target_point, nearest_s);
  auto line_label = on_left ? "left" : "right";

  plot_point(ax1, target_point, "red", "Target Point");
  plot_point(ax1, target_pose.position, "orange", "Target s");
  plot_point(ax1, vertical_pose.position, "blue", "Perpendicular Point");

  draw_line(ax1, target_point, vertical_pose.position, "orange", "lateral distance");
  draw_line(ax1, target_point, target_pose.position, "red", "distance2d");
  draw_square_angle_line(ax1, target_pose, target_point, "black", line_label);
  draw_tangent_line(ax1, target_pose);

  plot_trajectory(ax2, traj, "Parabolic to the right");
  plot_point(ax2, target_point, "red", "Target Point", true);
  plot_point(ax2, target_pose.position, "orange", "Target s", true);
  plot_point(ax2, vertical_pose.position, "blue", "Perpendicular Point", true);

  draw_line(ax2, target_point, vertical_pose.position, "orange", "lateral distance", true);
  draw_line(ax2, target_point, target_pose.position, "red", "distance2d", true);
  draw_square_angle_line(ax2, target_pose, target_point, "black", line_label);
  draw_tangent_line(ax2, target_pose);

  ax2.set_title(Args("To the right (Point on the Right)"), Kwargs("fontsize"_a = 16));
  ax2.set_xlim(Args(std::make_tuple(0, 1.5)));
  ax2.set_ylim(Args(std::make_tuple(0, 1)));

  for (auto & ax : axes) {
    ax.grid();
    ax.legend();
    ax.set_aspect(Args("equal"));
  }
  fig.tight_layout();
  plt.show();

  return 0;
}

int main2()
{
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 2, Kwargs("figsize"_a = std::make_tuple(18, 15)));
  auto & ax1 = axes[0];
  auto & ax2 = axes[1];

  geometry_msgs::msg::Point target_point;
  target_point.x = 1.0;
  target_point.y = 0.5;
  target_point.z = 0.0;

  // To the left

  auto traj = build_parabolic_trajectory(11, 0.5, true);

  plot_trajectory(ax1, traj, "Parabolic to the left");
  plot_traj_with_orientation(ax1, traj);

  const auto nearest_s = autoware::experimental::trajectory::find_nearest_index(traj, target_point);

  const auto target_s = nearest_s + 0.5;
  const auto target_pose = traj.compute(target_s);

  const auto longitudinal_offset =
    autoware_utils_geometry::calc_longitudinal_deviation(target_pose, target_point);
  const auto vertical_pose =
    autoware_utils_geometry::calc_offset_pose(target_pose, longitudinal_offset, 0, 0, 0);

  auto on_left = autoware::experimental::trajectory::is_left_side(traj, target_point, nearest_s);
  auto line_label = on_left ? "left" : "right";

  plot_point(ax1, target_point, "red", "Target Point");
  plot_point(ax1, target_pose.position, "orange", "Target s");
  plot_point(ax1, vertical_pose.position, "blue", "Perpendicular Point");
  draw_line(ax1, target_point, vertical_pose.position, "orange", "lateral distance");
  draw_line(ax1, target_point, target_pose.position, "red", "distance2d");
  draw_square_angle_line(ax1, target_pose, target_point, "black", line_label);
  draw_tangent_line(ax1, target_pose);

  plot_trajectory(ax2, traj, "Parabolic to the right");
  plot_point(ax2, target_point, "red", "Target Point", true);
  plot_point(ax2, target_pose.position, "orange", "Target s", true);
  plot_point(ax2, vertical_pose.position, "blue", "Perpendicular Point", true);

  draw_line(ax2, target_point, vertical_pose.position, "orange", "lateral distance", true);
  draw_line(ax2, target_point, target_pose.position, "red", "distance2d", true);
  draw_square_angle_line(ax2, target_pose, target_point, "black", line_label);
  draw_tangent_line(ax2, target_pose);

  ax2.set_title(Args("To the left (Point on the Left)"), Kwargs("fontsize"_a = 16));
  ax2.set_xlim(Args(std::make_tuple(0, 1.5)));
  ax2.set_ylim(Args(std::make_tuple(0, 1)));

  for (auto & ax : axes) {
    ax.grid();
    ax.legend();
    ax.set_aspect(Args("equal"));
  }
  fig.tight_layout();
  plt.show();

  return 0;
}

int main()
{
  pybind11::scoped_interpreter guard{};
  main1();
  main2();
  return 0;
}
