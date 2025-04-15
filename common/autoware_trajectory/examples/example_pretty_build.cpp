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

#include "autoware/trajectory/utils/pretty_build.hpp"
#include "autoware_utils_geometry/geometry.hpp"

#include <autoware/pyplot/pyplot.hpp>
#include <range/v3/all.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <iostream>
#include <string>
#include <vector>

using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_utils_geometry::create_quaternion_from_yaw;
using autoware_utils_geometry::get_rpy;
using geometry_msgs::build;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Quaternion;
using ranges::to;
using ranges::views::transform;

void plot_original_point(
  const PathPointWithLaneId & point, const std::string & label, autoware::pyplot::Axes & ax)
{
  const auto & pose = point.point.pose;
  ax.scatter(Args(pose.position.x, pose.position.y), Kwargs("color"_a = "red"));
  ax.text(Args(pose.position.x, pose.position.y - 0.05, label));

  const auto yaw = get_rpy(pose.orientation).z;
  ax.quiver(
    Args(pose.position.x, pose.position.y, std::cos(yaw), std::sin(yaw)),
    Kwargs(
      "color"_a = "red", "scale"_a = 5.0, "angles"_a = "xy", "scale_units"_a = "xy",
      "linewidth"_a = 2.0));
}

int main1()
{
  auto plt = autoware::pyplot::import();

  std::vector<PathPointWithLaneId> points;
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(1.0).y(1.0).z(0.0))
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
                         .position(build<Point>().x(0.7).y(0.3).z(0.0))
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
                         .position(build<Point>().x(2.0).y(2.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(M_PI / 2.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  auto trajectory_opt = autoware::experimental::trajectory::pretty_build(points);
  if (!trajectory_opt) {
    return 1;
  }
  auto & trajectory = trajectory_opt.value();

  auto [fig, axes] = plt.subplots(1, 2);
  auto & ax1 = axes[0];
  auto & ax2 = axes[1];

  // NOLINTBEGIN
  const auto plot =
    [&](
      const autoware::experimental::trajectory::Trajectory<PathPointWithLaneId> & traj,
      autoware::pyplot::Axes & axes) {
      const auto s = traj.base_arange(0.05);
      const auto C =
        s | transform([&](const double s) { return traj.compute(s); }) | to<std::vector>();
      const auto Cx =
        C | transform([](const auto & p) { return p.point.pose.position.x; }) | to<std::vector>();
      const auto Cy =
        C | transform([](const auto & p) { return p.point.pose.position.y; }) | to<std::vector>();
      const auto th = traj.azimuth(s);
      const auto cos_th =
        th | transform([&](const double s) { return std::cos(s); }) | to<std::vector>();
      const auto sin_th =
        th | transform([&](const double s) { return std::sin(s); }) | to<std::vector>();

      const auto yaw = s | transform([&](const double s) {
                         return get_rpy(traj.compute(s).point.pose.orientation).z;
                       }) |
                       to<std::vector>();
      const auto cos_yaw =
        yaw | transform([&](const double s) { return std::cos(s); }) | to<std::vector>();
      const auto sin_yaw =
        yaw | transform([&](const double s) { return std::sin(s); }) | to<std::vector>();

      axes.plot(Args(Cx, Cy), Kwargs("label"_a = "interpolated", "color"_a = "magenta"));
      axes.quiver(
        Args(Cx, Cy, cos_th, sin_th),
        Kwargs(
          "color"_a = "green", "scale"_a = 8.0, "angles"_a = "xy", "scale_units"_a = "xy",
          "label"_a = "azimuth", "alpha"_a = 0.5));
      axes.quiver(
        Args(Cx, Cy, cos_yaw, sin_yaw),
        Kwargs(
          "color"_a = "orange", "scale"_a = 11.0, "angles"_a = "xy", "scale_units"_a = "xy",
          "label"_a = "orientation", "alpha"_a = 0.5));
    };

  plot(trajectory, ax1);
  plot_original_point(points[0], "input 1", ax1);
  plot_original_point(points[1], "input 2", ax1);
  plot_original_point(points[2], "input 3", ax1);
  ax1.set_title(Args("Before align_orientation"));

  trajectory.align_orientation_with_trajectory_direction();

  plot(trajectory, ax2);
  ax2.set_title(Args("After align_orientation"));

  for (auto & ax : axes) {
    ax.grid();
    ax.legend();
    ax.set_aspect(Args("equal"));
  }
  fig.tight_layout();
  plt.show();
  // NOLINTEND
  return 0;
}

int main2()
{
  auto plt = autoware::pyplot::import();

  std::vector<PathPointWithLaneId> points;
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(1.0).y(1.0).z(0.0))
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
                         .position(build<Point>().x(1.5).y(0.5).z(0.0))
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
                         .position(build<Point>().x(2.0).y(2.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(M_PI / 2.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(2.5).y(3.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(M_PI / 2.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }

  auto trajectory = *autoware::experimental::trajectory::pretty_build(points);

  auto [fig, axes] = plt.subplots(1, 2);
  auto & ax1 = axes[0];
  auto & ax2 = axes[1];

  // NOLINTBEGIN
  const auto plot =
    [&](
      const autoware::experimental::trajectory::Trajectory<PathPointWithLaneId> & traj,
      autoware::pyplot::Axes & axes) {
      const auto s = traj.base_arange(0.05);
      const auto C =
        s | transform([&](const double s) { return traj.compute(s); }) | to<std::vector>();
      const auto Cx =
        C | transform([](const auto & p) { return p.point.pose.position.x; }) | to<std::vector>();
      const auto Cy =
        C | transform([](const auto & p) { return p.point.pose.position.y; }) | to<std::vector>();
      const auto th = traj.azimuth(s);
      const auto cos_th =
        th | transform([&](const double s) { return std::cos(s); }) | to<std::vector>();
      const auto sin_th =
        th | transform([&](const double s) { return std::sin(s); }) | to<std::vector>();

      const auto yaw = s | transform([&](const double s) {
                         return get_rpy(traj.compute(s).point.pose.orientation).z;
                       }) |
                       to<std::vector>();
      const auto cos_yaw =
        yaw | transform([&](const double s) { return std::cos(s); }) | to<std::vector>();
      const auto sin_yaw =
        yaw | transform([&](const double s) { return std::sin(s); }) | to<std::vector>();

      axes.plot(Args(Cx, Cy), Kwargs("label"_a = "interpolated", "color"_a = "magenta"));
      axes.quiver(
        Args(Cx, Cy, cos_th, sin_th),
        Kwargs(
          "color"_a = "green", "scale"_a = 8.0, "angles"_a = "xy", "scale_units"_a = "xy",
          "label"_a = "azimuth", "alpha"_a = 0.5));
      axes.quiver(
        Args(Cx, Cy, cos_yaw, sin_yaw),
        Kwargs(
          "color"_a = "orange", "scale"_a = 11.0, "angles"_a = "xy", "scale_units"_a = "xy",
          "label"_a = "orientation", "alpha"_a = 0.5));
    };

  plot(trajectory, ax1);
  plot_original_point(points[0], "input 1", ax1);
  plot_original_point(points[1], "input 2", ax1);
  plot_original_point(points[2], "input 3", ax1);
  plot_original_point(points[3], "input 4", ax1);
  ax1.set_title(Args("Before align_orientation"));

  trajectory.align_orientation_with_trajectory_direction();

  plot(trajectory, ax2);
  ax2.set_title(Args("After align_orientation"));

  for (auto & ax : axes) {
    ax.grid();
    ax.legend();
    ax.set_aspect(Args("equal"));
  }
  fig.tight_layout();
  plt.show();
  // NOLINTEND
  return 0;
}

int main()
{
  pybind11::scoped_interpreter guard{};
  main1();
  main2();
  return 0;
}
