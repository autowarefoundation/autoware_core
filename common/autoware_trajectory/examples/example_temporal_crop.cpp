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

#include "autoware/trajectory/temporal_trajectory.hpp"
#include "autoware/trajectory/utils/pretty_build.hpp"
#include "autoware_utils_geometry/geometry.hpp"

#include <autoware/pyplot/pyplot.hpp>
#include <rclcpp/duration.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <iostream>
#include <tuple>
#include <utility>
#include <vector>

using autoware::experimental::trajectory::TemporalTrajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_geometry::create_quaternion_from_yaw;

namespace
{

TrajectoryPoint make_point(const double x, const double y, const double yaw, const double time)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.orientation = create_quaternion_from_yaw(yaw);
  point.longitudinal_velocity_mps = 4.0F;
  point.time_from_start = rclcpp::Duration::from_seconds(time);
  return point;
}

TemporalTrajectory make_trajectory()
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0, 0.00, 0.0), make_point(1.0, 0.2, 0.05, 0.7),
    make_point(2.2, 0.9, 0.25, 1.6), make_point(3.6, 1.8, 0.45, 2.6),
    make_point(5.1, 2.0, 0.20, 3.8), make_point(6.7, 1.5, -0.15, 5.0),
    make_point(8.1, 0.4, -0.35, 6.2)};

  const auto trajectory = autoware::experimental::trajectory::pretty_build_temporal(points, true);
  if (!trajectory) {
    throw std::runtime_error("failed to pretty-build temporal trajectory");
  }
  return *trajectory;
}

std::pair<std::vector<double>, std::vector<double>> sample_xy(const TemporalTrajectory & trajectory)
{
  const auto points = trajectory.restore();
  std::vector<double> x;
  std::vector<double> y;
  x.reserve(points.size());
  y.reserve(points.size());
  for (const auto & point : points) {
    x.push_back(point.pose.position.x);
    y.push_back(point.pose.position.y);
  }
  return {x, y};
}

std::pair<std::vector<double>, std::vector<double>> sample_time_distance(
  const TemporalTrajectory & trajectory)
{
  const auto points = trajectory.restore();
  std::vector<double> time;
  std::vector<double> distance;
  time.reserve(points.size());
  distance.reserve(points.size());
  for (const auto & point : points) {
    const auto t = rclcpp::Duration(point.time_from_start).seconds();
    time.push_back(t);
    distance.push_back(trajectory.time_to_distance(t));
  }
  return {time, distance};
}

void plot_xy(
  autoware::pyplot::Axes & ax, const TemporalTrajectory & trajectory, const char * label,
  const char * color)
{
  const auto [x, y] = sample_xy(trajectory);
  ax.plot(Args(x, y), Kwargs("label"_a = label, "color"_a = color, "marker"_a = "o"));
}

void plot_time_distance(
  autoware::pyplot::Axes & ax, const TemporalTrajectory & trajectory, const char * label,
  const char * color)
{
  const auto [time, distance] = sample_time_distance(trajectory);
  ax.plot(Args(time, distance), Kwargs("label"_a = label, "color"_a = color, "marker"_a = "o"));
}
}  // namespace

int main()
{
  pybind11::scoped_interpreter guard{};
  auto plt = autoware::pyplot::import();

  const auto original = make_trajectory();

  auto early_window = original;
  early_window.crop_time(0.8, 2.4);

  auto late_window = original;
  late_window.crop_time(2.8, 2.2);

  auto rebased_early_window = early_window;
  rebased_early_window.set_time_offset(early_window.start_time());

  std::cout << "original: [" << original.start_time() << ", " << original.end_time() << "]\n";
  std::cout << "early_window: [" << early_window.start_time() << ", " << early_window.end_time()
            << "]\n";
  std::cout << "rebased_early_window: [" << rebased_early_window.start_time() << ", "
            << rebased_early_window.end_time() << "]\n";
  std::cout << "late_window: [" << late_window.start_time() << ", " << late_window.end_time()
            << "]\n";

  auto [fig, axes] = plt.subplots(1, 2, Kwargs("figsize"_a = std::make_tuple(14, 6)));

  {
    auto ax = axes[0];
    plot_xy(ax, original, "original", "navy");
    plot_xy(ax, early_window, "window [0.8, 3.2]", "darkorange");
    plot_xy(ax, late_window, "window [2.8, 5.0]", "crimson");
    ax.set_title(Args("Spatial Windows From crop_time"));
    ax.grid();
    ax.legend();
    ax.set_aspect(Args("equal"));
  }

  {
    auto ax = axes[1];
    plot_time_distance(ax, original, "original", "navy");
    plot_time_distance(ax, early_window, "window [0.8, 3.2]", "darkorange");
    plot_time_distance(ax, late_window, "window [2.8, 5.0]", "crimson");
    ax.set_title(Args("Absolute Time / Cropped Distance"));
    ax.set_xlabel(Args("time [s]"));
    ax.set_ylabel(Args("distance [m]"));
    ax.grid();
    ax.legend();
  }

  fig.tight_layout();
  plt.savefig(Args("temporal_trajectory_crop.svg"));
  plt.show();
  return 0;
}
