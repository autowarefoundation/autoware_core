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
#include "autoware/trajectory/utils/crossed.hpp"
#include "autoware/trajectory/utils/pretty_build.hpp"
#include "autoware/trajectory/utils/temporal/find_stop_points.hpp"

#include <autoware/pyplot/pyplot.hpp>
#include <rclcpp/duration.hpp>

#include <lanelet2_core/primitives/LineString.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <iostream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using autoware::experimental::trajectory::TemporalTrajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

constexpr size_t num_plot_samples = 30;

TrajectoryPoint make_point(const double x, const double y, const double time_from_start)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.longitudinal_velocity_mps = 3.0F;
  point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
  return point;
}

TemporalTrajectory build_temporal_trajectory()
{
  const std::vector<TrajectoryPoint> points{make_point(0.0, 0.0, 0.0), make_point(1.0, 0.3, 0.8),
                                            make_point(2.2, 1.2, 1.7), make_point(3.4, 2.1, 2.5),
                                            make_point(4.8, 2.6, 3.4), make_point(6.2, 2.2, 4.3),
                                            make_point(7.6, 1.0, 5.2)};

  auto trajectory = autoware::experimental::trajectory::pretty_build_temporal(points);
  if (!trajectory) {
    throw std::runtime_error("failed to pretty-build temporal trajectory");
  }
  return *trajectory;
}

std::vector<double> sample_times(const TemporalTrajectory & trajectory)
{
  if (num_plot_samples <= 1U || trajectory.duration() <= 0.0) {
    return {trajectory.start_time()};
  }

  std::vector<double> times;
  times.reserve(num_plot_samples);
  const auto start = trajectory.start_time();
  const auto step = trajectory.duration() / static_cast<double>(num_plot_samples - 1U);
  for (size_t i = 0; i < num_plot_samples; ++i) {
    times.push_back(start + step * static_cast<double>(i));
  }
  return times;
}

std::pair<std::vector<double>, std::vector<double>> sample_xy(const TemporalTrajectory & trajectory)
{
  const auto points = trajectory.compute_from_time(sample_times(trajectory));
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
  const auto points = trajectory.compute_from_time(sample_times(trajectory));
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

std::pair<std::vector<double>, std::vector<double>> sample_time_velocity(
  const TemporalTrajectory & trajectory)
{
  const auto points = trajectory.compute_from_time(sample_times(trajectory));
  std::vector<double> time;
  std::vector<double> velocity;
  time.reserve(points.size());
  velocity.reserve(points.size());
  for (const auto & point : points) {
    time.push_back(rclcpp::Duration(point.time_from_start).seconds());
    velocity.push_back(point.longitudinal_velocity_mps);
  }
  return {time, velocity};
}

void plot_spatial_trajectory(
  autoware::pyplot::Axes & ax, const TemporalTrajectory & trajectory, const std::string & label,
  const std::string & color)
{
  const auto [x, y] = sample_xy(trajectory);
  ax.plot(Args(x, y), Kwargs("label"_a = label, "color"_a = color, "marker"_a = "o"));
}

void plot_time_series(
  autoware::pyplot::Axes & ax, const std::vector<double> & x, const std::vector<double> & y,
  const std::string & label, const std::string & color)
{
  ax.plot(Args(x, y), Kwargs("label"_a = label, "color"_a = color, "marker"_a = "o"));
}

int main()
{
  pybind11::scoped_interpreter guard{};
  auto plt = autoware::pyplot::import();

  const auto original = build_temporal_trajectory();

  lanelet::LineString2d stop_line(lanelet::InvalId);
  stop_line.push_back(lanelet::Point3d(lanelet::InvalId, 4.1, 0.4, 0.0));
  stop_line.push_back(lanelet::Point3d(lanelet::InvalId, 4.1, 3.5, 0.0));

  const auto crossed_points = autoware::experimental::trajectory::crossed(original, stop_line);
  if (crossed_points.empty()) {
    std::cerr << "Failed to find a stopline intersection" << std::endl;
    return 1;
  }
  const auto stop_length = crossed_points.front();

  auto stop_immediate = original;
  stop_immediate.set_stopline(stop_length);

  auto stop_with_wait = original;
  stop_with_wait.set_stopline(stop_length, 2.0);

  const auto crossed_point = original.compute_from_distance(stop_length);
  const auto [original_time, original_distance] = sample_time_distance(original);
  const auto [immediate_time, immediate_distance] = sample_time_distance(stop_immediate);
  const auto [wait_time, wait_distance] = sample_time_distance(stop_with_wait);
  const auto [original_vel_time, original_vel] = sample_time_velocity(original);
  const auto [immediate_vel_time, immediate_vel] = sample_time_velocity(stop_immediate);
  const auto [wait_vel_time, wait_vel] = sample_time_velocity(stop_with_wait);
  const auto wait_stop_points =
    autoware::experimental::trajectory::find_stop_points(stop_with_wait);

  auto [fig, axes] = plt.subplots(1, 3, Kwargs("figsize"_a = std::make_tuple(18, 6)));

  {
    auto ax = axes[0];
    plot_spatial_trajectory(ax, original, "original", "navy");
    plot_spatial_trajectory(ax, stop_immediate, "set_stopline(length)", "darkorange");
    plot_spatial_trajectory(ax, stop_with_wait, "set_stopline(length, duration)", "crimson");
    ax.plot(
      Args(
        std::vector<double>{stop_line[0].x(), stop_line[1].x()},
        std::vector<double>{stop_line[0].y(), stop_line[1].y()}),
      Kwargs("color"_a = "forestgreen", "linestyle"_a = "--", "label"_a = "stop line"));
    ax.scatter(
      Args(crossed_point.pose.position.x, crossed_point.pose.position.y),
      Kwargs("color"_a = "black", "s"_a = 60, "label"_a = "crossing point"));
    ax.set_title(Args("Spatial Trajectories"));
    ax.grid();
    ax.legend();
    ax.set_aspect(Args("equal"));
  }

  {
    auto ax = axes[1];
    plot_time_series(ax, original_time, original_distance, "original", "navy");
    plot_time_series(ax, immediate_time, immediate_distance, "set_stopline(length)", "darkorange");
    plot_time_series(ax, wait_time, wait_distance, "set_stopline(length, duration)", "crimson");
    ax.scatter(
      Args(
        std::vector<double>{rclcpp::Duration(crossed_point.time_from_start).seconds()},
        std::vector<double>{stop_length}),
      Kwargs("color"_a = "black", "s"_a = 60, "label"_a = "crossing point"));
    ax.set_title(Args("Time to Distance"));
    ax.set_xlabel(Args("time [s]"));
    ax.set_ylabel(Args("distance [m]"));
    ax.grid();
    ax.legend();
  }

  {
    auto ax = axes[2];
    plot_time_series(ax, original_vel_time, original_vel, "original", "navy");
    plot_time_series(ax, immediate_vel_time, immediate_vel, "set_stopline(length)", "darkorange");
    plot_time_series(ax, wait_vel_time, wait_vel, "set_stopline(length, duration)", "crimson");
    if (!wait_stop_points.empty()) {
      ax.plot(
        Args(
          std::vector<double>{
            wait_stop_points.front().start_time, wait_stop_points.front().start_time},
          std::vector<double>{0.0, 3.5}),
        Kwargs("color"_a = "grey", "linestyle"_a = "--", "label"_a = "stop start/end"));
      ax.plot(
        Args(
          std::vector<double>{wait_stop_points.front().end_time, wait_stop_points.front().end_time},
          std::vector<double>{0.0, 3.5}),
        Kwargs("color"_a = "grey", "linestyle"_a = "--"));
    }
    ax.set_title(Args("Velocity Profile"));
    ax.set_xlabel(Args("time [s]"));
    ax.set_ylabel(Args("velocity [m/s]"));
    ax.grid();
    ax.legend();
  }

  fig.tight_layout();
  plt.savefig(Args("temporal_trajectory_stopline.svg"));
  plt.show();
  return 0;
}
