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
#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/utils/find_intervals.hpp"

#include <autoware/pyplot/pyplot.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/duration.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_geometry::create_quaternion_from_yaw;

TrajectoryPoint make_point(const double x, const double y, const double yaw, const double time)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.orientation = create_quaternion_from_yaw(yaw);
  point.longitudinal_velocity_mps = 2.0F;
  point.time_from_start = rclcpp::Duration::from_seconds(time);
  return point;
}

int main()
{
  try {
    pybind11::scoped_interpreter guard{};
    auto plt = autoware::pyplot::import();

    constexpr double radius = 20.0;
    constexpr double center_x = 0.0;
    constexpr double center_y = radius;
    constexpr double start_angle = -M_PI_2;  // -90 deg
    constexpr double end_angle = 0.0;        // 0 deg
    constexpr double velocity = 2.0;
    constexpr size_t num_arc_points = 15;
    constexpr size_t stop_index = num_arc_points / 2;  // middle of arc
    // constexpr size_t stop_index = -1;
    constexpr double stop_duration = 3.0;  // total stop time

    std::vector<TrajectoryPoint> points;
    points.reserve(num_arc_points + 2);

    double prev_x = 0.0;
    double prev_y = 0.0;
    double accumulated_time = 0.0;
    double stop_x = 0.0;
    double stop_y = 0.0;

    for (size_t i = 0; i < num_arc_points; ++i) {
      const double angle = start_angle + (end_angle - start_angle) * static_cast<double>(i) /
                                           static_cast<double>(num_arc_points - 1);
      const double x = center_x + radius * std::cos(angle);
      const double y = center_y + radius * std::sin(angle);
      const double yaw = angle + M_PI_2;  // tangent direction

      if (i > 0) {
        accumulated_time += std::hypot(x - prev_x, y - prev_y) / velocity;
      }

      auto point = make_point(x, y, yaw, accumulated_time);

      if (i == stop_index) {
        stop_x = x;
        stop_y = y;
        // Insert stop point with multiple duplicated points
        point.longitudinal_velocity_mps = 0.0F;
        points.push_back(point);

        accumulated_time += stop_duration / 2.0;
        auto p2 = make_point(x, y, yaw, accumulated_time);
        p2.longitudinal_velocity_mps = 0.0F;
        points.push_back(p2);

        accumulated_time += stop_duration / 2.0;
        auto p3 = make_point(x, y, yaw, accumulated_time);
        p3.longitudinal_velocity_mps = 0.0F;
        points.push_back(p3);
      } else {
        points.push_back(point);
      }

      prev_x = x;
      prev_y = y;
    }

    const auto trajectory_opt =
      autoware::experimental::trajectory::TemporalTrajectory::Builder{}.build(points);
    if (!trajectory_opt) {
      std::cerr << "Failed to build temporal trajectory" << std::endl;
      return 1;
    }
    const auto & trajectory = *trajectory_opt;

    // Sample for plotting based on time
    constexpr size_t num_samples = 200;
    const auto start_time = trajectory.start_time();
    const auto duration = trajectory.duration();
    std::vector<double> sample_times;
    sample_times.reserve(num_samples);
    for (size_t i = 0; i < num_samples; ++i) {
      sample_times.push_back(
        start_time + duration * static_cast<double>(i) / static_cast<double>(num_samples - 1));
    }

    std::vector<double> sample_x;
    std::vector<double> sample_y;
    std::vector<double> sample_distances;
    sample_x.reserve(num_samples);
    sample_y.reserve(num_samples);
    sample_distances.reserve(num_samples);

    for (const auto t : sample_times) {
      const auto point = trajectory.compute_from_time(t);
      sample_x.push_back(point.pose.position.x);
      sample_y.push_back(point.pose.position.y);
      sample_distances.push_back(trajectory.time_to_distance(t));
    }

    const auto curvature_values = trajectory.spatial_trajectory().curvature(sample_distances);

    // Restore original input points for stop-point marker
    const auto restored = trajectory.restore();
    std::vector<double> restored_x;
    std::vector<double> restored_y;
    for (const auto & p : restored) {
      restored_x.push_back(p.pose.position.x);
      restored_y.push_back(p.pose.position.y);
    }

    const auto stop_intervals =
      autoware::experimental::trajectory::find_intervals(trajectory, [](const TrajectoryPoint & p) {
        return std::abs(p.longitudinal_velocity_mps) <=
               autoware::experimental::trajectory::k_epsilon_velocity;
      });
    const bool has_stop = !stop_intervals.empty();
    const double stop_time = has_stop ? stop_intervals.front().start.time : 0.0;
    const double stop_distance = has_stop ? stop_intervals.front().start.distance : 0.0;

    auto [fig, axes] = plt.subplots(1, 3, Kwargs("figsize"_a = std::make_tuple(18, 6)));

    {
      auto ax = axes[0];
      ax.plot(Args(sample_x, sample_y), Kwargs("label"_a = "trajectory", "color"_a = "navy"));
      ax.scatter(
        Args(restored_x, restored_y), Kwargs("color"_a = "navy", "s"_a = 20, "zorder"_a = 3));
      if (has_stop) {
        ax.scatter(
          Args(std::vector<double>{stop_x}, std::vector<double>{stop_y}),
          Kwargs("color"_a = "crimson", "s"_a = 100, "zorder"_a = 5, "label"_a = "stop point"));
      }
      ax.set_title(Args("Arc Trajectory with Stop Point"));
      ax.set_xlabel(Args("x [m]"));
      ax.set_ylabel(Args("y [m]"));
      ax.grid();
      ax.legend();
      ax.set_aspect(Args("equal"));
    }

    {
      auto ax = axes[1];
      ax.plot(
        Args(sample_times, sample_distances),
        Kwargs("label"_a = "time-distance", "color"_a = "darkorange"));
      if (has_stop) {
        ax.scatter(
          Args(std::vector<double>{stop_time}, std::vector<double>{stop_distance}),
          Kwargs("color"_a = "crimson", "s"_a = 100, "zorder"_a = 5, "label"_a = "stop point"));
      }
      ax.set_title(Args("Time vs Arc Length"));
      ax.set_xlabel(Args("time [s]"));
      ax.set_ylabel(Args("distance [m]"));
      ax.grid();
      ax.legend();
    }

    {
      auto ax = axes[2];
      ax.plot(
        Args(sample_times, curvature_values),
        Kwargs("label"_a = "curvature", "color"_a = "darkgreen"));
      ax.set_title(Args("Curvature Profile"));
      ax.set_xlabel(Args("time [s]"));
      ax.set_ylabel(Args("curvature [1/m]"));
      ax.grid();
      ax.legend();
    }

    fig.tight_layout();
    plt.show();

    return 0;
  } catch (const std::exception & e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 1;
  }
}
