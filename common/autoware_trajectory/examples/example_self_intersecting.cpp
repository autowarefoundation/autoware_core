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

#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/utils/find_intervals.hpp"
#include "autoware/trajectory/utils/find_nearest.hpp"

#include <autoware/pyplot/pyplot.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/all.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using autoware::experimental::trajectory::Trajectory;
using autoware_utils_geometry::create_point;
using autoware_utils_geometry::create_quaternion_from_rpy;
using ranges::to;
using ranges::views::transform;

static geometry_msgs::msg::Pose make_pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::Pose p;
  p.position = create_point(x, y, 0.0);
  p.orientation = create_quaternion_from_rpy(0.0, 0.0, yaw);
  return p;
}

static Trajectory<geometry_msgs::msg::Pose> build_bow_trajectory(
  const size_t num_points, const double size, const double total_angle)
{
  std::vector<geometry_msgs::msg::Pose> raw_poses;
  raw_poses.reserve(num_points);

  auto delta_theta = total_angle / num_points;

  for (size_t i = 0; i < num_points; ++i) {
    double theta = M_PI / 4 + i * delta_theta;

    double x = size * cos(theta);
    double y = size * sin(theta) * cos(theta);
    double yaw = std::atan2(cos(2 * theta), -1 * sin(theta));
    raw_poses.push_back(make_pose(x, y, yaw));
  }
  auto traj = Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(raw_poses);
  return traj.value();
}

static double calculate_bow_trajectory_yaw(const double theta)
{
  return std::atan2(cos(2 * theta), -1 * sin(theta));
}

static double calculate_bow_trajectory_yaw_from_x(const double x, double size = 3)
{
  auto theta = std::acos(x / size);
  return calculate_bow_trajectory_yaw(theta);
}

static Trajectory<geometry_msgs::msg::Pose> build_vertical_loop_trajectory(
  const size_t num_points, const double radius, const double start_x, const double offset)
{
  std::vector<geometry_msgs::msg::Pose> raw_poses;
  raw_poses.reserve(num_points);

  size_t first_part = static_cast<size_t>(num_points / 3);
  size_t last_part = static_cast<size_t>(num_points / 3);
  size_t second_part = static_cast<size_t>(num_points - first_part - last_part);

  // First part: going in (from right to left)
  double rate = (start_x) / (first_part);
  for (size_t i = 0u; i < first_part; ++i) {
    double x = start_x - rate * i;
    double y = offset;
    raw_poses.push_back(make_pose(x, y, M_PI));
  }

  // Second part: go in the loop
  double start_theta = M_PI * 3 / 2;
  double loop_rate = 2 * M_PI / (second_part - 1);
  for (size_t i = 0u; i < second_part; ++i) {
    // from 3/2 pi to -1/2pi
    double theta = start_theta - i * loop_rate;
    double x = radius * cos(theta);
    double y = (offset + radius) + radius * sin(theta);
    raw_poses.push_back(make_pose(x, y, theta - M_PI / 2));
  }

  // Last part: go out to the left
  double out_rate = (start_x) / (first_part);
  for (size_t i = 1u; i <= last_part; ++i) {
    double x = 0 - out_rate * i;
    double y = offset;
    raw_poses.push_back(make_pose(x, y, M_PI));
  }

  auto traj = Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(raw_poses);
  return traj.value();
}

static Trajectory<geometry_msgs::msg::Pose> build_lollipop_trajectory(
  const size_t num_points, const double radius, const double phase_dif = M_PI / 6,
  const double start_x = 3, const double offset = 0)
{
  std::vector<geometry_msgs::msg::Pose> raw_poses;
  raw_poses.reserve(num_points);

  size_t first_part = static_cast<size_t>(num_points / 3);
  size_t last_part = static_cast<size_t>(num_points / 3);
  size_t second_part = static_cast<size_t>(num_points - first_part - last_part);

  // First part: go into bottom
  double rate = (start_x) / (first_part);
  for (size_t i = 0u; i < first_part; ++i) {
    double x = start_x - rate * i;
    double y = offset - radius * sin(phase_dif / 2);
    raw_poses.push_back(make_pose(x, y, M_PI));
  }

  // Second part: go in the loop
  double start_theta = 2 * M_PI - phase_dif / 2;
  double end_theta = phase_dif / 2;
  double loop_rate = (end_theta - start_theta) / (second_part - 1);
  for (size_t i = 0u; i < second_part; ++i) {
    // from -phase_dif/2 to +phase_dif/2
    double theta = start_theta + i * loop_rate;
    double x = -radius * cos(phase_dif / 2) + radius * cos(theta);
    double y = offset + radius * sin(theta);
    raw_poses.push_back(make_pose(x, y, theta - M_PI / 2));
  }

  // Last part: go out from top
  double out_rate = (start_x) / (last_part);
  for (size_t i = 1u; i <= last_part; ++i) {
    double x = 0 + out_rate * i;
    double y = offset + radius * sin(phase_dif / 2);
    raw_poses.push_back(make_pose(x, y, 0));
  }

  auto traj = Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(raw_poses);
  return traj.value();
}

template <class TrajectoryPointType>
static void plot_trajectory(
  autoware::pyplot::PyPlot & plt, const Trajectory<TrajectoryPointType> & traj,
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

  plt.plot(Args(x_all, y_all), Kwargs("color"_a = "green", "label"_a = label));
}

template <class TrajectoryPointType>
static void plot_trajectory_base_with_orientation(
  autoware::pyplot::PyPlot & plt, const Trajectory<TrajectoryPointType> & traj)
{
  const auto s = traj.get_underlying_bases();

  const auto c = traj.compute(s);
  const auto x =
    c | transform([](const auto & point) { return point.position.x; }) | to<std::vector>();
  const auto y =
    c | transform([](const auto & point) { return point.position.y; }) | to<std::vector>();

  const auto th =
    c | transform([](const auto & quat) { return autoware_utils_geometry::get_rpy(quat).z; }) |
    to<std::vector>();
  const auto cos_th = th | transform([](const auto v) { return std::cos(v); }) | to<std::vector>();
  const auto sin_th = th | transform([](const auto v) { return std::sin(v); }) | to<std::vector>();
  if (s.size() < 20) {
    plt.scatter(
      Args(x, y), Kwargs("color"_a = "black", "label"_a = "Base Point", "marker"_a = "x"));
    plt.quiver(
      Args(x, y, cos_th, sin_th),
      Kwargs(
        "color"_a = "red", "scale"_a = 30, "label"_a = "Base Point Orientation", "alpha"_a = 0.3));
  } else {
    std::vector<double> skipped_x;
    std::vector<double> skipped_y;
    std::vector<double> skipped_cos_th;
    std::vector<double> skipped_sin_th;
    for (size_t i = 0; i < x.size(); i += static_cast<int>(x.size() / 20)) {
      skipped_x.push_back(x[i]);
      skipped_y.push_back(y[i]);
      skipped_cos_th.push_back(cos_th[i]);
      skipped_sin_th.push_back(sin_th[i]);
    }
    plt.scatter(
      Args(skipped_x, skipped_y),
      Kwargs("color"_a = "black", "label"_a = "Base Point", "marker"_a = "x"));
    plt.quiver(
      Args(skipped_x, skipped_y, skipped_cos_th, skipped_sin_th),
      Kwargs(
        "color"_a = "red", "scale"_a = 30, "label"_a = "Base Point Orientation", "alpha"_a = 0.3));
  }
}

static void plot_queries_pose(
  autoware::pyplot::PyPlot & plt, const std::vector<geometry_msgs::msg::Pose> & queries,
  const std::vector<std::string> & yaws)
{
  assert(queries.size() == yaws.size());
  std::vector<std::string> colors{"orange", "purple", "cyan", "brown", "magenta"};
  const auto x =
    queries | transform([](const auto & pose) { return pose.position.x; }) | to<std::vector>();
  const auto y =
    queries | transform([](const auto & pose) { return pose.position.y; }) | to<std::vector>();
  const auto yaw = queries |
                   transform([](auto & pose) { return autoware_utils_geometry::get_rpy(pose).z; }) |
                   to<std::vector>();
  const auto cos_yaw =
    yaw | transform([](const auto v) { return std::cos(v); }) | to<std::vector>();
  const auto sin_yaw =
    yaw | transform([](const auto v) { return std::sin(v); }) | to<std::vector>();

  for (size_t i = 0; i < queries.size(); ++i) {
    std::ostringstream oss;
    oss << "query point " << i << ": (" << std::fixed << std::setprecision(2) << x[i] << ", "
        << y[i] << ") at phase = " << yaws[i];
    const std::string label = oss.str();
    plt.scatter(Args(x[i], y[i]), Kwargs("color"_a = colors[i], "label"_a = label));
    plt.quiver(Args(x[i], y[i], cos_yaw[i], sin_yaw[i]), Kwargs("color"_a = colors[i]));
  }
}

template <class TrajectoryPointType>
static void plot_nearest_point(
  autoware::pyplot::PyPlot & plt, const Trajectory<TrajectoryPointType> & traj,
  const std::vector<double> & nearest_points)
{
  std::vector<std::string> colors{"orange", "purple", "cyan", "brown", "magenta"};
  const auto c = traj.compute(nearest_points);

  const auto x =
    c | transform([](const auto & point) { return point.position.x; }) | to<std::vector>();
  const auto y =
    c | transform([](const auto & point) { return point.position.y; }) | to<std::vector>();

  for (size_t i = 0; i < x.size(); ++i) {
    std::string label = "Nearest Point of Query point" + std::to_string(i);
    plt.scatter(Args(x[i], y[i]), Kwargs("color"_a = colors[i], "label"_a = label));
  }
}

int main()
{
  pybind11::scoped_interpreter guard{};

  // Bow Trajectory
  {
    auto plt = autoware::pyplot::import();
    auto fig = plt.figure(Args(), Kwargs("figsize"_a = std::make_tuple(18, 6)));

    const size_t num_points = 1000;

    auto traj = build_bow_trajectory(num_points, 3, 1.5 * M_PI);

    std::vector<geometry_msgs::msg::Pose> queries;
    std::vector<std::string> yaws;
    std::vector<double> s_nearests;
    plot_trajectory(plt, traj, "Bow");
    plot_trajectory_base_with_orientation(plt, traj);

    auto yaw = calculate_bow_trajectory_yaw_from_x(1);
    auto query = make_pose(1, 1, yaw);
    queries.push_back(query);
    yaws.push_back("-39/50 * M_PI");

    yaw = calculate_bow_trajectory_yaw(M_PI / 2);
    query = make_pose(-0.05, 0, yaw);
    queries.push_back(query);
    yaws.push_back("-3/4 * M_PI");

    yaw = calculate_bow_trajectory_yaw(M_PI * 3 / 2);
    query = make_pose(-0.05, 0, yaw);
    queries.push_back(query);
    yaws.push_back("-1/4 * M_PI");
    plot_queries_pose(plt, queries, yaws);

    for (const auto & q : queries) {
      auto s_opt = find_first_nearest_index(traj, q, std::numeric_limits<double>::max(), 0.4);
      double s_nearest = *s_opt;
      s_nearests.push_back(s_nearest);
    }
    plot_nearest_point(plt, traj, s_nearests);
    plt.gca().set_aspect(Args("equal", "box"));
    std::string title = "Bow Trajectory with Base Point of " + std::to_string(num_points);
    plt.title(Args(title), Kwargs("fontsize"_a = 18));
    plt.grid();
    plt.legend(
      Args(), Kwargs("loc"_a = "center left", "bbox_to_anchor"_a = std::make_tuple(1.0, 0.5)));
    fig.tight_layout();
    // plt.xlim(Args(-1,1));
    // plt.ylim(Args(-1,1));
    plt.show();
  }

  // Lollipop Trajectory
  {
    auto plt = autoware::pyplot::import();
    auto fig = plt.figure(Args(), Kwargs("figsize"_a = std::make_tuple(18, 6)));
    size_t num_points = 1000;

    auto traj = build_vertical_loop_trajectory(num_points, 3, 3, 0);

    std::vector<geometry_msgs::msg::Pose> queries;
    std::vector<std::string> yaws;
    std::vector<double> s_nearests;
    plot_trajectory(plt, traj, "Bow");
    plot_trajectory_base_with_orientation(plt, traj);

    auto query = make_pose(1, 1, -M_PI);
    queries.push_back(query);
    yaws.push_back("-M_PI");

    query = make_pose(0, 0.05, -M_PI);
    queries.push_back(query);
    yaws.push_back("-M_PI");

    query = make_pose(0, -0.05, M_PI);
    queries.push_back(query);
    yaws.push_back("-M_PI");
    plot_queries_pose(plt, queries, yaws);

    for (const auto & q : queries) {
      auto s_opt = find_first_nearest_index(traj, q, std::numeric_limits<double>::max(), 0.4);
      double s_nearest = *s_opt;
      s_nearests.push_back(s_nearest);
    }
    plot_nearest_point(plt, traj, s_nearests);
    plt.gca().set_aspect(Args("equal", "box"));
    std::string title = "Vertical Loop Trajectory with Base Point of " + std::to_string(num_points);
    plt.title(Args(title), Kwargs("fontsize"_a = 18));
    fig.tight_layout();
    plt.grid();
    plt.legend(
      Args(), Kwargs("loc"_a = "center left", "bbox_to_anchor"_a = std::make_tuple(1.0, 0.5)));

    plt.show();
  }
  {
    auto plt = autoware::pyplot::import();
    auto fig = plt.figure(Args(), Kwargs("figsize"_a = std::make_tuple(18, 6)));

    size_t num_points = 1000;
    auto traj = build_lollipop_trajectory(num_points, 3);

    std::vector<geometry_msgs::msg::Pose> queries;
    std::vector<std::string> yaws;
    std::vector<double> s_nearests;
    plot_trajectory(plt, traj, "Bow");
    plot_trajectory_base_with_orientation(plt, traj);

    auto query = make_pose(1.5, 0, -M_PI);
    queries.push_back(query);
    yaws.push_back("-M_PI");

    query = make_pose(1.5, 0, 0);
    queries.push_back(query);
    yaws.push_back("0");

    query = make_pose(-(3 * cos(M_PI / 12)), 0, M_PI);
    queries.push_back(query);
    yaws.push_back("M_PI");

    query = make_pose(-(3 * cos(M_PI / 12)), 0, M_PI / 2);
    queries.push_back(query);
    yaws.push_back("M_PI/2");

    query = make_pose(-(3 * cos(M_PI / 12)), 0, -M_PI / 4);
    queries.push_back(query);
    yaws.push_back("-M_PI/4");

    plot_queries_pose(plt, queries, yaws);

    for (const auto & q : queries) {
      auto s_opt = find_first_nearest_index(traj, q, std::numeric_limits<double>::max(), M_PI / 3);
      double s_nearest = *s_opt;
      s_nearests.push_back(s_nearest);
    }
    plot_nearest_point(plt, traj, s_nearests);
    plt.gca().set_aspect(Args("equal", "box"));
    std::string title = "Lollipop Trajectory with Base Point of " + std::to_string(num_points);
    plt.title(Args(title), Kwargs("fontsize"_a = 18));
    plt.grid();
    plt.legend(
      Args(), Kwargs("loc"_a = "center left", "bbox_to_anchor"_a = std::make_tuple(1.0, 0.5)));

    fig.tight_layout();

    plt.show();
  }

  return 0;
}
