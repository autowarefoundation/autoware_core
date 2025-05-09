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

#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/reference_path.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/pyplot/pyplot.hpp>
#include <autoware_test_utils/visualization.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/all.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace autoware::experimental;  // NOLINT

int main1()
{
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots();

  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
    "sample_map/straight_waypoint";
  const auto intersection_crossing_map_path = sample_map_dir / "lanelet2_map.osm";

  const auto lanelet_map_ptr =
    lanelet2_utils::load_mgrs_coordinate_map(intersection_crossing_map_path.string());
  const auto [routing_graph, traffic_rules] =
    lanelet2_utils::instantiate_routing_graph_and_traffic_rules(lanelet_map_ptr);

  const std::vector<lanelet::Id> ids = {1043, 1047, 1049};
  const auto lanelet_sequence = ids | ranges::views::transform([&](const auto & id) {
                                  return lanelet_map_ptr->laneletLayer.get(id);
                                }) |
                                ranges::to<std::vector>();
  const auto ego_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(autoware_utils_geometry::create_point(1573.68, 389.857, 100.0))
      .orientation(autoware_utils_geometry::create_quaternion(0.0, 0.0, 0.999997, 0.00250111));
  const auto reference_path_opt = trajectory::build_reference_path(
    lanelet_sequence, lanelet_sequence.front(), ego_pose, lanelet_map_ptr, routing_graph,
    traffic_rules, 1.0, 10.0, 200, 50);

  if (!reference_path_opt) {
    return 0;
  }
  const auto & reference_path = reference_path_opt.value();
  /*
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  path.points = reference_path.restore();
  autoware::test_utils::plot_autoware_object(path, axes);
  for (const auto & route_lanelet : lanelet_sequence) {
    autoware::test_utils::plot_lanelet2_object(route_lanelet, axes);
  }
  axes.set_aspect(Args("equal"));
  */
  const auto ss = reference_path.base_arange(0.1);
  const auto ks = reference_path.curvature(ss);
  axes.plot(Args(ss, ks), Kwargs("label"_a = "continuous", "alpha"_a = 0.5));

  const auto underlying_points = reference_path.compute(reference_path.get_underlying_bases());
  autoware_internal_planning_msgs::msg::PathWithLaneId discrete_path;
  for (unsigned i = 0; i < underlying_points.size(); ++i) {
    const auto & point = underlying_points.at(i);
    autoware_internal_planning_msgs::msg::PathPointWithLaneId point_with_lane_id;
    point_with_lane_id.point.pose = point.point.pose;
    discrete_path.points.push_back(point_with_lane_id);
  }
  const auto discrete_ss = reference_path.base_arange(0.1);
  const auto discrete_interpolated_path = autoware::motion_utils::resamplePath(
    discrete_path, discrete_ss /* arc lengths for this interpolation */);
  const auto discrete_curvature =
    autoware::motion_utils::calcCurvature(discrete_interpolated_path.points);
  axes.plot(Args(discrete_ss, discrete_curvature), Kwargs("label"_a = "discrete", "alpha"_a = 0.5));
  axes.legend();
  plt.show();

  return 0;
}

int main2()
{
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(2, 2);

  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_test_utils")) /
    "test_map/overlap";
  const auto intersection_crossing_map_path = sample_map_dir / "lanelet2_map.osm";

  const auto lanelet_map_ptr =
    lanelet2_utils::load_mgrs_coordinate_map(intersection_crossing_map_path.string());
  const auto [routing_graph, traffic_rules] =
    lanelet2_utils::instantiate_routing_graph_and_traffic_rules(lanelet_map_ptr);

  const std::vector<lanelet::Id> ids = {609, 610, 612, 611, 613};
  const auto lanelet_sequence = ids | ranges::views::transform([&](const auto & id) {
                                  return lanelet_map_ptr->laneletLayer.get(id);
                                }) |
                                ranges::to<std::vector>();
  const auto ego_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(autoware_utils_geometry::create_point(1573.68, 389.857, 100.0))
      .orientation(autoware_utils_geometry::create_quaternion(0.0, 0.0, 0.999997, 0.00250111));

  {
    auto & ax = axes[0];
    const auto reference_path_opt = trajectory::build_reference_path(
      lanelet_sequence, lanelet_sequence.front(), ego_pose, lanelet_map_ptr, routing_graph,
      traffic_rules, 1.0, 10.0, 500, 0);
    if (!reference_path_opt) {
      return 0;
    }
    const auto & reference_path = reference_path_opt.value();
    autoware_internal_planning_msgs::msg::PathWithLaneId path;
    path.points = reference_path.restore();
    autoware::test_utils::plot_autoware_object(path, ax);
    for (const auto & route_lanelet : lanelet_sequence) {
      autoware::test_utils::plot_lanelet2_object(route_lanelet, ax);
    }
    ax.scatter(Args(ego_pose.position.x, ego_pose.position.y), Kwargs("label"_a = "ego position"));
    ax.set_aspect(Args("equal"));
    ax.set_title(Args("forward = 500, backward = 0"));
    ax.legend();
    ax.grid();
  }
  {
    auto & ax = axes[1];
    const auto reference_path_opt = trajectory::build_reference_path(
      lanelet_sequence, lanelet_sequence.front(), ego_pose, lanelet_map_ptr, routing_graph,
      traffic_rules, 1.0, 10.0, 0, 100);
    if (!reference_path_opt) {
      return 0;
    }
    const auto & reference_path = reference_path_opt.value();
    autoware_internal_planning_msgs::msg::PathWithLaneId path;
    path.points = reference_path.restore();
    autoware::test_utils::plot_autoware_object(path, ax);
    for (const auto & route_lanelet : lanelet_sequence) {
      autoware::test_utils::plot_lanelet2_object(route_lanelet, ax);
    }
    ax.scatter(Args(ego_pose.position.x, ego_pose.position.y), Kwargs("label"_a = "ego position"));
    ax.set_aspect(Args("equal"));
    ax.set_title(Args("forward = 0, backward = 100"));
    ax.legend();
    ax.grid();
  }
  {
    auto & ax = axes[2];
    const auto reference_path_opt = trajectory::build_reference_path(
      lanelet_sequence, lanelet_sequence.front(), ego_pose, lanelet_map_ptr, routing_graph,
      traffic_rules, 1.0, 10.0, 50, 10);
    if (!reference_path_opt) {
      return 0;
    }
    const auto & reference_path = reference_path_opt.value();
    autoware_internal_planning_msgs::msg::PathWithLaneId path;
    path.points = reference_path.restore();
    autoware::test_utils::plot_autoware_object(path, ax);
    for (const auto & route_lanelet : lanelet_sequence) {
      autoware::test_utils::plot_lanelet2_object(route_lanelet, ax);
    }
    ax.scatter(Args(ego_pose.position.x, ego_pose.position.y), Kwargs("label"_a = "ego position"));
    ax.set_aspect(Args("equal"));
    ax.set_title(Args("forward = 50, backward = 10"));
    ax.legend();
    ax.grid();
  }
  {
    auto & ax = axes[3];
    const auto reference_path_opt = trajectory::build_reference_path(
      lanelet_sequence, lanelet_sequence.front(), ego_pose, lanelet_map_ptr, routing_graph,
      traffic_rules, 1.0, 10.0, 0, 0);
    if (reference_path_opt) {
      const auto & reference_path = reference_path_opt.value();
      autoware_internal_planning_msgs::msg::PathWithLaneId path;
      path.points = reference_path.restore();
      autoware::test_utils::plot_autoware_object(path, ax);
    }
    for (const auto & route_lanelet : lanelet_sequence) {
      autoware::test_utils::plot_lanelet2_object(route_lanelet, ax);
    }
    ax.scatter(Args(ego_pose.position.x, ego_pose.position.y), Kwargs("label"_a = "ego position"));
    ax.set_aspect(Args("equal"));
    ax.set_title(Args("forward = 0, backward = 0"));
    ax.legend();
    ax.grid();
  }
  plt.show();

  return 0;
}

int main()
{
  pybind11::scoped_interpreter guard{};
  main1();
  main2();
}
