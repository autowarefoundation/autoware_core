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
#include "autoware/trajectory/utils/reference_path.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/pyplot/pyplot.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>
#include <autoware_test_utils/visualization.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/all.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace fs = std::filesystem;
using namespace autoware::experimental;  // NOLINT

[[maybe_unused]] int main1()
{
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 1);

  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
    "sample_map/vm_01_10-12/dense_centerline";
  const std::vector<lanelet::Id> ids = {140, 137, 136, 138, 139, 135};
  const auto ego_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(autoware_utils_geometry::create_point(108, 99, 100.0))
      .orientation(autoware_utils_geometry::create_quaternion(0.0, 0.0, 0.999997, 0.00250111));
  const auto map_path = sample_map_dir / "lanelet2_map.osm";

  const auto lanelet_map_ptr = lanelet2_utils::load_mgrs_coordinate_map(map_path.string());
  const auto [routing_graph, traffic_rules] =
    lanelet2_utils::instantiate_routing_graph_and_traffic_rules(lanelet_map_ptr);
  const auto current_lanelet = lanelet_map_ptr->laneletLayer.get(138);

  const auto lanelet_sequence = ids | ranges::views::transform([&](const auto & id) {
                                  return lanelet_map_ptr->laneletLayer.get(id);
                                }) |
                                ranges::to<std::vector>();
  auto path_plot_config = autoware::test_utils::PathWithLaneIdConfig::defaults();
  path_plot_config.linewidth = 2.0;
  path_plot_config.color = "orange";
  path_plot_config.quiver_size = 2.0;

  auto lane_plot_config = autoware::test_utils::LaneConfig::defaults();
  lane_plot_config.line_config = autoware::test_utils::LineConfig::defaults();

  {
    auto & ax = axes[0];
    const double forward_length = 40;
    const double backward_length = 0.0;
    const auto reference_path_opt = trajectory::build_reference_path(
      lanelet_sequence, current_lanelet, ego_pose, lanelet_map_ptr, routing_graph, traffic_rules,
      forward_length, backward_length);
    if (reference_path_opt) {
      const auto & reference_path = reference_path_opt.value();
      autoware_internal_planning_msgs::msg::PathWithLaneId path;
      path.points = reference_path.restore();
      autoware::test_utils::plot_autoware_object(path, ax, path_plot_config);
      ax.set_title(Args(
        "forward = 40, backward = 0 (actual length = " + std::to_string(reference_path.length()) +
        ")"));
    }
    for (const auto & route_lanelet : lanelet_sequence) {
      autoware::test_utils::plot_lanelet2_object(route_lanelet, ax, lane_plot_config);
    }
    ax.scatter(Args(ego_pose.position.x, ego_pose.position.y), Kwargs("label"_a = "ego position"));
    ax.set_aspect(Args("equal"));
    ax.legend();
    ax.grid();
  }
  plt.show();

  return 0;
}

[[maybe_unused]] int main2()
{
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 1);

  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_test_utils")) /
    "test_map/overlap";
  const std::vector<lanelet::Id> ids = {609, 610, 612, 611, 613};
  const auto ego_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(autoware_utils_geometry::create_point(1573.68, 389.857, 100.0))
      .orientation(autoware_utils_geometry::create_quaternion(0.0, 0.0, 0.999997, 0.00250111));
  const auto map_path = sample_map_dir / "lanelet2_map.osm";

  const auto lanelet_map_ptr = lanelet2_utils::load_mgrs_coordinate_map(map_path.string());
  const auto [routing_graph, traffic_rules] =
    lanelet2_utils::instantiate_routing_graph_and_traffic_rules(lanelet_map_ptr);
  const auto current_lanelet = lanelet_map_ptr->laneletLayer.get(609);

  const auto lanelet_sequence = ids | ranges::views::transform([&](const auto & id) {
                                  return lanelet_map_ptr->laneletLayer.get(id);
                                }) |
                                ranges::to<std::vector>();
  auto path_plot_config = autoware::test_utils::PathWithLaneIdConfig::defaults();
  path_plot_config.linewidth = 2.0;
  path_plot_config.color = "orange";
  path_plot_config.quiver_size = 2.0;

  auto lane_plot_config = autoware::test_utils::LaneConfig::defaults();
  lane_plot_config.line_config = autoware::test_utils::LineConfig::defaults();

  {
    const double forward_length = 50;
    const double backward_length = 10.0;
    auto & ax = axes[0];
    const auto reference_path_opt = trajectory::build_reference_path(
      lanelet_sequence, current_lanelet, ego_pose, lanelet_map_ptr, routing_graph, traffic_rules,
      forward_length, backward_length);
    if (reference_path_opt) {
      const auto & reference_path = reference_path_opt.value();
      autoware_internal_planning_msgs::msg::PathWithLaneId path;
      path.points = reference_path.restore();
      autoware::test_utils::plot_autoware_object(path, ax, path_plot_config);
      ax.set_title(Args(
        "forward = 50, backward = 10 (actual length = " + std::to_string(reference_path.length()) +
        ")"));
    }
    for (const auto & route_lanelet : lanelet_sequence) {
      autoware::test_utils::plot_lanelet2_object(route_lanelet, ax, lane_plot_config);
    }
    ax.scatter(Args(ego_pose.position.x, ego_pose.position.y), Kwargs("label"_a = "ego position"));
    ax.set_aspect(Args("equal"));
    ax.legend();
    ax.grid();
  }
  plt.show();

  return 0;
}

namespace autoware::test_utils
{

struct TestCaseFormat1
{
  std::string map_abs_path;
  std::unordered_map<std::string, geometry_msgs::msg::Pose> manual_poses;
};

inline TestCaseFormat1 load_test_case_format1(const YAML::Node & node)
{
  const auto map_rel_path = node["map_rel_path"].as<std::string>();
  const auto map_abs_path =
    std::filesystem::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
    "sample_map" / map_rel_path;
  int cnt = 0;
  std::unordered_map<std::string, geometry_msgs::msg::Pose> manual_poses;
  for (const auto & pose : node["manual_poses"]) {
    const auto name = "P" + std::to_string(cnt);
    manual_poses[name] = autoware::test_utils::parse<geometry_msgs::msg::Pose>(pose);
    cnt++;
  }
  return {map_abs_path.string(), manual_poses};
}

inline TestCaseFormat1 load_test_case(const std::string & test_case_file_path)
{
  try {
    YAML::Node node = YAML::LoadFile(test_case_file_path);
    const auto version = node["version"].as<int>();
    if (version == 1) {
      return load_test_case_format1(node);
    }
    throw std::invalid_argument("unsupported version: " + std::to_string(version));
  } catch (std::exception & e) {
    std::cerr << "failed to load:  " << e.what() << std::endl;
    throw e;
  }
}

}  // namespace autoware::test_utils

[[maybe_unused]] int main3()
{
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 1);

  const auto test_case_data = autoware::test_utils::load_test_case(
    fs::path(ament_index_cpp::get_package_share_directory("autoware_trajectory")) /
    "test_data/test_reference_path_valid_02.yaml");
  const std::vector<lanelet::Id> ids = {60, 57, 56, 58, 59, 55};
  const auto ego_pose = test_case_data.manual_poses.at("P0");

  const auto lanelet_map_ptr =
    lanelet2_utils::load_mgrs_coordinate_map(test_case_data.map_abs_path);
  const auto [routing_graph, traffic_rules] =
    lanelet2_utils::instantiate_routing_graph_and_traffic_rules(lanelet_map_ptr);
  const auto current_lanelet = lanelet_map_ptr->laneletLayer.get(60);

  const auto lanelet_sequence = ids | ranges::views::transform([&](const auto & id) {
                                  return lanelet_map_ptr->laneletLayer.get(id);
                                }) |
                                ranges::to<std::vector>();
  auto path_plot_config = autoware::test_utils::PathWithLaneIdConfig::defaults();
  path_plot_config.linewidth = 2.0;
  path_plot_config.color = "orange";
  path_plot_config.quiver_size = 2.0;

  auto lane_plot_config = autoware::test_utils::LaneConfig::defaults();
  lane_plot_config.line_config = autoware::test_utils::LineConfig::defaults();

  {
    static constexpr auto inf = std::numeric_limits<double>::infinity();
    const double forward_length = inf;
    const double backward_length = inf;
    auto & ax = axes[0];
    const auto reference_path_opt = trajectory::build_reference_path(
      lanelet_sequence, current_lanelet, ego_pose, lanelet_map_ptr, routing_graph, traffic_rules,
      forward_length, backward_length);
    if (reference_path_opt) {
      const auto & reference_path = reference_path_opt.value();
      autoware_internal_planning_msgs::msg::PathWithLaneId path;
      path.points = reference_path.restore();
      autoware::test_utils::plot_autoware_object(path, ax, path_plot_config);
      ax.set_title(Args(
        "forward = 50, backward = 10 (actual length = " + std::to_string(reference_path.length()) +
        ")"));
    }
    for (const auto & route_lanelet : lanelet_sequence) {
      autoware::test_utils::plot_lanelet2_object(route_lanelet, ax, lane_plot_config);
    }
    ax.scatter(Args(ego_pose.position.x, ego_pose.position.y), Kwargs("label"_a = "ego position"));
    ax.set_aspect(Args("equal"));
    ax.legend();
    ax.grid();
  }
  plt.show();

  return 0;
}

[[maybe_unused]] int main4()
{
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 1);

  const auto test_case_data = autoware::test_utils::load_test_case(
    fs::path(ament_index_cpp::get_package_share_directory("autoware_trajectory")) /
    "test_data/test_reference_path_invalid_01.yaml");
  const std::vector<lanelet::Id> ids = {60, 57, 56, 58, 59, 55};
  const auto ego_pose = test_case_data.manual_poses.at("P0");

  const auto lanelet_map_ptr =
    lanelet2_utils::load_mgrs_coordinate_map(test_case_data.map_abs_path);
  const auto [routing_graph, traffic_rules] =
    lanelet2_utils::instantiate_routing_graph_and_traffic_rules(lanelet_map_ptr);
  const auto current_lanelet = lanelet_map_ptr->laneletLayer.get(60);

  const auto lanelet_sequence = ids | ranges::views::transform([&](const auto & id) {
                                  return lanelet_map_ptr->laneletLayer.get(id);
                                }) |
                                ranges::to<std::vector>();
  auto path_plot_config = autoware::test_utils::PathWithLaneIdConfig::defaults();
  path_plot_config.linewidth = 2.0;
  path_plot_config.color = "orange";
  path_plot_config.quiver_size = 2.0;
  path_plot_config.lane_id = true;

  auto lane_plot_config = autoware::test_utils::LaneConfig::defaults();
  lane_plot_config.line_config = autoware::test_utils::LineConfig::defaults();

  {
    static constexpr auto inf = std::numeric_limits<double>::infinity();
    const double forward_length = inf;
    const double backward_length = inf;
    auto & ax = axes[0];
    const auto reference_path_opt = trajectory::build_reference_path(
      lanelet_sequence, current_lanelet, ego_pose, lanelet_map_ptr, routing_graph, traffic_rules,
      forward_length, backward_length);
    if (reference_path_opt) {
      const auto & reference_path = reference_path_opt.value();
      autoware_internal_planning_msgs::msg::PathWithLaneId path;
      path.points = reference_path.restore();
      autoware::test_utils::plot_autoware_object(path, ax, path_plot_config);
      ax.set_title(Args(
        "forward = 50, backward = 10 (actual length = " + std::to_string(reference_path.length()) +
        ")"));
    }
    for (const auto & route_lanelet : lanelet_sequence) {
      autoware::test_utils::plot_lanelet2_object(route_lanelet, ax, lane_plot_config);
    }
    ax.scatter(Args(ego_pose.position.x, ego_pose.position.y), Kwargs("label"_a = "ego position"));
    ax.set_aspect(Args("equal"));
    ax.legend();
    ax.grid();
  }
  plt.show();

  return 0;
}

int main()
{
  pybind11::scoped_interpreter guard{};
  // main1();
  // main2();
  // main3();
  main4();
}
