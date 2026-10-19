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

#ifndef AUTOWARE__PATH_GENERATOR__PATH_GENERATOR_HPP_
#define AUTOWARE__PATH_GENERATOR__PATH_GENERATOR_HPP_

#include <autoware/lanelet2_utils/route_manager.hpp>
#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware_path_generator/path_generator_parameters.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <optional>
#include <string>

namespace autoware::path_generator
{
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_vehicle_msgs::msg::HazardLightsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using nav_msgs::msg::Odometry;
using ::path_generator::Params;
using Trajectory = autoware::experimental::trajectory::Trajectory<PathPointWithLaneId>;

struct PathGeneratorConfig
{
  double vehicle_length_m{0.0};
  double max_longitudinal_offset_m{0.0};
};

struct PathGeneratorInput
{
  LaneletMapBin::ConstSharedPtr map_ptr{nullptr};
  LaneletRoute::ConstSharedPtr route_ptr{nullptr};
  Odometry odometry;
  Params params;
};

struct PathGeneratorResult
{
  PathWithLaneId path;
  TurnIndicatorsCommand turn_indicators_cmd;
  HazardLightsCommand hazard_lights_cmd;
};

class PathGeneratorCore
{
public:
  PathGeneratorCore() = default;
  explicit PathGeneratorCore(const PathGeneratorConfig & config);

  tl::expected<PathGeneratorResult, std::string> generate(const PathGeneratorInput & input);

  void set_config(const PathGeneratorConfig & config);
  const PathGeneratorConfig & get_config() const;

private:
  tl::expected<PathWithLaneId, std::string> generate_path(
    const geometry_msgs::msg::Pose & current_pose, const LaneletRoute & route,
    const Params & params,
    const std::optional<builtin_interfaces::msg::Time> & stamp = std::nullopt);

  tl::expected<PathWithLaneId, std::string> generate_path(
    const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::msg::Pose & current_pose,
    double s_ego, double s_start, double s_end, bool connect_to_goal, const LaneletRoute & route,
    const Params & params,
    const std::optional<builtin_interfaces::msg::Time> & stamp = std::nullopt) const;

  PathGeneratorConfig config_;
  LaneletMapBin::ConstSharedPtr prev_map_ptr_{nullptr};
  LaneletRoute::ConstSharedPtr prev_route_ptr_{nullptr};
  std::optional<experimental::lanelet2_utils::RouteManager> route_manager_{std::nullopt};
};

namespace core
{
using PathGenerator = PathGeneratorCore;
}

}  // namespace autoware::path_generator

#endif  // AUTOWARE__PATH_GENERATOR__PATH_GENERATOR_HPP_
