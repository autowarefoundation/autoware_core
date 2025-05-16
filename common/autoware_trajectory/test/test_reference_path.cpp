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

#include "autoware/trajectory/reference_path.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/topology.hpp>

#include <gtest/gtest.h>
#include <lanelet2_io/Io.h>

#include <filesystem>
#include <string>

namespace fs = std::filesystem;

namespace autoware::experimental
{

TEST(reference_path, NormalMap)
{
  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
    "sample_map/straight_waypoint";
  const auto map_path = sample_map_dir / "lanelet2_map.osm";
  const auto lanelet_map_ptr = lanelet2_utils::load_mgrs_coordinate_map(map_path.string());
  const auto routing_graph_ptr =
    autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
      lanelet_map_ptr)
      .first;
}
}  // namespace autoware::experimental
