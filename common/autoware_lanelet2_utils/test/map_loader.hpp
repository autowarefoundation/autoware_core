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

#ifndef MAP_LOADER_HPP_
#define MAP_LOADER_HPP_

#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>

#include <string>
#include <utility>

template <typename LaneletMapPtrType>
LaneletMapPtrType load_mgrs_coordinate_map(const std::string & path)
{
  lanelet::ErrorMessages errors{};
  lanelet::projection::MGRSProjector projector;
  auto lanelet_map_ptr = lanelet::load(path, projector, &errors);

  if constexpr (std::is_const_v<std::remove_pointer_t<typename LaneletMapPtrType::element_type>>) {
    return lanelet::LaneletMapConstPtr{std::move(lanelet_map_ptr)};
  } else {
    return lanelet::LaneletMapPtr{std::move(lanelet_map_ptr)};
  }
}

#endif  // MAP_LOADER_HPP_
