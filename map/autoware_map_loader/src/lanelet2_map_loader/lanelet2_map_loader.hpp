// Copyright 2021 TierIV
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

/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 *
 */

#ifndef LANELET2_MAP_LOADER__LANELET2_MAP_LOADER_HPP_
#define LANELET2_MAP_LOADER__LANELET2_MAP_LOADER_HPP_

#include "utils/lanelet2_map_cell_metadata.hpp"

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::map_loader
{

class MapLoadException : public std::runtime_error
{
public:
  explicit MapLoadException(const std::vector<std::string> & errs)
  : std::runtime_error("Map load failed"), errors(errs)
  {
  }
  std::vector<std::string> errors;
};

struct Lanelet2MapLoaderParameters
{
  std::string lanelet2_map_path;
  std::string lanelet2_map_metadata_path;
  double center_line_resolution;
  bool use_waypoints;
  bool allow_unsupported_version;
};

struct Lanelet2MapLoaderResult
{
  autoware_map_msgs::msg::LaneletMapBin map_bin_msg;
  std::map<std::string, Lanelet2FileMetaData> cell_metadata_dict;
  std::vector<std::string> warnings;
  std::vector<std::string> infos;
};

class Lanelet2MapLoaderCore
{
public:
  explicit Lanelet2MapLoaderCore(const Lanelet2MapLoaderParameters & params);

  [[nodiscard]] Lanelet2MapLoaderResult execute(
    const autoware_map_msgs::msg::MapProjectorInfo & projector_info) const;

  lanelet::LaneletMapPtr load_map(
    const std::string & lanelet2_filename,
    const autoware_map_msgs::msg::MapProjectorInfo & projector_info,
    std::vector<std::string> & warnings) const;

  [[nodiscard]] autoware_map_msgs::msg::LaneletMapBin create_map_bin_msg(
    const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename) const;

private:
  Lanelet2MapLoaderParameters params_;
};

}  // namespace autoware::map_loader
#endif  // LANELET2_MAP_LOADER__LANELET2_MAP_LOADER_HPP_
