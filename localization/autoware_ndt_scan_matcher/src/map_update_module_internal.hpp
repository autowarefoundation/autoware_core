// Copyright 2026 Autoware Foundation
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

#ifndef MAP_UPDATE_MODULE_INTERNAL_HPP_
#define MAP_UPDATE_MODULE_INTERNAL_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher::detail
{

template <typename CloudPtrT>
struct MapUpdateDiffTemplate
{
  struct Addition
  {
    CloudPtrT cloud;
    std::string id;
  };

  std::vector<Addition> additions;
  std::vector<std::string> removals;
};

struct MapUpdateResult
{
  std::size_t added{0};
  std::size_t removed{0};
  bool updated{false};
  double execution_time_ms{0.0};
};

using MapUpdateDiff = MapUpdateDiffTemplate<pcl::PointCloud<pcl::PointXYZ>::Ptr>;

template <typename NdtT, typename DiffT>
MapUpdateResult apply_map_update(NdtT & ndt, const DiffT & diff)
{
  const auto start = std::chrono::steady_clock::now();

  std::size_t added = 0;
  for (const auto & addition : diff.additions) {
    ndt.addTarget(addition.cloud, addition.id);
    ++added;
  }

  std::size_t removed = 0;
  for (const auto & id : diff.removals) {
    ndt.removeTarget(id);
    ++removed;
  }

  const bool updated = (added + removed) > 0;
  if (updated) {
    ndt.createVoxelKdtree();
  }

  const auto end = std::chrono::steady_clock::now();
  const double execution_ms =
    std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start).count();

  return MapUpdateResult{added, removed, updated, execution_ms};
}

}  // namespace autoware::ndt_scan_matcher::detail

#endif  // MAP_UPDATE_MODULE_INTERNAL_HPP_
