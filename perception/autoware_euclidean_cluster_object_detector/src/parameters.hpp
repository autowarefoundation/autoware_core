// Copyright 2020 TIER IV, Inc.
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

#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

namespace autoware::euclidean_cluster
{
struct EuclideanClusterParams
{
  bool use_height{false};
  int min_cluster_size{1};
  int max_cluster_size{500};
  float tolerance{1.0f};
  float voxel_leaf_size{0.0f};
  int min_points_number_per_voxel{1};
};

}  // namespace autoware::euclidean_cluster

#endif  // PARAMETERS_HPP_
