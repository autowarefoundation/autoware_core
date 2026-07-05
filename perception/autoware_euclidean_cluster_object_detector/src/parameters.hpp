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

/**
 * @brief Struct to hold params for Euclidean Cluster Object Detector node
 *
 * @param use_height Whether to use height information in clustering
 * @param min_cluster_size Min number of points to form a cluster
 * @param max_cluster_size Max number of points to form a cluster
 * @param tolerance Distance tolerance for clustering
 * @param voxel_leaf_size Leaf size for voxel grid downsampling (if > 0.0, enables downsampling)
 * @param min_points_number_per_voxel Min number of points per voxel to keep it during downsampling
 */
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
