// Copyright 2024 Autoware Foundation
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

// C ABI exported by the `autoware_ndt_scan_matcher_rs` Rust crate.
// Hand-written for now; switch to cbindgen-generated output once the surface grows.

#ifndef AUTOWARE_NDT_SCAN_MATCHER_RS_H_
#define AUTOWARE_NDT_SCAN_MATCHER_RS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint64_t autoware_ndt_scan_matcher_rs_add(uint64_t left, uint64_t right);

// Rotate the 3x3 position block of a 6x6 row-major pose covariance: out_cov = R * src_cov * R^T.
// src_cov/out_cov point to 36 doubles (row-major 6x6); rot points to 9 doubles (row-major 3x3).
// No-op if any pointer is null.
void autoware_ndt_scan_matcher_rs_rotate_covariance(
  const double * src_cov, const double * rot, double * out_cov);

// Maximum number of consecutive direction inversions over a pose trajectory (zero-copy).
// `poses` points to `num_poses` contiguous geometry_msgs::msg::Pose; only position.{x,y,z} is
// read. The Pose memory layout is asserted on the C++ side (see ndt_scan_matcher_helper_rs.cpp).
// Returns 0 if `poses` is null or `num_poses` is 0.
int32_t autoware_ndt_scan_matcher_rs_count_oscillation(const void * poses, size_t num_poses);

// --- estimate_covariance pure helpers (matrices are row-major) ---

// Temperature-scaled softmax: out[0..n] = softmax(scores[0..n] / temperature). No-op if null.
void autoware_ndt_scan_matcher_rs_calc_weight_vec(
  const double * scores, size_t n, double temperature, double * out);

// Weighted mean and covariance of n 2D points. poses2d = [x0,y0, x1,y1, ...] (2*n doubles),
// weights = n doubles; mean_out = 2 doubles, cov_out = 4 doubles (row-major 2x2). No-op if null.
void autoware_ndt_scan_matcher_rs_calculate_weighted_mean_and_cov(
  const double * poses2d, const double * weights, size_t n, double * mean_out, double * cov_out);

// Laplace approximation: cov_out (row-major 2x2) = -inverse of the top-left 2x2 of the row-major
// 6x6 hessian (36 doubles). cov_out filled with NaN if singular. No-op if null.
void autoware_ndt_scan_matcher_rs_laplace_xy_covariance(const double * hessian, double * cov_out);

// Rotate a 2x2 covariance (row-major) by the row-major 2x2 yaw block `rot`. *_to_base_link computes
// R^T*C*R; *_to_map computes R*C*R^T. out = 4 doubles (row-major 2x2). No-op if null.
void autoware_ndt_scan_matcher_rs_rotate_covariance_to_base_link(
  const double * cov, const double * rot, double * out);
void autoware_ndt_scan_matcher_rs_rotate_covariance_to_map(
  const double * cov, const double * rot, double * out);

// Clamp the base_link-frame diagonal to (fixed_cov00, fixed_cov11), then rotate back to map.
// cov/rot = row-major 2x2 (4 doubles each); out = row-major 2x2. No-op if null.
void autoware_ndt_scan_matcher_rs_adjust_diagonal_covariance(
  const double * cov, const double * rot, double fixed_cov00, double fixed_cov11, double * out);

// --- voxel-grid covariance (NDT leaves); opaque handle owned by Rust ---

// Opaque single voxel grid. Build with _voxel_grid_build, free with _voxel_grid_free.
typedef struct AwNdtVoxelGrid AwNdtVoxelGrid;

// Build a voxel grid from `n` xyz f32 triples (3*n floats at `points`). `leaf_size` = 3 doubles.
// `min_points` (<=0 -> default 6) and `eig_mult` (e.g. 0.01) match MultiVoxelGridCovariance.
// Returns an owned handle (free with _voxel_grid_free), or null if `points`/`leaf_size` is null.
AwNdtVoxelGrid * autoware_ndt_scan_matcher_rs_voxel_grid_build(
  const float * points, size_t n, const double * leaf_size, int32_t min_points, double eig_mult);

// Look up the leaf whose voxel contains `point` (3 floats). On hit, writes mean (3 doubles) and
// row-major inverse covariance (9 doubles) and returns true; otherwise returns false.
bool autoware_ndt_scan_matcher_rs_voxel_grid_leaf_at(
  const AwNdtVoxelGrid * grid, const float * point, double * mean_out, double * icov_out);

// Free a grid handle returned by _voxel_grid_build (no-op if null).
void autoware_ndt_scan_matcher_rs_voxel_grid_free(AwNdtVoxelGrid * grid);

// --- multi-grid map + kd-tree radius search (MultiVoxelGridCovariance equivalent) ---

// Opaque id-keyed map of voxel grids + a kd-tree over centroids. Integer grid ids (the C++ side
// maps its string ids to integers). Build with _map_new, free with _map_free.
typedef struct AwNdtVoxelGridMap AwNdtVoxelGridMap;

AwNdtVoxelGridMap * autoware_ndt_scan_matcher_rs_voxel_grid_map_new(
  const double * leaf_size, int32_t min_points, double eig_mult);

// Build a grid from `3*n` floats at `points` and register it under `id` (replaces existing).
void autoware_ndt_scan_matcher_rs_voxel_grid_map_add_target(
  AwNdtVoxelGridMap * map, const float * points, size_t n, uint64_t id);

void autoware_ndt_scan_matcher_rs_voxel_grid_map_remove_target(AwNdtVoxelGridMap * map, uint64_t id);

// Build the kd-tree over the current grids' centroids (call after add/remove, before searching).
void autoware_ndt_scan_matcher_rs_voxel_grid_map_create_kdtree(AwNdtVoxelGridMap * map);

// Leaf (flat) indices whose centroid is within `radius` of `point` (3 floats). `max_nn`=0 means
// unlimited. Writes up to `cap` indices to `out_idx`; returns the total number found.
uint32_t autoware_ndt_scan_matcher_rs_voxel_grid_map_radius_search(
  const AwNdtVoxelGridMap * map, const float * point, double radius, uint32_t max_nn,
  uint32_t * out_idx, uint32_t cap);

// Fetch a leaf's mean (3 doubles) and row-major inverse covariance (9 doubles) by flat index.
bool autoware_ndt_scan_matcher_rs_voxel_grid_map_leaf(
  const AwNdtVoxelGridMap * map, uint32_t idx, double * mean_out, double * icov_out);

void autoware_ndt_scan_matcher_rs_voxel_grid_map_free(AwNdtVoxelGridMap * map);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AUTOWARE_NDT_SCAN_MATCHER_RS_H_
