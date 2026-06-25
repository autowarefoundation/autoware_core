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

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AUTOWARE_NDT_SCAN_MATCHER_RS_H_
