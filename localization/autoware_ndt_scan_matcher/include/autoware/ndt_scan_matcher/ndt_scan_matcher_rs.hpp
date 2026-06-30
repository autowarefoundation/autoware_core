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

// Roadmap foundation (plan/ndt_in_rust_next.md → Phase 0/1): the opaque Rust node handle
// (`AwNdtScanMatcher`) + its RAII C++ owner, plus the param conversion that crosses the FFI once at
// construction. Built only under NDT_USE_RUST. The handle is inert this slice (held as a node member,
// exercised by tests); later phases move node state into it and thin the callbacks to forwarders.

#ifndef AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_RS_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_RS_HPP_

#ifdef NDT_USE_RUST

#include "autoware/ndt_scan_matcher/hyper_parameters.hpp"

#include "autoware_ndt_scan_matcher_rs.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace autoware::ndt_scan_matcher
{

/// Build the FFI param struct from the node's `HyperParameters`. The covariance offset-model vectors
/// are borrowed by pointer for the duration of the `_new` call only (Rust copies them), so the
/// referenced `HyperParameters` must outlive that call. The fixed `min_points` / `eig_mult` /
/// `outlier_ratio` mirror the values `NdtRustAdapter` configures the engine with.
inline AwNdtParams make_aw_ndt_params(const HyperParameters & p)
{
  AwNdtParams out{};
  out.resolution = static_cast<double>(p.ndt.resolution);
  out.min_points = 6;
  out.eig_mult = 0.01;
  out.trans_epsilon = p.ndt.trans_epsilon;
  out.step_size = p.ndt.step_size;
  out.max_iterations = p.ndt.max_iterations;
  out.outlier_ratio = 0.55;
  out.num_threads = p.ndt.num_threads;
  out.converged_param_type = static_cast<int32_t>(p.score_estimation.converged_param_type);
  out.converged_param_transform_probability =
    p.score_estimation.converged_param_transform_probability;
  out.converged_param_nearest_voxel_transformation_likelihood =
    p.score_estimation.converged_param_nearest_voxel_transformation_likelihood;
  out.covariance_estimation_type =
    static_cast<int32_t>(p.covariance.covariance_estimation.covariance_estimation_type);
  out.covariance_scale_factor = p.covariance.covariance_estimation.scale_factor;
  out.covariance_temperature = p.covariance.covariance_estimation.temperature;
  std::copy(
    p.covariance.output_pose_covariance.begin(), p.covariance.output_pose_covariance.end(),
    out.output_pose_covariance);
  out.initial_pose_offset_model_x =
    p.covariance.covariance_estimation.initial_pose_offset_model_x.data();
  out.initial_pose_offset_model_x_len =
    p.covariance.covariance_estimation.initial_pose_offset_model_x.size();
  out.initial_pose_offset_model_y =
    p.covariance.covariance_estimation.initial_pose_offset_model_y.data();
  out.initial_pose_offset_model_y_len =
    p.covariance.covariance_estimation.initial_pose_offset_model_y.size();
  return out;
}

/// RAII owner of the opaque Rust node handle (`AwNdtScanMatcher *`). Constructs via `_new` (throwing
/// on a null result), frees via `_free`. Non-copyable, non-movable — a single owner per node.
class NDTScanMatcherRS
{
public:
  explicit NDTScanMatcherRS(const AwNdtParams & params)
  : handle_(autoware_ndt_scan_matcher_rs_new(&params))
  {
    if (handle_ == nullptr) {
      throw std::runtime_error("failed to create Rust NdtScanMatcherRs");
    }
  }

  ~NDTScanMatcherRS() { autoware_ndt_scan_matcher_rs_free(handle_); }

  NDTScanMatcherRS(const NDTScanMatcherRS &) = delete;
  NDTScanMatcherRS & operator=(const NDTScanMatcherRS &) = delete;
  NDTScanMatcherRS(NDTScanMatcherRS &&) = delete;
  NDTScanMatcherRS & operator=(NDTScanMatcherRS &&) = delete;

  AwNdtScanMatcher * raw() { return handle_; }

private:
  AwNdtScanMatcher * handle_{nullptr};
};

}  // namespace autoware::ndt_scan_matcher

#endif  // NDT_USE_RUST

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_RS_HPP_
