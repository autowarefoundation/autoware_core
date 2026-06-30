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

// Roadmap foundation (plan/ndt_in_rust_next.md → Phase 0/1): the C++ RAII owner of the opaque Rust
// node handle. Builds an AwNdtParams directly (no live node needed) and exercises the construct/throw
// lifecycle across the FFI boundary.

#include "autoware/ndt_scan_matcher/ndt_scan_matcher_rs.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{
AwNdtParams make_minimal_params(const std::vector<double> & ox, const std::vector<double> & oy)
{
  AwNdtParams p{};
  p.resolution = 2.0;
  p.min_points = 6;
  p.eig_mult = 0.01;
  p.trans_epsilon = 0.01;
  p.step_size = 0.1;
  p.max_iterations = 30;
  p.outlier_ratio = 0.55;
  p.num_threads = 4;
  p.converged_param_type = 0;
  p.converged_param_transform_probability = 3.0;
  p.converged_param_nearest_voxel_transformation_likelihood = 2.3;
  p.covariance_estimation_type = 0;
  p.covariance_scale_factor = 1.0;
  p.covariance_temperature = 0.05;
  p.output_pose_covariance[0] = 1.0;
  p.initial_pose_offset_model_x = ox.data();
  p.initial_pose_offset_model_x_len = ox.size();
  p.initial_pose_offset_model_y = oy.data();
  p.initial_pose_offset_model_y_len = oy.size();
  return p;
}
}  // namespace

// A valid param set constructs a non-null handle and frees cleanly on scope exit.
TEST(NdtScanMatcherRsHandle, ConstructAndDestroy)
{
  const std::vector<double> ox{0.0, 0.5, -0.5};
  const std::vector<double> oy{0.0, 0.5, 0.5};
  const AwNdtParams params = make_minimal_params(ox, oy);
  NDTScanMatcherRS rs(params);
  EXPECT_NE(rs.raw(), nullptr);
}

// Mismatched offset-model lengths are rejected Rust-side (_new returns null), so the RAII wrapper
// throws rather than holding a null handle.
TEST(NdtScanMatcherRsHandle, MismatchedOffsetLengthsThrows)
{
  const std::vector<double> ox{0.0, 0.5};
  const std::vector<double> oy{0.0};
  const AwNdtParams params = make_minimal_params(ox, oy);
  EXPECT_THROW({ NDTScanMatcherRS rs(params); }, std::runtime_error);
}

}  // namespace autoware::ndt_scan_matcher
