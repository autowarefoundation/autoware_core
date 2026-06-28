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

// The single place where the persistent NDT engine type is selected. Under NDT_USE_RUST it is the
// Rust-backed NdtRustAdapter (a thin engine handle over the C ABI); otherwise the original pclomp
// engine. The node + map_update alias their own NormalDistributionsTransform / NdtType to NdtBackend
// so this `#ifdef` is the only engine-type switch — the per-callback compute dispatch lives in the
// .cpp callbacks (see plan/ndt_in_rust.md, 案B).

#ifndef AUTOWARE__NDT_SCAN_MATCHER__NDT_BACKEND_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__NDT_BACKEND_HPP_

#include "ndt_omp/multigrid_ndt_omp.h"
#ifdef NDT_USE_RUST
#include "ndt_rust_adapter.hpp"
#endif

#include <pcl/point_types.h>

namespace autoware::ndt_scan_matcher
{
#ifdef NDT_USE_RUST
using NdtBackend = NdtRustAdapter;
#else
using NdtBackend = pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
#endif
}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_BACKEND_HPP_
