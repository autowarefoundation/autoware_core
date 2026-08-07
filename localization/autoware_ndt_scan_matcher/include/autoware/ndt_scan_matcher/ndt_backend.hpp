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

// Legacy C++ NDT backend aliases. The Rust-enabled production node owns its live engine in
// NdtScanMatcherRs and borrows it through the Rust node handle; this header remains for the
// NDT_USE_RUST=OFF path and for legacy helper declarations that still name NdtBackend.

#ifndef AUTOWARE__NDT_SCAN_MATCHER__NDT_BACKEND_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__NDT_BACKEND_HPP_

#include "guarded.hpp"
#include "ndt_omp/multigrid_ndt_omp.h"

#include <pcl/point_types.h>

#include <memory>

namespace autoware::ndt_scan_matcher
{
using NdtBackend = pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;

// Legacy C++ engine holder. The Rust-enabled node no longer owns this holder; its live engine is
// inside NdtScanMatcherRs.
using NdtBackendPtr = std::shared_ptr<NdtBackend>;
using EngineHolder = Guarded<NdtBackendPtr>;
}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_BACKEND_HPP_
