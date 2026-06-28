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

// E6b: a drop-in C++ adapter for pclomp::MultiGridNormalDistributionsTransform that forwards to the
// Rust NDT engine over the C ABI (an opaque AwNdtEngine*). Built only under NDT_USE_RUST; E6c swaps
// the node's NormalDistributionsTransform typedef to this. The node copies the NDT (map-update
// double-buffer), so the adapter is copy-constructible/assignable via the engine clone.

#ifndef AUTOWARE__NDT_SCAN_MATCHER__NDT_RUST_ADAPTER_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__NDT_RUST_ADAPTER_HPP_

#include "autoware_ndt_scan_matcher_rs.h"

#include <Eigen/Core>
#include <autoware/ndt_scan_matcher/ndt_omp/ndt_struct.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ndt_scan_matcher
{

/// Thin owning handle to the Rust NDT engine (the node's `NormalDistributionsTransform`/`NdtType`
/// under `NDT_USE_RUST`). Owns the engine handle (Rule-of-Five: copy = engine clone) and exposes the
/// lifecycle + params + map-management surface the node + `map_update` still call on the typedef
/// (`addTarget`/`removeTarget`/`createVoxelKdtree`/`hasTarget`/`getCurrentMapIDs`/`getParams`/
/// `setParams`/`getMaximumIterations`) plus `raw_handle()`. The compute (align / scoring / covariance /
/// regularization) is driven by the node directly through the engine FFIs on `raw_handle()` — this is
/// **not** a pclomp drop-in (N4c/N4e). The cell-id mapping now lives in the engine (N4d).
class NdtRustAdapter
{
public:
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudTarget = pcl::PointCloud<PointTarget>;
  using PointCloudSourceConstPtr = PointCloudSource::ConstPtr;
  using PointCloudTargetConstPtr = PointCloudTarget::ConstPtr;

  NdtRustAdapter() : handle_(autoware_ndt_scan_matcher_rs_ndt_engine_new(1.0, 6, 0.01)) {}
  ~NdtRustAdapter() { autoware_ndt_scan_matcher_rs_ndt_engine_free(handle_); }

  // The cell-id -> tile mapping now lives in the Rust engine (N4d), so the clone carries it; the
  // adapter holds no id state of its own.
  NdtRustAdapter(const NdtRustAdapter & o)
  : handle_(autoware_ndt_scan_matcher_rs_ndt_engine_clone(o.handle_)), params_(o.params_)
  {
  }
  NdtRustAdapter & operator=(const NdtRustAdapter & o)
  {
    if (this != &o) {
      autoware_ndt_scan_matcher_rs_ndt_engine_free(handle_);
      handle_ = autoware_ndt_scan_matcher_rs_ndt_engine_clone(o.handle_);
      params_ = o.params_;
    }
    return *this;
  }
  NdtRustAdapter(NdtRustAdapter && o) noexcept : handle_(o.handle_), params_(o.params_)
  {
    o.handle_ = nullptr;
  }
  NdtRustAdapter & operator=(NdtRustAdapter && o) noexcept
  {
    if (this != &o) {
      autoware_ndt_scan_matcher_rs_ndt_engine_free(handle_);
      handle_ = o.handle_;
      o.handle_ = nullptr;
      params_ = o.params_;
    }
    return *this;
  }

  void setParams(const pclomp::NdtParams & p)
  {
    params_ = p;
    autoware_ndt_scan_matcher_rs_ndt_engine_set_params(
      handle_, p.trans_epsilon, p.step_size, static_cast<double>(p.resolution), p.max_iterations,
      0.55 /* outlier_ratio (fixed in C++) */, p.num_threads);
  }
  pclomp::NdtParams getParams() const { return params_; }
  int getMaximumIterations() const
  {
    return autoware_ndt_scan_matcher_rs_ndt_engine_max_iterations(handle_);
  }

  // The live Rust engine handle, as a `const` pointer (the engine is Sync and exposes &self-only ops
  // over the C ABI; mutation is interior). The node drives align/scoring/covariance/commit through
  // this. Rust does not retain it past a call.
  const AwNdtEngine * raw_handle() const { return handle_; }

  // Atomically publish `src`'s freshly-built map into this engine (the map-update commit): one store,
  // so a concurrent align observes the complete new map or the old one, never a partially-built one.
  void commitFrom(const NdtRustAdapter & src)
  {
    autoware_ndt_scan_matcher_rs_ndt_engine_commit_from(handle_, src.handle_);
  }

  void addTarget(const PointCloudTargetConstPtr & cloud, const std::string & id)
  {
    const std::vector<float> flat = to_flat(*cloud);
    autoware_ndt_scan_matcher_rs_ndt_engine_add_target_str(
      handle_, flat.data(), cloud->size(), reinterpret_cast<const std::uint8_t *>(id.data()),
      id.size());
  }
  void removeTarget(const std::string & id)
  {
    autoware_ndt_scan_matcher_rs_ndt_engine_remove_target_str(
      handle_, reinterpret_cast<const std::uint8_t *>(id.data()), id.size());
  }
  void createVoxelKdtree() { autoware_ndt_scan_matcher_rs_ndt_engine_create_kdtree(handle_); }
  bool hasTarget() const { return autoware_ndt_scan_matcher_rs_ndt_engine_has_target(handle_); }
  std::vector<std::string> getCurrentMapIDs() const
  {
    // Two-pass: size, then fill (the engine owns the cell-id set; ids come back sorted).
    std::uint32_t count = 0;
    std::uint32_t total = 0;
    autoware_ndt_scan_matcher_rs_ndt_engine_get_current_map_ids(
      handle_, nullptr, 0, nullptr, 0, &count, &total);
    std::vector<std::uint32_t> lengths(count);
    std::vector<std::uint8_t> bytes(total);
    autoware_ndt_scan_matcher_rs_ndt_engine_get_current_map_ids(
      handle_, lengths.data(), count, bytes.data(), total, &count, &total);
    std::vector<std::string> ids;
    ids.reserve(count);
    std::size_t off = 0;
    for (std::uint32_t i = 0; i < count; ++i) {
      ids.emplace_back(reinterpret_cast<const char *>(bytes.data()) + off, lengths[i]);
      off += lengths[i];
    }
    return ids;
  }

  // N4c/N4e: the pclomp-shaped compute methods (align / getResult / calculate* /
  // set[/unset]RegularizationPose) were removed — the node drives align/scoring/covariance/
  // regularization directly through the engine FFIs on `raw_handle()`. This adapter is now a thin
  // engine handle: lifecycle + `raw_handle` + map management + params. (The engine FFIs are
  // differential-tested directly by test_ndt_engine / test_align / test_estimate_covariance_multi.)

private:
  static std::vector<float> to_flat(const PointCloudSource & cloud)
  {
    std::vector<float> f;
    f.reserve(cloud.size() * 3);
    for (const auto & p : cloud) {
      f.push_back(p.x);
      f.push_back(p.y);
      f.push_back(p.z);
    }
    return f;
  }

  AwNdtEngine * handle_;
  pclomp::NdtParams params_{};
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_RUST_ADAPTER_HPP_
