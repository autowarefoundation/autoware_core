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
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ndt_scan_matcher
{

/// Drop-in replacement for `pclomp::MultiGridNormalDistributionsTransform<PointXYZ, PointXYZ>` that
/// forwards to the Rust engine handle. Owns the handle (Rule-of-Five: copy = engine clone) and the
/// string `cell_id` -> `u64` mapping the node-facing API needs.
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

  NdtRustAdapter(const NdtRustAdapter & o)
  : handle_(autoware_ndt_scan_matcher_rs_ndt_engine_clone(o.handle_)),
    params_(o.params_),
    id_map_(o.id_map_),
    next_id_(o.next_id_)
  {
  }
  NdtRustAdapter & operator=(const NdtRustAdapter & o)
  {
    if (this != &o) {
      autoware_ndt_scan_matcher_rs_ndt_engine_free(handle_);
      handle_ = autoware_ndt_scan_matcher_rs_ndt_engine_clone(o.handle_);
      params_ = o.params_;
      id_map_ = o.id_map_;
      next_id_ = o.next_id_;
    }
    return *this;
  }
  NdtRustAdapter(NdtRustAdapter && o) noexcept
  : handle_(o.handle_), params_(o.params_), id_map_(std::move(o.id_map_)), next_id_(o.next_id_)
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
      id_map_ = std::move(o.id_map_);
      next_id_ = o.next_id_;
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

  void addTarget(const PointCloudTargetConstPtr & cloud, const std::string & id)
  {
    const std::vector<float> flat = to_flat(*cloud);
    autoware_ndt_scan_matcher_rs_ndt_engine_add_target(handle_, flat.data(), cloud->size(),
                                                        id_for(id));
  }
  void removeTarget(const std::string & id)
  {
    const auto it = id_map_.find(id);
    if (it == id_map_.end()) {
      return;
    }
    autoware_ndt_scan_matcher_rs_ndt_engine_remove_target(handle_, it->second);
    id_map_.erase(it);
  }
  void createVoxelKdtree() { autoware_ndt_scan_matcher_rs_ndt_engine_create_kdtree(handle_); }
  bool hasTarget() const { return autoware_ndt_scan_matcher_rs_ndt_engine_has_target(handle_); }
  std::vector<std::string> getCurrentMapIDs() const
  {
    std::vector<std::string> ids;
    ids.reserve(id_map_.size());
    for (const auto & kv : id_map_) {
      ids.push_back(kv.first);
    }
    return ids;
  }

  void align(
    PointCloudSource & output, const Eigen::Matrix4f & guess,
    const PointCloudSourceConstPtr & source)
  {
    const std::vector<float> src = to_flat(*source);
    std::array<float, 16> g{};
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        g[(r * 4) + c] = guess(r, c);
      }
    }
    autoware_ndt_scan_matcher_rs_ndt_engine_align(handle_, g.data(), src.data(), source->size());
    // C++ align fills `output` with the source transformed by the final pose.
    pcl::transformPointCloud(*source, output, getResult().pose);
  }

  pclomp::NdtResult getResult() const
  {
    pclomp::NdtResult r;
    std::array<float, 16> pose{};
    std::int32_t iter = 0;
    float tp = 0.0F;
    float nvl = 0.0F;
    std::array<double, 36> hess{};
    constexpr std::uint32_t kCap = 256;
    std::vector<float> ta(static_cast<size_t>(kCap) * 16);
    std::uint32_t count = 0;
    AwNdtAlignOutput out{};
    out.pose = pose.data();
    out.iteration_num = &iter;
    out.transform_probability = &tp;
    out.nearest_voxel_likelihood = &nvl;
    out.hessian = hess.data();
    out.transformation_array = ta.data();
    out.transforms_cap = kCap;
    out.transforms_count = &count;
    autoware_ndt_scan_matcher_rs_ndt_engine_get_result(handle_, &out);

    for (int rr = 0; rr < 4; ++rr) {
      for (int cc = 0; cc < 4; ++cc) {
        r.pose(rr, cc) = pose[(rr * 4) + cc];
      }
    }
    r.iteration_num = iter;
    r.transform_probability = tp;
    r.nearest_voxel_transformation_likelihood = nvl;
    for (int rr = 0; rr < 6; ++rr) {
      for (int cc = 0; cc < 6; ++cc) {
        r.hessian(rr, cc) = hess[(rr * 6) + cc];
      }
    }
    const std::uint32_t n = std::min(count, kCap);
    r.transformation_array.resize(n);
    for (std::uint32_t k = 0; k < n; ++k) {
      Eigen::Matrix4f m;
      for (int rr = 0; rr < 4; ++rr) {
        for (int cc = 0; cc < 4; ++cc) {
          m(rr, cc) = ta[(static_cast<size_t>(k) * 16) + (rr * 4) + cc];
        }
      }
      r.transformation_array[k] = m;
    }

    std::vector<float> tps(kCap);
    std::vector<float> nvls(kCap);
    std::uint32_t scount = 0;
    autoware_ndt_scan_matcher_rs_ndt_engine_get_score_arrays(
      handle_, tps.data(), nvls.data(), kCap, &scount);
    const std::uint32_t sn = std::min(scount, kCap);
    r.transform_probability_array.assign(tps.begin(), tps.begin() + sn);
    r.nearest_voxel_transformation_likelihood_array.assign(nvls.begin(), nvls.begin() + sn);
    return r;
  }

  double calculateTransformationProbability(const PointCloudSource & cloud) const
  {
    const std::vector<float> c = to_flat(cloud);
    return autoware_ndt_scan_matcher_rs_ndt_engine_calc_transformation_probability(
      handle_, c.data(), cloud.size());
  }
  double calculateNearestVoxelTransformationLikelihood(const PointCloudSource & cloud) const
  {
    const std::vector<float> c = to_flat(cloud);
    return autoware_ndt_scan_matcher_rs_ndt_engine_calc_nearest_voxel_likelihood(
      handle_, c.data(), cloud.size());
  }
  pcl::PointCloud<pcl::PointXYZI> calculateNearestVoxelScoreEachPoint(
    const PointCloudSource & cloud) const
  {
    const std::vector<float> c = to_flat(cloud);
    std::vector<float> scores(cloud.size(), 0.0F);
    autoware_ndt_scan_matcher_rs_ndt_engine_calc_nearest_voxel_score_each_point(
      handle_, c.data(), cloud.size(), scores.data());
    pcl::PointCloud<pcl::PointXYZI> out;
    for (size_t i = 0; i < cloud.size(); ++i) {
      if (scores[i] > 0.0F) {  // > 0 iff the point found a neighbor (matches the C++ output set)
        pcl::PointXYZI p;
        p.x = cloud.points[i].x;
        p.y = cloud.points[i].y;
        p.z = cloud.points[i].z;
        p.intensity = scores[i];
        out.points.push_back(p);
      }
    }
    return out;
  }

  void setRegularizationPose(const Eigen::Matrix4f & pose)
  {
    autoware_ndt_scan_matcher_rs_ndt_engine_set_regularization(
      handle_, pose(0, 3), pose(1, 3), params_.regularization_scale_factor);
  }
  void unsetRegularizationPose()
  {
    autoware_ndt_scan_matcher_rs_ndt_engine_set_regularization(handle_, 0.0F, 0.0F, 0.0F);
  }

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
  std::uint64_t id_for(const std::string & id)
  {
    const auto it = id_map_.find(id);
    if (it != id_map_.end()) {
      return it->second;
    }
    const std::uint64_t v = next_id_++;
    id_map_[id] = v;
    return v;
  }

  AwNdtEngine * handle_;
  pclomp::NdtParams params_{};
  std::map<std::string, std::uint64_t> id_map_;
  std::uint64_t next_id_{0};
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_RUST_ADAPTER_HPP_
