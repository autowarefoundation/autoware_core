// Copyright 2026 Autoware Foundation
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

// A minimal backend in a plugin library, for test_ndt_backend_abi.

#include <autoware/ndt_scan_matcher_common/ndt_backend.hpp>
#include <class_loader/class_loader.hpp>

#include <memory>
#include <string>
#include <vector>

namespace fake
{

using autoware::ndt_scan_matcher::NdtBackend;
using autoware::ndt_scan_matcher::NdtParams;
using autoware::ndt_scan_matcher::NdtResult;

class FakeBackend : public NdtBackend
{
public:
  void initialize(const NdtParams & params) override { params_ = params; }
  std::string name() const override { return "fake"; }
  NdtParams get_params() const override { return params_; }
  void add_target(const pcl::PointCloud<PointTarget>::ConstPtr &, const std::string &) override {}
  void remove_target(const std::string &) override {}
  void commit_target() override {}
  std::vector<std::string> get_current_map_ids() const override { return {}; }
  bool has_target() const override { return false; }
  SharedPtr clone() const override { return std::make_shared<FakeBackend>(*this); }
  SharedPtr clone_empty() const override { return std::make_shared<FakeBackend>(); }
  void set_regularization_pose(const Eigen::Matrix4f &) override {}
  void unset_regularization_pose() override {}
  NdtResult align(
    const Eigen::Matrix4f & guess, const pcl::PointCloud<PointSource>::ConstPtr &) override
  {
    NdtResult result;
    result.pose = guess;
    return result;
  }
  double calculate_transformation_probability(const pcl::PointCloud<PointSource> &) const override
  {
    return 0.0;
  }
  double calculate_nearest_voxel_transformation_likelihood(
    const pcl::PointCloud<PointSource> &) const override
  {
    return 0.0;
  }
  pcl::PointCloud<pcl::PointXYZI> calculate_nearest_voxel_score_each_point(
    const pcl::PointCloud<PointSource> &) const override
  {
    return {};
  }

private:
  NdtParams params_{};
};

}  // namespace fake

CLASS_LOADER_REGISTER_CLASS(fake::FakeBackend, autoware::ndt_scan_matcher::NdtBackend)

// The Eigen configuration of this library, from the macros of Eigen and not from the name of the
// inline namespace. The test compares it with its own configuration.
extern "C" int fake_ndt_backend_static_alignment()
{
  return EIGEN_MAX_STATIC_ALIGN_BYTES;
}

// 1 if Eigen::internal::aligned_malloc() is std::malloc().
extern "C" int fake_ndt_backend_std_malloc()
{
  return ((EIGEN_DEFAULT_ALIGN_BYTES == 0) || EIGEN_MALLOC_ALREADY_ALIGNED) ? 1 : 0;
}
