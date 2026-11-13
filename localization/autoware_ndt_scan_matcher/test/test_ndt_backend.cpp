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

// A backend for the tests of the error messages of the node. It is built three times as a plugin
// library: with a constructor that throws, with an initialize() that throws, and with another heap
// allocator of Eigen, as a backend that was built with other compiler options.

#include <autoware/ndt_scan_matcher_common/ndt_backend.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Each library has its own class name. pluginlib uses one class loader for all libraries, and it
// finds a class by its name.
#if defined(TEST_NDT_BACKEND_CONSTRUCTOR_THROWS)
#define TEST_NDT_BACKEND_CLASS ConstructorThrowsBackend
#elif defined(TEST_NDT_BACKEND_INITIALIZE_THROWS)
#define TEST_NDT_BACKEND_CLASS InitializeThrowsBackend
#else
#define TEST_NDT_BACKEND_CLASS OtherAbiBackend
#endif

namespace test_ndt_backend
{

using autoware::ndt_scan_matcher::NdtBackend;
using autoware::ndt_scan_matcher::NdtParams;
using autoware::ndt_scan_matcher::NdtResult;

class TEST_NDT_BACKEND_CLASS : public NdtBackend
{
public:
  TEST_NDT_BACKEND_CLASS()
  {
#ifdef TEST_NDT_BACKEND_CONSTRUCTOR_THROWS
    throw std::logic_error("the test backend cannot be made");
#endif
  }
  TEST_NDT_BACKEND_CLASS(const TEST_NDT_BACKEND_CLASS &) = default;

  void initialize(const NdtParams & params) override
  {
#ifdef TEST_NDT_BACKEND_INITIALIZE_THROWS
    throw std::runtime_error("this computer has no test device");
#endif
    params_ = params;
  }
  std::string name() const override { return "test"; }
  NdtParams get_params() const override { return params_; }
  void add_target(const pcl::PointCloud<PointTarget>::ConstPtr &, const std::string &) override {}
  void remove_target(const std::string &) override {}
  void commit_target() override {}
  std::vector<std::string> get_current_map_ids() const override { return {}; }
  bool has_target() const override { return false; }
  SharedPtr clone() const override { return std::make_shared<TEST_NDT_BACKEND_CLASS>(*this); }
  SharedPtr clone_empty() const override { return std::make_shared<TEST_NDT_BACKEND_CLASS>(); }
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

}  // namespace test_ndt_backend

PLUGINLIB_EXPORT_CLASS(
  test_ndt_backend::TEST_NDT_BACKEND_CLASS, autoware::ndt_scan_matcher::NdtBackend)
