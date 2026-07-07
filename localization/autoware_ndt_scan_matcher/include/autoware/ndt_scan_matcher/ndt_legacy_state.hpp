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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__NDT_LEGACY_STATE_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__NDT_LEGACY_STATE_HPP_

#include "ndt_backend.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

namespace autoware::ndt_scan_matcher
{

class NdtLegacyState
{
  using PointSource = pcl::PointXYZ;

public:
  NdtLegacyState() = default;
  ~NdtLegacyState() = default;

  NdtLegacyState(const NdtLegacyState &) = delete;
  NdtLegacyState & operator=(const NdtLegacyState &) = delete;
  NdtLegacyState(NdtLegacyState &&) = delete;
  NdtLegacyState & operator=(NdtLegacyState &&) = delete;

#ifndef NDT_USE_RUST
  EngineHolder & ndt() { return ndt_; }
  const EngineHolder & ndt() const { return ndt_; }

  pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_baselink_frame()
  {
    return sensor_points_in_baselink_frame_;
  }

  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_baselink_frame() const
  {
    return sensor_points_in_baselink_frame_;
  }

private:
  EngineHolder ndt_{std::make_shared<NdtBackend>()};
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_baselink_frame_;
#endif
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_LEGACY_STATE_HPP_
