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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__VISUALIZE_POINT_SCORE_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__VISUALIZE_POINT_SCORE_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace autoware::ndt_scan_matcher::detail
{

pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorize_point_scores(
  const pcl::PointCloud<pcl::PointXYZI> & nvs_points_in_map_ptr_i, float lower_nvs, float upper_nvs);

}  // namespace autoware::ndt_scan_matcher::detail

#endif  // AUTOWARE__NDT_SCAN_MATCHER__VISUALIZE_POINT_SCORE_HPP_
