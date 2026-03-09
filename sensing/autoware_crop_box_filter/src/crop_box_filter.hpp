// Copyright 2026 TIER IV, Inc.
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

#ifndef CROP_BOX_FILTER_HPP_
#define CROP_BOX_FILTER_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

namespace autoware::crop_box_filter
{

struct ValidationResult
{
  bool is_valid;
  std::string reason;
};

ValidationResult validate_pointcloud2(const PointCloud2 & cloud);

}  // namespace autoware::crop_box_filter

#endif  // CROP_BOX_FILTER_HPP_
