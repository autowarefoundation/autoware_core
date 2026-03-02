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

#include "crop_box_filter.hpp"

#include <sensor_msgs/msg/point_field.hpp>

#include <memory>
#include <sstream>

namespace autoware::crop_box_filter
{

ValidationResult validate_pointcloud2(const PointCloud2ConstPtr & cloud)
{
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;

  for (const auto & field : cloud->fields) {
    if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
      continue;
    }
    if (field.name == "x")
      has_x = true;
    else if (field.name == "y")
      has_y = true;
    else if (field.name == "z")
      has_z = true;
  }

  if (!has_x || !has_y || !has_z) {
    return {false, "The pointcloud does not have the required x, y, z FLOAT32 fields."};
  }

  // verify the total size of the point cloud
  if (cloud->width * cloud->height * cloud->point_step != cloud->data.size()) {
    std::ostringstream oss;
    oss << "Invalid PointCloud (data = " << cloud->data.size() << ", width = " << cloud->width
        << ", height = " << cloud->height << ", step = " << cloud->point_step << ") with stamp "
        << cloud->header.stamp.sec + cloud->header.stamp.nanosec * 1e-9 << ", and frame "
        << cloud->header.frame_id << " received!";
    return {false, oss.str()};
  }

  return {true, ""};
}

}  // namespace autoware::crop_box_filter
