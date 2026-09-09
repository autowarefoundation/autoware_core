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

#include "gnss_poser.hpp"

#include <tf2/LinearMath/Quaternion.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace autoware::gnss_poser
{
bool is_fixed(const sensor_msgs::msg::NavSatStatus & nav_sat_status_msg)
{
  return nav_sat_status_msg.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX;
}

bool can_get_covariance(const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg)
{
  return nav_sat_fix_msg.position_covariance_type >
         sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
}

geometry_msgs::msg::Point get_median_position(
  const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer)
{
  auto get_median = [](std::vector<double> array) {
    std::sort(std::begin(array), std::end(array));
    const size_t median_index = array.size() / 2;
    double median = (array.size() % 2)
                      ? (array.at(median_index))
                      : ((array.at(median_index) + array.at(median_index - 1)) / 2);
    return median;
  };

  std::vector<double> array_x;
  std::vector<double> array_y;
  std::vector<double> array_z;
  for (const auto & position : position_buffer) {
    array_x.push_back(position.x);
    array_y.push_back(position.y);
    array_z.push_back(position.z);
  }

  geometry_msgs::msg::Point median_point;
  median_point.x = get_median(array_x);
  median_point.y = get_median(array_y);
  median_point.z = get_median(array_z);
  return median_point;
}

geometry_msgs::msg::Point get_average_position(
  const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer)
{
  std::vector<double> array_x;
  std::vector<double> array_y;
  std::vector<double> array_z;
  for (const auto & position : position_buffer) {
    array_x.push_back(position.x);
    array_y.push_back(position.y);
    array_z.push_back(position.z);
  }

  geometry_msgs::msg::Point average_point;
  average_point.x =
    std::reduce(array_x.begin(), array_x.end()) / static_cast<double>(array_x.size());
  average_point.y =
    std::reduce(array_y.begin(), array_y.end()) / static_cast<double>(array_y.size());
  average_point.z =
    std::reduce(array_z.begin(), array_z.end()) / static_cast<double>(array_z.size());
  return average_point;
}

geometry_msgs::msg::Quaternion get_quaternion_by_position_difference(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Point & prev_point)
{
  const double yaw = std::atan2(point.y - prev_point.y, point.x - prev_point.x);
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, yaw);
  return tf2::toMsg(quaternion);
}
}  // namespace autoware::gnss_poser
