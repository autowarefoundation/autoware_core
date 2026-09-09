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
#ifndef GNSS_POSER_HPP_
#define GNSS_POSER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <boost/circular_buffer.hpp>

// The rclcpp-free part of the GNSS poser: pure helpers of the pose computation. They take and
// return message types but need no node, clock, TF or publisher.
namespace autoware::gnss_poser
{
// How the published position is derived from the incoming fixes: the latest one as-is, or the
// average / component-wise median of the last `buff_epoch` ones.
enum class GnssPosePubMethod { Instant = 0, Average = 1, Median = 2 };

/// \brief True when the receiver reports at least STATUS_FIX.
bool is_fixed(const sensor_msgs::msg::NavSatStatus & nav_sat_status_msg);

/// \brief True when the receiver reports a covariance type other than UNKNOWN.
bool can_get_covariance(const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg);

/// \brief Component-wise median of the buffered positions. The buffer must be non-empty.
geometry_msgs::msg::Point get_median_position(
  const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer);

/// \brief Component-wise mean of the buffered positions. The buffer must be non-empty.
geometry_msgs::msg::Point get_average_position(
  const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer);

/// \brief Yaw-only orientation pointing from `prev_point` to `point` (atan2 of the xy
/// displacement); identity when the two points coincide.
geometry_msgs::msg::Quaternion get_quaternion_by_position_difference(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Point & prev_point);
}  // namespace autoware::gnss_poser

#endif  // GNSS_POSER_HPP_
