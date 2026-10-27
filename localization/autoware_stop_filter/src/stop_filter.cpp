// Copyright 2021-2025 TIER IV
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

#include "stop_filter.hpp"

#include <cmath>

namespace autoware::stop_filter
{
namespace
{
bool is_stopped(
  const nav_msgs::msg::Odometry & input, const double linear_x_threshold,
  const double angular_z_threshold)
{
  const bool linear_stopped = std::fabs(input.twist.twist.linear.x) < linear_x_threshold;
  const bool angular_stopped = std::fabs(input.twist.twist.angular.z) < angular_z_threshold;
  return linear_stopped && angular_stopped;
}
}  // namespace

StopFilter::StopFilter(const StopFilterConfig & config) : config_(config)
{
}

StopFilterResult StopFilter::filter(const nav_msgs::msg::Odometry & input) const
{
  const bool stopped = is_stopped(input, config_.linear_x_threshold, config_.angular_z_threshold);

  StopFilterResult result;
  result.stop_flag.stamp = input.header.stamp;
  result.stop_flag.data = stopped;

  result.filtered_odometry = input;
  if (stopped) {
    result.filtered_odometry.twist.twist.linear.x = 0.0;
    result.filtered_odometry.twist.twist.linear.y = 0.0;
    result.filtered_odometry.twist.twist.linear.z = 0.0;
    result.filtered_odometry.twist.twist.angular.x = 0.0;
    result.filtered_odometry.twist.twist.angular.y = 0.0;
    result.filtered_odometry.twist.twist.angular.z = 0.0;
  }

  return result;
}

}  // namespace autoware::stop_filter
