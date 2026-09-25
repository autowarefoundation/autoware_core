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

#ifndef STOP_FILTER_HPP_
#define STOP_FILTER_HPP_

#include <autoware_internal_debug_msgs/msg/bool_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace autoware::stop_filter
{

class StopFilter
{
public:
  StopFilter(double linear_x_threshold, double angular_z_threshold);

  autoware_internal_debug_msgs::msg::BoolStamped create_stop_flag_msg(
    const nav_msgs::msg::Odometry & input) const;

  nav_msgs::msg::Odometry create_filtered_msg(const nav_msgs::msg::Odometry & input) const;

private:
  double linear_x_threshold_;
  double angular_z_threshold_;
};
}  // namespace autoware::stop_filter
#endif  // STOP_FILTER_HPP_
