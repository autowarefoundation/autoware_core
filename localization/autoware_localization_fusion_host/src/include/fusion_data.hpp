// Copyright 2025 Autoware Foundation
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

#ifndef FUSION_DATA_HPP_
#define FUSION_DATA_HPP_

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace autoware::localization_fusion_host
{

struct FusionData
{
  nav_msgs::msg::Odometry ekf_odom;
  geometry_msgs::msg::TwistWithCovarianceStamped ekf_twist_cov;
  bool has_ekf_estimate{false};

  nav_msgs::msg::Odometry kinematic_state;
  bool has_kinematic_state{false};

  geometry_msgs::msg::TwistWithCovarianceStamped twist_estimator;
  bool has_twist_estimator{false};
};

}  // namespace autoware::localization_fusion_host

#endif  // FUSION_DATA_HPP_
