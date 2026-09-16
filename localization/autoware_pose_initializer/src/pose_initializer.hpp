// Copyright 2022 The Autoware Contributors
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

#ifndef POSE_INITIALIZER_HPP_
#define POSE_INITIALIZER_HPP_

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autoware::pose_initializer
{
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using Pose = geometry_msgs::msg::Pose;

struct CoreWarning
{
  std::string text;
  uint32_t throttle_ms{0};
};

struct DiagnosticsInfo
{
  bool is_reliable;
  std::optional<double> gnss_error_2d;
  std::optional<bool> is_gnss_pose_error_small;
};

struct InitializationResult
{
  bool is_success{false};
  uint16_t error_code{0};
  std::string error_message;

  std::vector<CoreWarning> warnings;
  std::optional<DiagnosticsInfo> diagnostics;
  std::optional<PoseWithCovarianceStamped> reset_pose;
};

class PoseInitializer
{
public:
  PoseInitializer() = default;

  // Validate user defined initial pose
  static Pose validate_user_defined_initial_pose(const std::vector<double> & initial_pose_array);

  // Evaluate the auto initialization request
  static InitializationResult evaluate_auto_pose(
    const PoseWithCovarianceStamped & aligned_pose, const bool is_reliable,
    const std::optional<PoseWithCovarianceStamped> & gnss_pose,
    const std::array<double, 36> & output_pose_covariance,
    const std::optional<double> & pose_error_threshold);
};

}  // namespace autoware::pose_initializer

#endif  // POSE_INITIALIZER_HPP_
