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

#include "pose_initializer.hpp"

#include "utils/localization_util.hpp"

#include <cmath>
#include <stdexcept>

namespace autoware::pose_initializer
{
Pose PoseInitializer::validate_user_defined_initial_pose(
  const std::vector<double> & initial_pose_array)
{
  if (initial_pose_array.size() != 7) {
    throw std::invalid_argument(
      "Could not set user defined initial pose. The size of initial_pose is " +
      std::to_string(initial_pose_array.size()) + ". It must be 7.");
  }
  if (
    std::abs(initial_pose_array[3]) < 1e-6 && std::abs(initial_pose_array[4]) < 1e-6 &&
    std::abs(initial_pose_array[5]) < 1e-6 && std::abs(initial_pose_array[6]) < 1e-6) {
    throw std::invalid_argument("Input quaternion is invalid. All elements are close to zero.");
  }

  Pose initial_pose;
  initial_pose.position.x = initial_pose_array[0];
  initial_pose.position.y = initial_pose_array[1];
  initial_pose.position.z = initial_pose_array[2];
  initial_pose.orientation.x = initial_pose_array[3];
  initial_pose.orientation.y = initial_pose_array[4];
  initial_pose.orientation.z = initial_pose_array[5];
  initial_pose.orientation.w = initial_pose_array[6];

  return initial_pose;
}

InitializationResult PoseInitializer::evaluate_auto_pose(
  const PoseWithCovarianceStamped & aligned_pose, const bool is_reliable,
  const std::optional<PoseWithCovarianceStamped> & gnss_pose,
  const std::array<double, 36> & output_pose_covariance,
  const std::optional<double> & pose_error_threshold)
{
  InitializationResult result;
  DiagnosticsInfo diag;
  diag.is_reliable = is_reliable;

  if (pose_error_threshold.has_value() && gnss_pose.has_value()) {
    double gnss_error_2d = 0.0;
    const bool is_error_small = autoware::pose_initializer::check_pose_error(
      gnss_pose.value().pose.pose, aligned_pose.pose.pose, pose_error_threshold.value(),
      gnss_error_2d);

    diag.gnss_error_2d = gnss_error_2d;
    diag.is_gnss_pose_error_small = is_error_small;

    if (!is_error_small) {
      CoreWarning warn;
      warn.text = " Large error between Initial Pose and GNSS Pose.";
      warn.throttle_ms = 0;
      result.warnings.push_back(warn);
    }
  }

  result.diagnostics = diag;

  PoseWithCovarianceStamped reset_pose = aligned_pose;
  reset_pose.pose.covariance = output_pose_covariance;
  result.reset_pose = reset_pose;
  result.is_success = true;

  return result;
}

InitializationResult PoseInitializer::evaluate_direct_pose(
  const std::vector<PoseWithCovarianceStamped> & request_poses,
  const PoseWithCovarianceStamped & pose,
  const std::array<double, 36> & output_pose_covariance)
{
  InitializationResult result;
  if (request_poses.empty()) {
    result.is_success = false;
    // autoware_common_msgs::msg::ResponseStatus::PARAMETER_ERROR is 1, as used in original code
    result.error_code = 1;
    result.error_message =
      "No input pose_with_covariance. If you want to use DIRECT method, please input "
      "pose_with_covariance.";
    return result;
  }

  PoseWithCovarianceStamped reset_pose = request_poses.front();
  PoseWithCovarianceStamped reset_pose = pose;
  reset_pose.pose.covariance = output_pose_covariance;
  result.reset_pose = reset_pose;
  result.is_success = true;

  return result;
}

}  // namespace autoware::pose_initializer

