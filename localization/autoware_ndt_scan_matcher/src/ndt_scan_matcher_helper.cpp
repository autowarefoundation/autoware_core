// Copyright 2024 Autoware Foundation
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

#include "ndt_scan_matcher_helper.hpp"

#include <autoware/localization_util/util_func.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <vector>

namespace autoware::ndt_scan_matcher
{
using autoware::localization_util::point_to_vector3d;

std::array<double, 36> rotate_covariance(
  const std::array<double, 36> & src_covariance, const Eigen::Matrix3d & rotation)
{
  std::array<double, 36> ret_covariance = src_covariance;

  Eigen::Matrix3d src_cov;
  src_cov << src_covariance[0], src_covariance[1], src_covariance[2], src_covariance[6],
    src_covariance[7], src_covariance[8], src_covariance[12], src_covariance[13],
    src_covariance[14];

  Eigen::Matrix3d ret_cov;
  ret_cov = rotation * src_cov * rotation.transpose();

  for (Eigen::Index i = 0; i < 3; ++i) {
    ret_covariance[i] = ret_cov(0, i);
    ret_covariance[i + 6] = ret_cov(1, i);
    ret_covariance[i + 12] = ret_cov(2, i);
  }

  return ret_covariance;
}

int count_oscillation(const std::vector<geometry_msgs::msg::Pose> & result_pose_msg_array)
{
  constexpr double inversion_vector_threshold = -0.9;

  int oscillation_cnt = 0;
  int max_oscillation_cnt = 0;

  for (size_t i = 2; i < result_pose_msg_array.size(); ++i) {
    const Eigen::Vector3d current_pose = point_to_vector3d(result_pose_msg_array.at(i).position);
    const Eigen::Vector3d prev_pose = point_to_vector3d(result_pose_msg_array.at(i - 1).position);
    const Eigen::Vector3d prev_prev_pose =
      point_to_vector3d(result_pose_msg_array.at(i - 2).position);
    const Eigen::Vector3d current_step = current_pose - prev_pose;
    const Eigen::Vector3d prev_step = prev_pose - prev_prev_pose;
    // A zero-length step (e.g. repeated poses) has no direction. normalized() on a zero vector
    // yields NaNs, so guard against it and treat such steps as non-oscillations.
    if (current_step.norm() == 0.0 || prev_step.norm() == 0.0) {
      oscillation_cnt = 0;  // reset
      continue;
    }
    const double cosine_value = current_step.normalized().dot(prev_step.normalized());
    const bool oscillation = cosine_value < inversion_vector_threshold;
    if (oscillation) {
      oscillation_cnt++;  // count consecutive oscillation
    } else {
      oscillation_cnt = 0;  // reset
    }
    max_oscillation_cnt = std::max(max_oscillation_cnt, oscillation_cnt);
  }
  return max_oscillation_cnt;
}

ScoreEvaluationResult evaluate_score(
  const pclomp::NdtResult & ndt_result, const ScoreEvaluationInput & input)
{
  ScoreEvaluationResult result;
  result.expected_array_size = ndt_result.iteration_num + 1;

  if (input.metric == ScoreMetric::TRANSFORM_PROBABILITY) {
    result.score = ndt_result.transform_probability;
    result.score_threshold = input.transform_probability_threshold;
  } else if (input.metric == ScoreMetric::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) {
    result.score = ndt_result.nearest_voxel_transformation_likelihood;
    result.score_threshold = input.nearest_voxel_transformation_likelihood_threshold;
  } else {
    result.is_supported_metric = false;
    return result;
  }

  const std::vector<float> & tp_array = ndt_result.transform_probability_array;
  result.transform_probability_array_size = tp_array.size();
  if (static_cast<int>(tp_array.size()) == result.expected_array_size) {
    result.transform_probability_diff = tp_array.back() - tp_array.front();
    result.transform_probability_before = tp_array.front();
  }

  const std::vector<float> & nvtl_array = ndt_result.nearest_voxel_transformation_likelihood_array;
  result.nearest_voxel_transformation_likelihood_array_size = nvtl_array.size();
  if (static_cast<int>(nvtl_array.size()) == result.expected_array_size) {
    result.nearest_voxel_transformation_likelihood_diff = nvtl_array.back() - nvtl_array.front();
    result.nearest_voxel_transformation_likelihood_before = nvtl_array.front();
  }

  result.is_score_above_threshold = (result.score > result.score_threshold);
  return result;
}

bool is_scan_matching_converged(
  bool is_ok_iteration_num, bool is_local_optimal_solution_oscillation, bool is_ok_score)
{
  return (is_ok_iteration_num || is_local_optimal_solution_oscillation) && is_ok_score;
}

pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> extract_no_ground_points(
  const pcl::PointCloud<pcl::PointXYZ> & sensor_points_in_map, const double result_pose_z,
  const double z_margin_for_ground_removal)
{
  auto no_ground_points_in_map_ptr =
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  no_ground_points_in_map_ptr->points.reserve(sensor_points_in_map.size());

  for (const auto & point : sensor_points_in_map.points) {
    if (point.z - result_pose_z > z_margin_for_ground_removal) {
      no_ground_points_in_map_ptr->points.push_back(point);
    }
  }

  return no_ground_points_in_map_ptr;
}

}  // namespace autoware::ndt_scan_matcher
