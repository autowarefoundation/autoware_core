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

#include <autoware/ndt_scan_matcher/hyper_parameters_loader.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{

HyperParameters declare_hyper_parameters(autoware::agnocast_wrapper::Node * node)
{
  HyperParameters param;

  param.frame.base_frame = node->declare_parameter<std::string>("frame.base_frame");
  param.frame.ndt_base_frame = node->declare_parameter<std::string>("frame.ndt_base_frame");
  param.frame.map_frame = node->declare_parameter<std::string>("frame.map_frame");

  param.sensor_points.timeout_sec = node->declare_parameter<double>("sensor_points.timeout_sec");
  param.sensor_points.required_distance =
    node->declare_parameter<double>("sensor_points.required_distance");

  param.ndt.trans_epsilon = node->declare_parameter<double>("ndt.trans_epsilon");
  param.ndt.step_size = node->declare_parameter<double>("ndt.step_size");
  param.ndt.resolution = node->declare_parameter<float>("ndt.resolution");
  param.ndt.max_iterations =
    static_cast<int>(node->declare_parameter<int64_t>("ndt.max_iterations"));
  param.ndt.num_threads = static_cast<int>(node->declare_parameter<int64_t>("ndt.num_threads"));
  param.ndt.num_threads = std::max(param.ndt.num_threads, 1);
  param.ndt_regularization_enable = node->declare_parameter<bool>("ndt.regularization.enable");
  param.ndt.regularization_scale_factor =
    static_cast<float>(node->declare_parameter<float>("ndt.regularization.scale_factor"));

  param.initial_pose_estimation.particles_num =
    node->declare_parameter<int64_t>("initial_pose_estimation.particles_num");
  param.initial_pose_estimation.n_startup_trials =
    node->declare_parameter<int64_t>("initial_pose_estimation.n_startup_trials");

  param.validation.initial_pose_timeout_sec =
    node->declare_parameter<double>("validation.initial_pose_timeout_sec");
  param.validation.initial_pose_distance_tolerance_m =
    node->declare_parameter<double>("validation.initial_pose_distance_tolerance_m");
  param.validation.initial_to_result_distance_tolerance_m =
    node->declare_parameter<double>("validation.initial_to_result_distance_tolerance_m");
  param.validation.critical_upper_bound_exe_time_ms =
    node->declare_parameter<double>("validation.critical_upper_bound_exe_time_ms");
  param.validation.skipping_publish_num =
    node->declare_parameter<int64_t>("validation.skipping_publish_num");

  const int64_t converged_param_type_tmp =
    node->declare_parameter<int64_t>("score_estimation.converged_param_type");
  param.score_estimation.converged_param_type =
    static_cast<ConvergedParamType>(converged_param_type_tmp);
  param.score_estimation.converged_param_transform_probability =
    node->declare_parameter<double>("score_estimation.converged_param_transform_probability");
  param.score_estimation.converged_param_nearest_voxel_transformation_likelihood =
    node->declare_parameter<double>(
      "score_estimation.converged_param_nearest_voxel_transformation_likelihood");
  param.score_estimation.no_ground_points.enable =
    node->declare_parameter<bool>("score_estimation.no_ground_points.enable");
  param.score_estimation.no_ground_points.z_margin_for_ground_removal =
    node->declare_parameter<double>(
      "score_estimation.no_ground_points.z_margin_for_ground_removal");

  std::vector<double> output_pose_covariance =
    node->declare_parameter<std::vector<double>>("covariance.output_pose_covariance");
  for (std::size_t i = 0; i < output_pose_covariance.size(); ++i) {
    param.covariance.output_pose_covariance[i] = output_pose_covariance[i];
  }
  const int64_t covariance_estimation_type_tmp =
    node->declare_parameter<int64_t>("covariance.covariance_estimation.covariance_estimation_type");
  param.covariance.covariance_estimation.covariance_estimation_type =
    static_cast<CovarianceEstimationType>(covariance_estimation_type_tmp);
  param.covariance.covariance_estimation.initial_pose_offset_model_x =
    node->declare_parameter<std::vector<double>>(
      "covariance.covariance_estimation.initial_pose_offset_model_x");
  param.covariance.covariance_estimation.initial_pose_offset_model_y =
    node->declare_parameter<std::vector<double>>(
      "covariance.covariance_estimation.initial_pose_offset_model_y");
  if (
    param.covariance.covariance_estimation.initial_pose_offset_model_x.size() !=
    param.covariance.covariance_estimation.initial_pose_offset_model_y.size()) {
    std::stringstream message;
    message << "Invalid initial pose offset model parameters."
            << "Please make sure that the number of elements in "
            << "initial_pose_offset_model_x and initial_pose_offset_model_y are the same.";
    throw std::runtime_error(message.str());
  }
  param.covariance.covariance_estimation.temperature =
    node->declare_parameter<double>("covariance.covariance_estimation.temperature");
  param.covariance.covariance_estimation.scale_factor =
    node->declare_parameter<double>("covariance.covariance_estimation.scale_factor");

  param.dynamic_map_loading.update_distance =
    node->declare_parameter<double>("dynamic_map_loading.update_distance");
  param.dynamic_map_loading.map_radius =
    node->declare_parameter<double>("dynamic_map_loading.map_radius");
  param.dynamic_map_loading.lidar_radius =
    node->declare_parameter<double>("dynamic_map_loading.lidar_radius");
  param.dynamic_map_loading.publish_loaded_map =
    node->declare_parameter<bool>("dynamic_map_loading.publish_loaded_map");

  return param;
}

}  // namespace autoware::ndt_scan_matcher
