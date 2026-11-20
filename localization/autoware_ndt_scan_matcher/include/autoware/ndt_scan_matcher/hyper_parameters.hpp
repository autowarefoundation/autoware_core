// Copyright 2024 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use node file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__NDT_SCAN_MATCHER__HYPER_PARAMETERS_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__HYPER_PARAMETERS_HPP_

#include "ndt_omp/ndt_struct.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{

enum class ConvergedParamType {
  TRANSFORM_PROBABILITY = 0,
  NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD = 1
};

enum class CovarianceEstimationType {
  FIXED_VALUE = 0,
  LAPLACE_APPROXIMATION = 1,
  MULTI_NDT = 2,
  MULTI_NDT_SCORE = 3,
};

/**
 * \brief Every tunable the node runs on, as plain values.
 *
 * Deliberately holds nothing but data: no ROS node, no parameter declaration, no validation that
 * needs either. That is what lets the modules doing the actual work take a `HyperParameters`
 * sub-struct by reference without any of them depending on rclcpp.
 *
 * Reading the values off a node is the other half, and lives in `hyper_parameters_loader.hpp` —
 * the only place that knows these names correspond to ROS parameters.
 */
struct HyperParameters
{
  struct Frame
  {
    std::string base_frame{};
    std::string ndt_base_frame{};
    std::string map_frame{};
  } frame{};

  struct SensorPoints
  {
    double timeout_sec{};
    double required_distance{};
  } sensor_points{};

  pclomp::NdtParams ndt{};
  bool ndt_regularization_enable{};

  struct InitialPoseEstimation
  {
    int64_t particles_num{};
    int64_t n_startup_trials{};
  } initial_pose_estimation{};

  struct Validation
  {
    double initial_pose_timeout_sec{};
    double initial_pose_distance_tolerance_m{};
    double initial_to_result_distance_tolerance_m{};
    double critical_upper_bound_exe_time_ms{};
    int64_t skipping_publish_num{};
  } validation{};

  struct ScoreEstimation
  {
    ConvergedParamType converged_param_type{};
    double converged_param_transform_probability{};
    double converged_param_nearest_voxel_transformation_likelihood{};
    struct NoGroundPoints
    {
      bool enable{};
      double z_margin_for_ground_removal{};
    } no_ground_points{};
  } score_estimation{};

  struct Covariance
  {
    std::array<double, 36> output_pose_covariance{};

    struct CovarianceEstimation
    {
      CovarianceEstimationType covariance_estimation_type{};
      std::vector<double> initial_pose_offset_model_x{};
      std::vector<double> initial_pose_offset_model_y{};
      double temperature{};
      double scale_factor{};
    } covariance_estimation{};
  } covariance{};

  struct DynamicMapLoading
  {
    double update_distance{};
    double map_radius{};
    double lidar_radius{};
    bool publish_loaded_map{};
  } dynamic_map_loading{};
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__HYPER_PARAMETERS_HPP_
