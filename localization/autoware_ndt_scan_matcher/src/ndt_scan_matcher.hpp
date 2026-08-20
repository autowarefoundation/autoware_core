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

#ifndef NDT_SCAN_MATCHER_HPP_
#define NDT_SCAN_MATCHER_HPP_

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{

/** \brief Rotate the 3x3 position block of a 6x6 (row-major, 36-element) pose covariance by the
 * given rotation matrix, i.e. compute R * C * R^T for the upper-left 3x3 block while leaving the
 * remaining entries untouched. */
std::array<double, 36> rotate_covariance(
  const std::array<double, 36> & src_covariance, const Eigen::Matrix3d & rotation);

/** \brief Compose the final 6x6 (row-major, 36-element) output pose covariance from an estimated
 * 2D (xy) covariance. The estimated covariance is scaled by `scale_factor`, its diagonal is
 * adjusted against the default xx/yy variances, and the resulting xx/xy/yx/yy values are written
 * into the position block of `base_covariance`. All other entries are left untouched. */
std::array<double, 36> compose_output_covariance(
  const std::array<double, 36> & base_covariance, const Eigen::Matrix2d & estimated_covariance_2d,
  const Eigen::Matrix4f & ndt_pose, double scale_factor, double default_cov_xx,
  double default_cov_yy);

// Defined in hyper_parameters.hpp. Forward-declared here so the core interface stays free of the
// (rclcpp-dependent) hyper_parameters / ROS node headers.
enum class CovarianceEstimationType;

/** \brief Parameters controlling covariance estimation. Mirrors the relevant fields of the node's
 * HyperParameters so that the estimation logic itself does not depend on the parameter headers. */
struct CovarianceEstimationConfig
{
  CovarianceEstimationType type{};
  std::vector<double> initial_pose_offset_model_x{};
  std::vector<double> initial_pose_offset_model_y{};
  double temperature{};
  double fixed_covariance_value{};  // covariance used when type == FIXED_VALUE
};

/** \brief Result of estimate_covariance(): the estimated 2D (xy) covariance plus the optional
 * debug pose arrays for visualization. A pose array is present only when the estimation type
 * produces it (MULTI_NDT fills both, MULTI_NDT_SCORE fills only the initial poses, the others
 * leave both empty), and the caller is expected to publish whichever arrays are present. */
struct CovarianceEstimationResult
{
  Eigen::Matrix2d covariance;
  std::optional<geometry_msgs::msg::PoseArray> multi_ndt_result_poses;
  std::optional<geometry_msgs::msg::PoseArray> multi_initial_poses;
};

/** \brief Estimate the 2D (xy) covariance of an NDT result according to `config.type`. This is
 * pure computation: it performs no publishing. The debug pose arrays that the node visualizes are
 * returned in the result (with the given `stamp` / `frame_id`) instead of being published here. */
CovarianceEstimationResult estimate_covariance(
  const CovarianceEstimationConfig & config, const pclomp::NdtResult & ndt_result,
  const Eigen::Matrix4f & initial_pose_matrix, const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id,
  pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> & ndt_ref,
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> & sensor_points);

/** \brief Count the maximum number of consecutive direction inversions ("oscillations") in a
 * sequence of poses. A step is counted as an inversion when the cosine between consecutive motion
 * vectors falls below an internal threshold. */
int count_oscillation(const std::vector<geometry_msgs::msg::Pose> & result_pose_msg_array);

/** \brief Build the arrow MarkerArray visualizing the NDT optimization pose sequence. One ARROW
 * marker is created per pose in `pose_array`, and the remaining ids up to `max_iteration_num + 2`
 * are filled with blank markers (to overwrite stale markers from previous iterations). */
visualization_msgs::msg::MarkerArray create_marker_array(
  const builtin_interfaces::msg::Time & stamp, const std::string & frame_id,
  const std::vector<geometry_msgs::msg::Pose> & pose_array, int max_iteration_num);

/** \brief Convert a point cloud whose per-point intensity holds a nearest-voxel score into an RGB
 * point cloud, mapping each score linearly from the [lower_nvs, upper_nvs] range onto the color
 * scale. Point positions are copied unchanged. */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorize_points_by_score(
  const pcl::PointCloud<pcl::PointXYZI> & scored_points, float lower_nvs, float upper_nvs);

/** \brief Mean and standard deviation of the (x, y, z, roll, pitch) sampling distribution used to
 * seed the initial-pose optimizer. Yaw is sampled uniformly and is therefore not included. */
struct TpeSampleDistribution
{
  std::vector<double> mean;
  std::vector<double> stddev;
};

/** \brief Build the sampling distribution from an initial pose: the means are the pose position
 * and roll/pitch, and the standard deviations are the square roots of the diagonal of the pose
 * covariance (x, y, z, roll, pitch). */
TpeSampleDistribution create_tpe_sample_distribution(
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov);

/** \brief Build a pose from a 6-DoF optimization vector (x, y, z, roll, pitch, yaw). */
geometry_msgs::msg::Pose pose_from_optimization_variables(const std::vector<double> & variables);

/** \brief Decompose a pose into a 6-DoF optimization vector (x, y, z, roll, pitch, yaw). */
std::vector<double> optimization_variables_from_pose(const geometry_msgs::msg::Pose & pose);

enum class ScoreMetric {
  TRANSFORM_PROBABILITY = 0,
  NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD = 1,
};

struct ScoreEvaluationInput
{
  ScoreMetric metric{};
  double transform_probability_threshold{};
  double nearest_voxel_transformation_likelihood_threshold{};
};

struct ScoreEvaluationResult
{
  bool is_supported_metric{true};

  double score{};
  double score_threshold{};
  bool is_score_above_threshold{};

  int expected_array_size{};
  std::size_t transform_probability_array_size{};
  std::size_t nearest_voxel_transformation_likelihood_array_size{};

  std::optional<float> transform_probability_diff{};
  std::optional<float> transform_probability_before{};

  std::optional<float> nearest_voxel_transformation_likelihood_diff{};
  std::optional<float> nearest_voxel_transformation_likelihood_before{};
};

ScoreEvaluationResult evaluate_score(
  const pclomp::NdtResult & ndt_result, const ScoreEvaluationInput & input);

pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> extract_no_ground_points(
  const pcl::PointCloud<pcl::PointXYZ> & sensor_points_in_map, double result_pose_z,
  double z_margin_for_ground_removal);

}  // namespace autoware::ndt_scan_matcher

#endif  // NDT_SCAN_MATCHER_HPP_
