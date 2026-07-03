// Copyright 2015-2019 Autoware Foundation
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
#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>
#include <autoware/ndt_scan_matcher/ndt_scan_matcher_core.hpp>
#include <autoware_utils_pcl/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{
using autoware::localization_util::matrix4f_to_pose;
using autoware::localization_util::pose_to_matrix4f;
using autoware::localization_util::SmartPoseBuffer;

autoware_internal_debug_msgs::msg::Float32Stamped make_float32_stamped(
  const builtin_interfaces::msg::Time & stamp, float data);
autoware_internal_debug_msgs::msg::Int32Stamped make_int32_stamped(
  const builtin_interfaces::msg::Time & stamp, int32_t data);

bool NDTScanMatcher::callback_sensor_points_main(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame)
{
  const auto exe_start_time = std::chrono::system_clock::now();

  const rclcpp::Time sensor_ros_time = sensor_points_msg_in_sensor_frame->header.stamp;



  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_baselink_frame(
    new pcl::PointCloud<PointSource>);

  // check topic_time_stamp
  diagnostics_scan_points_->add_key_value("topic_time_stamp", sensor_ros_time.nanoseconds());

  // check sensor_points_size
  const size_t sensor_points_size = sensor_points_msg_in_sensor_frame->width;
  diagnostics_scan_points_->add_key_value("sensor_points_size", sensor_points_size);
  if (sensor_points_size == 0) {
    std::stringstream message;
    message << "Sensor points is empty.";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return false;
  }

  // check sensor_points_delay_time_sec
  const double sensor_points_delay_time_sec =
    (this->now() - sensor_points_msg_in_sensor_frame->header.stamp).seconds();
  diagnostics_scan_points_->add_key_value(
    "sensor_points_delay_time_sec", sensor_points_delay_time_sec);
  if (sensor_points_delay_time_sec > param_.sensor_points.timeout_sec) {
    std::stringstream message;
    message << "sensor points is experiencing latency."
            << "The delay time is " << sensor_points_delay_time_sec << "[sec] "
            << "(the tolerance is " << param_.sensor_points.timeout_sec << "[sec]).";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());

    // If the delay time of the LiDAR topic exceeds the delay compensation time of ekf_localizer,
    // even if further processing continues, the estimated result will be rejected by ekf_localizer.
    // Therefore, it would be acceptable to exit the function here.
    // However, for now, we will continue the processing as it is.

    // return false;
  }

  // preprocess input pointcloud
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_sensor_frame(
    new pcl::PointCloud<PointSource>);
  const std::string & sensor_frame = sensor_points_msg_in_sensor_frame->header.frame_id;

  pcl::fromROSMsg(*sensor_points_msg_in_sensor_frame, *sensor_points_in_sensor_frame);

  // transform sensor points from sensor-frame to base_link
  try {
    transform_sensor_measurement(
      sensor_frame, param_.frame.base_frame, sensor_points_in_sensor_frame,
      sensor_points_in_baselink_frame);
  } catch (const std::exception & ex) {
    std::stringstream message;
    message << ex.what() << ". Please publish TF " << sensor_frame << " to "
            << param_.frame.base_frame;
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_scan_points_->add_key_value("is_succeed_transform_sensor_points", false);
    return false;
  }
  diagnostics_scan_points_->add_key_value("is_succeed_transform_sensor_points", true);

  // check sensor_points_max_distance
  double max_distance = 0.0;
  for (const auto & point : sensor_points_in_baselink_frame->points) {
    const double distance = std::hypot(point.x, point.y, point.z);
    max_distance = std::max(max_distance, distance);
  }

  diagnostics_scan_points_->add_key_value("sensor_points_max_distance", max_distance);
  if (max_distance < param_.sensor_points.required_distance) {
    std::stringstream message;
    message << "Max distance of sensor points = " << std::fixed << std::setprecision(3)
            << max_distance << " [m] < " << param_.sensor_points.required_distance << " [m]";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return false;
  }

  return ndt_ptr_.with([&](const auto & ndt_ptr) {
    // store sensor points for ndt alignment
    sensor_points_in_baselink_frame_ = sensor_points_in_baselink_frame;

    // The still-C++ cloud publishers read the aligned pose; the return feeds `skipping_publish_num`.
    pclomp::NdtResult ndt_result;
    bool is_converged = false;
    SmartPoseBuffer::InterpolateResult interpolation_result;
    geometry_msgs::msg::Pose result_pose_msg;
    std::vector<geometry_msgs::msg::Pose> transformation_msg_array;
    std::array<double, 36> ndt_covariance{};

    // check is_activated
    const bool node_is_activated = is_activated_;
    diagnostics_scan_points_->add_key_value("is_activated", node_is_activated);
    if (!node_is_activated) {
      std::stringstream message;
      message << "Node is not activated.";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
      return false;
    }

    // calculate initial pose
    std::optional<SmartPoseBuffer::InterpolateResult> interpolation_result_opt =
      initial_pose_buffer_->interpolate(sensor_ros_time);

    // check is_succeed_interpolate_initial_pose
    const bool is_succeed_interpolate_initial_pose = (interpolation_result_opt != std::nullopt);
    diagnostics_scan_points_->add_key_value(
      "is_succeed_interpolate_initial_pose", is_succeed_interpolate_initial_pose);
    if (!is_succeed_interpolate_initial_pose) {
      std::stringstream message;
      message << "Couldn't interpolate pose. Please verify that "
                 "(1) the initial pose topic (primarily come from the EKF) is being published, and "
                 "(2) the timestamps of the sensor PCD messages and pose messages are synchronized "
                 "correctly.";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
      return false;
    }

    initial_pose_buffer_->pop_old(sensor_ros_time);
    interpolation_result = interpolation_result_opt.value();

    // if regularization is enabled and available, set pose to NDT for regularization
    if (param_.ndt_regularization_enable) {
      add_regularization_pose(sensor_ros_time, *ndt_ptr);
    }

    // Warn if the lidar has gone out of the map range
    if (map_update_module_->out_of_map_range(
          interpolation_result.interpolated_pose.pose.pose.position)) {
      std::stringstream msg;

      msg << "Lidar has gone out of the map range";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, msg.str());

      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, msg.str());
    }

    // check is_set_map_points
    const bool is_set_map_points = ndt_ptr->hasTarget();
    diagnostics_scan_points_->add_key_value("is_set_map_points", is_set_map_points);
    if (!is_set_map_points) {
      std::stringstream message;
      message << "Map points is not set.";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
      return false;
    }

    // perform ndt scan matching
    const Eigen::Matrix4f initial_pose_matrix =
      pose_to_matrix4f(interpolation_result.interpolated_pose.pose.pose);
    auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
    ndt_ptr->align(*output_cloud, initial_pose_matrix, sensor_points_in_baselink_frame_);

    ndt_result = ndt_ptr->getResult();

    result_pose_msg = matrix4f_to_pose(ndt_result.pose);
    for (const auto & pose_matrix : ndt_result.transformation_array) {
      geometry_msgs::msg::Pose pose_ros = matrix4f_to_pose(pose_matrix);
      transformation_msg_array.push_back(pose_ros);
    }

    // --- convergence decision ---
    constexpr int oscillation_num_threshold = 10;
    const int max_iterations = ndt_ptr->getMaximumIterations();
    const int oscillation_num = count_oscillation(transformation_msg_array);
    const bool is_ok_iteration_num = (ndt_result.iteration_num < max_iterations);
    const bool is_local_optimal_solution_oscillation =
      (oscillation_num > oscillation_num_threshold);
    bool valid_param_type = false;
    bool is_ok_score = false;
    double score = 0.0;
    double score_threshold = 0.0;
    if (param_.score_estimation.converged_param_type == ConvergedParamType::TRANSFORM_PROBABILITY) {
      valid_param_type = true;
      score = ndt_result.transform_probability;
      score_threshold = param_.score_estimation.converged_param_transform_probability;
    } else if (
      param_.score_estimation.converged_param_type ==
      ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) {
      valid_param_type = true;
      score = ndt_result.nearest_voxel_transformation_likelihood;
      score_threshold =
        param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood;
    }
    if (valid_param_type) {
      is_ok_score = (score > score_threshold);
      is_converged = (is_ok_iteration_num || is_local_optimal_solution_oscillation) && is_ok_score;
    }

    // check iteration_num
    diagnostics_scan_points_->add_key_value("iteration_num", ndt_result.iteration_num);
    if (!is_ok_iteration_num) {
      std::stringstream message;
      message << "The number of iterations has reached its upper limit. The number of iterations: "
              << ndt_result.iteration_num << ", Limit: " << max_iterations << ".";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    }

    // check local_optimal_solution_oscillation_num
    diagnostics_scan_points_->add_key_value(
      "local_optimal_solution_oscillation_num", oscillation_num);
    if (is_local_optimal_solution_oscillation) {
      std::stringstream message;
      message << "There is a possibility of oscillation in a local minimum";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    }

    // check score
    diagnostics_scan_points_->add_key_value(
      "transform_probability", ndt_result.transform_probability);
    diagnostics_scan_points_->add_key_value(
      "nearest_voxel_transformation_likelihood",
      ndt_result.nearest_voxel_transformation_likelihood);
    if (!valid_param_type) {
      std::stringstream message;
      message
        << "Unknown converged param type. Please check `score_estimation.converged_param_type`";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
      return false;
    }

    // check score diff
    const std::vector<float> & tp_array = ndt_result.transform_probability_array;
    if (static_cast<int>(tp_array.size()) != ndt_result.iteration_num + 1) {
      // only publish warning to /diagnostics, not skip publishing pose
      std::stringstream message;
      message << "transform_probability_array size is not equal to iteration_num + 1."
              << " transform_probability_array size: " << tp_array.size()
              << ", iteration_num: " << ndt_result.iteration_num;
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    } else {
      const float diff = tp_array.back() - tp_array.front();
      diagnostics_scan_points_->add_key_value("transform_probability_diff", diff);
      diagnostics_scan_points_->add_key_value("transform_probability_before", tp_array.front());
    }
    const std::vector<float> & nvtl_array =
      ndt_result.nearest_voxel_transformation_likelihood_array;
    if (static_cast<int>(nvtl_array.size()) != ndt_result.iteration_num + 1) {
      // only publish warning to /diagnostics, not skip publishing pose
      std::stringstream message;
      message
        << "nearest_voxel_transformation_likelihood_array size is not equal to iteration_num + 1."
        << " nearest_voxel_transformation_likelihood_array size: " << nvtl_array.size()
        << ", iteration_num: " << ndt_result.iteration_num;
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    } else {
      const float diff = nvtl_array.back() - nvtl_array.front();
      diagnostics_scan_points_->add_key_value("nearest_voxel_transformation_likelihood_diff", diff);
      diagnostics_scan_points_->add_key_value(
        "nearest_voxel_transformation_likelihood_before", nvtl_array.front());
    }

    if (!is_ok_score) {
      std::stringstream message;
      message << "Score is below the threshold. Score: " << score
              << ", Threshold: " << score_threshold;
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    }

    // covariance estimation
    const Eigen::Quaterniond map_to_base_link_quat = Eigen::Quaterniond(
      result_pose_msg.orientation.w, result_pose_msg.orientation.x, result_pose_msg.orientation.y,
      result_pose_msg.orientation.z);
    const Eigen::Matrix3d map_to_base_link_rotation =
      map_to_base_link_quat.normalized().toRotationMatrix();

    ndt_covariance =
      rotate_covariance(param_.covariance.output_pose_covariance, map_to_base_link_rotation);
    if (
      param_.covariance.covariance_estimation.covariance_estimation_type !=
      CovarianceEstimationType::FIXED_VALUE) {
      const Eigen::Matrix2d estimated_covariance_2d =
        estimate_covariance(ndt_result, initial_pose_matrix, sensor_ros_time, *ndt_ptr);
      const Eigen::Matrix2d estimated_covariance_2d_scaled =
        estimated_covariance_2d * param_.covariance.covariance_estimation.scale_factor;
      const double default_cov_xx = param_.covariance.output_pose_covariance[0];
      const double default_cov_yy = param_.covariance.output_pose_covariance[7];
      const Eigen::Matrix2d estimated_covariance_2d_adj = pclomp::adjust_diagonal_covariance(
        estimated_covariance_2d_scaled, ndt_result.pose, default_cov_xx, default_cov_yy);
      ndt_covariance[0 + 6 * 0] = estimated_covariance_2d_adj(0, 0);
      ndt_covariance[1 + 6 * 1] = estimated_covariance_2d_adj(1, 1);
      ndt_covariance[1 + 6 * 0] = estimated_covariance_2d_adj(1, 0);
      ndt_covariance[0 + 6 * 1] = estimated_covariance_2d_adj(0, 1);
    }

    // check distance_initial_to_result
    const auto distance_initial_to_result = static_cast<double>(autoware::localization_util::norm(
      interpolation_result.interpolated_pose.pose.pose.position, result_pose_msg.position));
    diagnostics_scan_points_->add_key_value(
      "distance_initial_to_result", distance_initial_to_result);
    if (distance_initial_to_result > param_.validation.initial_to_result_distance_tolerance_m) {
      std::stringstream message;
      message << "distance_initial_to_result is too large (" << distance_initial_to_result
              << " [m]).";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    }

    // check execution_time
    const auto exe_end_time = std::chrono::system_clock::now();
    const auto duration_micro_sec =
      std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
    const auto exe_time = static_cast<float>(duration_micro_sec) / 1000.0f;
    diagnostics_scan_points_->add_key_value("execution_time", exe_time);
    if (exe_time > param_.validation.critical_upper_bound_exe_time_ms) {
      std::stringstream message;
      message << "NDT exe time is too long (took " << exe_time << " [ms]).";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    }

    // publish legacy C++ outputs. exe_time wraps the prologue + middle.
    exe_time_pub_->publish(make_float32_stamped(sensor_ros_time, exe_time));
    initial_pose_with_covariance_pub_->publish(interpolation_result.interpolated_pose);
    transform_probability_pub_->publish(
      make_float32_stamped(sensor_ros_time, ndt_result.transform_probability));
    nearest_voxel_transformation_likelihood_pub_->publish(
      make_float32_stamped(sensor_ros_time, ndt_result.nearest_voxel_transformation_likelihood));
    iteration_num_pub_->publish(make_int32_stamped(sensor_ros_time, ndt_result.iteration_num));
    publish_tf(sensor_ros_time, result_pose_msg);
    publish_pose(sensor_ros_time, result_pose_msg, ndt_covariance, is_converged);
    publish_marker(sensor_ros_time, transformation_msg_array, ndt_ptr->getMaximumIterations());
    publish_initial_to_result(
      sensor_ros_time, result_pose_msg, interpolation_result.interpolated_pose,
      interpolation_result.old_pose, interpolation_result.new_pose);

    pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_map_ptr(
      new pcl::PointCloud<PointSource>);
    autoware_utils_pcl::transform_pointcloud(
      *sensor_points_in_baselink_frame, *sensor_points_in_map_ptr, ndt_result.pose);
    publish_point_cloud(sensor_ros_time, param_.frame.map_frame, sensor_points_in_map_ptr);

    // check each of point score
    const float lower_nvs = 1.0f;
    const float upper_nvs = 3.5f;
    if (voxel_score_points_pub_->get_subscription_count() > 0) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr nvs_points_in_map_ptr_rgb{
        new pcl::PointCloud<pcl::PointXYZRGB>};
      nvs_points_in_map_ptr_rgb =
        visualize_point_score(sensor_points_in_map_ptr, lower_nvs, upper_nvs, *ndt_ptr);
      sensor_msgs::msg::PointCloud2 nvs_points_msg_in_map;
      pcl::toROSMsg(*nvs_points_in_map_ptr_rgb, nvs_points_msg_in_map);
      nvs_points_msg_in_map.header.stamp = sensor_ros_time;
      nvs_points_msg_in_map.header.frame_id = param_.frame.map_frame;
      voxel_score_points_pub_->publish(nvs_points_msg_in_map);
    }

    // whether use no ground points to calculate score
    if (param_.score_estimation.no_ground_points.enable) {
      // remove ground
      pcl::shared_ptr<pcl::PointCloud<PointSource>> no_ground_points_in_map_ptr(
        new pcl::PointCloud<PointSource>);
      no_ground_points_in_map_ptr->points.reserve(sensor_points_in_map_ptr->size());
      // The aligned pose z is constant over the loop; the translation z of the 4x4 matrix equals
      // matrix4f_to_pose(ndt_result.pose).position.z. Hoist it to avoid rebuilding a full Pose
      // (including a quaternion extraction) for every point in the scan.
      const double result_pose_z = ndt_result.pose(2, 3);
      for (std::size_t i = 0; i < sensor_points_in_map_ptr->size(); i++) {
        const float point_z = sensor_points_in_map_ptr->points[i].z;  // NOLINT
        if (
          point_z - result_pose_z >
          param_.score_estimation.no_ground_points.z_margin_for_ground_removal) {
          no_ground_points_in_map_ptr->points.push_back(sensor_points_in_map_ptr->points[i]);
        }
      }
      // pub remove-ground points
      sensor_msgs::msg::PointCloud2 no_ground_points_msg_in_map;
      pcl::toROSMsg(*no_ground_points_in_map_ptr, no_ground_points_msg_in_map);
      no_ground_points_msg_in_map.header.stamp = sensor_ros_time;
      no_ground_points_msg_in_map.header.frame_id = param_.frame.map_frame;
      no_ground_points_aligned_pose_pub_->publish(no_ground_points_msg_in_map);
      // calculate score
      const auto no_ground_transform_probability = static_cast<float>(
        ndt_ptr->calculateTransformationProbability(*no_ground_points_in_map_ptr));
      const auto no_ground_nearest_voxel_transformation_likelihood = static_cast<float>(
        ndt_ptr->calculateNearestVoxelTransformationLikelihood(*no_ground_points_in_map_ptr));
      // pub score
      no_ground_transform_probability_pub_->publish(
        make_float32_stamped(sensor_ros_time, no_ground_transform_probability));
      no_ground_nearest_voxel_transformation_likelihood_pub_->publish(
        make_float32_stamped(sensor_ros_time, no_ground_nearest_voxel_transformation_likelihood));
    }

    return is_converged;
  });

}

}  // namespace autoware::ndt_scan_matcher
