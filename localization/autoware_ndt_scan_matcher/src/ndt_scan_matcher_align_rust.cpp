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

#include <autoware/localization_util/matrix_type.hpp>
#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/ndt_scan_matcher_core.hpp>
#include <autoware/ndt_scan_matcher/particle.hpp>
#include <autoware_utils_pcl/transforms.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

namespace autoware::ndt_scan_matcher
{
using autoware::localization_util::pose_to_matrix4f;
namespace
{
pcl::PointCloud<pcl::PointXYZ> flat_xyz_to_cloud(
  const std::vector<float> & points, const std::size_t point_count)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.reserve(point_count);
  for (std::size_t i = 0; i < point_count; ++i) {
    const std::size_t base = i * 3U;
    cloud.points.emplace_back(points[base], points[base + 1U], points[base + 2U]);
  }
  cloud.width = static_cast<std::uint32_t>(cloud.points.size());
  cloud.height = 1U;
  cloud.is_dense = true;
  return cloud;
}

geometry_msgs::msg::Pose aw_pose_to_msg(const AwPose & pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.position[0];
  msg.position.y = pose.position[1];
  msg.position.z = pose.position[2];
  msg.orientation.x = pose.orientation[0];
  msg.orientation.y = pose.orientation[1];
  msg.orientation.z = pose.orientation[2];
  msg.orientation.w = pose.orientation[3];
  return msg;
}

AwNdtAlignServiceInput make_align_service_input(
  const bool transform_initial_pose_ok, const bool map_points_ok, const bool sensor_points_ok,
  const bool align_score_available, const double align_score,
  const double reliable_score_threshold)
{
  AwNdtAlignServiceInput input{};
  input.transform_initial_pose_ok = transform_initial_pose_ok ? 1U : 0U;
  input.map_points_ok = map_points_ok ? 1U : 0U;
  input.sensor_points_ok = sensor_points_ok ? 1U : 0U;
  input.align_score_available = align_score_available ? 1U : 0U;
  input.align_score = align_score;
  input.reliable_score_threshold = reliable_score_threshold;
  return input;
}

AwNdtAlignServiceGateAction evaluate_align_service_gate(
  const bool transform_initial_pose_ok, const bool map_points_ok, const bool sensor_points_ok,
  const bool align_score_available, const double align_score,
  const double reliable_score_threshold, AwNdtAlignServiceTrace * trace = nullptr)
{
  const AwNdtAlignServiceInput input = make_align_service_input(
    transform_initial_pose_ok, map_points_ok, sensor_points_ok, align_score_available, align_score,
    reliable_score_threshold);
  AwNdtAlignServiceGateAction action{};
  autoware_ndt_scan_matcher_rs_node_evaluate_align_service_gate(&input, trace, &action);
  return action;
}

std::string align_service_gate_message(
  const std::int32_t message_kind, const std::string & target_frame,
  const std::string & source_frame)
{
  if (message_kind == NDT_ALIGN_SERVICE_MESSAGE_TRANSFORM_UNAVAILABLE) {
    return "Please publish TF " + target_frame + " to " + source_frame;
  }
  if (message_kind == NDT_ALIGN_SERVICE_MESSAGE_MAP_UNAVAILABLE) {
    return "No InputTarget. Please check the map file and the map_loader service";
  }
  if (message_kind == NDT_ALIGN_SERVICE_MESSAGE_SENSOR_UNAVAILABLE) {
    return "No InputSource. Please check the input lidar topic";
  }
  return {};
}

AwNdtAlignServiceResponse assemble_align_service_response(
  const geometry_msgs::msg::PoseWithCovarianceStamped & aligned_pose,
  const geometry_msgs::msg::PoseWithCovarianceStamped & request_pose, const double align_score,
  const double reliable_score_threshold)
{
  AwNdtAlignServiceAlignedInput input{};
  input.stamp_ns = static_cast<rclcpp::Time>(aligned_pose.header.stamp).nanoseconds();
  input.position[0] = aligned_pose.pose.pose.position.x;
  input.position[1] = aligned_pose.pose.pose.position.y;
  input.position[2] = aligned_pose.pose.pose.position.z;
  input.orientation[0] = aligned_pose.pose.pose.orientation.x;
  input.orientation[1] = aligned_pose.pose.pose.orientation.y;
  input.orientation[2] = aligned_pose.pose.pose.orientation.z;
  input.orientation[3] = aligned_pose.pose.pose.orientation.w;
  std::copy(
    request_pose.pose.covariance.begin(), request_pose.pose.covariance.end(),
    input.request_covariance);
  input.align_score = align_score;
  input.reliable_score_threshold = reliable_score_threshold;

  AwNdtAlignServiceResponse response{};
  autoware_ndt_scan_matcher_rs_node_assemble_align_service_response(&input, &response);
  return response;
}

void append_align_search_summary_trace(
  const std::int64_t particles_requested, const std::int64_t particles_evaluated,
  const std::int64_t marker_publish_count, const std::int64_t cloud_publish_count,
  const std::int32_t best_iteration, const double best_score,
  const double reliable_score_threshold, AwNdtAlignServiceTrace * trace)
{
  if (trace == nullptr) {
    return;
  }

  AwNdtAlignServiceSearchSummaryInput input{};
  input.particles_requested = particles_requested;
  input.particles_evaluated = particles_evaluated;
  input.marker_publish_count = marker_publish_count;
  input.cloud_publish_count = cloud_publish_count;
  input.best_iteration = best_iteration;
  input.best_score = best_score;
  input.reliable_score_threshold = reliable_score_threshold;
  autoware_ndt_scan_matcher_rs_node_append_align_service_search_summary_trace(&input, trace);
}

void append_align_response_trace(
  const AwNdtAlignServiceResponse & response, AwNdtAlignServiceTrace * trace)
{
  if (trace == nullptr) {
    return;
  }
  autoware_ndt_scan_matcher_rs_node_append_align_service_response_trace(&response, trace);
}

void apply_align_service_response(
  const AwNdtAlignServiceResponse & align_response, const std::string & frame_id,
  autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Response & res)
{
  res.success = (align_response.success != 0U);
  res.reliable = (align_response.reliable != 0U);
  res.pose_with_covariance.header.stamp = rclcpp::Time(align_response.stamp_ns);
  res.pose_with_covariance.header.frame_id = frame_id;
  res.pose_with_covariance.pose.pose.position.x = align_response.position[0];
  res.pose_with_covariance.pose.pose.position.y = align_response.position[1];
  res.pose_with_covariance.pose.pose.position.z = align_response.position[2];
  res.pose_with_covariance.pose.pose.orientation.x = align_response.orientation[0];
  res.pose_with_covariance.pose.pose.orientation.y = align_response.orientation[1];
  res.pose_with_covariance.pose.pose.orientation.z = align_response.orientation[2];
  res.pose_with_covariance.pose.pose.orientation.w = align_response.orientation[3];
  std::copy(
    std::begin(align_response.covariance), std::end(align_response.covariance),
    res.pose_with_covariance.pose.covariance.begin());
}

}  // namespace

void NDTScanMatcher::service_ndt_align_main(
  const autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  const std::string & target_frame = param_.frame.map_frame;
  const std::string & source_frame = req->pose_with_covariance.header.frame_id;

  geometry_msgs::msg::TransformStamped transform_s2t;
  try {
    transform_s2t = tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    diagnostics_ndt_align_->add_key_value("is_succeed_transform_initial_pose", false);

    const AwNdtAlignServiceGateAction action =
      evaluate_align_service_gate(false, false, false, false, 0.0, 0.0, align_service_trace_);
    const std::string message =
      align_service_gate_message(action.message_kind, target_frame, source_frame);
    diagnostics_ndt_align_->update_level_and_message(
      static_cast<diagnostic_msgs::msg::DiagnosticStatus::_level_type>(action.diagnostic_level),
      message);
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message);
    res->success = (action.success != 0U);
    return;
  }
  diagnostics_ndt_align_->add_key_value("is_succeed_transform_initial_pose", true);

  auto initial_pose_msg_in_map_frame =
    autoware::localization_util::transform(req->pose_with_covariance, transform_s2t);
  initial_pose_msg_in_map_frame.header.stamp = req->pose_with_covariance.header.stamp;
  map_update_module_->update_map(
    initial_pose_msg_in_map_frame.pose.pose.position, diagnostics_ndt_align_);

  const AwNdtEngine * engine = rs_.engine_raw();
  const bool is_set_map_points =
    autoware_ndt_scan_matcher_rs_ndt_engine_has_target(engine);
  diagnostics_ndt_align_->add_key_value("is_set_map_points", is_set_map_points);
  if (!is_set_map_points) {
    const AwNdtAlignServiceGateAction action =
      evaluate_align_service_gate(true, false, false, false, 0.0, 0.0, align_service_trace_);
    const std::string message = align_service_gate_message(action.message_kind, target_frame, source_frame);
    diagnostics_ndt_align_->update_level_and_message(
      static_cast<diagnostic_msgs::msg::DiagnosticStatus::_level_type>(action.diagnostic_level),
      message);
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message);
    res->success = (action.success != 0U);
    return;
  }

  const bool is_set_sensor_points =
    autoware_ndt_scan_matcher_rs_latest_sensor_points_count(rs_.raw()) > 0U;
  diagnostics_ndt_align_->add_key_value("is_set_sensor_points", is_set_sensor_points);
  if (!is_set_sensor_points) {
    const AwNdtAlignServiceGateAction action =
      evaluate_align_service_gate(true, true, false, false, 0.0, 0.0, align_service_trace_);
    const std::string message = align_service_gate_message(action.message_kind, target_frame, source_frame);
    diagnostics_ndt_align_->update_level_and_message(
      static_cast<diagnostic_msgs::msg::DiagnosticStatus::_level_type>(action.diagnostic_level),
      message);
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message);
    res->success = (action.success != 0U);
    return;
  }

  const AwNdtAlignServiceGateAction ready_action =
    evaluate_align_service_gate(true, true, true, false, 0.0, 0.0, align_service_trace_);
  if (ready_action.should_align == 0U) {
    res->success = false;
    return;
  }

  const auto [pose_with_covariance, score] =
    align_pose(initial_pose_msg_in_map_frame, align_service_trace_);

  static_cast<void>(evaluate_align_service_gate(
    true, true, true, true, score,
    param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood,
    align_service_trace_));

  const AwNdtAlignServiceResponse align_response = assemble_align_service_response(
    pose_with_covariance, req->pose_with_covariance, score,
    param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood);
  append_align_response_trace(align_response, align_service_trace_);
  apply_align_service_response(align_response, pose_with_covariance.header.frame_id, *res);
  if (!res->reliable) {
    RCLCPP_WARN_STREAM(
      this->get_logger(), "Initial Pose Estimation is Unstable. Score is " << score);
  }
}

std::tuple<geometry_msgs::msg::PoseWithCovarianceStamped, double> NDTScanMatcher::align_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov,
  AwNdtAlignServiceTrace * trace)
{
  autoware::localization_util::output_pose_with_cov_to_log(
    get_logger(), "align_pose_input", initial_pose_with_cov);

  const Eigen::Map<const autoware::localization_util::RowMatrixXd> covariance = {
    initial_pose_with_cov.pose.covariance.data(), 6, 6};

  std::vector<Particle> particle_array;

  visualization_msgs::msg::MarkerArray marker_array;
  constexpr int64_t publish_num = 20;
  const int64_t publish_interval =
    std::max<int64_t>(param_.initial_pose_estimation.particles_num / publish_num, 1);

  auto return_search_failure = [&](const std::string & message) {
    RCLCPP_ERROR_STREAM(get_logger(), message);
    geometry_msgs::msg::PoseWithCovarianceStamped failure_pose = initial_pose_with_cov;
    failure_pose.header.frame_id = param_.frame.map_frame;
    append_align_search_summary_trace(
      param_.initial_pose_estimation.particles_num, static_cast<std::int64_t>(particle_array.size()),
      0, 0, -1, -std::numeric_limits<double>::infinity(),
      param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood, trace);
    return std::make_tuple(failure_pose, -std::numeric_limits<double>::infinity());
  };

  const std::size_t particles_capacity =
    static_cast<std::size_t>(std::max<int64_t>(param_.initial_pose_estimation.particles_num, 0));
  std::vector<AwPose> initial_poses(particles_capacity);
  std::vector<AwPose> result_poses(particles_capacity);
  std::vector<double> scores(particles_capacity);
  std::vector<std::int32_t> iterations(particles_capacity);

  AwNdtAlignServiceSearchInput search_input{};
  search_input.position[0] = initial_pose_with_cov.pose.pose.position.x;
  search_input.position[1] = initial_pose_with_cov.pose.pose.position.y;
  search_input.position[2] = initial_pose_with_cov.pose.pose.position.z;
  search_input.orientation[0] = initial_pose_with_cov.pose.pose.orientation.x;
  search_input.orientation[1] = initial_pose_with_cov.pose.pose.orientation.y;
  search_input.orientation[2] = initial_pose_with_cov.pose.pose.orientation.z;
  search_input.orientation[3] = initial_pose_with_cov.pose.pose.orientation.w;
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      search_input.covariance[(r * 6) + c] = covariance(r, c);
    }
  }
  search_input.particles_num = param_.initial_pose_estimation.particles_num;
  search_input.n_startup_trials = param_.initial_pose_estimation.n_startup_trials;
  search_input.reliable_score_threshold =
    param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood;

  const std::size_t source_points_capacity =
    autoware_ndt_scan_matcher_rs_latest_sensor_points_count(rs_.raw());
  std::vector<float> source_flat(source_points_capacity * 3U);

  AwNdtAlignServiceSearchOutput search_output{};
  search_output.particles_capacity = particles_capacity;
  search_output.initial_poses = initial_poses.data();
  search_output.result_poses = result_poses.data();
  search_output.scores = scores.data();
  search_output.iterations = iterations.data();
  search_output.source_points = source_flat.data();
  search_output.source_points_capacity = source_points_capacity;

  const std::int32_t search_status =
    autoware_ndt_scan_matcher_rs_node_run_align_service_search_latest(
      rs_.raw(), rs_.engine_raw(), &search_input, &search_output);
  if (search_status != NDT_ALIGN_SERVICE_STATUS_ALIGNED || search_output.valid == 0U) {
    return return_search_failure(
      "Rust align-service search failed with status " + std::to_string(search_status));
  }

  const auto source_cloud = std::make_shared<pcl::PointCloud<PointSource>>(
    flat_xyz_to_cloud(source_flat, search_output.source_points_len));

  for (std::size_t i = 0; i < search_output.particles_len; ++i) {
    Particle particle(
      aw_pose_to_msg(initial_poses[i]), aw_pose_to_msg(result_poses[i]), scores[i], iterations[i]);
    particle_array.push_back(particle);
    push_debug_markers(marker_array, get_clock()->now(), param_.frame.map_frame, particle, i);

    const int64_t one_based = static_cast<int64_t>(i) + 1;
    if (one_based % publish_interval == 0 || one_based == search_output.particles_requested) {
      ndt_monte_carlo_initial_pose_marker_pub_->publish(marker_array);
      marker_array.markers.clear();
    }

    auto sensor_points_in_map_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
    autoware_utils_pcl::transform_pointcloud(
      *source_cloud, *sensor_points_in_map_ptr, pose_to_matrix4f(particle.result_pose));
    publish_point_cloud(
      initial_pose_with_cov.header.stamp, param_.frame.map_frame, sensor_points_in_map_ptr);
  }

  auto best_particle_ptr = std::max_element(
    std::begin(particle_array), std::end(particle_array),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = initial_pose_with_cov.header.stamp;
  result_pose_with_cov_msg.header.frame_id = param_.frame.map_frame;
  result_pose_with_cov_msg.pose.pose = best_particle_ptr->result_pose;

  autoware::localization_util::output_pose_with_cov_to_log(
    get_logger(), "align_pose_output", result_pose_with_cov_msg);
  diagnostics_ndt_align_->add_key_value("best_particle_score", best_particle_ptr->score);
  append_align_search_summary_trace(
    search_output.particles_requested, search_output.particles_evaluated,
    search_output.marker_publish_count, search_output.cloud_publish_count,
    static_cast<std::int32_t>(best_particle_ptr->iteration), best_particle_ptr->score,
    param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood, trace);

  return std::make_tuple(result_pose_with_cov_msg, best_particle_ptr->score);
}

}  // namespace autoware::ndt_scan_matcher
