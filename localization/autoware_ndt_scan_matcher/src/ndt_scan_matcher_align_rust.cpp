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
#include <autoware/localization_util/tree_structured_parzen_estimator.hpp>
#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/ndt_scan_matcher_core.hpp>
#include <autoware/ndt_scan_matcher/particle.hpp>
#include <autoware_utils_pcl/transforms.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
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
using autoware::localization_util::matrix4f_to_pose;
using autoware::localization_util::pose_to_matrix4f;
using autoware::localization_util::TreeStructuredParzenEstimator;

namespace
{
std::vector<float> cloud_to_flat(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  std::vector<float> f;
  f.reserve(cloud.size() * 3);
  for (const auto & p : cloud) {
    f.push_back(p.x);
    f.push_back(p.y);
    f.push_back(p.z);
  }
  return f;
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

pclomp::NdtResult ndt_result_from_engine(const AwNdtEngine * handle)
{
  pclomp::NdtResult r;
  std::array<float, 16> pose{};
  std::int32_t iter = 0;
  float tp = 0.0F;
  float nvl = 0.0F;
  std::array<double, 36> hess{};
  constexpr std::uint32_t kCap = 256;
  std::vector<float> ta(static_cast<size_t>(kCap) * 16);
  std::uint32_t count = 0;
  AwNdtAlignOutput out{};
  out.pose = pose.data();
  out.iteration_num = &iter;
  out.transform_probability = &tp;
  out.nearest_voxel_likelihood = &nvl;
  out.hessian = hess.data();
  out.transformation_array = ta.data();
  out.transforms_cap = kCap;
  out.transforms_count = &count;
  autoware_ndt_scan_matcher_rs_ndt_engine_get_result(handle, &out);

  for (int rr = 0; rr < 4; ++rr) {
    for (int cc = 0; cc < 4; ++cc) {
      r.pose(rr, cc) = pose[(rr * 4) + cc];
    }
  }
  r.iteration_num = iter;
  r.transform_probability = tp;
  r.nearest_voxel_transformation_likelihood = nvl;
  for (int rr = 0; rr < 6; ++rr) {
    for (int cc = 0; cc < 6; ++cc) {
      r.hessian(rr, cc) = hess[(rr * 6) + cc];
    }
  }
  const std::uint32_t n = std::min(count, kCap);
  r.transformation_array.resize(n);
  for (std::uint32_t k = 0; k < n; ++k) {
    Eigen::Matrix4f m;
    for (int rr = 0; rr < 4; ++rr) {
      for (int cc = 0; cc < 4; ++cc) {
        m(rr, cc) = ta[(static_cast<size_t>(k) * 16) + (rr * 4) + cc];
      }
    }
    r.transformation_array[k] = m;
  }

  std::vector<float> tps(kCap);
  std::vector<float> nvls(kCap);
  std::uint32_t scount = 0;
  autoware_ndt_scan_matcher_rs_ndt_engine_get_score_arrays(
    handle, tps.data(), nvls.data(), kCap, &scount);
  const std::uint32_t sn = std::min(scount, kCap);
  r.transform_probability_array.assign(tps.begin(), tps.begin() + sn);
  r.nearest_voxel_transformation_likelihood_array.assign(nvls.begin(), nvls.begin() + sn);
  return r;
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
      evaluate_align_service_gate(false, false, false, false, 0.0, 0.0);
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

  ndt_ptr_.with([&](auto & ndt_ptr) {
    bool is_set_map_points =
      autoware_ndt_scan_matcher_rs_ndt_engine_has_target(ndt_ptr->raw_handle());
    diagnostics_ndt_align_->add_key_value("is_set_map_points", is_set_map_points);
    if (!is_set_map_points) {
      const AwNdtAlignServiceGateAction action =
        evaluate_align_service_gate(true, false, false, false, 0.0, 0.0);
      const std::string message =
        align_service_gate_message(action.message_kind, target_frame, source_frame);
      diagnostics_ndt_align_->update_level_and_message(
        static_cast<diagnostic_msgs::msg::DiagnosticStatus::_level_type>(action.diagnostic_level),
        message);
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message);
      res->success = (action.success != 0U);
      return;
    }

    bool is_set_sensor_points = (sensor_points_in_baselink_frame_ != nullptr);
    diagnostics_ndt_align_->add_key_value("is_set_sensor_points", is_set_sensor_points);
    if (!is_set_sensor_points) {
      const AwNdtAlignServiceGateAction action =
        evaluate_align_service_gate(true, true, false, false, 0.0, 0.0);
      const std::string message =
        align_service_gate_message(action.message_kind, target_frame, source_frame);
      diagnostics_ndt_align_->update_level_and_message(
        static_cast<diagnostic_msgs::msg::DiagnosticStatus::_level_type>(action.diagnostic_level),
        message);
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message);
      res->success = (action.success != 0U);
      return;
    }

    const AwNdtAlignServiceGateAction ready_action =
      evaluate_align_service_gate(true, true, true, false, 0.0, 0.0);
    if (ready_action.should_align == 0U) {
      res->success = false;
      return;
    }

    const auto [pose_with_covariance, score] =
      align_pose(initial_pose_msg_in_map_frame, *ndt_ptr);

    const AwNdtAlignServiceResponse align_response = assemble_align_service_response(
      pose_with_covariance, req->pose_with_covariance, score,
      param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood);
    apply_align_service_response(align_response, pose_with_covariance.header.frame_id, *res);
    if (!res->reliable) {
      RCLCPP_WARN_STREAM(
        this->get_logger(), "Initial Pose Estimation is Unstable. Score is " << score);
    }
  });
}

std::tuple<geometry_msgs::msg::PoseWithCovarianceStamped, double> NDTScanMatcher::align_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov,
  NormalDistributionsTransform & ndt_ref, AwNdtAlignServiceTrace * trace)
{
  autoware::localization_util::output_pose_with_cov_to_log(
    get_logger(), "align_pose_input", initial_pose_with_cov);

  const auto base_rpy = autoware::localization_util::get_rpy(initial_pose_with_cov);
  const Eigen::Map<const autoware::localization_util::RowMatrixXd> covariance = {
    initial_pose_with_cov.pose.covariance.data(), 6, 6};
  const double stddev_x = std::sqrt(covariance(0, 0));
  const double stddev_y = std::sqrt(covariance(1, 1));
  const double stddev_z = std::sqrt(covariance(2, 2));
  const double stddev_roll = std::sqrt(covariance(3, 3));
  const double stddev_pitch = std::sqrt(covariance(4, 4));

  const std::vector<double> sample_mean{
    initial_pose_with_cov.pose.pose.position.x,
    initial_pose_with_cov.pose.pose.position.y,
    initial_pose_with_cov.pose.pose.position.z,
    base_rpy.x,
    base_rpy.y};
  const std::vector<double> sample_stddev{stddev_x, stddev_y, stddev_z, stddev_roll, stddev_pitch};

  TreeStructuredParzenEstimator tpe(
    TreeStructuredParzenEstimator::Direction::MAXIMIZE,
    param_.initial_pose_estimation.n_startup_trials, sample_mean, sample_stddev);

  std::vector<Particle> particle_array;
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
  const std::vector<float> source_flat = cloud_to_flat(*sensor_points_in_baselink_frame_);

  visualization_msgs::msg::MarkerArray marker_array;
  std::int64_t marker_publish_count = 0;
  std::int64_t cloud_publish_count = 0;
  constexpr int64_t publish_num = 20;
  const int64_t publish_interval =
    std::max<int64_t>(param_.initial_pose_estimation.particles_num / publish_num, 1);

  for (int64_t i = 0; i < param_.initial_pose_estimation.particles_num; i++) {
    const TreeStructuredParzenEstimator::Input input = tpe.get_next_input();

    geometry_msgs::msg::Pose initial_pose;
    initial_pose.position.x = input[0];
    initial_pose.position.y = input[1];
    initial_pose.position.z = input[2];
    geometry_msgs::msg::Vector3 init_rpy;
    init_rpy.x = input[3];
    init_rpy.y = input[4];
    init_rpy.z = input[5];
    tf2::Quaternion tf_quaternion;
    tf_quaternion.setRPY(init_rpy.x, init_rpy.y, init_rpy.z);
    initial_pose.orientation = tf2::toMsg(tf_quaternion);

    const Eigen::Matrix4f initial_pose_matrix = pose_to_matrix4f(initial_pose);
    std::array<float, 16> guess16{};
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        guess16[(r * 4) + c] = initial_pose_matrix(r, c);
      }
    }
    autoware_ndt_scan_matcher_rs_ndt_engine_align(
      ndt_ref.raw_handle(), guess16.data(), source_flat.data(),
      sensor_points_in_baselink_frame_->size());
    const pclomp::NdtResult ndt_result = ndt_result_from_engine(ndt_ref.raw_handle());

    Particle particle(
      initial_pose, matrix4f_to_pose(ndt_result.pose),
      ndt_result.nearest_voxel_transformation_likelihood, ndt_result.iteration_num);
    particle_array.push_back(particle);
    push_debug_markers(marker_array, get_clock()->now(), param_.frame.map_frame, particle, i);

    if (
      (i + 1) % publish_interval == 0 || (i + 1) == param_.initial_pose_estimation.particles_num) {
      ndt_monte_carlo_initial_pose_marker_pub_->publish(marker_array);
      ++marker_publish_count;
      marker_array.markers.clear();
    }

    const geometry_msgs::msg::Pose pose = matrix4f_to_pose(ndt_result.pose);
    const geometry_msgs::msg::Vector3 rpy = autoware::localization_util::get_rpy(pose);

    TreeStructuredParzenEstimator::Input result(6);
    result[0] = pose.position.x;
    result[1] = pose.position.y;
    result[2] = pose.position.z;
    result[3] = rpy.x;
    result[4] = rpy.y;
    result[5] = rpy.z;
    tpe.add_trial(TreeStructuredParzenEstimator::Trial{result, ndt_result.transform_probability});

    auto sensor_points_in_map_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
    autoware_utils_pcl::transform_pointcloud(
      *sensor_points_in_baselink_frame_, *sensor_points_in_map_ptr, ndt_result.pose);
    publish_point_cloud(
      initial_pose_with_cov.header.stamp, param_.frame.map_frame, sensor_points_in_map_ptr);
    ++cloud_publish_count;
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
    param_.initial_pose_estimation.particles_num, static_cast<std::int64_t>(particle_array.size()),
    marker_publish_count, cloud_publish_count,
    static_cast<std::int32_t>(best_particle_ptr->iteration), best_particle_ptr->score,
    param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood, trace);

  return std::make_tuple(result_pose_with_cov_msg, best_particle_ptr->score);
}

}  // namespace autoware::ndt_scan_matcher
