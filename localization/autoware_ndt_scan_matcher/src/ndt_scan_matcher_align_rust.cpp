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

AwNdtAlignServiceDecision decide_align_service(
  const bool transform_initial_pose_ok, const bool map_points_ok, const bool sensor_points_ok,
  const bool align_score_available, const double align_score,
  const double reliable_score_threshold, AwNdtAlignServiceTrace * trace = nullptr)
{
  AwNdtAlignServiceInput input{};
  input.transform_initial_pose_ok = transform_initial_pose_ok ? 1U : 0U;
  input.map_points_ok = map_points_ok ? 1U : 0U;
  input.sensor_points_ok = sensor_points_ok ? 1U : 0U;
  input.align_score_available = align_score_available ? 1U : 0U;
  input.align_score = align_score;
  input.reliable_score_threshold = reliable_score_threshold;
  AwNdtAlignServiceDecision decision{};
  autoware_ndt_scan_matcher_rs_node_decide_align_service_traced(&input, trace, &decision);
  return decision;
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

    std::stringstream message;
    message << "Please publish TF " << target_frame.c_str() << " to " << source_frame.c_str();
    diagnostics_ndt_align_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    const AwNdtAlignServiceDecision decision =
      decide_align_service(false, false, false, false, 0.0, 0.0);
    res->success = (decision.success != 0U);
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
      std::stringstream message;
      message << "No InputTarget. Please check the map file and the map_loader service";
      diagnostics_ndt_align_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
      const AwNdtAlignServiceDecision decision =
        decide_align_service(true, false, false, false, 0.0, 0.0);
      res->success = (decision.success != 0U);
      return;
    }

    bool is_set_sensor_points = (sensor_points_in_baselink_frame_ != nullptr);
    diagnostics_ndt_align_->add_key_value("is_set_sensor_points", is_set_sensor_points);
    if (!is_set_sensor_points) {
      std::stringstream message;
      message << "No InputSource. Please check the input lidar topic";
      diagnostics_ndt_align_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
      const AwNdtAlignServiceDecision decision =
        decide_align_service(true, true, false, false, 0.0, 0.0);
      res->success = (decision.success != 0U);
      return;
    }

    const AwNdtAlignServiceDecision ready_decision =
      decide_align_service(true, true, true, false, 0.0, 0.0);
    if (ready_decision.should_align == 0U) {
      res->success = false;
      return;
    }

    const auto [pose_with_covariance, score] =
      align_pose(initial_pose_msg_in_map_frame, *ndt_ptr);

    const AwNdtAlignServiceDecision aligned_decision = decide_align_service(
      true, true, true, true, score,
      param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood);
    res->reliable = (aligned_decision.reliable != 0U);
    if (!res->reliable) {
      RCLCPP_WARN_STREAM(
        this->get_logger(), "Initial Pose Estimation is Unstable. Score is " << score);
    }
    res->success = (aligned_decision.success != 0U);
    res->pose_with_covariance = pose_with_covariance;
    res->pose_with_covariance.pose.covariance = req->pose_with_covariance.pose.covariance;
  });
}

std::tuple<geometry_msgs::msg::PoseWithCovarianceStamped, double> NDTScanMatcher::align_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov,
  NormalDistributionsTransform & ndt_ref)
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

  return std::make_tuple(result_pose_with_cov_msg, best_particle_ptr->score);
}

}  // namespace autoware::ndt_scan_matcher
