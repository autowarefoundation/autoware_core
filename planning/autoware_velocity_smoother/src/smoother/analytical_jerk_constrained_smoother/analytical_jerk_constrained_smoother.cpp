// Copyright 2021 Tier IV, Inc.
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

#include "autoware/velocity_smoother/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.hpp"

#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/trajectory/trajectory_point.hpp"
#include "autoware/velocity_smoother/trajectory_utils.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/all.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace
{
using TrajectoryPoints = std::vector<autoware_planning_msgs::msg::TrajectoryPoint>;
using TrajectoryExperimental =
  autoware::experimental::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>;

geometry_msgs::msg::Pose lerpByPose(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2, const double t)
{
  tf2::Transform tf_transform1, tf_transform2;
  tf2::fromMsg(p1, tf_transform1);
  tf2::fromMsg(p2, tf_transform2);
  const auto & tf_point = tf2::lerp(tf_transform1.getOrigin(), tf_transform2.getOrigin(), t);
  const auto & tf_quaternion =
    tf2::slerp(tf_transform1.getRotation(), tf_transform2.getRotation(), t);

  geometry_msgs::msg::Pose pose;
  pose.position.x = tf_point.getX();
  pose.position.y = tf_point.getY();
  pose.position.z = tf_point.getZ();
  pose.orientation = tf2::toMsg(tf_quaternion);
  return pose;
}

bool applyMaxVelocity(
  const double max_velocity, const size_t start_index, const size_t end_index,
  TrajectoryPoints & output_trajectory)
{
  if (end_index < start_index || output_trajectory.size() < end_index) {
    return false;
  }

  for (size_t i = start_index; i <= end_index; ++i) {
    output_trajectory.at(i).longitudinal_velocity_mps =
      std::min(output_trajectory.at(i).longitudinal_velocity_mps, static_cast<float>(max_velocity));
    output_trajectory.at(i).acceleration_mps2 = 0.0;
  }
  return true;
}

[[maybe_unused]] bool applyMaxVelocity(
  const double max_velocity, const double start_distance, const double end_distance,
  TrajectoryExperimental & trajectory)
{
  if (end_distance < start_distance || trajectory.length() < end_distance) {
    return false;
  }

  trajectory.longitudinal_velocity_mps().range(start_distance, end_distance).set(max_velocity);

  return true;
}

}  // namespace

namespace autoware::velocity_smoother
{
AnalyticalJerkConstrainedSmoother::AnalyticalJerkConstrainedSmoother(
  rclcpp::Node & node, const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
: SmootherBase(node, time_keeper)
{
  auto & p = smoother_param_;
  p.resample.ds_resample = node.declare_parameter<double>("resample.ds_resample");
  p.resample.num_resample = node.declare_parameter<int>("resample.num_resample");
  p.resample.delta_yaw_threshold = node.declare_parameter<double>("resample.delta_yaw_threshold");
  p.latacc.enable_constant_velocity_while_turning =
    node.declare_parameter<bool>("latacc.enable_constant_velocity_while_turning");
  p.latacc.constant_velocity_dist_threshold =
    node.declare_parameter<double>("latacc.constant_velocity_dist_threshold");
  p.forward.max_acc = node.declare_parameter<double>("forward.max_acc");
  p.forward.min_acc = node.declare_parameter<double>("forward.min_acc");
  p.forward.max_jerk = node.declare_parameter<double>("forward.max_jerk");
  p.forward.min_jerk = node.declare_parameter<double>("forward.min_jerk");
  p.forward.kp = node.declare_parameter<double>("forward.kp");
  p.backward.start_jerk = node.declare_parameter<double>("backward.start_jerk");
  p.backward.min_jerk_mild_stop = node.declare_parameter<double>("backward.min_jerk_mild_stop");
  p.backward.min_jerk = node.declare_parameter<double>("backward.min_jerk");
  p.backward.min_acc_mild_stop = node.declare_parameter<double>("backward.min_acc_mild_stop");
  p.backward.min_acc = node.declare_parameter<double>("backward.min_acc");
  p.backward.span_jerk = node.declare_parameter<double>("backward.span_jerk");
}

void AnalyticalJerkConstrainedSmoother::setParam(const Param & smoother_param)
{
  smoother_param_ = smoother_param;
}

AnalyticalJerkConstrainedSmoother::Param AnalyticalJerkConstrainedSmoother::getParam() const
{
  return smoother_param_;
}

bool AnalyticalJerkConstrainedSmoother::apply(
  const double initial_vel, const double initial_acc, const TrajectoryPoints & input,
  TrajectoryPoints & output, [[maybe_unused]] std::vector<TrajectoryPoints> & debug_trajectories,
  [[maybe_unused]] const bool publish_debug_trajs)
{
  RCLCPP_DEBUG(logger_, "-------------------- Start --------------------");

  // guard
  if (input.empty()) {
    RCLCPP_DEBUG(logger_, "Fail. input trajectory is empty");
    return false;
  }

  // intput trajectory is cropped, so closest_index = 0
  const size_t closest_index = 0;

  // Find deceleration targets
  if (input.size() == 1) {
    RCLCPP_DEBUG(
      logger_,
      "Input trajectory size is too short. Cannot find decel targets and "
      "return v0, a0");
    output = input;
    output.front().longitudinal_velocity_mps = initial_vel;
    output.front().acceleration_mps2 = initial_acc;
    return true;
  }
  std::vector<std::pair<size_t, double>> decel_target_indices;
  searchDecelTargetIndices(input, closest_index, decel_target_indices);
  RCLCPP_DEBUG(logger_, "Num deceleration targets: %zd", decel_target_indices.size());
  for (auto & index : decel_target_indices) {
    RCLCPP_DEBUG(
      logger_, "Target deceleration index: %ld, target velocity: %f", index.first, index.second);
  }

  // Apply filters according to deceleration targets
  TrajectoryPoints reference_trajectory = input;
  TrajectoryPoints filtered_trajectory = input;
  for (size_t i = 0; i < decel_target_indices.size(); ++i) {
    size_t fwd_start_index;
    double fwd_start_vel;
    double fwd_start_acc;
    if (i == 0) {
      fwd_start_index = closest_index;
      fwd_start_vel = initial_vel;
      fwd_start_acc = initial_acc;
    } else {
      fwd_start_index = decel_target_indices.at(i - 1).first;
      fwd_start_vel = filtered_trajectory.at(fwd_start_index).longitudinal_velocity_mps;
      fwd_start_acc = filtered_trajectory.at(fwd_start_index).acceleration_mps2;
    }

    RCLCPP_DEBUG(logger_, "Apply forward jerk filter from: %ld", fwd_start_index);
    applyForwardJerkFilter(
      reference_trajectory, fwd_start_index, fwd_start_vel, fwd_start_acc, smoother_param_,
      filtered_trajectory);

    size_t bwd_start_index = closest_index;
    double bwd_start_vel = initial_vel;
    double bwd_start_acc = initial_acc;
    for (int j = i; j >= 0; --j) {
      if (j == 0) {
        bwd_start_index = closest_index;
        bwd_start_vel = initial_vel;
        bwd_start_acc = initial_acc;
        break;
      }
      if (decel_target_indices.at(j - 1).second < decel_target_indices.at(j).second) {
        bwd_start_index = decel_target_indices.at(j - 1).first;
        bwd_start_vel = filtered_trajectory.at(bwd_start_index).longitudinal_velocity_mps;
        bwd_start_acc = filtered_trajectory.at(bwd_start_index).acceleration_mps2;
        break;
      }
    }
    std::vector<size_t> start_indices;
    if (bwd_start_index != fwd_start_index) {
      start_indices.push_back(bwd_start_index);
      start_indices.push_back(fwd_start_index);
    } else {
      start_indices.push_back(bwd_start_index);
    }

    const size_t decel_target_index = decel_target_indices.at(i).first;
    const double decel_target_vel = decel_target_indices.at(i).second;
    RCLCPP_DEBUG(
      logger_, "Apply backward decel filter from: %s, to: %ld (%f)",
      strStartIndices(start_indices).c_str(), decel_target_index, decel_target_vel);
    if (!applyBackwardDecelFilter(
          start_indices, decel_target_index, decel_target_vel, smoother_param_,
          filtered_trajectory)) {
      RCLCPP_DEBUG(
        logger_,
        "Failed to apply backward decel filter, so apply max velocity filter. max velocity = %f, "
        "start_index = %s, end_index = %zd",
        decel_target_vel, strStartIndices(start_indices).c_str(), filtered_trajectory.size() - 1);

      const double ep = 0.001;
      if (std::abs(decel_target_vel) < ep) {
        applyMaxVelocity(0.0, bwd_start_index, filtered_trajectory.size() - 1, filtered_trajectory);
        output = filtered_trajectory;
        RCLCPP_DEBUG(logger_, "-------------------- Finish --------------------");
        return true;
      }
      applyMaxVelocity(decel_target_vel, bwd_start_index, decel_target_index, reference_trajectory);
      RCLCPP_DEBUG(logger_, "Apply forward jerk filter from: %ld", bwd_start_index);
      applyForwardJerkFilter(
        reference_trajectory, bwd_start_index, bwd_start_vel, bwd_start_acc, smoother_param_,
        filtered_trajectory);
    }
  }

  size_t start_index;
  double start_vel;
  double start_acc;
  if (decel_target_indices.empty() == true) {
    start_index = closest_index;
    start_vel = initial_vel;
    start_acc = initial_acc;
  } else {
    start_index = decel_target_indices.back().first;
    start_vel = filtered_trajectory.at(start_index).longitudinal_velocity_mps;
    start_acc = filtered_trajectory.at(start_index).acceleration_mps2;
  }
  RCLCPP_DEBUG(logger_, "Apply forward jerk filter from: %ld", start_index);
  applyForwardJerkFilter(
    reference_trajectory, start_index, start_vel, start_acc, smoother_param_, filtered_trajectory);

  output = filtered_trajectory;

  RCLCPP_DEBUG(logger_, "-------------------- Finish --------------------");
  return true;
}

bool AnalyticalJerkConstrainedSmoother::apply(
  const double initial_vel, const double initial_acc, const TrajectoryExperimental & input,
  TrajectoryExperimental & output,
  [[maybe_unused]] std::vector<TrajectoryExperimental> & debug_trajectories,
  [[maybe_unused]] const bool publish_debug_trajs)
{
  RCLCPP_DEBUG(logger_, "-------------------- Start (Continuous) --------------------");

  // guard
  const auto [bases, velocities] = input.longitudinal_velocity_mps().get_data();
  if (bases.empty() || velocities.empty()) {
    RCLCPP_DEBUG(logger_, "Fail. input trajectory is empty");
    return false;
  }

  // closest_distance = 0
  const double closest_distance = 0.0;

  if (bases.size() == 1) {
    RCLCPP_DEBUG(
      logger_,
      "Input trajectory size is too short. Cannot find decel targets and "
      "return v0, a0");
    output = input;
    output.longitudinal_velocity_mps().range(0.0, 0.0).set(initial_vel);
    output.acceleration_mps2().range(0.0, 0.0).set(initial_acc);
    return true;
  }

  std::vector<std::pair<double, double>> decel_target_distances;
  searchDecelTargetIndices(input, closest_distance, decel_target_distances);
  RCLCPP_DEBUG(logger_, "Num deceleration targets: %zd", decel_target_distances.size());
  for (auto & target : decel_target_distances) {
    RCLCPP_DEBUG(
      logger_, "Target deceleration distance: %f, target velocity: %f", target.first,
      target.second);
  }

  // apply filters according to deceleration targets
  TrajectoryExperimental reference_trajectory = input;
  TrajectoryExperimental filtered_trajectory = input;

  for (size_t i = 0; i < decel_target_distances.size(); ++i) {
    double fwd_start_distance;
    double fwd_start_vel;
    double fwd_start_acc;
    if (i == 0) {
      fwd_start_distance = closest_distance;
      fwd_start_vel = initial_vel;
      fwd_start_acc = initial_acc;
    } else {
      fwd_start_distance = decel_target_distances.at(i - 1).first;
      fwd_start_vel = filtered_trajectory.longitudinal_velocity_mps().compute(fwd_start_distance);
      fwd_start_acc = filtered_trajectory.acceleration_mps2().compute(fwd_start_distance);
    }

    RCLCPP_DEBUG(logger_, "Apply forward jerk filter from: %f", fwd_start_distance);
    applyForwardJerkFilter(
      reference_trajectory, fwd_start_distance, fwd_start_vel, fwd_start_acc, smoother_param_,
      filtered_trajectory);

    double bwd_start_distance = closest_distance;
    double bwd_start_vel = initial_vel;
    double bwd_start_acc = initial_acc;
    for (int j = static_cast<int>(i); j >= 0; --j) {
      if (j == 0) {
        bwd_start_distance = closest_distance;
        bwd_start_vel = initial_vel;
        bwd_start_acc = initial_acc;
        break;
      }
      if (decel_target_distances.at(j - 1).second < decel_target_distances.at(j).second) {
        bwd_start_distance = decel_target_distances.at(j - 1).first;
        bwd_start_vel = filtered_trajectory.longitudinal_velocity_mps().compute(bwd_start_distance);
        bwd_start_acc = filtered_trajectory.acceleration_mps2().compute(bwd_start_distance);
        break;
      }
    }

    std::vector<double> start_distances;
    if (bwd_start_distance != fwd_start_distance) {
      start_distances.push_back(bwd_start_distance);
      start_distances.push_back(fwd_start_distance);
    } else {
      start_distances.push_back(bwd_start_distance);
    }

    const double decel_target_distance = decel_target_distances.at(i).first;
    const double decel_target_vel = decel_target_distances.at(i).second;
    RCLCPP_DEBUG(
      logger_, "Apply backward decel filter from: %f, to: %f (%f)", bwd_start_distance,
      decel_target_distance, decel_target_vel);

    if (!applyBackwardDecelFilter(
          start_distances, decel_target_distance, decel_target_vel, smoother_param_,
          filtered_trajectory)) {
      RCLCPP_DEBUG(
        logger_,
        "Failed to apply backward decel filter, so apply max velocity filter. max velocity = %f, "
        "start_distance = %f, end_distance = %f",
        decel_target_vel, bwd_start_distance, input.length());

      const double ep = 0.001;
      if (std::abs(decel_target_vel) < ep) {
        applyMaxVelocity(0.0, bwd_start_distance, input.length(), filtered_trajectory);
        output = filtered_trajectory;
        RCLCPP_DEBUG(logger_, "-------------------- Finish (Continuous) --------------------");
        return true;
      }
      applyMaxVelocity(
        decel_target_vel, bwd_start_distance, decel_target_distance, reference_trajectory);
      RCLCPP_DEBUG(logger_, "Apply forward jerk filter from: %f", bwd_start_distance);
      applyForwardJerkFilter(
        reference_trajectory, bwd_start_distance, bwd_start_vel, bwd_start_acc, smoother_param_,
        filtered_trajectory);
    }
  }

  double start_distance;
  double start_vel;
  double start_acc;
  if (decel_target_distances.empty()) {
    start_distance = closest_distance;
    start_vel = initial_vel;
    start_acc = initial_acc;
  } else {
    start_distance = decel_target_distances.back().first;
    start_vel = filtered_trajectory.longitudinal_velocity_mps().compute(start_distance);
    start_acc = filtered_trajectory.acceleration_mps2().compute(start_distance);
  }
  RCLCPP_DEBUG(logger_, "Apply forward jerk filter from: %f", start_distance);
  applyForwardJerkFilter(
    reference_trajectory, start_distance, start_vel, start_acc, smoother_param_,
    filtered_trajectory);

  output = filtered_trajectory;

  RCLCPP_DEBUG(logger_, "-------------------- Finish (Continuous) --------------------");
  return true;
}

TrajectoryPoints AnalyticalJerkConstrainedSmoother::resampleTrajectory(
  const TrajectoryPoints & input, [[maybe_unused]] const double v0,
  [[maybe_unused]] const geometry_msgs::msg::Pose & current_pose,
  [[maybe_unused]] const double nearest_dist_threshold,
  [[maybe_unused]] const double nearest_yaw_threshold) const
{
  TrajectoryPoints output;
  if (input.empty()) {
    RCLCPP_WARN(logger_, "Input trajectory is empty");
    return input;
  }

  const double ds = 1.0 / static_cast<double>(smoother_param_.resample.num_resample);

  for (size_t i = 0; i < input.size() - 1; ++i) {
    double s = 0.0;
    const auto tp0 = input.at(i);
    const auto tp1 = input.at(i + 1);

    const double dist_thr = 0.001;  // 1mm
    const double dist_tp0_tp1 = autoware_utils_geometry::calc_distance2d(tp0, tp1);
    if (std::fabs(dist_tp0_tp1) < dist_thr) {
      output.push_back(input.at(i));
      continue;
    }

    for (size_t j = 0; j < static_cast<size_t>(smoother_param_.resample.num_resample); ++j) {
      auto tp = input.at(i);

      tp.pose = lerpByPose(tp0.pose, tp1.pose, s);
      tp.longitudinal_velocity_mps = tp0.longitudinal_velocity_mps;
      tp.heading_rate_rps = (1.0 - s) * tp0.heading_rate_rps + s * tp1.heading_rate_rps;
      tp.acceleration_mps2 = tp0.acceleration_mps2;
      // tp.accel.angular.z = (1.0 - s) * tp0.accel.angular.z + s * tp1.accel.angular.z;

      output.push_back(tp);

      s += ds;
    }
  }

  output.push_back(input.back());

  return output;
}

TrajectoryPoints AnalyticalJerkConstrainedSmoother::applyLateralAccelerationFilter(
  const TrajectoryPoints & input, [[maybe_unused]] const double v0,
  [[maybe_unused]] const double a0, [[maybe_unused]] const bool enable_smooth_limit,
  const bool use_resampling, const double input_points_interval) const
{
  if (input.size() < 3) {
    return input;  // cannot calculate lateral acc. do nothing.
  }

  // Interpolate with constant interval distance for lateral acceleration calculation.
  const double points_interval = use_resampling ? 0.1 : input_points_interval;  // [m]

  TrajectoryPoints output;
  // since the resampling takes a long time, omit the resampling when it is not requested
  if (use_resampling) {
    std::vector<double> out_arclength;
    const std::vector<double> in_arclength = trajectory_utils::calcArclengthArray(input);
    for (double s = 0; s < in_arclength.back(); s += points_interval) {
      out_arclength.push_back(s);
    }
    const auto output_traj = autoware::motion_utils::resampleTrajectory(
      autoware::motion_utils::convertToTrajectory(input), out_arclength);
    output = autoware::motion_utils::convertToTrajectoryPointArray(output_traj);
    output.back() = input.back();  // keep the final speed.
  } else {
    output = input;
  }

  constexpr double curvature_calc_dist = 5.0;  // [m] calc curvature with 5m away points
  const size_t idx_dist =
    static_cast<size_t>(std::max(static_cast<int>((curvature_calc_dist) / points_interval), 1));

  // Calculate curvature assuming the trajectory points interval is constant
  const auto curvature_v = trajectory_utils::calcTrajectoryCurvatureFrom3Points(output, idx_dist);

  // Decrease speed according to lateral G
  const size_t before_decel_index =
    static_cast<size_t>(std::round(base_param_.decel_distance_before_curve / points_interval));
  const size_t after_decel_index =
    static_cast<size_t>(std::round(base_param_.decel_distance_after_curve / points_interval));

  const auto lateral_acceleration_velocity_square_ratio_limits =
    computeLateralAccelerationVelocitySquareRatioLimits();

  std::vector<int> filtered_points;
  for (size_t i = 0; i < output.size(); ++i) {
    double curvature = 0.0;
    const size_t start = i > before_decel_index ? i - before_decel_index : 0;
    const size_t end = std::min(output.size(), i + after_decel_index);
    for (size_t j = start; j < end; ++j) {
      curvature = std::max(curvature, std::fabs(curvature_v.at(j)));
    }
    double v_curvature_max = computeVelocityLimitFromLateralAcc(
      curvature, lateral_acceleration_velocity_square_ratio_limits);
    v_curvature_max = std::max(v_curvature_max, base_param_.min_curve_velocity);
    if (output.at(i).longitudinal_velocity_mps > v_curvature_max) {
      output.at(i).longitudinal_velocity_mps = v_curvature_max;
      filtered_points.push_back(i);
    }
  }

  // Keep constant velocity while turning
  const double dist_threshold = smoother_param_.latacc.constant_velocity_dist_threshold;
  std::vector<std::tuple<size_t, size_t, double>> latacc_filtered_ranges;
  size_t start_index = 0;
  size_t end_index = 0;
  bool is_updated = false;
  double min_latacc_velocity;
  for (size_t i = 0; i < filtered_points.size(); ++i) {
    const size_t index = filtered_points.at(i);

    if (is_updated == false) {
      start_index = index;
      end_index = index;
      min_latacc_velocity = output.at(index).longitudinal_velocity_mps;
      is_updated = true;
      continue;
    }

    if (
      autoware_utils_geometry::calc_distance2d(output.at(end_index), output.at(index)) <
      dist_threshold) {
      end_index = index;
      min_latacc_velocity = std::min(
        static_cast<double>(output.at(index).longitudinal_velocity_mps), min_latacc_velocity);
    } else {
      latacc_filtered_ranges.emplace_back(start_index, end_index, min_latacc_velocity);
      start_index = index;
      end_index = index;
      min_latacc_velocity = output.at(index).longitudinal_velocity_mps;
    }
  }
  if (is_updated) {
    latacc_filtered_ranges.emplace_back(start_index, end_index, min_latacc_velocity);
  }

  for (size_t i = 0; i < output.size(); ++i) {
    for (const auto & lat_acc_filtered_range : latacc_filtered_ranges) {
      const size_t filtered_start_index = std::get<0>(lat_acc_filtered_range);
      const size_t filtered_end_index = std::get<1>(lat_acc_filtered_range);
      const double filtered_min_latacc_velocity = std::get<2>(lat_acc_filtered_range);

      if (
        filtered_start_index <= i && i <= filtered_end_index &&
        smoother_param_.latacc.enable_constant_velocity_while_turning) {
        output.at(i).longitudinal_velocity_mps = filtered_min_latacc_velocity;
        break;
      }
    }
  }

  return output;
}

TrajectoryExperimental AnalyticalJerkConstrainedSmoother::applyLateralAccelerationFilter(
  TrajectoryExperimental & trajectory, [[maybe_unused]] const double v0,
  [[maybe_unused]] const double a0, [[maybe_unused]] const bool enable_smooth_limit,
  const double input_distance_interval) const
{
  const auto curvature_v =
    trajectory_utils::calcTrajectoryCurvatureFrom3Points(trajectory, input_distance_interval);

  const auto lateral_acceleration_velocity_square_ratio_limits =
    computeLateralAccelerationVelocitySquareRatioLimits();

  const double before_decel_dist = base_param_.decel_distance_before_curve;
  const double after_decel_dist = base_param_.decel_distance_after_curve;

  // calculate and apply velocity limits based on curvature
  const auto velocity_data = trajectory.longitudinal_velocity_mps().get_data();
  const auto [bases, velocities] = velocity_data;

  std::vector<std::pair<double, double>> filtered_points;  // arc_length and velocity
  for (size_t i = 0; i < bases.size(); ++i) {
    double curvature = 0.0;

    const double arc_length_i = bases.at(i);
    const double start_arc = std::max(0.0, arc_length_i - before_decel_dist);
    const double end_arc = std::min(trajectory.length(), arc_length_i + after_decel_dist);

    for (size_t j = 0; j < curvature_v.size(); ++j) {
      const double arc_length_j = bases.at(j);  // Use bases instead of trajectory.arc_length(j)
      if (arc_length_j >= start_arc && arc_length_j <= end_arc) {
        curvature = std::max(curvature, std::fabs(curvature_v.at(j)));
      }
    }

    double v_curvature_max = computeVelocityLimitFromLateralAcc(
      curvature, lateral_acceleration_velocity_square_ratio_limits);
    v_curvature_max = std::max(v_curvature_max, base_param_.min_curve_velocity);

    if (velocities.at(i) > v_curvature_max) {
      trajectory.longitudinal_velocity_mps().range(arc_length_i, arc_length_i).set(v_curvature_max);
      filtered_points.push_back({arc_length_i, v_curvature_max});
    }
  }

  // keep constant velocity while turning
  const double dist_threshold = smoother_param_.latacc.constant_velocity_dist_threshold;
  std::vector<std::tuple<double, double, double>>
    latacc_filtered_ranges;  // start_arc, end_arc, velocity

  if (!filtered_points.empty()) {
    double start_arc = filtered_points.front().first;
    double end_arc = start_arc;
    double min_latacc_velocity = filtered_points.front().second;

    for (size_t i = 1; i < filtered_points.size(); ++i) {
      const double current_arc = filtered_points.at(i).first;
      const double velocity = filtered_points.at(i).second;
      const double arc_dist = current_arc - end_arc;

      if (arc_dist < dist_threshold) {
        end_arc = current_arc;
        min_latacc_velocity = std::min(velocity, min_latacc_velocity);
      } else {
        latacc_filtered_ranges.emplace_back(start_arc, end_arc, min_latacc_velocity);
        start_arc = current_arc;
        end_arc = current_arc;
        min_latacc_velocity = velocity;
      }
    }
    latacc_filtered_ranges.emplace_back(start_arc, end_arc, min_latacc_velocity);
  }

  // apply constant velocity within filtered ranges
  for (const auto & lat_acc_filtered_range : latacc_filtered_ranges) {
    const double start_arc = std::get<0>(lat_acc_filtered_range);
    const double end_arc = std::get<1>(lat_acc_filtered_range);
    const double filtered_min_latacc_velocity = std::get<2>(lat_acc_filtered_range);

    if (smoother_param_.latacc.enable_constant_velocity_while_turning) {
      trajectory.longitudinal_velocity_mps()
        .range(start_arc, end_arc)
        .set(filtered_min_latacc_velocity);
    }
  }

  return trajectory;
}

bool AnalyticalJerkConstrainedSmoother::searchDecelTargetIndices(
  const TrajectoryPoints & trajectory, const size_t closest_index,
  std::vector<std::pair<size_t, double>> & decel_target_indices) const
{
  const double ep = -0.00001;
  const size_t start_index = std::max<size_t>(1, closest_index);
  std::vector<std::pair<size_t, double>> tmp_indices;
  for (size_t i = start_index; i < trajectory.size() - 1; ++i) {
    const double dv_before =
      trajectory.at(i).longitudinal_velocity_mps - trajectory.at(i - 1).longitudinal_velocity_mps;
    const double dv_after =
      trajectory.at(i + 1).longitudinal_velocity_mps - trajectory.at(i).longitudinal_velocity_mps;
    if (dv_before < ep && dv_after > ep) {
      tmp_indices.emplace_back(i, trajectory.at(i).longitudinal_velocity_mps);
    }
  }

  const unsigned int i = trajectory.size() - 1;
  const double dv_before =
    trajectory.at(i).longitudinal_velocity_mps - trajectory.at(i - 1).longitudinal_velocity_mps;
  if (dv_before < ep) {
    tmp_indices.emplace_back(i, trajectory.at(i).longitudinal_velocity_mps);
  }

  if (!tmp_indices.empty()) {
    for (unsigned int j = 0; j < tmp_indices.size() - 1; ++j) {
      const size_t index_err = 10;
      if (
        (tmp_indices.at(j + 1).first - tmp_indices.at(j).first < index_err) &&
        (tmp_indices.at(j + 1).second < tmp_indices.at(j).second)) {
        continue;
      }

      decel_target_indices.emplace_back(tmp_indices.at(j).first, tmp_indices.at(j).second);
    }
  }
  if (!tmp_indices.empty()) {
    decel_target_indices.emplace_back(tmp_indices.back().first, tmp_indices.back().second);
  }
  return true;
}

bool AnalyticalJerkConstrainedSmoother::searchDecelTargetIndices(
  const TrajectoryExperimental & trajectory, const double closest_distance,
  std::vector<std::pair<double, double>> & decel_target_indices) const
{
  const double ep = -0.00001;
  const double start_distance = std::max(0.0, closest_distance);
  const double traj_length = trajectory.length();

  const auto [bases, velocities] = trajectory.longitudinal_velocity_mps().get_data();

  if (velocities.size() < 2) {
    return true;
  }

  size_t start_idx = 0;
  for (size_t i = 0; i < bases.size(); ++i) {
    if (bases.at(i) >= start_distance) {
      start_idx = i;
      break;
    }
  }

  std::vector<std::pair<double, double>> tmp_indices;

  for (const auto [curr_distance, next_distance, curr_vel, next_vel] : ranges::views::zip(
         bases | ranges::views::drop(start_idx), bases | ranges::views::drop(start_idx + 1),
         velocities | ranges::views::drop(start_idx),
         velocities | ranges::views::drop(start_idx + 1))) {
    if (curr_distance > traj_length) {
      break;
    }

    const double dv_before = (start_idx > 0) ? (curr_vel - velocities.at(start_idx - 1)) : 0.0;
    const double dv_after = next_vel - curr_vel;

    if (dv_before < ep && dv_after > ep) {
      const double valley_distance =
        curr_distance - curr_vel * (next_distance - curr_distance) / (next_vel - curr_vel);
      tmp_indices.emplace_back(valley_distance, curr_vel);
    }
  }

  const size_t last_idx = velocities.size() - 1;
  if (last_idx > 0) {
    const double dv_before_last = velocities.at(last_idx) - velocities.at(last_idx - 1);
    if (dv_before_last < ep) {
      tmp_indices.emplace_back(bases.at(last_idx), velocities.at(last_idx));
    }
  }

  const double arc_dist_threshold = 10.0;
  if (!tmp_indices.empty()) {
    for (unsigned int j = 0; j < tmp_indices.size() - 1; ++j) {
      const double arc_dist = tmp_indices.at(j + 1).first - tmp_indices.at(j).first;
      if (
        (arc_dist < arc_dist_threshold) &&
        (tmp_indices.at(j + 1).second < tmp_indices.at(j).second)) {
        continue;
      }

      decel_target_indices.emplace_back(tmp_indices.at(j).first, tmp_indices.at(j).second);
    }
  }
  if (!tmp_indices.empty()) {
    decel_target_indices.emplace_back(tmp_indices.back().first, tmp_indices.back().second);
  }
  return true;
}

bool AnalyticalJerkConstrainedSmoother::applyForwardJerkFilter(
  const TrajectoryPoints & base_trajectory, const size_t start_index, const double initial_vel,
  const double initial_acc, const Param & params, TrajectoryPoints & output_trajectory) const
{
  output_trajectory.at(start_index).longitudinal_velocity_mps = initial_vel;
  output_trajectory.at(start_index).acceleration_mps2 = initial_acc;

  for (size_t i = start_index + 1; i < base_trajectory.size(); ++i) {
    const double prev_vel = output_trajectory.at(i - 1).longitudinal_velocity_mps;
    const double ds =
      autoware_utils_geometry::calc_distance2d(base_trajectory.at(i - 1), base_trajectory.at(i));
    const double dt = ds / std::max(prev_vel, 1.0);

    const double prev_acc = output_trajectory.at(i - 1).acceleration_mps2;
    const double curr_vel = std::max(prev_vel + prev_acc * dt, 0.0);

    const double error_vel = base_trajectory.at(i).longitudinal_velocity_mps - curr_vel;
    const double fb_acc = params.forward.kp * error_vel;
    const double limited_acc =
      std::max(params.forward.min_acc, std::min(params.forward.max_acc, fb_acc));
    const double fb_jerk = (limited_acc - prev_acc) / dt;
    const double limited_jerk =
      std::max(params.forward.min_jerk, std::min(params.forward.max_jerk, fb_jerk));

    const double curr_acc = prev_acc + limited_jerk * dt;

    output_trajectory.at(i).longitudinal_velocity_mps = curr_vel;
    output_trajectory.at(i).acceleration_mps2 = curr_acc;
  }

  return true;
}

bool AnalyticalJerkConstrainedSmoother::applyForwardJerkFilter(
  const TrajectoryExperimental & base_trajectory, const double start_distance,
  const double initial_vel, const double initial_acc, const Param & params,
  TrajectoryExperimental & output_trajectory) const
{
  const auto [bases, velocities] = base_trajectory.longitudinal_velocity_mps().get_data();

  if (bases.empty() || velocities.empty()) {
    return false;
  }

  const double traj_length = output_trajectory.length();

  output_trajectory.longitudinal_velocity_mps()
    .range(start_distance, start_distance)
    .set(initial_vel);

  double prev_acc = initial_acc;
  double prev_vel = initial_vel;

  for (const auto [curr_distance, next_distance, curr_base_vel, next_base_vel] : ranges::views::zip(
         bases, bases | ranges::views::drop(1), velocities, velocities | ranges::views::drop(1))) {
    if (curr_distance < start_distance) {
      continue;
    }

    if (curr_distance >= traj_length) {
      break;
    }

    const double ds = next_distance - curr_distance;
    if (ds <= 0.0) continue;

    const double dt = ds / std::max(prev_vel, 1.0);

    const double curr_vel_predicted = std::max(prev_vel + prev_acc * dt, 0.0);
    const double error_vel = curr_base_vel - curr_vel_predicted;

    const double fb_acc = params.forward.kp * error_vel;
    const double limited_acc =
      std::max(params.forward.min_acc, std::min(params.forward.max_acc, fb_acc));
    const double fb_jerk = (limited_acc - prev_acc) / dt;
    const double limited_jerk =
      std::max(params.forward.min_jerk, std::min(params.forward.max_jerk, fb_jerk));

    const double curr_acc = prev_acc + limited_jerk * dt;

    output_trajectory.longitudinal_velocity_mps()
      .range(next_distance, next_distance)
      .set(curr_vel_predicted);

    prev_acc = curr_acc;
    prev_vel = curr_vel_predicted;
  }

  return true;
}

bool AnalyticalJerkConstrainedSmoother::applyBackwardDecelFilter(
  const std::vector<size_t> & start_indices, const size_t decel_target_index,
  const double decel_target_vel, const Param & params, TrajectoryPoints & output_trajectory) const
{
  const double ep = 0.001;

  double output_planning_jerk = -100.0;
  size_t output_start_index = 0;
  std::vector<double> output_dist_to_target;
  int output_type;
  std::vector<double> output_times;

  for (size_t start_index : start_indices) {
    double dist = 0.0;
    std::vector<double> dist_to_target(output_trajectory.size(), 0);
    dist_to_target.at(decel_target_index) = dist;
    for (size_t i = start_index; i < decel_target_index; ++i) {
      if (output_trajectory.at(i).longitudinal_velocity_mps >= decel_target_vel) {
        start_index = i;
        break;
      }
    }
    for (size_t i = decel_target_index; i > start_index; --i) {
      dist += autoware_utils_geometry::calc_distance2d(
        output_trajectory.at(i - 1), output_trajectory.at(i));
      dist_to_target.at(i - 1) = dist;
    }

    RCLCPP_DEBUG(logger_, "Check enough dist to decel. start_index: %ld", start_index);
    double planning_jerk;
    int type;
    std::vector<double> times;
    double stop_dist;
    bool is_enough_dist = false;
    for (planning_jerk = params.backward.start_jerk; planning_jerk > params.backward.min_jerk - ep;
         planning_jerk += params.backward.span_jerk) {
      if (calcEnoughDistForDecel(
            output_trajectory, start_index, decel_target_vel, planning_jerk, params, dist_to_target,
            is_enough_dist, type, times, stop_dist)) {
        break;
      }
    }

    if (!is_enough_dist) {
      RCLCPP_DEBUG(logger_, "Distance is not enough for decel with all jerk condition");
      continue;
    }

    if (planning_jerk >= output_planning_jerk) {
      output_planning_jerk = planning_jerk;
      output_start_index = start_index;
      output_dist_to_target = dist_to_target;
      output_type = type;
      output_times = times;
      RCLCPP_DEBUG(
        logger_, "Update planning jerk: %f, start_index: %ld", planning_jerk, start_index);
    }
  }

  if (output_planning_jerk == -100.0) {
    RCLCPP_DEBUG(
      logger_,
      "Distance is not enough for decel with all jerk and start index "
      "condition");
    return false;
  }

  RCLCPP_DEBUG(logger_, "Search decel start index");
  size_t decel_start_index = output_start_index;
  if (output_planning_jerk == params.backward.start_jerk) {
    for (size_t i = decel_target_index - 1; i >= output_start_index; --i) {
      bool is_enough_dist = false;
      double stop_dist;
      if (calcEnoughDistForDecel(
            output_trajectory, i, decel_target_vel, output_planning_jerk, params,
            output_dist_to_target, is_enough_dist, output_type, output_times, stop_dist)) {
        decel_start_index = i;
        break;
      }
    }
  }

  RCLCPP_DEBUG(
    logger_,
    "Apply filter. decel_start_index: %ld, target_vel: %f, "
    "planning_jerk: %f, type: %d, times: %s",
    decel_start_index, decel_target_vel, output_planning_jerk, output_type,
    strTimes(output_times).c_str());
  if (!applyDecelVelocityFilter(
        decel_start_index, decel_target_vel, output_planning_jerk, params, output_type,
        output_times, output_trajectory)) {
    RCLCPP_DEBUG(
      logger_,
      "[applyDecelVelocityFilter] dist is enough, but fail to plan backward decel velocity");
    return false;
  }

  return true;
}

bool AnalyticalJerkConstrainedSmoother::applyBackwardDecelFilter(
  const std::vector<double> & start_distances, const double decel_target_distance,
  const double decel_target_vel, const Param & params,
  TrajectoryExperimental & output_trajectory) const
{
  const double ep = 0.001;

  double output_planning_jerk = -100.0;
  double output_start_distance = 0.0;
  double output_dist_to_target = 0.0;
  int output_type;
  std::vector<double> output_times;

  for (double start_distance : start_distances) {
    double dist = 0.0;

    double actual_start_distance = start_distance;
    const auto [bases, velocities] = output_trajectory.longitudinal_velocity_mps().get_data();

    for (const auto [curr_arc, next_arc, curr_vel, next_vel] : ranges::views::zip(
           bases, bases | ranges::views::drop(1), velocities,
           velocities | ranges::views::drop(1))) {
      if (curr_arc >= start_distance && next_arc <= decel_target_distance) {
        if (
          (curr_vel >= decel_target_vel && next_vel <= decel_target_vel) ||
          (curr_vel <= decel_target_vel && next_vel >= decel_target_vel)) {
          if (std::abs(next_vel - curr_vel) > 1e-6) {
            actual_start_distance = curr_arc + (decel_target_vel - curr_vel) *
                                                 (next_arc - curr_arc) / (next_vel - curr_vel);
          } else {
            actual_start_distance = curr_arc;
          }
          break;
        }
      }
    }

    for (int i = static_cast<int>(bases.size()) - 1; i > 0; --i) {
      const double curr_arc = bases.at(i);
      const double prev_arc = bases.at(i - 1);

      if (curr_arc <= decel_target_distance && prev_arc <= decel_target_distance) {
        dist += curr_arc - prev_arc;
      }

      if (curr_arc <= actual_start_distance) {
        break;
      }
    }

    RCLCPP_DEBUG(logger_, "Check enough dist to decel. start_distance: %f", actual_start_distance);
    double planning_jerk;
    int type;
    std::vector<double> times;
    double stop_dist;
    bool is_enough_dist = false;
    for (planning_jerk = params.backward.start_jerk; planning_jerk > params.backward.min_jerk - ep;
         planning_jerk += params.backward.span_jerk) {
      if (calcEnoughDistForDecel(
            output_trajectory, actual_start_distance, decel_target_vel, planning_jerk, params, dist,
            is_enough_dist, type, times, stop_dist)) {
        break;
      }
    }

    if (!is_enough_dist) {
      RCLCPP_DEBUG(logger_, "Distance is not enough for decel with all jerk condition");
      continue;
    }

    if (planning_jerk >= output_planning_jerk) {
      output_planning_jerk = planning_jerk;
      output_start_distance = actual_start_distance;
      output_dist_to_target = dist;
      output_type = type;
      output_times = times;
      RCLCPP_DEBUG(
        logger_, "Update planning jerk: %f, start_distance: %f", planning_jerk,
        actual_start_distance);
    }
  }

  if (output_planning_jerk == -100.0) {
    RCLCPP_DEBUG(
      logger_,
      "Distance is not enough for decel with all jerk and start distance "
      "condition");
    return false;
  }

  RCLCPP_DEBUG(logger_, "Search decel start distance");
  double decel_start_distance = output_start_distance;
  if (output_planning_jerk == params.backward.start_jerk) {
    const auto [bases, velocities] = output_trajectory.longitudinal_velocity_mps().get_data();
    for (int i = static_cast<int>(bases.size()) - 1; i >= 0; --i) {
      const double test_distance = bases.at(i);
      if (test_distance >= output_start_distance && test_distance < decel_target_distance) {
        bool is_enough_dist = false;
        double stop_dist;
        if (calcEnoughDistForDecel(
              output_trajectory, test_distance, decel_target_vel, output_planning_jerk, params,
              output_dist_to_target, is_enough_dist, output_type, output_times, stop_dist)) {
          decel_start_distance = test_distance;
        }
      }
    }
  }

  RCLCPP_DEBUG(
    logger_,
    "Apply filter. decel_start_distance: %f, target_vel: %f, "
    "planning_jerk: %f, type: %d, times: %s",
    decel_start_distance, decel_target_vel, output_planning_jerk, output_type,
    strTimes(output_times).c_str());
  if (!applyDecelVelocityFilter(
        decel_start_distance, decel_target_vel, output_planning_jerk, params, output_type,
        output_times, output_trajectory)) {
    RCLCPP_DEBUG(
      logger_,
      "[applyDecelVelocityFilter] dist is enough, but fail to plan backward decel velocity");
    return false;
  }

  return true;
}

bool AnalyticalJerkConstrainedSmoother::calcEnoughDistForDecel(
  const TrajectoryPoints & trajectory, const size_t start_index, const double decel_target_vel,
  const double planning_jerk, const Param & params, const std::vector<double> & dist_to_target,
  bool & is_enough_dist, int & type, std::vector<double> & times, double & stop_dist) const
{
  const double v0 = trajectory.at(start_index).longitudinal_velocity_mps;
  const double a0 = trajectory.at(start_index).acceleration_mps2;
  const double jerk_acc = std::abs(planning_jerk);
  const double jerk_dec = planning_jerk;
  auto calcMinAcc = [&params](const double planning_jerk) {
    if (planning_jerk < params.backward.min_jerk_mild_stop) {
      return params.backward.min_acc;
    }
    return params.backward.min_acc_mild_stop;
  };
  const double min_acc = calcMinAcc(planning_jerk);
  type = 0;
  times.clear();
  stop_dist = 0.0;

  if (!analytical_velocity_planning_utils::calcStopDistWithJerkAndAccConstraints(
        v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type, times, stop_dist)) {
    return false;
  }

  const double allowed_dist = dist_to_target.at(start_index);
  if (0.0 <= stop_dist && stop_dist <= allowed_dist) {
    RCLCPP_DEBUG(
      logger_,
      "Distance is enough. v0: %f, a0: %f, jerk: %f, stop_dist: %f, "
      "allowed_dist: %f",
      v0, a0, planning_jerk, stop_dist, allowed_dist);
    is_enough_dist = true;
    return true;
  }
  RCLCPP_DEBUG(
    logger_,
    "Distance is not enough. v0: %f, a0: %f, jerk: %f, stop_dist: %f, "
    "allowed_dist: %f",
    v0, a0, planning_jerk, stop_dist, allowed_dist);
  return false;
}

bool AnalyticalJerkConstrainedSmoother::calcEnoughDistForDecel(
  const TrajectoryExperimental & trajectory, const double start_distance,
  const double decel_target_vel, const double planning_jerk, const Param & params,
  const double dist_to_target, bool & is_enough_dist, int & type, std::vector<double> & times,
  double & stop_dist) const
{
  const double v0 = trajectory.longitudinal_velocity_mps().compute(start_distance);
  const double a0 = trajectory.acceleration_mps2().compute(start_distance);
  const double jerk_acc = std::abs(planning_jerk);
  const double jerk_dec = planning_jerk;
  auto calcMinAcc = [&params](const double planning_jerk) {
    if (planning_jerk < params.backward.min_jerk_mild_stop) {
      return params.backward.min_acc;
    }
    return params.backward.min_acc_mild_stop;
  };
  const double min_acc = calcMinAcc(planning_jerk);
  type = 0;
  times.clear();
  stop_dist = 0.0;

  if (!analytical_velocity_planning_utils::calcStopDistWithJerkAndAccConstraints(
        v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type, times, stop_dist)) {
    return false;
  }

  if (0.0 <= stop_dist && stop_dist <= dist_to_target) {
    RCLCPP_DEBUG(
      logger_,
      "Distance is enough. v0: %f, a0: %f, jerk: %f, stop_dist: %f, "
      "allowed_dist: %f",
      v0, a0, planning_jerk, stop_dist, dist_to_target);
    is_enough_dist = true;
    return true;
  }
  RCLCPP_DEBUG(
    logger_,
    "Distance is not enough. v0: %f, a0: %f, jerk: %f, stop_dist: %f, "
    "allowed_dist: %f",
    v0, a0, planning_jerk, stop_dist, dist_to_target);
  return false;
}

bool AnalyticalJerkConstrainedSmoother::applyDecelVelocityFilter(
  const size_t decel_start_index, const double decel_target_vel, const double planning_jerk,
  const Param & params, const int type, const std::vector<double> & times,
  TrajectoryPoints & output_trajectory) const
{
  const double v0 = output_trajectory.at(decel_start_index).longitudinal_velocity_mps;
  const double a0 = output_trajectory.at(decel_start_index).acceleration_mps2;
  const double jerk_acc = std::abs(planning_jerk);
  const double jerk_dec = planning_jerk;
  auto calcMinAcc = [&params](const double planning_jerk) {
    if (planning_jerk < params.backward.min_jerk_mild_stop) {
      return params.backward.min_acc;
    }
    return params.backward.min_acc_mild_stop;
  };
  const double min_acc = calcMinAcc(planning_jerk);

  if (!analytical_velocity_planning_utils::calcStopVelocityWithConstantJerkAccLimit(
        v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type, times, decel_start_index,
        output_trajectory)) {
    return false;
  }

  return true;
}

bool AnalyticalJerkConstrainedSmoother::applyDecelVelocityFilter(
  const double decel_start_distance, const double decel_target_vel, const double planning_jerk,
  const Param & params, const int type, const std::vector<double> & times,
  TrajectoryExperimental & output_trajectory) const
{
  const double v0 = output_trajectory.longitudinal_velocity_mps().compute(decel_start_distance);
  const double a0 = output_trajectory.acceleration_mps2().compute(decel_start_distance);
  const double jerk_acc = std::abs(planning_jerk);
  const double jerk_dec = planning_jerk;
  auto calcMinAcc = [&params](const double planning_jerk) {
    if (planning_jerk < params.backward.min_jerk_mild_stop) {
      return params.backward.min_acc;
    }
    return params.backward.min_acc_mild_stop;
  };
  const double min_acc = calcMinAcc(planning_jerk);

  if (!analytical_velocity_planning_utils::calcStopVelocityWithConstantJerkAccLimit(
        v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type, times, decel_start_distance,
        output_trajectory)) {
    return false;
  }

  return true;
}

std::string AnalyticalJerkConstrainedSmoother::strTimes(const std::vector<double> & times) const
{
  std::stringstream ss;
  unsigned int i = 0;
  for (double time : times) {
    ss << "time[" << i << "] = " << time << ", ";
    i++;
  }
  return ss.str();
}

std::string AnalyticalJerkConstrainedSmoother::strStartIndices(
  const std::vector<size_t> & start_indices) const
{
  std::stringstream ss;
  for (size_t i = 0; i < start_indices.size(); ++i) {
    if (i != (start_indices.size() - 1)) {
      ss << start_indices.at(i) << ", ";
    } else {
      ss << start_indices.at(i);
    }
  }
  return ss.str();
}

}  // namespace autoware::velocity_smoother
