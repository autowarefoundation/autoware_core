// Copyright 2022 Autoware Foundation
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

#ifndef HYPER_PARAMETERS_HPP_
#define HYPER_PARAMETERS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <string>

namespace autoware::ekf_localizer
{

class HyperParameters
{
public:
  // Additive seam: a default constructor so the parameters can be hand-set in a unit test
  // without a live rclcpp::Node. The fields below are intentionally non-const so a test can
  // populate them directly. The node-based constructor still fully initializes every field.
  //
  // The in-class member initializers below give the default-constructed instance SAFE, non-zero
  // values for every field that EKFModule relies on at construction time. In particular
  // extend_state_step is set to a realistic >= 1 value: EKFModule sizes accumulated_delay_times_
  // as std::vector<double>(extend_state_step, 1.0E15) and find_closest_delay_time_index() reads
  // accumulated_delay_times_.back(). A misconfigured extend_state_step == 0 would leave that table
  // empty; find_closest_delay_time_index() now guards the empty table (returns 0) instead of
  // dereferencing .back(), so the degenerate configuration no longer triggers undefined behaviour.
  // The other sizing/rate defaults (ekf_rate/ekf_dt, smoothing steps, queue sizes, gate distances,
  // process/filter noise) are likewise non-zero so a default-constructed instance is immediately
  // usable.
  HyperParameters() = default;

  explicit HyperParameters(rclcpp::Node * node)
  : show_debug_info(node->declare_parameter<bool>("node.show_debug_info")),
    ekf_rate(node->declare_parameter<double>("node.predict_frequency")),
    ekf_dt(1.0 / std::max(ekf_rate, 0.1)),
    tf_rate_(node->declare_parameter<double>("node.tf_rate")),
    enable_yaw_bias_estimation(node->declare_parameter<bool>("node.enable_yaw_bias_estimation")),
    extend_state_step(node->declare_parameter<int>("node.extend_state_step")),
    pose_frame_id(node->declare_parameter<std::string>("misc.pose_frame_id")),
    pose_additional_delay(
      node->declare_parameter<double>("pose_measurement.pose_additional_delay")),
    pose_gate_dist(node->declare_parameter<double>("pose_measurement.pose_gate_dist")),
    pose_smoothing_steps(node->declare_parameter<int>("pose_measurement.pose_smoothing_steps")),
    max_pose_queue_size(node->declare_parameter<int>("pose_measurement.max_pose_queue_size")),
    twist_additional_delay(
      node->declare_parameter<double>("twist_measurement.twist_additional_delay")),
    twist_gate_dist(node->declare_parameter<double>("twist_measurement.twist_gate_dist")),
    twist_smoothing_steps(node->declare_parameter<int>("twist_measurement.twist_smoothing_steps")),
    max_twist_queue_size(node->declare_parameter<int>("twist_measurement.max_twist_queue_size")),
    proc_stddev_vx_c(node->declare_parameter<double>("process_noise.proc_stddev_vx_c")),
    proc_stddev_wz_c(node->declare_parameter<double>("process_noise.proc_stddev_wz_c")),
    proc_stddev_yaw_c(node->declare_parameter<double>("process_noise.proc_stddev_yaw_c")),
    z_filter_proc_dev(
      node->declare_parameter<double>("simple_1d_filter_parameters.z_filter_proc_dev")),
    roll_filter_proc_dev(
      node->declare_parameter<double>("simple_1d_filter_parameters.roll_filter_proc_dev")),
    pitch_filter_proc_dev(
      node->declare_parameter<double>("simple_1d_filter_parameters.pitch_filter_proc_dev")),
    pose_no_update_count_threshold_warn(
      node->declare_parameter<int>("diagnostics.pose_no_update_count_threshold_warn")),
    pose_no_update_count_threshold_error(
      node->declare_parameter<int>("diagnostics.pose_no_update_count_threshold_error")),
    twist_no_update_count_threshold_warn(
      node->declare_parameter<int>("diagnostics.twist_no_update_count_threshold_warn")),
    twist_no_update_count_threshold_error(
      node->declare_parameter<int>("diagnostics.twist_no_update_count_threshold_error")),
    ellipse_scale(node->declare_parameter<double>("diagnostics.ellipse_scale")),
    error_ellipse_size(node->declare_parameter<double>("diagnostics.error_ellipse_size")),
    warn_ellipse_size(node->declare_parameter<double>("diagnostics.warn_ellipse_size")),
    error_ellipse_size_lateral_direction(
      node->declare_parameter<double>("diagnostics.error_ellipse_size_lateral_direction")),
    warn_ellipse_size_lateral_direction(
      node->declare_parameter<double>("diagnostics.warn_ellipse_size_lateral_direction")),
    diagnostics_publish_frequency(
      node->declare_parameter<double>("diagnostics.diagnostics_publish_frequency")),
    diagnostics_publish_period(1.0 / diagnostics_publish_frequency),
    threshold_observable_velocity_mps(
      node->declare_parameter<double>("misc.threshold_observable_velocity_mps"))
  {
  }

  // NOTE: these members are non-const so the default constructor (additive test seam) can set
  // them. The node-based constructor still initializes all of them in its member initializer list.
  bool show_debug_info{false};
  double ekf_rate{50.0};      // ekf update frequency = predict_frequency [Hz]
  double ekf_dt{1.0 / 50.0};  // ekf update period [s]
  double tf_rate_{10.0};
  bool enable_yaw_bias_estimation{true};
  // Sizes accumulated_delay_times_ to this length; 0 leaves the table empty, which
  // find_closest_delay_time_index() guards against (see ekf_module.cpp).
  size_t extend_state_step{50};
  std::string pose_frame_id{"map"};
  double pose_additional_delay{0.0};
  double pose_gate_dist{10000.0};
  size_t pose_smoothing_steps{5};
  size_t max_pose_queue_size{5};
  double twist_additional_delay{0.0};
  double twist_gate_dist{10000.0};
  size_t twist_smoothing_steps{2};
  size_t max_twist_queue_size{5};
  double proc_stddev_vx_c{10.0};    //!< @brief  vx process noise
  double proc_stddev_wz_c{5.0};     //!< @brief  wz process noise
  double proc_stddev_yaw_c{0.005};  //!< @brief  yaw process noise
  double z_filter_proc_dev{1.0};
  double roll_filter_proc_dev{0.01};
  double pitch_filter_proc_dev{0.01};
  size_t pose_no_update_count_threshold_warn{0};
  size_t pose_no_update_count_threshold_error{0};
  size_t twist_no_update_count_threshold_warn{0};
  size_t twist_no_update_count_threshold_error{0};
  double ellipse_scale{0.0};
  double error_ellipse_size{0.0};
  double warn_ellipse_size{0.0};
  double error_ellipse_size_lateral_direction{0.0};
  double warn_ellipse_size_lateral_direction{0.0};
  double diagnostics_publish_frequency{0.0};  //!< @brief diagnostics publish frequency [Hz]
  double diagnostics_publish_period{0.0};     //!< @brief diagnostics publish period [s]

  double threshold_observable_velocity_mps{0.0};
};

}  // namespace autoware::ekf_localizer

#endif  // HYPER_PARAMETERS_HPP_
