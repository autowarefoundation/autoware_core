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

#include "include/hyper_parameters.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <string>

namespace autoware::ekf_localizer
{

HyperParameters load_hyper_parameters(rclcpp::Node * node)
{
  HyperParameters p;
  p.show_debug_info = node->declare_parameter<bool>("node.show_debug_info");
  p.ekf_rate = node->declare_parameter<double>("node.predict_frequency");
  p.ekf_dt = 1.0 / std::max(p.ekf_rate, 0.1);
  p.tf_rate_ = node->declare_parameter<double>("node.tf_rate");
  p.enable_yaw_bias_estimation = node->declare_parameter<bool>("node.enable_yaw_bias_estimation");
  p.extend_state_step = node->declare_parameter<int>("node.extend_state_step");
  p.pose_frame_id = node->declare_parameter<std::string>("misc.pose_frame_id");
  p.pose_additional_delay =
    node->declare_parameter<double>("pose_measurement.pose_additional_delay");
  p.pose_gate_dist = node->declare_parameter<double>("pose_measurement.pose_gate_dist");
  p.pose_smoothing_steps = node->declare_parameter<int>("pose_measurement.pose_smoothing_steps");
  p.max_pose_queue_size = node->declare_parameter<int>("pose_measurement.max_pose_queue_size");
  p.twist_additional_delay =
    node->declare_parameter<double>("twist_measurement.twist_additional_delay");
  p.twist_gate_dist = node->declare_parameter<double>("twist_measurement.twist_gate_dist");
  p.twist_smoothing_steps = node->declare_parameter<int>("twist_measurement.twist_smoothing_steps");
  p.max_twist_queue_size = node->declare_parameter<int>("twist_measurement.max_twist_queue_size");
  p.proc_stddev_vx_c = node->declare_parameter<double>("process_noise.proc_stddev_vx_c");
  p.proc_stddev_wz_c = node->declare_parameter<double>("process_noise.proc_stddev_wz_c");
  p.proc_stddev_yaw_c = node->declare_parameter<double>("process_noise.proc_stddev_yaw_c");
  p.z_filter_proc_dev =
    node->declare_parameter<double>("simple_1d_filter_parameters.z_filter_proc_dev");
  p.roll_filter_proc_dev =
    node->declare_parameter<double>("simple_1d_filter_parameters.roll_filter_proc_dev");
  p.pitch_filter_proc_dev =
    node->declare_parameter<double>("simple_1d_filter_parameters.pitch_filter_proc_dev");
  p.pose_no_update_count_threshold_warn =
    node->declare_parameter<int>("diagnostics.pose_no_update_count_threshold_warn");
  p.pose_no_update_count_threshold_error =
    node->declare_parameter<int>("diagnostics.pose_no_update_count_threshold_error");
  p.twist_no_update_count_threshold_warn =
    node->declare_parameter<int>("diagnostics.twist_no_update_count_threshold_warn");
  p.twist_no_update_count_threshold_error =
    node->declare_parameter<int>("diagnostics.twist_no_update_count_threshold_error");
  p.ellipse_scale = node->declare_parameter<double>("diagnostics.ellipse_scale");
  p.error_ellipse_size = node->declare_parameter<double>("diagnostics.error_ellipse_size");
  p.warn_ellipse_size = node->declare_parameter<double>("diagnostics.warn_ellipse_size");
  p.error_ellipse_size_lateral_direction =
    node->declare_parameter<double>("diagnostics.error_ellipse_size_lateral_direction");
  p.warn_ellipse_size_lateral_direction =
    node->declare_parameter<double>("diagnostics.warn_ellipse_size_lateral_direction");
  p.diagnostics_publish_frequency =
    node->declare_parameter<double>("diagnostics.diagnostics_publish_frequency");
  p.diagnostics_publish_period = 1.0 / p.diagnostics_publish_frequency;
  p.threshold_observable_velocity_mps =
    node->declare_parameter<double>("misc.threshold_observable_velocity_mps");
  return p;
}

}  // namespace autoware::ekf_localizer
