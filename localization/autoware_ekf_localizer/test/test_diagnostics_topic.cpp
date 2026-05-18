// Copyright 2023 Autoware Foundation
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

#include "include/ekf_localizer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace autoware::ekf_localizer
{

static rclcpp::NodeOptions make_minimal_ekf_options()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"node.show_debug_info", false},
    {"node.enable_yaw_bias_estimation", true},
    {"node.predict_frequency", 100.0},
    {"node.tf_rate", 50.0},
    {"node.extend_state_step", 50},
    {"misc.pose_frame_id", std::string("map")},
    {"pose_measurement.pose_additional_delay", 0.0},
    {"pose_measurement.pose_gate_dist", 49.5},
    {"pose_measurement.pose_smoothing_steps", 5},
    {"pose_measurement.max_pose_queue_size", 5},
    {"twist_measurement.twist_additional_delay", 0.0},
    {"twist_measurement.twist_gate_dist", 46.1},
    {"twist_measurement.twist_smoothing_steps", 2},
    {"twist_measurement.max_twist_queue_size", 2},
    {"process_noise.proc_stddev_vx_c", 10.0},
    {"process_noise.proc_stddev_wz_c", 5.0},
    {"process_noise.proc_stddev_yaw_c", 0.005},
    {"simple_1d_filter_parameters.z_filter_proc_dev", 5.0},
    {"simple_1d_filter_parameters.roll_filter_proc_dev", 0.1},
    {"simple_1d_filter_parameters.pitch_filter_proc_dev", 0.1},
    {"diagnostics.pose_no_update_count_threshold_warn", 50},
    {"diagnostics.pose_no_update_count_threshold_error", 100},
    {"diagnostics.twist_no_update_count_threshold_warn", 50},
    {"diagnostics.twist_no_update_count_threshold_error", 100},
    {"diagnostics.ellipse_scale", 3.0},
    {"diagnostics.error_ellipse_size", 1.5},
    {"diagnostics.warn_ellipse_size", 1.2},
    {"diagnostics.error_ellipse_size_lateral_direction", 0.3},
    {"diagnostics.warn_ellipse_size_lateral_direction", 0.25},
    {"diagnostics.diagnostics_publish_frequency", 10.0},
    {"misc.threshold_observable_velocity_mps", 0.0},
  });
  return options;
}

TEST(DiagnosticsTopicTest, logs_published_diagnostic_status_names)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto ekf = std::make_shared<EKFLocalizer>(make_minimal_ekf_options());

  size_t message_count = 0;
  auto sub = ekf->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10,
    [&message_count](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
      ++message_count;
      std::cout << "[DiagnosticsTopicTest] /diagnostics message #" << message_count << " — "
                << msg->status.size() << " status(es)\n";
      for (size_t i = 0; i < msg->status.size(); ++i) {
        const auto & st = msg->status[i];
        std::cout << "[DiagnosticsTopicTest]   [" << i << "] name='" << st.name << "'"
                  << " hardware_id='" << st.hardware_id << "'"
                  << " level=" << static_cast<int>(st.level) << " message='" << st.message << "'\n";
      }
      std::cout.flush();
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ekf);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (message_count == 0 && std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    executor.spin_some(std::chrono::milliseconds(20));
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  executor.remove_node(ekf);

  EXPECT_GE(message_count, 1u)
    << "Expected at least one /diagnostics message (timer publishes when not activated)";
}

}  // namespace autoware::ekf_localizer
