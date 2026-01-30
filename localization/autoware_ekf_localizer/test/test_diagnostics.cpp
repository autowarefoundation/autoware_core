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

#include "autoware/ekf_localizer/diagnostics.hpp"
#include "autoware/ekf_localizer/ekf_localizer.hpp"

#include <rclcpp/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::ekf_localizer
{

TEST(TestEkfDiagnostics, check_process_activated)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  bool is_activated = true;
  stat = check_process_activated(is_activated);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  is_activated = false;
  stat = check_process_activated(is_activated);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST(TestEkfDiagnostics, check_set_initialpose)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  bool is_set_initialpose = true;
  stat = check_set_initialpose(is_set_initialpose);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  is_set_initialpose = false;
  stat = check_set_initialpose(is_set_initialpose);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST(TestEkfDiagnostics, check_measurement_updated)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  const std::string measurement_type = "pose";  // not effect for stat.level
  const size_t no_update_count_threshold_warn = 50;
  const size_t no_update_count_threshold_error = 250;

  size_t no_update_count = 0;
  stat = check_measurement_updated(
    measurement_type, no_update_count, no_update_count_threshold_warn,
    no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  no_update_count = 1;
  stat = check_measurement_updated(
    measurement_type, no_update_count, no_update_count_threshold_warn,
    no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  no_update_count = 49;
  stat = check_measurement_updated(
    measurement_type, no_update_count, no_update_count_threshold_warn,
    no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  no_update_count = 50;
  stat = check_measurement_updated(
    measurement_type, no_update_count, no_update_count_threshold_warn,
    no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  no_update_count = 249;
  stat = check_measurement_updated(
    measurement_type, no_update_count, no_update_count_threshold_warn,
    no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  no_update_count = 250;
  stat = check_measurement_updated(
    measurement_type, no_update_count, no_update_count_threshold_warn,
    no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST(TestEkfDiagnostics, check_measurement_queue_size)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  const std::string measurement_type = "pose";  // not effect for stat.level

  size_t queue_size = 0;  // not effect for stat.level
  stat = check_measurement_queue_size(measurement_type, queue_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  queue_size = 1;  // not effect for stat.level
  stat = check_measurement_queue_size(measurement_type, queue_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST(TestEkfDiagnostics, check_measurement_delay_gate)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  const std::string measurement_type = "pose";  // not effect for stat.level
  const double delay_time = 0.1;                // not effect for stat.level
  const double delay_time_threshold = 1.0;      // not effect for stat.level

  bool is_passed_delay_gate = true;
  stat = check_measurement_delay_gate(
    measurement_type, is_passed_delay_gate, delay_time, delay_time_threshold);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  is_passed_delay_gate = false;
  stat = check_measurement_delay_gate(
    measurement_type, is_passed_delay_gate, delay_time, delay_time_threshold);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST(TestEkfDiagnostics, check_measurement_mahalanobis_gate)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  const std::string measurement_type = "pose";        // not effect for stat.level
  const double mahalanobis_distance = 0.1;            // not effect for stat.level
  const double mahalanobis_distance_threshold = 1.0;  // not effect for stat.level

  bool is_passed_mahalanobis_gate = true;
  stat = check_measurement_mahalanobis_gate(
    measurement_type, is_passed_mahalanobis_gate, mahalanobis_distance,
    mahalanobis_distance_threshold);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  is_passed_mahalanobis_gate = false;
  stat = check_measurement_mahalanobis_gate(
    measurement_type, is_passed_mahalanobis_gate, mahalanobis_distance,
    mahalanobis_distance_threshold);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST(TestLocalizationErrorMonitorDiagnostics, merge_diagnostic_status)
{
  diagnostic_msgs::msg::DiagnosticStatus merged_stat;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> stat_array(2);

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat_array.at(0).message = "OK";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat_array.at(1).message = "OK";
  merged_stat = merge_diagnostic_status(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(merged_stat.message, "OK");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  stat_array.at(0).message = "WARN0";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat_array.at(1).message = "OK";
  merged_stat = merge_diagnostic_status(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(merged_stat.message, "WARN0");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat_array.at(0).message = "OK";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  stat_array.at(1).message = "WARN1";
  merged_stat = merge_diagnostic_status(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(merged_stat.message, "WARN1");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  stat_array.at(0).message = "WARN0";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  stat_array.at(1).message = "WARN1";
  merged_stat = merge_diagnostic_status(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(merged_stat.message, "WARN0; WARN1");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat_array.at(0).message = "OK";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  stat_array.at(1).message = "ERROR1";
  merged_stat = merge_diagnostic_status(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(merged_stat.message, "ERROR1");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  stat_array.at(0).message = "WARN0";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  stat_array.at(1).message = "ERROR1";
  merged_stat = merge_diagnostic_status(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(merged_stat.message, "WARN0; ERROR1");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  stat_array.at(0).message = "ERROR0";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  stat_array.at(1).message = "ERROR1";
  merged_stat = merge_diagnostic_status(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(merged_stat.message, "ERROR0; ERROR1");
}

class TestEkfDiagnosticsWithNode : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    // Don't shutdown here as other tests might need rclcpp context
  }

  std::shared_ptr<rclcpp::Node> create_test_node(const std::string & node_name)
  {
    // Create a new node - parameters will be declared by HyperParameters constructor
    return std::make_shared<rclcpp::Node>(node_name);
  }
};

// Friend class to access private members of EKFLocalizer for testing
class EKFLocalizerTestSuite : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    // Don't shutdown here as other tests might need rclcpp context
  }

  std::shared_ptr<autoware::ekf_localizer::EKFLocalizer> create_ekf_localizer(
    const std::string & /* node_name */, double diagnostics_publish_period, double ekf_rate)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      // Node parameters
      {"node.show_debug_info", false},
      {"node.enable_yaw_bias_estimation", true},
      {"node.predict_frequency", ekf_rate},
      {"node.tf_rate", 50.0},
      {"node.extend_state_step", 50},
      // Pose measurement parameters
      {"pose_measurement.pose_additional_delay", 0.0},
      {"pose_measurement.pose_measure_uncertainty_time", 0.01},
      {"pose_measurement.pose_smoothing_steps", 5},
      {"pose_measurement.pose_gate_dist", 49.5},
      // Twist measurement parameters
      {"twist_measurement.twist_additional_delay", 0.0},
      {"twist_measurement.twist_smoothing_steps", 2},
      {"twist_measurement.twist_gate_dist", 46.1},
      // Process noise parameters
      {"process_noise.proc_stddev_yaw_c", 0.005},
      {"process_noise.proc_stddev_vx_c", 10.0},
      {"process_noise.proc_stddev_wz_c", 5.0},
      // Simple 1D filter parameters
      {"simple_1d_filter_parameters.z_filter_proc_dev", 5.0},
      {"simple_1d_filter_parameters.roll_filter_proc_dev", 0.1},
      {"simple_1d_filter_parameters.pitch_filter_proc_dev", 0.1},
      // Diagnostics parameters
      {"diagnostics.pose_no_update_count_threshold_warn", 50},
      {"diagnostics.pose_no_update_count_threshold_error", 100},
      {"diagnostics.twist_no_update_count_threshold_warn", 50},
      {"diagnostics.twist_no_update_count_threshold_error", 100},
      {"diagnostics.ellipse_scale", 3.0},
      {"diagnostics.error_ellipse_size", 1.5},
      {"diagnostics.warn_ellipse_size", 1.2},
      {"diagnostics.error_ellipse_size_lateral_direction", 0.3},
      {"diagnostics.warn_ellipse_size_lateral_direction", 0.25},
      {"diagnostics.diagnostics_publish_frequency",
       diagnostics_publish_period > 0.0 ? 1.0 / diagnostics_publish_period : 0.0},
      // Misc parameters
      {"misc.threshold_observable_velocity_mps", 0.0},
      {"misc.pose_frame_id", "map"},
    });
    return std::make_shared<autoware::ekf_localizer::EKFLocalizer>(options);
  }

  // Helper methods to access private members through friend class
  bool should_publish_diagnostics(
    autoware::ekf_localizer::EKFLocalizer * ekf_localizer, const rclcpp::Time & current_time)
  {
    return ekf_localizer->should_publish_diagnostics(current_time);
  }

  void set_diagnostics_publish_counter(
    autoware::ekf_localizer::EKFLocalizer * ekf_localizer, double value)
  {
    ekf_localizer->diagnostics_publish_counter_ = value;
  }

  double get_diagnostics_publish_counter(autoware::ekf_localizer::EKFLocalizer * ekf_localizer)
  {
    return ekf_localizer->diagnostics_publish_counter_;
  }
};

TEST_F(EKFLocalizerTestSuite, should_publish_diagnostics_period_zero)
{
  // Test that should_publish_diagnostics returns true when diagnostics_publish_period <= 0.0
  const double ekf_rate = 100.0;                  // 100 Hz
  const double diagnostics_publish_period = 0.0;  // 0.0 means publish every callback

  auto ekf_localizer =
    create_ekf_localizer("test_should_publish_period_zero", diagnostics_publish_period, ekf_rate);

  rclcpp::Time current_time = ekf_localizer->now();

  // Should always return true when period is 0.0
  EXPECT_TRUE(should_publish_diagnostics(ekf_localizer.get(), current_time));
  EXPECT_TRUE(should_publish_diagnostics(ekf_localizer.get(), current_time));
  EXPECT_TRUE(should_publish_diagnostics(ekf_localizer.get(), current_time));
}

TEST_F(EKFLocalizerTestSuite, should_publish_diagnostics_period_negative)
{
  // Test that should_publish_diagnostics returns true when diagnostics_publish_period < 0.0
  const double ekf_rate = 100.0;                   // 100 Hz
  const double diagnostics_publish_period = -1.0;  // Negative period

  auto ekf_localizer = create_ekf_localizer(
    "test_should_publish_period_negative", diagnostics_publish_period, ekf_rate);

  rclcpp::Time current_time = ekf_localizer->now();

  // Should always return true when period is negative
  EXPECT_TRUE(should_publish_diagnostics(ekf_localizer.get(), current_time));
  EXPECT_TRUE(should_publish_diagnostics(ekf_localizer.get(), current_time));
}

TEST_F(EKFLocalizerTestSuite, should_publish_diagnostics_period_less_than_ekf_dt)
{
  // Test that should_publish_diagnostics returns true when diagnostics_publish_period < ekf_dt
  const double ekf_rate = 100.0;                    // 100 Hz, so ekf_dt = 0.01 seconds
  const double diagnostics_publish_period = 0.005;  // 0.005 seconds < 0.01 seconds (ekf_dt)

  auto ekf_localizer = create_ekf_localizer(
    "test_should_publish_period_less_than_dt", diagnostics_publish_period, ekf_rate);

  rclcpp::Time current_time = ekf_localizer->now();

  // Should always return true when period < ekf_dt
  EXPECT_TRUE(should_publish_diagnostics(ekf_localizer.get(), current_time));
  EXPECT_TRUE(should_publish_diagnostics(ekf_localizer.get(), current_time));
}

TEST_F(EKFLocalizerTestSuite, should_publish_diagnostics_with_counter)
{
  // Test that should_publish_diagnostics uses counter correctly when period > ekf_dt
  const double ekf_rate = 100.0;                  // 100 Hz, so ekf_dt = 0.01 seconds
  const double diagnostics_publish_period = 0.1;  // 0.1 seconds = 10 timer callbacks

  auto ekf_localizer =
    create_ekf_localizer("test_should_publish_with_counter", diagnostics_publish_period, ekf_rate);

  rclcpp::Time current_time = ekf_localizer->now();

  // Calculate expected threshold: ekf_rate * diagnostics_publish_period = 100.0 * 0.1 = 10.0
  const double expected_threshold = ekf_rate * diagnostics_publish_period;

  // Counter starts at 0.0, so first call should return false (0.0 < 10.0)
  EXPECT_FALSE(should_publish_diagnostics(ekf_localizer.get(), current_time));

  // Access private member diagnostics_publish_counter_ through friend class helper methods
  // Set counter to just below threshold
  set_diagnostics_publish_counter(ekf_localizer.get(), expected_threshold - 1.0);
  EXPECT_FALSE(should_publish_diagnostics(ekf_localizer.get(), current_time));

  // Set counter to exactly at threshold
  set_diagnostics_publish_counter(ekf_localizer.get(), expected_threshold);
  EXPECT_TRUE(should_publish_diagnostics(ekf_localizer.get(), current_time));
  // After returning true, counter should be decremented by threshold
  EXPECT_DOUBLE_EQ(get_diagnostics_publish_counter(ekf_localizer.get()), 0.0);

  // Set counter to above threshold
  set_diagnostics_publish_counter(ekf_localizer.get(), expected_threshold + 5.0);
  EXPECT_TRUE(should_publish_diagnostics(ekf_localizer.get(), current_time));
  // Counter should be decremented by threshold, leaving remainder
  EXPECT_DOUBLE_EQ(get_diagnostics_publish_counter(ekf_localizer.get()), 5.0);

  // Test multiple increments
  set_diagnostics_publish_counter(ekf_localizer.get(), 0.0);
  EXPECT_FALSE(should_publish_diagnostics(ekf_localizer.get(), current_time));

  // Simulate 9 timer callbacks (counter = 9.0)
  set_diagnostics_publish_counter(ekf_localizer.get(), 9.0);
  EXPECT_FALSE(should_publish_diagnostics(ekf_localizer.get(), current_time));

  // Simulate 10th timer callback (counter = 10.0, should trigger publish)
  set_diagnostics_publish_counter(ekf_localizer.get(), 10.0);
  EXPECT_TRUE(should_publish_diagnostics(ekf_localizer.get(), current_time));
  EXPECT_DOUBLE_EQ(get_diagnostics_publish_counter(ekf_localizer.get()), 0.0);
}

}  // namespace autoware::ekf_localizer
