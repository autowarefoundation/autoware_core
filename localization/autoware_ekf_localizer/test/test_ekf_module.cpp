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

#include "include/ekf_module.hpp"
#include "include/hyper_parameters.hpp"
#include "include/state_index.hpp"
#include "include/warning.hpp"

#include <autoware_utils_geometry/msg/covariance.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>

namespace autoware::ekf_localizer
{

namespace
{
// Build a HyperParameters instance with hand-set values, so EKFModule can be exercised without a
// live rclcpp::Node. HyperParameters is a plain data struct (no rclcpp dependency); the test owns
// the values it cares about and value-initializes the rest to zero/empty.
HyperParameters make_params()
{
  HyperParameters params{};  // value-initialize every field to zero/empty
  params.show_debug_info = false;
  params.ekf_rate = 50.0;
  params.ekf_dt = 1.0 / 50.0;
  params.tf_rate_ = 10.0;
  params.enable_yaw_bias_estimation = true;
  params.extend_state_step = 50;
  params.pose_frame_id = "map";
  params.pose_additional_delay = 0.0;
  params.pose_gate_dist = 10000.0;
  params.pose_smoothing_steps = 5;
  params.max_pose_queue_size = 5;
  params.twist_additional_delay = 0.0;
  params.twist_gate_dist = 10000.0;
  params.twist_smoothing_steps = 2;
  params.max_twist_queue_size = 5;
  params.proc_stddev_vx_c = 10.0;
  params.proc_stddev_wz_c = 5.0;
  params.proc_stddev_yaw_c = 0.005;
  params.z_filter_proc_dev = 1.0;
  params.roll_filter_proc_dev = 0.01;
  params.pitch_filter_proc_dev = 0.01;
  return params;
}

std::shared_ptr<EKFModule> make_module(const HyperParameters & params)
{
  // Warning(nullptr) is an explicitly-requested no-op logger (node_ == nullptr).
  auto warning = std::make_shared<Warning>(nullptr);
  return std::make_shared<EKFModule>(warning, params);
}

geometry_msgs::msg::PoseWithCovarianceStamped make_pose(
  const double x, const double y, const double yaw, const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = stamp;
  pose.pose.pose.position.x = x;
  pose.pose.pose.position.y = y;
  pose.pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.pose.orientation.x = q.x();
  pose.pose.pose.orientation.y = q.y();
  pose.pose.pose.orientation.z = q.z();
  pose.pose.pose.orientation.w = q.w();
  pose.pose.covariance.fill(0.0);
  using COV_IDX = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  pose.pose.covariance[COV_IDX::X_X] = 1.0;
  pose.pose.covariance[COV_IDX::Y_Y] = 1.0;
  pose.pose.covariance[COV_IDX::Z_Z] = 1.0;
  pose.pose.covariance[COV_IDX::ROLL_ROLL] = 0.01;
  pose.pose.covariance[COV_IDX::PITCH_PITCH] = 0.01;
  pose.pose.covariance[COV_IDX::YAW_YAW] = 0.01;
  return pose;
}

geometry_msgs::msg::TwistWithCovarianceStamped make_twist(
  const double vx, const double wz, const std::string & frame_id, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TwistWithCovarianceStamped twist;
  twist.header.frame_id = frame_id;
  twist.header.stamp = stamp;
  twist.twist.twist.linear.x = vx;
  twist.twist.twist.angular.z = wz;
  twist.twist.covariance.fill(0.0);
  using COV_IDX = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  twist.twist.covariance[COV_IDX::X_X] = 1.0;
  twist.twist.covariance[COV_IDX::YAW_YAW] = 1.0;
  return twist;
}

geometry_msgs::msg::TransformStamped identity_transform()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.transform.rotation.w = 1.0;
  return transform;
}

}  // namespace

// ---------------------------------------------------------------------------
// find_closest_delay_time_index
// ---------------------------------------------------------------------------
TEST(TestEKFModule, FindClosestDelayTimeIndex)
{
  const auto params = make_params();
  auto module = make_module(params);

  // Build a strictly increasing delay-time table: front becomes 0 and the rest accumulate.
  // After one accumulate_delay_time(dt), the table is [0, 1e15 + dt, 1e15 + dt, ...].
  // Repeatedly accumulating builds a monotonically-increasing prefix.
  const double dt = 0.1;
  for (size_t i = 0; i < params.extend_state_step; ++i) {
    module->accumulate_delay_time(dt);
  }
  // After extend_state_step accumulations the table is [0, dt, 2*dt, ..., (n-1)*dt].

  // target below the first element -> lower_bound at begin -> index 0
  EXPECT_EQ(module->find_closest_delay_time_index(-1.0), 0u);

  // target exactly the first element (0) -> begin -> index 0
  EXPECT_EQ(module->find_closest_delay_time_index(0.0), 0u);

  // closest-of-two: a value between two grid points snaps to the nearer one.
  // grid is 0, 0.1, 0.2, ...; 0.14 is closer to 0.1 (index 1) than 0.2 (index 2).
  EXPECT_EQ(module->find_closest_delay_time_index(0.14), 1u);
  // 0.16 is closer to 0.2 (index 2).
  EXPECT_EQ(module->find_closest_delay_time_index(0.16), 2u);

  // target beyond the last element -> returns size (== extend_state_step).
  const double beyond = static_cast<double>(params.extend_state_step) * dt + 1.0;
  EXPECT_EQ(module->find_closest_delay_time_index(beyond), params.extend_state_step);
}

// Degenerate configuration: extend_state_step == 0 leaves accumulated_delay_times_ empty.
// find_closest_delay_time_index() must not dereference .back() on the empty table; it returns the
// safe index 0 (== size()) for any target instead of crashing. The default of 50 must NOT mask
// this, so the test sets extend_state_step to 0 explicitly.
TEST(TestEKFModule, FindClosestDelayTimeIndexEmptyTable)
{
  HyperParameters params = make_params();
  params.extend_state_step = 0;
  auto module = make_module(params);

  EXPECT_EQ(module->find_closest_delay_time_index(-1.0), 0u);
  EXPECT_EQ(module->find_closest_delay_time_index(0.0), 0u);
  EXPECT_EQ(module->find_closest_delay_time_index(1.0), 0u);
  EXPECT_EQ(module->find_closest_delay_time_index(1.0e15), 0u);
}

// ---------------------------------------------------------------------------
// accumulate_delay_time: copy_backward shift + accumulation
// ---------------------------------------------------------------------------
TEST(TestEKFModule, AccumulateDelayTime)
{
  HyperParameters params = make_params();
  params.extend_state_step = 4;
  auto module = make_module(params);

  // Initial table is filled with 1.0E15. find_closest_delay_time_index uses the table directly,
  // so we can probe its boundary behaviour to characterize the shift/accumulation.
  const double dt = 0.2;

  // First accumulation: front -> 0, others -> 1e15 + dt (still huge).
  module->accumulate_delay_time(dt);
  // Index 0 corresponds to delay 0.
  EXPECT_EQ(module->find_closest_delay_time_index(0.0), 0u);

  // Second accumulation shifts and the second element becomes 0 + dt = dt, the rest stay huge.
  module->accumulate_delay_time(dt);
  // Now table[0] = 0, table[1] = dt, table[2..] huge. A target at dt snaps to index 1.
  EXPECT_EQ(module->find_closest_delay_time_index(dt), 1u);

  // Third accumulation: table[0]=0, table[1]=dt, table[2]=2*dt, table[3] huge.
  module->accumulate_delay_time(dt);
  EXPECT_EQ(module->find_closest_delay_time_index(2.0 * dt), 2u);

  // A target larger than the (still huge) last element returns size().
  EXPECT_EQ(module->find_closest_delay_time_index(2.0e15), params.extend_state_step);
}

// ---------------------------------------------------------------------------
// Simple1DFilter init + update
// ---------------------------------------------------------------------------
TEST(TestSimple1DFilter, InitAndUpdate)
{
  Simple1DFilter filter;
  // Before init, update() must initialize from the first observation.
  filter.update(5.0, 2.0, 0.1);
  EXPECT_DOUBLE_EQ(filter.get_x(), 5.0);
  EXPECT_DOUBLE_EQ(filter.get_var(), 2.0);

  // A subsequent update performs the Kalman blend.
  // With proc_var = 0, var stays 2.0 during prediction.
  // kalman_gain = var / (var + obs_var) = 2 / (2 + 2) = 0.5.
  // x = 5 + 0.5 * (7 - 5) = 6.0; var = (1 - 0.5) * 2 = 1.0.
  filter.update(7.0, 2.0, 0.1);
  EXPECT_DOUBLE_EQ(filter.get_x(), 6.0);
  EXPECT_DOUBLE_EQ(filter.get_var(), 1.0);
}

TEST(TestSimple1DFilter, ProcessVarianceInflatesPrediction)
{
  Simple1DFilter filter;
  filter.set_proc_var(4.0);  // proc_var_x_c_
  filter.init(0.0, 1.0);     // x_ = 0, var_ = 1.0

  // Prediction step: proc_var_x_d = 4.0 * dt^2 = 4.0 * 0.25 = 1.0; var = 1.0 + 1.0 = 2.0.
  // Update step: kalman_gain = 2 / (2 + 2) = 0.5; x = 0 + 0.5 * (10 - 0) = 5.0;
  // var = (1 - 0.5) * 2 = 1.0.
  filter.update(10.0, 2.0, 0.5);
  EXPECT_DOUBLE_EQ(filter.get_x(), 5.0);
  EXPECT_DOUBLE_EQ(filter.get_var(), 1.0);
}

// ---------------------------------------------------------------------------
// compensate_rph_with_delay: zero vs non-zero angular velocity, delta_z correction
// ---------------------------------------------------------------------------
TEST(TestEKFModule, CompensateRphWithDelayZeroAngularVelocity)
{
  const auto params = make_params();
  auto module = make_module(params);

  const rclcpp::Time stamp(100, 0, RCL_ROS_TIME);
  auto pose = make_pose(1.0, 2.0, 0.3, "map", stamp);

  const tf2::Vector3 zero_angular_velocity(0.0, 0.0, 0.0);
  const double delay_time = 0.2;
  const auto compensated =
    module->compensate_rph_with_delay(pose, zero_angular_velocity, delay_time);

  // With zero angular velocity the orientation is unchanged (delta is identity).
  EXPECT_NEAR(compensated.pose.pose.orientation.x, pose.pose.pose.orientation.x, 1e-9);
  EXPECT_NEAR(compensated.pose.pose.orientation.y, pose.pose.pose.orientation.y, 1e-9);
  EXPECT_NEAR(compensated.pose.pose.orientation.z, pose.pose.pose.orientation.z, 1e-9);
  EXPECT_NEAR(compensated.pose.pose.orientation.w, pose.pose.pose.orientation.w, 1e-9);

  // The header stamp is shifted forward by delay_time.
  const rclcpp::Time compensated_stamp(compensated.header.stamp);
  EXPECT_NEAR(compensated_stamp.seconds(), stamp.seconds() + delay_time, 1e-9);

  // With a fresh module (VX == 0) and zero pitch, delta_z is 0.
  EXPECT_NEAR(compensated.pose.pose.position.z, pose.pose.pose.position.z, 1e-9);
}

TEST(TestEKFModule, CompensateRphWithDelayNonZeroAngularVelocity)
{
  const auto params = make_params();
  auto module = make_module(params);

  const rclcpp::Time stamp(100, 0, RCL_ROS_TIME);
  auto pose = make_pose(0.0, 0.0, 0.0, "map", stamp);

  // Non-zero yaw rate -> orientation must rotate by omega * delay_time about Z.
  const tf2::Vector3 angular_velocity(0.0, 0.0, 1.0);
  const double delay_time = 0.5;
  const auto compensated = module->compensate_rph_with_delay(pose, angular_velocity, delay_time);

  // Expected yaw delta = |omega| * delay_time = 1.0 * 0.5 = 0.5 rad.
  tf2::Quaternion q(
    compensated.pose.pose.orientation.x, compensated.pose.pose.orientation.y,
    compensated.pose.pose.orientation.z, compensated.pose.pose.orientation.w);
  const double yaw = tf2::getYaw(q);
  EXPECT_NEAR(yaw, 0.5, 1e-6);
}

// ---------------------------------------------------------------------------
// measurement_update_pose: success and safety-critical rejection branches
// ---------------------------------------------------------------------------
class MeasurementUpdatePose : public ::testing::Test
{
protected:
  void SetUp() override
  {
    params_ = make_params();
    reset_module();
  }

  // (Re)build the module from the current params_, initialize at the origin, fill the delay-time
  // table with realistic small values (one accumulation per predict cycle in the real node), and
  // run a prediction so the EKF state is well-defined.
  void reset_module()
  {
    module_ = make_module(params_);
    const rclcpp::Time t0(100, 0, RCL_ROS_TIME);
    auto initial_pose = make_pose(0.0, 0.0, 0.0, "map", t0);
    module_->initialize(initial_pose, identity_transform());
    for (size_t i = 0; i < params_.extend_state_step; ++i) {
      module_->accumulate_delay_time(params_.ekf_dt);
    }
    module_->predict_with_delay(params_.ekf_dt);
  }

  HyperParameters params_;
  std::shared_ptr<EKFModule> module_;
};

TEST_F(MeasurementUpdatePose, AcceptsValidMeasurement)
{
  using COV_IDX = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  const rclcpp::Time t_curr(100, 0, RCL_ROS_TIME);

  // Feed a measurement offset from the predicted state (origin) so a real Kalman update has an
  // externally-observable effect: the state must move toward the measurement and the position
  // covariance must shrink. A silent no-op after passing the gates would leave the state at the
  // origin and the covariance unchanged, failing these postconditions.
  auto pose = make_pose(1.0, 2.0, 0.0, "map", t_curr);

  const auto pose_before = module_->get_current_pose(t_curr, false);
  const auto cov_before = module_->get_current_pose_covariance();
  EXPECT_DOUBLE_EQ(pose_before.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(pose_before.pose.position.y, 0.0);

  EKFDiagnosticInfo diag;
  const bool ok = module_->measurement_update_pose(pose, t_curr, diag);

  EXPECT_TRUE(ok);
  EXPECT_TRUE(diag.is_passed_delay_gate);
  EXPECT_TRUE(diag.is_passed_mahalanobis_gate);

  // Postcondition: the state is blended toward the measurement (strictly between the prior
  // estimate and the measurement), not snapped to it.
  const auto pose_after = module_->get_current_pose(t_curr, false);
  EXPECT_GT(pose_after.pose.position.x, 0.0);
  EXPECT_LT(pose_after.pose.position.x, 1.0);
  EXPECT_GT(pose_after.pose.position.y, 0.0);
  EXPECT_LT(pose_after.pose.position.y, 2.0);

  // Postcondition: the position covariance is reduced by incorporating the measurement.
  const auto cov_after = module_->get_current_pose_covariance();
  EXPECT_LT(cov_after[COV_IDX::X_X], cov_before[COV_IDX::X_X]);
  EXPECT_LT(cov_after[COV_IDX::Y_Y], cov_before[COV_IDX::Y_Y]);
}

TEST_F(MeasurementUpdatePose, RejectsOnDelayGate)
{
  // A pose timestamped far in the past produces a delay larger than the table threshold,
  // so delay_step >= extend_state_step and the update is rejected.
  const rclcpp::Time t_curr(1000, 0, RCL_ROS_TIME);
  const rclcpp::Time t_old(100, 0, RCL_ROS_TIME);
  auto pose = make_pose(0.0, 0.0, 0.0, "map", t_old);

  EKFDiagnosticInfo diag;
  const bool ok = module_->measurement_update_pose(pose, t_curr, diag);

  EXPECT_FALSE(ok);
  EXPECT_FALSE(diag.is_passed_delay_gate);
}

TEST_F(MeasurementUpdatePose, RejectsOnNan)
{
  const rclcpp::Time t_curr(100, 0, RCL_ROS_TIME);
  auto pose = make_pose(0.0, 0.0, 0.0, "map", t_curr);
  pose.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();

  EKFDiagnosticInfo diag;
  const bool ok = module_->measurement_update_pose(pose, t_curr, diag);

  EXPECT_FALSE(ok);
  // The NaN gate is reached after the delay gate, so the delay gate is still marked passed.
  EXPECT_TRUE(diag.is_passed_delay_gate);
}

TEST_F(MeasurementUpdatePose, RejectsOnInf)
{
  const rclcpp::Time t_curr(100, 0, RCL_ROS_TIME);
  auto pose = make_pose(0.0, 0.0, 0.0, "map", t_curr);
  pose.pose.pose.position.y = std::numeric_limits<double>::infinity();

  EKFDiagnosticInfo diag;
  const bool ok = module_->measurement_update_pose(pose, t_curr, diag);

  EXPECT_FALSE(ok);
}

TEST_F(MeasurementUpdatePose, RejectsOnMahalanobisGate)
{
  // Tighten the gate so a far-away measurement is rejected by the Mahalanobis distance check.
  params_.pose_gate_dist = 1e-6;
  reset_module();

  const rclcpp::Time t_curr(100, 0, RCL_ROS_TIME);
  auto pose = make_pose(1000.0, 1000.0, 0.0, "map", t_curr);

  EKFDiagnosticInfo diag;
  const bool ok = module_->measurement_update_pose(pose, t_curr, diag);

  EXPECT_FALSE(ok);
  EXPECT_TRUE(diag.is_passed_delay_gate);
  EXPECT_FALSE(diag.is_passed_mahalanobis_gate);
  EXPECT_GT(diag.mahalanobis_distance, 0.0);
}

// ---------------------------------------------------------------------------
// measurement_update_twist: success and safety-critical rejection branches
// ---------------------------------------------------------------------------
class MeasurementUpdateTwist : public ::testing::Test
{
protected:
  void SetUp() override
  {
    params_ = make_params();
    reset_module();
  }

  void reset_module()
  {
    module_ = make_module(params_);
    const rclcpp::Time t0(100, 0, RCL_ROS_TIME);
    auto initial_pose = make_pose(0.0, 0.0, 0.0, "map", t0);
    module_->initialize(initial_pose, identity_transform());
    for (size_t i = 0; i < params_.extend_state_step; ++i) {
      module_->accumulate_delay_time(params_.ekf_dt);
    }
    module_->predict_with_delay(params_.ekf_dt);
  }

  HyperParameters params_;
  std::shared_ptr<EKFModule> module_;
};

TEST_F(MeasurementUpdateTwist, AcceptsValidMeasurement)
{
  using COV_IDX = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  const rclcpp::Time t_curr(100, 0, RCL_ROS_TIME);

  // Feed a velocity offset from the predicted state (vx == wz == 0) so the update is observable:
  // the velocity estimate must move toward the measurement and the velocity covariance shrink.
  auto twist = make_twist(3.0, 1.0, "base_link", t_curr);

  const auto twist_before = module_->get_current_twist(t_curr);
  const auto cov_before = module_->get_current_twist_covariance();
  EXPECT_DOUBLE_EQ(twist_before.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(twist_before.twist.angular.z, 0.0);

  EKFDiagnosticInfo diag;
  const bool ok = module_->measurement_update_twist(twist, t_curr, diag);

  EXPECT_TRUE(ok);
  EXPECT_TRUE(diag.is_passed_delay_gate);
  EXPECT_TRUE(diag.is_passed_mahalanobis_gate);

  // Postcondition: the velocity estimate is blended toward the measurement, not snapped to it.
  const auto twist_after = module_->get_current_twist(t_curr);
  EXPECT_GT(twist_after.twist.linear.x, 0.0);
  EXPECT_LT(twist_after.twist.linear.x, 3.0);
  EXPECT_GT(twist_after.twist.angular.z, 0.0);
  EXPECT_LT(twist_after.twist.angular.z, 1.0);

  // Postcondition: the velocity covariance (vx maps to twist covariance X_X) is reduced.
  const auto cov_after = module_->get_current_twist_covariance();
  EXPECT_LT(cov_after[COV_IDX::X_X], cov_before[COV_IDX::X_X]);
}

TEST_F(MeasurementUpdateTwist, RejectsOnDelayGate)
{
  const rclcpp::Time t_curr(1000, 0, RCL_ROS_TIME);
  const rclcpp::Time t_old(100, 0, RCL_ROS_TIME);
  auto twist = make_twist(0.0, 0.0, "base_link", t_old);

  EKFDiagnosticInfo diag;
  const bool ok = module_->measurement_update_twist(twist, t_curr, diag);

  EXPECT_FALSE(ok);
  EXPECT_FALSE(diag.is_passed_delay_gate);
}

TEST_F(MeasurementUpdateTwist, RejectsOnNan)
{
  const rclcpp::Time t_curr(100, 0, RCL_ROS_TIME);
  auto twist = make_twist(0.0, 0.0, "base_link", t_curr);
  twist.twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();

  EKFDiagnosticInfo diag;
  const bool ok = module_->measurement_update_twist(twist, t_curr, diag);

  EXPECT_FALSE(ok);
  EXPECT_TRUE(diag.is_passed_delay_gate);
}

TEST_F(MeasurementUpdateTwist, RejectsOnInf)
{
  const rclcpp::Time t_curr(100, 0, RCL_ROS_TIME);
  auto twist = make_twist(0.0, 0.0, "base_link", t_curr);
  twist.twist.twist.angular.z = std::numeric_limits<double>::infinity();

  EKFDiagnosticInfo diag;
  const bool ok = module_->measurement_update_twist(twist, t_curr, diag);

  EXPECT_FALSE(ok);
}

TEST_F(MeasurementUpdateTwist, RejectsOnMahalanobisGate)
{
  params_.twist_gate_dist = 1e-6;
  reset_module();

  const rclcpp::Time t_curr(100, 0, RCL_ROS_TIME);
  auto twist = make_twist(1000.0, 1000.0, "base_link", t_curr);

  EKFDiagnosticInfo diag;
  const bool ok = module_->measurement_update_twist(twist, t_curr, diag);

  EXPECT_FALSE(ok);
  EXPECT_TRUE(diag.is_passed_delay_gate);
  EXPECT_FALSE(diag.is_passed_mahalanobis_gate);
  EXPECT_GT(diag.mahalanobis_distance, 0.0);
}

}  // namespace autoware::ekf_localizer
