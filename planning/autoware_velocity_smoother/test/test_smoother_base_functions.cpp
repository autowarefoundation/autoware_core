// Copyright 2022 Tier IV, Inc.
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

#include "autoware/velocity_smoother/smoother/jerk_filtered_smoother.hpp"
#include "autoware/velocity_smoother/smoother/smoother_base.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <vector>

using autoware::velocity_smoother::JerkFilteredSmoother;
using autoware::velocity_smoother::SmootherBase;

// Test fixture to create a SmootherBase instance with controlled parameters
class TestSmootherBase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    auto node_options = rclcpp::NodeOptions{};
    node_options.append_parameter_override("algorithm_type", "JerkFiltered");
    node_options.append_parameter_override("publish_debug_trajs", false);
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    const auto velocity_smoother_dir =
      ament_index_cpp::get_package_share_directory("autoware_velocity_smoother");
    node_options.arguments(
      {"--ros-args", "--params-file", autoware_test_utils_dir + "/config/test_common.param.yaml",
       "--params-file", autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
       "--params-file", autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
       "--params-file", velocity_smoother_dir + "/config/default_velocity_smoother.param.yaml",
       "--params-file", velocity_smoother_dir + "/config/default_common.param.yaml",
       "--params-file", velocity_smoother_dir + "/config/JerkFiltered.param.yaml"});
    node = std::make_shared<rclcpp::Node>("test_smoother_base_node", node_options);

    auto time_keeper =
      std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_);

    SmootherBase::BaseParam params;
    params.max_accel = 1.0;
    params.min_decel = -1.0;
    params.stop_decel = 0.5;
    params.max_jerk = 0.8;
    params.min_jerk = -0.8;
    params.min_decel_for_lateral_acc_lim_filter = -0.5;
    params.sample_ds = 0.1;
    params.curvature_threshold = 0.01;
    params.lateral_acceleration_limits = {2.0, 1.8, 1.5};
    params.velocity_thresholds = {10.0, 20.0, 30.0};
    params.steering_angle_rate_limits = {30.0, 20.0, 10.0};
    params.curvature_calculation_distance = 1.0;
    params.decel_distance_before_curve = 3.0;
    params.decel_distance_after_curve = 2.0;
    params.min_curve_velocity = 2.0;
    params.wheel_base = 2.7;
    params.resample_param.max_trajectory_length = 200.0;
    params.resample_param.min_trajectory_length = 30.0;
    params.resample_param.resample_time = 0.1;
    params.resample_param.dense_resample_dt = 0.1;
    params.resample_param.dense_min_interval_distance = 0.1;
    params.resample_param.sparse_resample_dt = 0.5;
    params.resample_param.sparse_min_interval_distance = 4.0;

    smoother_base = std::make_shared<JerkFilteredSmoother>(*node, time_keeper);
    // smoother_base->setWheelBase(2.7); // Set a typical wheelbase value
    std::dynamic_pointer_cast<SmootherBase>(smoother_base)->setParam(params);
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<JerkFilteredSmoother> smoother_base;
  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_;
};

TEST_F(TestSmootherBase, ComputeLateralAccelerationVelocitySquareRatioLimits)
{
  const auto limits = smoother_base->computeLateralAccelerationVelocitySquareRatioLimits();

  // Check that we have the expected number of limits
  // Number of thresholds + 1 for the final segment
  EXPECT_EQ(limits.size(), 4);

  // Check the values for the first limit pair
  constexpr double epsilon = 1e-5;
  EXPECT_NEAR(limits[0].first, 2.0 / (0.0 * 0.0 + epsilon), 1e-10);
  EXPECT_NEAR(limits[0].second, 2.0 / (10.0 * 10.0 + epsilon), 1e-10);

  // Check the values for the second limit pair
  EXPECT_NEAR(limits[1].first, 1.8 / (10.0 * 10.0 + epsilon), 1e-10);
  EXPECT_NEAR(limits[1].second, 1.8 / (20.0 * 20.0 + epsilon), 1e-10);

  // Check the values for the third limit pair
  EXPECT_NEAR(limits[2].first, 1.5 / (20.0 * 20.0 + epsilon), 1e-10);
  EXPECT_NEAR(limits[2].second, 1.5 / (30.0 * 30.0 + epsilon), 1e-10);

  // Check the values for the last limit pair
  EXPECT_NEAR(limits[3].first, 1.5 / (30.0 * 30.0 + epsilon), 1e-10);
  EXPECT_NEAR(limits[3].second, 0.0, 1e-10);
}

TEST_F(TestSmootherBase, ComputeSteerRateVelocityRatioLimits)
{
  const auto limits = smoother_base->computeSteerRateVelocityRatioLimits();

  // Check that we have the expected number of limits
  // Number of thresholds + 1 for the final segment
  EXPECT_EQ(limits.size(), 4);

  constexpr double epsilon = 1e-5;
  constexpr double deg2rad = M_PI / 180.0;

  // Check the values for the first limit pair
  EXPECT_NEAR(limits[0].first, 30.0 * deg2rad / (0.0 + epsilon), 1e-10);
  EXPECT_NEAR(limits[0].second, 30.0 * deg2rad / (10.0 + epsilon), 1e-10);

  // Check the values for the second limit pair
  EXPECT_NEAR(limits[1].first, 20.0 * deg2rad / (10.0 + epsilon), 1e-10);
  EXPECT_NEAR(limits[1].second, 20.0 * deg2rad / (20.0 + epsilon), 1e-10);

  // Check the values for the third limit pair
  EXPECT_NEAR(limits[2].first, 10.0 * deg2rad / (20.0 + epsilon), 1e-10);
  EXPECT_NEAR(limits[2].second, 10.0 * deg2rad / (30.0 + epsilon), 1e-10);

  // Check the values for the last limit pair
  EXPECT_NEAR(limits[3].first, 10.0 * deg2rad / (30.0 + epsilon), 1e-10);
  EXPECT_NEAR(limits[3].second, 0.0, 1e-10);
}

TEST_F(TestSmootherBase, ComputeVelocityLimitFromLateralAcc)
{
  const auto limits = smoother_base->computeLateralAccelerationVelocitySquareRatioLimits();

  // Test case 1: Curvature below the lowest threshold
  // Should return a large velocity
  double curvature = 0.000001;  // Very small curvature
  double velocity_limit = smoother_base->computeVelocityLimitFromLateralAcc(curvature, limits);
  EXPECT_GT(velocity_limit, 100.0);

  // Test case 2: Curvature between thresholds
  // For curvature = 0.0025 (between first and second threshold)
  // Expected velocity = sqrt(1.5 / 0.0025) = 24.49
  curvature = 0.0025;
  velocity_limit = smoother_base->computeVelocityLimitFromLateralAcc(curvature, limits);
  EXPECT_NEAR(velocity_limit, 24.494897, 1e-5);

  // Test case 3: Curvature above all thresholds
  // Expected velocity = sqrt(2.0 / 1.0) = 1.414
  curvature = 1.0;  // Very high curvature
  velocity_limit = smoother_base->computeVelocityLimitFromLateralAcc(curvature, limits);
  EXPECT_NEAR(velocity_limit, 1.414, 1e-3);

  // Test case 4: Curvature exactly at a threshold
  // For curvature = 0.00166... (exactly at second threshold)
  // Should return the second velocity threshold (20.0)
  curvature = 1.8 / (20.0 * 20.0);
  velocity_limit = smoother_base->computeVelocityLimitFromLateralAcc(curvature, limits);
  EXPECT_NEAR(velocity_limit, 20.0, 1e-10);
}

TEST_F(TestSmootherBase, ComputeVelocityLimitFromSteerRate)
{
  const auto limits = smoother_base->computeSteerRateVelocityRatioLimits();
  // Limits: [+inf, 3.0, 1.0, 0.33]

  constexpr double deg2rad = M_PI / 180.0;

  // Test case 1: Steer rate ratio below the lowest threshold
  // Should return a large velocity
  // Expected velocity = 0.33 (selected threshold) / 0.1 (input) * 30 (speed threshold) = 100.0
  double steer_rate_ratio = 0.1 * deg2rad;  // Very small ratio
  double velocity_limit =
    smoother_base->computeVelocityLimitFromSteerRate(steer_rate_ratio, limits);
  EXPECT_NEAR(velocity_limit, 100.0, 1e-10);

  // Test case 2: Steer rate ratio on the threshold
  // For ratio = 1.0 * deg2rad (between first and second threshold)
  // Expected velocity = 20.0 * deg2rad / (1.0 * deg2rad) = 20.0
  steer_rate_ratio = 1.0 * deg2rad;
  velocity_limit = smoother_base->computeVelocityLimitFromSteerRate(steer_rate_ratio, limits);
  EXPECT_NEAR(velocity_limit, 20.0, 1e-10);

  // Test case 3: Steer rate ratio above all thresholds
  // For ratio = 1.0 * deg2rad (between first and second threshold)
  // Expected velocity = 3.0 (selected threshold) / 100 (input) * 10 (speed threshold) = 0.3
  steer_rate_ratio = 100.0 * deg2rad;  // Very high ratio
  velocity_limit = smoother_base->computeVelocityLimitFromSteerRate(steer_rate_ratio, limits);
  EXPECT_NEAR(velocity_limit, 0.3, 1e-10);

  // Test case 4: Steer rate ratio between thresholds
  // For ratio = 0.4 * deg2rad (exactly at second threshold)
  // Expected velocity = 0.33 (selected threshold) / 0.4 (input) * 30 (speed threshold) = 25.0
  steer_rate_ratio = 0.4 * deg2rad;
  velocity_limit = smoother_base->computeVelocityLimitFromSteerRate(steer_rate_ratio, limits);
  EXPECT_NEAR(velocity_limit, 25.0, 1e-10);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
