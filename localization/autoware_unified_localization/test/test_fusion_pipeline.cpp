// Copyright 2025 Autoware Foundation
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

#include "autoware/unified_localization_core/fusion_pipeline.hpp"
#include "autoware/unified_localization_core/types.hpp"

#include <gtest/gtest.h>

#include <cmath>

using autoware::unified_localization_core::FusionPipeline;
using autoware::unified_localization_core::FusionPipelineParams;
using autoware::unified_localization_core::OdometryOutput;
using autoware::unified_localization_core::PoseWithCovariance;
using autoware::unified_localization_core::TwistWithCovariance;

TEST(FusionPipeline, InitializeAndGetOdometry)
{
  FusionPipelineParams params;
  params.ekf.predict_frequency = 50.0;
  params.ekf.ekf_dt = 0.02;
  FusionPipeline pipeline(params);

  PoseWithCovariance initial_pose;
  initial_pose.timestamp_sec = 0.0;
  initial_pose.position_x = 1.0;
  initial_pose.position_y = 2.0;
  initial_pose.position_z = 0.5;
  initial_pose.orientation_w = 1.0;
  initial_pose.covariance[0] = 0.01;
  initial_pose.covariance[7] = 0.01;
  initial_pose.covariance[14] = 0.01;
  initial_pose.covariance[21] = 0.01;
  initial_pose.covariance[28] = 0.01;
  initial_pose.covariance[35] = 0.01;

  pipeline.initialize(initial_pose);
  ASSERT_TRUE(pipeline.is_initialized());

  OdometryOutput odom;
  pipeline.get_odometry(0.0, odom);
  EXPECT_NEAR(odom.position_x, 1.0, 1e-5);
  EXPECT_NEAR(odom.position_y, 2.0, 1e-5);
  EXPECT_NEAR(odom.position_z, 0.5, 1e-5);
  EXPECT_NEAR(odom.linear_x, 0.0, 1e-5);
  EXPECT_NEAR(odom.angular_z, 0.0, 1e-5);
}

TEST(FusionPipeline, StepWithPoseAndTwistThenGetOdometry)
{
  FusionPipelineParams params;
  params.ekf.predict_frequency = 50.0;
  params.ekf.ekf_dt = 0.02;
  FusionPipeline pipeline(params);

  PoseWithCovariance initial_pose;
  initial_pose.timestamp_sec = 0.0;
  initial_pose.position_x = 0.0;
  initial_pose.position_y = 0.0;
  initial_pose.position_z = 0.0;
  initial_pose.orientation_w = 1.0;
  for (size_t i = 0; i < 36; ++i) {
    initial_pose.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
  }
  pipeline.initialize(initial_pose);

  double t = 0.02;
  for (int i = 0; i < 10; ++i) {
    PoseWithCovariance pose;
    pose.timestamp_sec = t - 0.01;
    pose.position_x = 0.1 * (i + 1);
    pose.position_y = 0.0;
    pose.position_z = 0.0;
    pose.orientation_w = 1.0;
    pose.covariance[0] = pose.covariance[7] = pose.covariance[35] = 0.01;

    TwistWithCovariance twist;
    twist.timestamp_sec = t - 0.01;
    twist.linear_x = 5.0;
    twist.angular_z = 0.0;
    twist.covariance[0] = twist.covariance[35] = 0.1;

    pipeline.step(t, 0.02, &pose, &twist);
    t += 0.02;
  }

  OdometryOutput odom;
  pipeline.get_odometry(t, odom);
  EXPECT_GT(odom.position_x, 0.5);
  EXPECT_NEAR(odom.position_y, 0.0, 0.1);
  EXPECT_NEAR(odom.linear_x, 5.0, 1.0);
  EXPECT_NEAR(odom.angular_z, 0.0, 0.1);
}

TEST(FusionPipeline, StopFilterZeroesWhenStopped)
{
  FusionPipelineParams params;
  params.ekf.predict_frequency = 50.0;
  params.ekf.ekf_dt = 0.02;
  params.stop_filter.linear_x_threshold = 0.5;
  params.stop_filter.angular_z_threshold = 0.1;
  FusionPipeline pipeline(params);

  PoseWithCovariance initial_pose;
  initial_pose.timestamp_sec = 0.0;
  initial_pose.position_x = 0.0;
  initial_pose.position_y = 0.0;
  initial_pose.position_z = 0.0;
  initial_pose.orientation_w = 1.0;
  initial_pose.covariance[0] = initial_pose.covariance[7] = initial_pose.covariance[35] = 0.01;
  pipeline.initialize(initial_pose);

  TwistWithCovariance twist;
  twist.timestamp_sec = 0.01;
  twist.linear_x = 0.05;
  twist.angular_z = 0.01;
  twist.covariance[0] = twist.covariance[35] = 0.01;
  pipeline.step(0.02, 0.02, nullptr, &twist);

  OdometryOutput odom;
  pipeline.get_odometry(0.02, odom);
  EXPECT_NEAR(odom.linear_x, 0.0, 1e-6);
  EXPECT_NEAR(odom.angular_z, 0.0, 1e-6);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
