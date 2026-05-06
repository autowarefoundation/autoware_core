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

#ifndef TEST_CASES__PARTICLE_NUM_LESS_THAN_PUBLISH_NUM_HPP_
#define TEST_CASES__PARTICLE_NUM_LESS_THAN_PUBLISH_NUM_HPP_

#include "../test_fixture.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <thread>

class TestNDTScanMatcherLowParticleNum : public TestNDTScanMatcher
{
protected:
  int64_t particles_num_for_test() const override { return 10; }
  int64_t n_startup_trials_for_test() const override { return 10; }
};

TEST_F(TestNDTScanMatcherLowParticleNum, particle_num_less_than_publish_num)  // NOLINT
{
  //---------//
  // Arrange //
  //---------//
  std::thread t1([&]() {
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node_);
    exec.spin();
  });
  std::thread t2([&]() { rclcpp::spin(pcd_loader_); });

  //-----//
  // Act //
  //-----//
  EXPECT_TRUE(trigger_node_client_->send_trigger_node(true));

  const sensor_msgs::msg::PointCloud2 input_cloud = make_default_sensor_pcd();
  RCLCPP_INFO_STREAM(node_->get_logger(), "sensor cloud size: " << input_cloud.width);
  sensor_pcd_publisher_->publish_pcd(input_cloud);

  const geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg =
    make_pose(/* x = */ 100.0, /* y = */ 100.0);
  const geometry_msgs::msg::Pose result_pose =
    initialpose_client_->send_initialpose(initial_pose_msg).pose.pose;

  RCLCPP_INFO_STREAM(
    node_->get_logger(), std::fixed << "result_pose: " << result_pose.position.x << ", "
                                    << result_pose.position.y << ", " << result_pose.position.z);

  rclcpp::shutdown();
  t1.join();
  t2.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

#endif  // TEST_CASES__PARTICLE_NUM_LESS_THAN_PUBLISH_NUM_HPP_
