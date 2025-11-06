// Copyright 2024 TIER IV, Inc.
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

#include "../src/initial_pose_adaptor.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using autoware::adapi_adaptors::InitialPoseAdaptor;

class InitialPoseAdaptorTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }

  std::vector<double> create_valid_covariance()
  {
    std::vector<double> covariance(36, 0.0);
    // Set diagonal elements
    for (size_t i = 0; i < 6; ++i) {
      covariance[i * 6 + i] = 1.0;
    }
    return covariance;
  }

  std::vector<double> create_invalid_covariance(size_t size) { return std::vector<double>(size); }
};

// Test 1: Node should initialize successfully with valid covariance parameter
TEST_F(InitialPoseAdaptorTest, InitializeWithValidParameter)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("initial_pose_particle_covariance", create_valid_covariance());

  EXPECT_NO_THROW({
    auto node = std::make_shared<InitialPoseAdaptor>(options);
    EXPECT_NE(node, nullptr);
  });
}

// Test 2: Node should not crash when parameter is missing
TEST_F(InitialPoseAdaptorTest, InitializeWithMissingParameter)
{
  rclcpp::NodeOptions options;
  // Don't set the parameter

  EXPECT_NO_THROW({
    auto node = std::make_shared<InitialPoseAdaptor>(options);
    EXPECT_NE(node, nullptr);
  });
}

// Test 3: Node should not crash when parameter has wrong size (too small)
TEST_F(InitialPoseAdaptorTest, InitializeWithWrongSizeParameterTooSmall)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override(
    "initial_pose_particle_covariance", create_invalid_covariance(10));

  EXPECT_NO_THROW({
    auto node = std::make_shared<InitialPoseAdaptor>(options);
    EXPECT_NE(node, nullptr);
  });
}

// Test 4: Node should not crash when parameter has wrong size (too large)
TEST_F(InitialPoseAdaptorTest, InitializeWithWrongSizeParameterTooLarge)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override(
    "initial_pose_particle_covariance", create_invalid_covariance(50));

  EXPECT_NO_THROW({
    auto node = std::make_shared<InitialPoseAdaptor>(options);
    EXPECT_NE(node, nullptr);
  });
}

// Test 5: Node should not crash when parameter is empty
TEST_F(InitialPoseAdaptorTest, InitializeWithEmptyParameter)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("initial_pose_particle_covariance", std::vector<double>{});

  EXPECT_NO_THROW({
    auto node = std::make_shared<InitialPoseAdaptor>(options);
    EXPECT_NE(node, nullptr);
  });
}

// Test 6: Node should handle multiple initialization attempts
TEST_F(InitialPoseAdaptorTest, MultipleInitializationAttempts)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("initial_pose_particle_covariance", create_valid_covariance());

  EXPECT_NO_THROW({
    auto node1 = std::make_shared<InitialPoseAdaptor>(options);
    EXPECT_NE(node1, nullptr);
    node1.reset();  // Destroy the first node

    auto node2 = std::make_shared<InitialPoseAdaptor>(options);
    EXPECT_NE(node2, nullptr);
  });
}

// Test 7: Verify node can be created and destroyed without issues
TEST_F(InitialPoseAdaptorTest, CreateAndDestroyNode)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("initial_pose_particle_covariance", create_valid_covariance());

  EXPECT_NO_THROW({
    {
      auto node = std::make_shared<InitialPoseAdaptor>(options);
      EXPECT_NE(node, nullptr);
  // Node goes out of scope and should be destroyed cleanly
}
});
}

// Test 8: Test with various invalid parameter sizes
TEST_F(InitialPoseAdaptorTest, InitializeWithVariousInvalidSizes)
{
  std::vector<size_t> invalid_sizes = {0, 1, 5, 35, 37, 100};

  for (const auto & size : invalid_sizes) {
    rclcpp::NodeOptions options;
    options.append_parameter_override(
      "initial_pose_particle_covariance", create_invalid_covariance(size));

    EXPECT_NO_THROW({
      auto node = std::make_shared<InitialPoseAdaptor>(options);
      EXPECT_NE(node, nullptr);
    }) << "Failed with size: "
       << size;
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
