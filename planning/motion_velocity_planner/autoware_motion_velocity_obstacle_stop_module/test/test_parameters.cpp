// Copyright 2025 TIER IV, Inc.
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

#include "parameters.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace autoware::motion_velocity_planner
{

// Test ROS 2 node
class TestNode : public rclcpp::Node
{
public:
  TestNode() : rclcpp::Node("test_node") {}
};

// Test suite for get_object_parameter
class GetObjectParameterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<TestNode>();
  }
  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<TestNode> node_;
};

// --- Test cases (verify priority: ns > ns.object_label > ns.default) ---

// 1. Case where namespace-specific parameter is retrieved with the highest priority
TEST_F(GetObjectParameterTest, GetNamespaceParameter_HighestPriority)
{
  // ns exists
  node_->declare_parameter<int>("obstacle_stop", 100);
  // ns.car exists
  node_->declare_parameter<int>("obstacle_stop.car", 200);
  // ns.default exists
  node_->declare_parameter<int>("obstacle_stop.default", 300);

  // Call the function and verify that the highest priority value (obstacle_stop) is retrieved
  int result = get_object_parameter<int>(*node_, "obstacle_stop", "car");
  ASSERT_EQ(result, 100);
}

// 2. Case where namespace parameter does not exist, and object-specific parameter has the next
// priority
TEST_F(GetObjectParameterTest, GetSpecificObjectParameter_SecondPriority)
{
  // ns does not exist
  // ns.car exists
  node_->declare_parameter<double>("obstacle_stop.car", 10.5);
  // ns.default exists
  node_->declare_parameter<double>("obstacle_stop.default", 20.5);

  // Call the function and verify that the second highest priority value (obstacle_stop.car) is
  // retrieved
  double result = get_object_parameter<double>(*node_, "obstacle_stop", "car");
  ASSERT_DOUBLE_EQ(result, 10.5);
}

// 3. Case where namespace and object parameters do not exist, and the default parameter is
// retrieved
TEST_F(GetObjectParameterTest, GetDefaultParameter_LowestPriority)
{
  // ns does not exist
  // ns.bicycle also does not exist
  // ns.default exists
  node_->declare_parameter<bool>("obstacle_stop.default", true);

  // Call the function and verify that the default value is retrieved
  bool result = get_object_parameter<bool>(*node_, "obstacle_stop", "bicycle");
  ASSERT_TRUE(result);
}

// 4. Case where an exception is thrown when no parameter is found
TEST_F(GetObjectParameterTest, ThrowExceptionWhenNoParameterFound)
{
  ASSERT_THROW(
    get_object_parameter<std::string>(*node_, "obstacle_stop", "motorcycle"), std::runtime_error);
}

// 5. Verify that parameters with a suffix also follow this priority
TEST_F(GetObjectParameterTest, SuffixWithCustomPriority)
{
  // ns.value exists
  node_->declare_parameter<double>("obstacle_stop.value", 50.0);
  // ns.pedestrian.value exists
  node_->declare_parameter<double>("obstacle_stop.pedestrian.value", 10.0);

  // Call the function and verify that ns.value is prioritized
  double result = get_object_parameter<double>(*node_, "obstacle_stop", "pedestrian", "value");
  ASSERT_DOUBLE_EQ(result, 50.0);
}

}  // namespace autoware::motion_velocity_planner
