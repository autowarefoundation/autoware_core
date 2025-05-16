// Copyright 2023 The Autoware Contributors
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

#include "../src/lanelet2_map_visualization_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>

using autoware::lanelet2_map_visualizer::Lanelet2MapVisualizationNode;

class TestNodeInitialization : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(TestNodeInitialization, InitializationTest)
{
  // Test that the node can be constructed without errors
  rclcpp::NodeOptions options;
  std::shared_ptr<Lanelet2MapVisualizationNode> node =
    std::make_shared<Lanelet2MapVisualizationNode>(options);

  // Check that the node is not null
  ASSERT_NE(node, nullptr);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
