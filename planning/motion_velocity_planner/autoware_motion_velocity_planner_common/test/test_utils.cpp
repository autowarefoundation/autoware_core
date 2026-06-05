// Copyright 2026 TIER IV, Inc.
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

#include "autoware/motion_velocity_planner_common/utils.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>

namespace autoware::motion_velocity_planner::utils
{

class MotionVelocityPlannerCommonUtilsTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(MotionVelocityPlannerCommonUtilsTest, GetTargetObjectTypeSupportsAnimalAndHazard)
{
  auto options = rclcpp::NodeOptions{};
  options.append_parameter_override("target.unknown", false);
  options.append_parameter_override("target.car", false);
  options.append_parameter_override("target.truck", false);
  options.append_parameter_override("target.bus", false);
  options.append_parameter_override("target.trailer", false);
  options.append_parameter_override("target.motorcycle", false);
  options.append_parameter_override("target.bicycle", false);
  options.append_parameter_override("target.pedestrian", false);
  options.append_parameter_override("target.animal", true);
  options.append_parameter_override("target.hazard", true);

  auto node = std::make_shared<rclcpp::Node>("test_get_target_object_type", options);
  const auto types = get_target_object_type(*node, "target.");

  EXPECT_NE(std::find(types.begin(), types.end(), ObjectClassification::ANIMAL), types.end());
  EXPECT_NE(std::find(types.begin(), types.end(), ObjectClassification::HAZARD), types.end());
  EXPECT_EQ(std::find(types.begin(), types.end(), ObjectClassification::UNKNOWN), types.end());
}

TEST_F(MotionVelocityPlannerCommonUtilsTest, GetTargetObjectTypeDefaultsMissingLabelsToFalse)
{
  auto options = rclcpp::NodeOptions{};
  options.append_parameter_override("target.unknown", true);
  options.append_parameter_override("target.car", false);
  options.append_parameter_override("target.truck", false);
  options.append_parameter_override("target.bus", false);
  options.append_parameter_override("target.trailer", false);
  options.append_parameter_override("target.motorcycle", false);
  options.append_parameter_override("target.bicycle", false);
  options.append_parameter_override("target.pedestrian", false);

  auto node = std::make_shared<rclcpp::Node>("test_get_target_object_type_defaults", options);

  EXPECT_NO_THROW({
    const auto types = get_target_object_type(*node, "target.");
    EXPECT_NE(std::find(types.begin(), types.end(), ObjectClassification::UNKNOWN), types.end());
    EXPECT_EQ(std::find(types.begin(), types.end(), ObjectClassification::ANIMAL), types.end());
    EXPECT_EQ(std::find(types.begin(), types.end(), ObjectClassification::HAZARD), types.end());
  });
}

}  // namespace autoware::motion_velocity_planner::utils
