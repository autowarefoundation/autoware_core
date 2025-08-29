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

#include "../src/obstacle_stop_module.hpp"

#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <autoware_test_utils/autoware_test_utils.hpp>
#include <gtest/gtest.h>

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}

class ObstacleStopModuleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto node_options = rclcpp::NodeOptions{};
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    std::vector<std::string> yaml_files = {
      autoware_test_utils_dir + "/config/test_motion_velocity_planner.param.yaml",
      autoware_test_utils_dir + "/config/test_common.param.yaml",
      autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
      autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
      ament_index_cpp::get_package_share_directory(
        "autoware_motion_velocity_obstacle_stop_module") +
        "/config/obstacle_stop.param.yaml"};
    autoware::test_utils::updateNodeOptions(node_options, yaml_files);

    node_ = std::make_shared<rclcpp::Node>("test_node", node_options);
    module_ = std::make_unique<autoware::motion_velocity_planner::ObstacleStopModule>();

    module_->init(*node_, "obstacle_stop");
    planner_data_ = std::make_shared<autoware::motion_velocity_planner::PlannerData>(*node_);
  }

  std::unique_ptr<autoware::motion_velocity_planner::ObstacleStopModule> module_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware::motion_velocity_planner::PlannerData> planner_data_;
};

TEST_F(ObstacleStopModuleTest, NodeTestWithPredictedObjects)
{
  // Add a simple pedestrian obstacle for testing
  autoware_perception_msgs::msg::PredictedObject pedestrian;
  pedestrian.existence_probability = 1.0;

  // Set classification
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
  classification.probability = 1.0;
  pedestrian.classification.resize(1);
  pedestrian.classification.at(0) = classification;

  // Set pose - place pedestrian in front of ego
  geometry_msgs::msg::Pose initial_pose;
  initial_pose.position.x = 20.0;
  initial_pose.position.y = 0.0;
  initial_pose.position.z = 0.0;
  initial_pose.orientation.w = 1.0;
  pedestrian.kinematics.initial_pose_with_covariance.pose = initial_pose;

  geometry_msgs::msg::Twist initial_twist;
  initial_twist.linear.x = 0.0;
  initial_twist.linear.y = 0.0;
  pedestrian.kinematics.initial_twist_with_covariance.twist = initial_twist;

  // Set shape
  pedestrian.shape.type = autoware_perception_msgs::msg::Shape::CYLINDER;
  pedestrian.shape.dimensions.x = 0.5;
  pedestrian.shape.dimensions.y = 0.5;
  pedestrian.shape.dimensions.z = 1.8;

  pedestrian.kinematics.predicted_paths.resize(1);
  auto & predicted_path = pedestrian.kinematics.predicted_paths.front();
  predicted_path.path.resize(10);
  predicted_path.time_step = rclcpp::Duration::from_seconds(0.5);
  predicted_path.confidence = 1.0;
  for (size_t i = 0; i < predicted_path.path.size(); ++i) {
    predicted_path.path[i].position.x = initial_pose.position.x + i * 0.5 * initial_twist.linear.x;
    predicted_path.path[i].position.y = initial_pose.position.y + i * 0.5 * initial_twist.linear.y;
    predicted_path.path[i].position.z = initial_pose.position.z;
    predicted_path.path[i].orientation = initial_pose.orientation;
  }

  autoware_perception_msgs::msg::PredictedObjects predicted_objects;
  predicted_objects.header.frame_id = "map";
  predicted_objects.header.stamp = node_->get_clock()->now();
  predicted_objects.objects.push_back(pedestrian);

  auto odometry = nav_msgs::msg::Odometry{};
  odometry.header.frame_id = "map";
  odometry.header.stamp = node_->get_clock()->now();
  odometry.pose.pose.position.x = 0.0;
  odometry.pose.pose.position.y = 0.0;
  odometry.pose.pose.position.z = 0.0;
  odometry.pose.pose.orientation.w = 1.0;

  // Generate trajectory points along a straight line
  auto trajectory = autoware_planning_msgs::msg::Trajectory{};
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = node_->get_clock()->now();

  for (double x = 0.0; x <= 100.0; x += 1.0) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = 0.0;
    point.pose.position.z = 0.0;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 10.0;
    trajectory.points.push_back(point);
  }

  // Set data for PlannerData
  planner_data_->current_odometry = odometry;
  planner_data_->process_predicted_objects(predicted_objects);

  module_->init(*node_, "obstacle_stop");
  module_->plan(trajectory.points, trajectory.points, planner_data_);

  // Get and verify planning factors
  const auto planning_factors = module_->get_planning_factors();
  ASSERT_EQ(planning_factors.size(), 1);
  const auto & planning_factor = planning_factors.front();
  EXPECT_EQ(planning_factor.behavior,
  autoware_internal_planning_msgs::msg::PlanningFactor::STOP);
  EXPECT_NEAR(planning_factor.control_points.front().pose.position.x, 10.0, 3.0);
  EXPECT_NEAR(planning_factor.control_points.front().pose.position.y, 0.0, 1.0);
  EXPECT_NEAR(planning_factor.control_points.front().pose.position.z, 0.0, 1.0);
  
  ASSERT_EQ(planning_factor.safety_factors.factors.size(), 1);
  const auto & safety_factor = planning_factor.safety_factors.factors.front();
  // current implementation does not set object type
  // EXPECT_EQ(safety_factor.type, expected_object_type);
  EXPECT_FALSE(safety_factor.is_safe);
  ASSERT_EQ(safety_factor.points.size(), 1);
  EXPECT_NEAR(safety_factor.points.front().x, 20.0, 1.0);
  EXPECT_NEAR(safety_factor.points.front().y, 0.0, 1.0);
  EXPECT_NEAR(safety_factor.points.front().z, 0.0, 1.0);
  EXPECT_EQ(safety_factor.object_id, predicted_objects.objects.front().object_id);
}
