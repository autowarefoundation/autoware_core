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

#include "obstacle_stop_module.hpp"
#include "test_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/node_options.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

using autoware::motion_velocity_planner::CommonParam;
using autoware::motion_velocity_planner::ObstacleFilteringParam;
using autoware::motion_velocity_planner::ObstacleStopModule;
using autoware::motion_velocity_planner::PlannerData;
using autoware::motion_velocity_planner::Polygon2d;
using autoware::motion_velocity_planner::StopPlanningParam;
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_planning_msgs::msg::TrajectoryPoint;

namespace autoware::motion_velocity_planner
{

// Wrapper class for testing
class ObstacleStopModuleWrapper : public ObstacleStopModule
{
public:
  ObstacleStopModuleWrapper() : ObstacleStopModule()
  {
    // Initialize the module
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file",
       ament_index_cpp::get_package_share_directory(
         "autoware_motion_velocity_obstacle_stop_module") +
         "/config/obstacle_stop.param.yaml"});
    node_ = std::make_shared<rclcpp::Node>("test_node", options);

    // Set required parameters directly
    node_->declare_parameter("normal.min_acc", -1.0);
    node_->declare_parameter("normal.max_acc", 1.0);
    node_->declare_parameter("normal.min_jerk", -1.0);
    node_->declare_parameter("normal.max_jerk", 1.0);

    node_->declare_parameter("limit.min_acc", -2.5);
    node_->declare_parameter("limit.max_acc", 1.0);
    node_->declare_parameter("limit.min_jerk", -1.5);
    node_->declare_parameter("limit.max_jerk", 1.5);

    node_->declare_parameter("pointcloud.mask_lat_margin", 4.0);

    // Initialize the module
    init(*node_, "test_module");
  }

  // Wrapper method for testing
  std::optional<std::pair<geometry_msgs::msg::Point, double>> check_outside_cut_in_obstacle_wrapper(
    const std::shared_ptr<PlannerData::Object> object,
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<Polygon2d> & decimated_traj_polys_with_lat_margin,
    const double dist_to_bumper, const double estimation_time,
    const rclcpp::Time & predicted_objects_stamp) const
  {
    return check_outside_cut_in_obstacle(
      object, traj_points, decimated_traj_points, decimated_traj_polys_with_lat_margin,
      dist_to_bumper, estimation_time, predicted_objects_stamp);
  }

  // Helper function for testing
  geometry_msgs::msg::Pose calc_predicted_pose_wrapper(
    const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & time,
    const rclcpp::Time & predicted_objects_stamp) const
  {
    return object->calc_predicted_pose(time, predicted_objects_stamp);
  }

  // Wrapper method for testing
  std::vector<StopObstacle> check_consistency_wrapper(
    const rclcpp::Time & current_time, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
    std::vector<StopObstacle> stop_obstacles)
  {
    check_consistency(current_time, traj_points, objects, stop_obstacles);
    return stop_obstacles;
  }

  void set_prev_closest_stop_obstacles(const std::vector<StopObstacle> & stop_obstacles)
  {
    prev_closest_stop_obstacles_ = stop_obstacles;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
};

class OutsideCutInObstacleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    module_ = std::make_unique<ObstacleStopModuleWrapper>();
    // Initialize test parameters
    stop_planning_param_.hold_stop_distance_threshold = 2.0;

    // Prepare test trajectory data
    traj_points_ = test_utils::create_test_trajectory();
    decimated_traj_points_ = test_utils::create_test_decimated_trajectory();

    // Generate test polygons
    const double lat_margin = 1.0;
    const bool enable_to_consider_current_pose = true;
    const double time_to_convergence = 1.0;
    const double decimate_trajectory_step_length = 2.0;
    const geometry_msgs::msg::Pose current_ego_pose = test_utils::create_test_pose();

    // Initialize VehicleInfo
    vehicle_info_.vehicle_width_m = 2.0;
    vehicle_info_.vehicle_length_m = 4.0;
    vehicle_info_.min_longitudinal_offset_m = 1.0;
    vehicle_info_.max_longitudinal_offset_m = 3.0;

    // Generate test polygons
    decimated_traj_polys_with_lat_margin_ =
      autoware::motion_velocity_planner::polygon_utils::create_one_step_polygons(
        decimated_traj_points_, vehicle_info_, current_ego_pose, lat_margin,
        enable_to_consider_current_pose, time_to_convergence, decimate_trajectory_step_length);

    // Set test time
    current_time_ = module_->clock_->now();
  }

  void TearDown() override { rclcpp::shutdown(); }

  // Helper function for testing
  std::shared_ptr<PlannerData::Object> create_test_object(
    const double x_vel, const double y_vel, const geometry_msgs::msg::Pose & pose)
  {
    auto object = std::make_shared<PlannerData::Object>();
    object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x = x_vel;
    object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y = y_vel;
    object->predicted_object.kinematics.initial_pose_with_covariance.pose = pose;

    object->predicted_object.shape.dimensions.x = 2.0;
    object->predicted_object.shape.dimensions.y = 2.0;
    object->predicted_object.shape.dimensions.z = 2.0;

    object->predicted_object.classification.resize(1);
    object->predicted_object.classification.at(0).label = ObjectClassification::CAR;

    const double time_step = 1.0;
    const double prediction_time = 30.0;
    const size_t num_points = static_cast<size_t>(prediction_time / time_step) + 1;

    object->predicted_object.kinematics.predicted_paths.resize(1);
    auto & predicted_path = object->predicted_object.kinematics.predicted_paths[0];
    predicted_path.path.resize(num_points);
    predicted_path.time_step = rclcpp::Duration::from_seconds(time_step);

    for (size_t i = 0; i < num_points; ++i) {
      const double t = i * time_step;
      auto & path_point = predicted_path.path[i];
      path_point = pose;
      path_point.position.x += x_vel * t;
      path_point.position.y += y_vel * t;
    }

    return object;
  }

  VehicleInfo vehicle_info_;
  ObstacleFilteringParam obstacle_filtering_param_;
  StopPlanningParam stop_planning_param_;
  std::vector<TrajectoryPoint> traj_points_;
  std::vector<TrajectoryPoint> decimated_traj_points_;
  std::vector<Polygon2d> decimated_traj_polys_with_lat_margin_;
  rclcpp::Time current_time_;
  double dist_to_bumper_ =
    vehicle_info_.max_longitudinal_offset_m;  // only consider the forward direction
  double estimation_time_ = 10.0;             // set a large value for testing purposes
  std::unique_ptr<ObstacleStopModuleWrapper> module_;
};

TEST_F(OutsideCutInObstacleTest, HighLateralVelocity)
{
  auto pose = test_utils::create_test_pose();
  pose.position.x = 20.0;
  pose.position.y = -10.0;
  auto object = create_test_object(0.0, 10.0, pose);
  auto result = module_->check_outside_cut_in_obstacle_wrapper(
    object, traj_points_, decimated_traj_points_, decimated_traj_polys_with_lat_margin_,
    dist_to_bumper_, estimation_time_, current_time_);
  EXPECT_FALSE(result.has_value());
}

TEST_F(OutsideCutInObstacleTest, NoCollisionPoint)
{
  auto pose = test_utils::create_test_pose();
  pose.position.x = 20.0;
  pose.position.y = -5.0;
  auto object = create_test_object(1.0, 0.0, pose);
  auto result = module_->check_outside_cut_in_obstacle_wrapper(
    object, traj_points_, decimated_traj_points_, decimated_traj_polys_with_lat_margin_,
    dist_to_bumper_, estimation_time_, current_time_);
  EXPECT_FALSE(result.has_value());
}

TEST_F(OutsideCutInObstacleTest, NegativeCollisionDistance)
{
  auto pose = test_utils::create_test_pose();
  pose.position.x = 0.0;
  auto object = create_test_object(0.0, 0.0, pose);
  auto result = module_->check_outside_cut_in_obstacle_wrapper(
    object, traj_points_, decimated_traj_points_, decimated_traj_polys_with_lat_margin_,
    dist_to_bumper_, estimation_time_, current_time_);
  EXPECT_FALSE(result.has_value());
}

TEST_F(OutsideCutInObstacleTest, ValidCollisionPointWithStaticObject)
{
  auto pose = test_utils::create_test_pose();
  pose.position.x = 20.0;
  auto object = create_test_object(0.0, 0.0, pose);
  auto result = module_->check_outside_cut_in_obstacle_wrapper(
    object, traj_points_, decimated_traj_points_, decimated_traj_polys_with_lat_margin_,
    dist_to_bumper_, estimation_time_, current_time_);
  EXPECT_TRUE(result.has_value());
}

TEST_F(OutsideCutInObstacleTest, ValidCollisionPointWithCutInObject)
{
  auto pose = test_utils::create_test_pose();
  pose.position.x = 20.0;
  pose.position.y = -5.0;
  auto object = create_test_object(2.0, 0.5, pose);
  auto result = module_->check_outside_cut_in_obstacle_wrapper(
    object, traj_points_, decimated_traj_points_, decimated_traj_polys_with_lat_margin_,
    dist_to_bumper_, estimation_time_, current_time_);
  EXPECT_TRUE(result.has_value());
}

TEST_F(OutsideCutInObstacleTest, ValidCollisionPointWithOppositeCutInObject)
{
  auto pose = test_utils::create_test_pose();
  pose.position.x = 20.0;
  pose.position.y = -5.0;
  auto object = create_test_object(-2.0, 0.5, pose);
  auto result = module_->check_outside_cut_in_obstacle_wrapper(
    object, traj_points_, decimated_traj_points_, decimated_traj_polys_with_lat_margin_,
    dist_to_bumper_, estimation_time_, current_time_);
  EXPECT_FALSE(result.has_value());
}

// TODO(takagi): move this to autoware_motion_velocity_planner_common package
TEST_F(OutsideCutInObstacleTest, GetSpecifiedTimePoseStaticObject)
{
  // Create a static object
  auto pose = test_utils::create_test_pose();
  auto object = create_test_object(0.0, 0.0, pose);  // zero velocity

  // Check position at arbitrary time
  auto pose_0s = module_->calc_predicted_pose_wrapper(object, current_time_, current_time_);
  EXPECT_DOUBLE_EQ(pose_0s.position.x, pose.position.x);
  EXPECT_DOUBLE_EQ(pose_0s.position.y, pose.position.y);

  auto pose_1s = module_->calc_predicted_pose_wrapper(
    object, current_time_ + rclcpp::Duration::from_seconds(1.0), current_time_);
  EXPECT_DOUBLE_EQ(pose_1s.position.x, pose.position.x);
  EXPECT_DOUBLE_EQ(pose_1s.position.y, pose.position.y);
}

class CheckConsistencyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    module_ = std::make_unique<ObstacleStopModuleWrapper>();
    traj_points_ = test_utils::create_test_trajectory();
  }

  void TearDown() override { rclcpp::shutdown(); }

  // Create an object on the trajectory whose velocity is given in its own frame
  std::shared_ptr<PlannerData::Object> create_test_object(
    const double yaw, const double x_vel) const
  {
    auto object = std::make_shared<PlannerData::Object>();
    object->predicted_object.object_id.uuid.at(0) = 1;
    object->predicted_object.kinematics.initial_pose_with_covariance.pose =
      test_utils::create_test_pose(
        20.0, 0.0, 0.0, 0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0));
    object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x = x_vel;
    object->predicted_object.shape.dimensions.x = 2.0;
    object->predicted_object.shape.dimensions.y = 2.0;
    object->predicted_object.shape.dimensions.z = 2.0;
    object->predicted_object.classification.resize(1);
    object->predicted_object.classification.at(0).label = ObjectClassification::CAR;
    return object;
  }

  // Create the previous closest stop obstacle corresponding to the object
  static StopObstacle create_prev_stop_obstacle(
    const std::shared_ptr<PlannerData::Object> & object, const rclcpp::Time & stamp)
  {
    const auto & predicted_object = object->predicted_object;
    return StopObstacle{
      predicted_object.object_id,
      stamp,
      StopObstacleClassification{predicted_object.classification},
      predicted_object.kinematics.initial_pose_with_covariance.pose,
      predicted_object.shape,
      0.0,
      predicted_object.kinematics.initial_pose_with_covariance.pose.position,
      20.0,
      PolygonParam{}};
  }

  std::vector<TrajectoryPoint> traj_points_;
  // obstacle_velocity_threshold_enter_fixed_stop: 3.0, stop_obstacle_hold_time_threshold: 1.0
  const rclcpp::Time current_time_{10, 0, RCL_ROS_TIME};
  std::unique_ptr<ObstacleStopModuleWrapper> module_;
};

TEST_F(CheckConsistencyTest, HoldApproachingObstacle)
{
  // The object faces the opposite direction of the trajectory and approaches the ego.
  // Its velocity in its own frame is positive, but the longitudinal velocity against the trajectory
  // is negative, which is smaller than obstacle_velocity_threshold_enter_fixed_stop.
  const auto object = create_test_object(M_PI, 5.0);
  module_->set_prev_closest_stop_obstacles(
    {create_prev_stop_obstacle(object, current_time_ - rclcpp::Duration::from_seconds(0.5))});

  const auto result = module_->check_consistency_wrapper(current_time_, traj_points_, {object}, {});

  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result.front().uuid, object->predicted_object.object_id);
}

TEST_F(CheckConsistencyTest, HoldCrossingObstacle)
{
  // The object moves perpendicular to the trajectory, so its longitudinal velocity against the
  // trajectory is zero.
  const auto object = create_test_object(M_PI_2, 5.0);
  module_->set_prev_closest_stop_obstacles(
    {create_prev_stop_obstacle(object, current_time_ - rclcpp::Duration::from_seconds(0.5))});

  const auto result = module_->check_consistency_wrapper(current_time_, traj_points_, {object}, {});

  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result.front().uuid, object->predicted_object.object_id);
}

TEST_F(CheckConsistencyTest, NotHoldFastLeadingObstacle)
{
  // The object moves along the trajectory faster than obstacle_velocity_threshold_enter_fixed_stop.
  const auto object = create_test_object(0.0, 5.0);
  module_->set_prev_closest_stop_obstacles(
    {create_prev_stop_obstacle(object, current_time_ - rclcpp::Duration::from_seconds(0.5))});

  const auto result = module_->check_consistency_wrapper(current_time_, traj_points_, {object}, {});

  EXPECT_TRUE(result.empty());
}

TEST_F(CheckConsistencyTest, NotHoldAfterHoldTime)
{
  // The object is static, but stop_obstacle_hold_time_threshold has passed.
  const auto object = create_test_object(0.0, 0.0);
  module_->set_prev_closest_stop_obstacles(
    {create_prev_stop_obstacle(object, current_time_ - rclcpp::Duration::from_seconds(1.5))});

  const auto result = module_->check_consistency_wrapper(current_time_, traj_points_, {object}, {});

  EXPECT_TRUE(result.empty());
}

}  // namespace autoware::motion_velocity_planner
