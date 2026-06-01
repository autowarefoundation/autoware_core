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

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor_array.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

namespace autoware::planning_factor_interface
{
namespace
{
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_internal_planning_msgs::msg::PlanningFactorArray;
using autoware_internal_planning_msgs::msg::SafetyFactorArray;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;

Pose make_pose(const double x, const double y)
{
  Pose pose;
  pose.position = autoware_utils_geometry::create_point(x, y, 0.0);
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
  return pose;
}

// Straight line along +x with 1.0 m spacing, so that calcSignedArcLength from the
// origin to a point at x == d returns exactly d.
std::vector<TrajectoryPoint> make_straight_trajectory(const size_t num_points)
{
  std::vector<TrajectoryPoint> points;
  points.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    TrajectoryPoint p;
    p.pose = make_pose(static_cast<double>(i), 0.0);
    points.push_back(p);
  }
  return points;
}
}  // namespace

class PlanningFactorInterfaceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("planning_factor_interface_test_node");
    interface_ = std::make_unique<PlanningFactorInterface>(node_.get(), "test_module");
  }

  void TearDown() override
  {
    interface_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<PlanningFactorInterface> interface_;
};

// add(distance, ...) populates exactly one ControlPoint with the supplied fields and
// the PlanningFactor metadata (module name, behavior, detail, driving direction).
TEST_F(PlanningFactorInterfaceTest, AddSingleControlPoint)
{
  const auto control_pose = make_pose(3.0, 4.0);
  SafetyFactorArray safety_factors;
  safety_factors.is_safe = true;

  interface_->add(
    /*distance=*/12.5, control_pose, PlanningFactor::STOP, safety_factors,
    /*is_driving_forward=*/false, /*velocity=*/2.0, /*shift_length=*/-1.5, "detail-text");

  const auto & factors = interface_->get_factors();
  ASSERT_EQ(factors.size(), 1u);

  const auto & factor = factors.front();
  EXPECT_EQ(factor.module, "test_module");
  EXPECT_EQ(factor.behavior, PlanningFactor::STOP);
  EXPECT_EQ(factor.detail, "detail-text");
  EXPECT_FALSE(factor.is_driving_forward);
  EXPECT_TRUE(factor.safety_factors.is_safe);

  ASSERT_EQ(factor.control_points.size(), 1u);
  const auto & cp = factor.control_points.front();
  EXPECT_DOUBLE_EQ(cp.pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(cp.pose.position.y, 4.0);
  EXPECT_FLOAT_EQ(cp.velocity, 2.0f);
  EXPECT_FLOAT_EQ(cp.shift_length, -1.5f);
  EXPECT_FLOAT_EQ(cp.distance, 12.5f);
}

// add(start_distance, end_distance, ...) populates two ControlPoints in start->end order.
TEST_F(PlanningFactorInterfaceTest, AddTwoControlPoints)
{
  const auto start_pose = make_pose(1.0, 0.0);
  const auto end_pose = make_pose(5.0, 0.0);
  const SafetyFactorArray safety_factors;

  interface_->add(
    /*start_distance=*/1.0, /*end_distance=*/5.0, start_pose, end_pose, PlanningFactor::SLOW_DOWN,
    safety_factors, /*is_driving_forward=*/true, /*start_velocity=*/3.0, /*end_velocity=*/4.0,
    /*start_shift_length=*/0.5, /*end_shift_length=*/0.7, "section");

  const auto & factors = interface_->get_factors();
  ASSERT_EQ(factors.size(), 1u);

  const auto & factor = factors.front();
  EXPECT_EQ(factor.behavior, PlanningFactor::SLOW_DOWN);
  EXPECT_EQ(factor.detail, "section");
  EXPECT_TRUE(factor.is_driving_forward);

  ASSERT_EQ(factor.control_points.size(), 2u);
  const auto & start = factor.control_points.at(0);
  const auto & end = factor.control_points.at(1);

  EXPECT_DOUBLE_EQ(start.pose.position.x, 1.0);
  EXPECT_FLOAT_EQ(start.velocity, 3.0f);
  EXPECT_FLOAT_EQ(start.shift_length, 0.5f);
  EXPECT_FLOAT_EQ(start.distance, 1.0f);

  EXPECT_DOUBLE_EQ(end.pose.position.x, 5.0);
  EXPECT_FLOAT_EQ(end.velocity, 4.0f);
  EXPECT_FLOAT_EQ(end.shift_length, 0.7f);
  EXPECT_FLOAT_EQ(end.distance, 5.0f);
}

// The templated single-point add() forwards the calcSignedArcLength result into
// ControlPoint.distance. With a 1 m-spaced straight trajectory the signed arc length
// from the origin to (x, 0) is exactly x.
TEST_F(PlanningFactorInterfaceTest, AddTemplatedSinglePointForwardsArcLength)
{
  const auto points = make_straight_trajectory(6);
  const auto ego_pose = make_pose(0.0, 0.0);
  const auto control_pose = make_pose(4.0, 0.0);
  const SafetyFactorArray safety_factors;

  interface_->add(points, ego_pose, control_pose, PlanningFactor::STOP, safety_factors);

  const auto & factors = interface_->get_factors();
  ASSERT_EQ(factors.size(), 1u);
  ASSERT_EQ(factors.front().control_points.size(), 1u);
  EXPECT_FLOAT_EQ(factors.front().control_points.front().distance, 4.0f);
}

// The templated two-point add() forwards both calcSignedArcLength results in order.
TEST_F(PlanningFactorInterfaceTest, AddTemplatedTwoPointsForwardsArcLength)
{
  const auto points = make_straight_trajectory(8);
  const auto ego_pose = make_pose(0.0, 0.0);
  const auto start_pose = make_pose(2.0, 0.0);
  const auto end_pose = make_pose(6.0, 0.0);
  const SafetyFactorArray safety_factors;

  interface_->add(
    points, ego_pose, start_pose, end_pose, PlanningFactor::SLOW_DOWN, safety_factors);

  const auto & factors = interface_->get_factors();
  ASSERT_EQ(factors.size(), 1u);
  ASSERT_EQ(factors.front().control_points.size(), 2u);
  EXPECT_FLOAT_EQ(factors.front().control_points.at(0).distance, 2.0f);
  EXPECT_FLOAT_EQ(factors.front().control_points.at(1).distance, 6.0f);
}

// Multiple add() calls accumulate into factors_ in insertion order.
TEST_F(PlanningFactorInterfaceTest, MultipleAddAccumulates)
{
  const SafetyFactorArray safety_factors;
  interface_->add(1.0, make_pose(1.0, 0.0), PlanningFactor::STOP, safety_factors);
  interface_->add(2.0, make_pose(2.0, 0.0), PlanningFactor::SLOW_DOWN, safety_factors);
  interface_->add(3.0, make_pose(3.0, 0.0), PlanningFactor::NONE, safety_factors);

  const auto & factors = interface_->get_factors();
  ASSERT_EQ(factors.size(), 3u);
  EXPECT_EQ(factors.at(0).behavior, PlanningFactor::STOP);
  EXPECT_EQ(factors.at(1).behavior, PlanningFactor::SLOW_DOWN);
  EXPECT_EQ(factors.at(2).behavior, PlanningFactor::NONE);
  EXPECT_FLOAT_EQ(factors.at(0).control_points.front().distance, 1.0f);
  EXPECT_FLOAT_EQ(factors.at(1).control_points.front().distance, 2.0f);
  EXPECT_FLOAT_EQ(factors.at(2).control_points.front().distance, 3.0f);
}

// publish() stamps frame_id='map', forwards the accumulated factors into the message,
// and clears the internal buffer afterwards.
TEST_F(PlanningFactorInterfaceTest, PublishStampsAndForwardsFactors)
{
  const SafetyFactorArray safety_factors;
  interface_->add(7.0, make_pose(7.0, 0.0), PlanningFactor::STOP, safety_factors, true, 1.0, 0.0);

  PlanningFactorArray received;
  bool got_msg = false;
  auto sub = node_->create_subscription<PlanningFactorArray>(
    "/planning/planning_factors/test_module", rclcpp::QoS{1},
    [&](const PlanningFactorArray::ConstSharedPtr msg) {
      received = *msg;
      got_msg = true;
    });

  interface_->publish();

  // Spin until the subscription receives the published message (bounded wait).
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!got_msg && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(got_msg);
  EXPECT_EQ(received.header.frame_id, "map");
  ASSERT_EQ(received.factors.size(), 1u);
  EXPECT_EQ(received.factors.front().behavior, PlanningFactor::STOP);
  ASSERT_EQ(received.factors.front().control_points.size(), 1u);
  EXPECT_FLOAT_EQ(received.factors.front().control_points.front().distance, 7.0f);
}

// publish() clears the internal factor buffer so the next cycle starts empty
// (the clear-after-publish contract).
TEST_F(PlanningFactorInterfaceTest, PublishClearsFactors)
{
  const SafetyFactorArray safety_factors;
  interface_->add(1.0, make_pose(1.0, 0.0), PlanningFactor::STOP, safety_factors);
  ASSERT_EQ(interface_->get_factors().size(), 1u);

  interface_->publish();

  EXPECT_TRUE(interface_->get_factors().empty());

  // A subsequent add starts from an empty buffer.
  interface_->add(2.0, make_pose(2.0, 0.0), PlanningFactor::SLOW_DOWN, safety_factors);
  EXPECT_EQ(interface_->get_factors().size(), 1u);
}

}  // namespace autoware::planning_factor_interface

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
