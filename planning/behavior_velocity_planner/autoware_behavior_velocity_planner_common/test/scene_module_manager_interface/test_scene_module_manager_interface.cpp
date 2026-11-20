// Copyright 2026 The Autoware Contributors
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

#include "autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp"
#include "autoware/behavior_velocity_planner_common/planner_data.hpp"
#include "autoware/behavior_velocity_planner_common/scene_module_interface.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{
namespace bvp = autoware::behavior_velocity_planner;
using autoware::planning_factor_interface::PlanningFactorInterface;
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_utils_debug::TimeKeeper;
using visualization_msgs::msg::MarkerArray;

constexpr const char * module_name = "test_module";
constexpr const char * node_name = "test_node";

// Waits until `predicate` is true, for at most 10 seconds.
bool wait_until(const std::function<bool()> & predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!predicate()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

struct CallCounts
{
  int debug_markers{0};
  int virtual_walls{0};
};

// A scene module that counts how many times its debug markers and virtual walls are built.
class CountingSceneModule : public bvp::SceneModuleInterface
{
public:
  CountingSceneModule(
    rclcpp::Node & node, const std::shared_ptr<TimeKeeper> & time_keeper,
    const std::shared_ptr<PlanningFactorInterface> & planning_factor_interface, CallCounts & counts)
  : SceneModuleInterface(
      1, node.get_logger(), node.get_clock(), time_keeper, planning_factor_interface),
    counts_(counts)
  {
  }

  bool modifyPathVelocity(PathWithLaneId * /*path*/) override { return true; }

  MarkerArray createDebugMarkerArray() override
  {
    ++counts_.debug_markers;
    MarkerArray markers;
    markers.markers.emplace_back();
    return markers;
  }

  std::vector<autoware::motion_utils::VirtualWall> createVirtualWalls() override
  {
    ++counts_.virtual_walls;
    return {};
  }

private:
  CallCounts & counts_;
};

class TestManager : public bvp::SceneModuleManagerInterface<bvp::SceneModuleInterface>
{
public:
  explicit TestManager(rclcpp::Node & node) : SceneModuleManagerInterface(node, module_name) {}

  const char * getModuleName() override { return module_name; }

  bvp::RequiredSubscriptionInfo getRequiredSubscriptions() const override { return {}; }

  void addCountingModule(CallCounts & counts)
  {
    registerModule(
      std::make_shared<CountingSceneModule>(
        node_, time_keeper_, planning_factor_interface_, counts));
  }

  // Same check as isDebugMarkerSubscribed(), written here so that this test also builds without it.
  size_t debugMarkerSubscriptionCount() const { return pub_debug_->get_subscription_count(); }

protected:
  void launchNewModules(const PathWithLaneId & /*path*/) override {}

  std::function<bool(const std::shared_ptr<bvp::SceneModuleInterface> &)> getModuleExpiredFunction(
    const PathWithLaneId & /*path*/) override
  {
    return [](const std::shared_ptr<bvp::SceneModuleInterface> & /*module*/) { return false; };
  }
};

// The same scene module for the experimental interface.
class CountingExperimentalSceneModule : public bvp::experimental::SceneModuleInterface
{
public:
  CountingExperimentalSceneModule(
    rclcpp::Node & node, const std::shared_ptr<TimeKeeper> & time_keeper,
    const std::shared_ptr<PlanningFactorInterface> & planning_factor_interface, CallCounts & counts)
  : SceneModuleInterface(
      1, node.get_logger(), node.get_clock(), time_keeper, planning_factor_interface),
    counts_(counts)
  {
  }

  bool modifyPathVelocity(
    bvp::experimental::Trajectory & /*path*/,
    const std::vector<geometry_msgs::msg::Point> & /*left_bound*/,
    const std::vector<geometry_msgs::msg::Point> & /*right_bound*/,
    const bvp::PlannerData & /*planner_data*/) override
  {
    return true;
  }

  MarkerArray createDebugMarkerArray() override
  {
    ++counts_.debug_markers;
    MarkerArray markers;
    markers.markers.emplace_back();
    return markers;
  }

  std::vector<autoware::motion_utils::VirtualWall> createVirtualWalls() override
  {
    ++counts_.virtual_walls;
    return {};
  }

private:
  CallCounts & counts_;
};

class TestExperimentalManager
: public bvp::experimental::SceneModuleManagerInterface<bvp::experimental::SceneModuleInterface>
{
public:
  explicit TestExperimentalManager(rclcpp::Node & node)
  : SceneModuleManagerInterface(node, module_name)
  {
  }

  const char * getModuleName() override { return module_name; }

  bvp::RequiredSubscriptionInfo getRequiredSubscriptions() const override { return {}; }

  void addCountingModule(CallCounts & counts, const bvp::PlannerData & planner_data)
  {
    registerModule(
      std::make_shared<CountingExperimentalSceneModule>(
        node_, time_keeper_, planning_factor_interface_, counts),
      planner_data);
  }

  // Same check as isDebugMarkerSubscribed(), written here so that this test also builds without it.
  size_t debugMarkerSubscriptionCount() const { return pub_debug_->get_subscription_count(); }

protected:
  void launchNewModules(
    const bvp::experimental::Trajectory & /*path*/, const rclcpp::Time & /*stamp*/,
    const bvp::PlannerData & /*planner_data*/) override
  {
  }

  std::function<bool(const std::shared_ptr<bvp::experimental::SceneModuleInterface> &)>
  getModuleExpiredFunction(
    const bvp::experimental::Trajectory & /*path*/,
    const bvp::PlannerData & /*planner_data*/) override
  {
    return [](const std::shared_ptr<bvp::experimental::SceneModuleInterface> & /*module*/) {
      return false;
    };
  }
};

class SceneModuleManagerDebugMarkerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_behavior_velocity_planner_common") +
         "/config/behavior_velocity_planner_common.param.yaml",
       "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_test_utils") +
         "/config/test_vehicle_info.param.yaml"});
    options.parameter_overrides(
      {{"planning_factor_console_output.enable", false},
       {"planning_factor_console_output.duration", 1000}});
    node_ = std::make_shared<rclcpp::Node>(node_name, options);
    subscriber_node_ = std::make_shared<rclcpp::Node>("test_debug_marker_subscriber");
  }

  void TearDown() override
  {
    subscription_.reset();
    subscriber_node_.reset();
    node_.reset();
  }

  void subscribeDebugMarkers()
  {
    subscription_ = subscriber_node_->create_subscription<MarkerArray>(
      std::string("/") + node_name + "/debug/" + module_name, 1,
      [this](const MarkerArray::ConstSharedPtr /*msg*/) { ++received_count_; });
  }

  bool spinUntilReceived()
  {
    return wait_until([this]() {
      rclcpp::spin_some(subscriber_node_);
      return received_count_ > 0;
    });
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> subscriber_node_;
  rclcpp::Subscription<MarkerArray>::SharedPtr subscription_;
  int received_count_{0};
};

TEST_F(SceneModuleManagerDebugMarkerTest, BuildsDebugMarkersOnlyWhenSubscribed)
{
  // Arrange
  TestManager manager(*node_);
  CallCounts counts;
  manager.addCountingModule(counts);
  PathWithLaneId path;

  // Act and assert: no subscriber, so no debug markers are built. Virtual walls still are.
  manager.plan(&path);
  EXPECT_EQ(counts.debug_markers, 0);
  EXPECT_EQ(counts.virtual_walls, 1);

  // Act and assert: with a subscriber, the debug markers are built and published.
  subscribeDebugMarkers();
  ASSERT_TRUE(wait_until([&manager]() { return manager.debugMarkerSubscriptionCount() > 0; }));
  manager.plan(&path);
  EXPECT_EQ(counts.debug_markers, 1);
  EXPECT_EQ(counts.virtual_walls, 2);
  EXPECT_TRUE(spinUntilReceived());
}

TEST_F(SceneModuleManagerDebugMarkerTest, ExperimentalBuildsDebugMarkersOnlyWhenSubscribed)
{
  // Arrange
  const bvp::PlannerData planner_data(*node_);
  TestExperimentalManager manager(*node_);
  CallCounts counts;
  manager.addCountingModule(counts, planner_data);

  std::vector<PathPointWithLaneId> points(10);
  for (size_t i = 0; i < points.size(); ++i) {
    points.at(i).point.pose.position.x = static_cast<double>(i);
    points.at(i).lane_ids = {1};
  }
  auto path = bvp::experimental::Trajectory::Builder{}.build(points);
  ASSERT_TRUE(path);
  const std_msgs::msg::Header header;
  const std::vector<geometry_msgs::msg::Point> left_bound;
  const std::vector<geometry_msgs::msg::Point> right_bound;

  // Act and assert: no subscriber, so no debug markers are built. Virtual walls still are.
  manager.plan(*path, header, left_bound, right_bound, planner_data);
  EXPECT_EQ(counts.debug_markers, 0);
  EXPECT_EQ(counts.virtual_walls, 1);

  // Act and assert: with a subscriber, the debug markers are built and published.
  subscribeDebugMarkers();
  ASSERT_TRUE(wait_until([&manager]() { return manager.debugMarkerSubscriptionCount() > 0; }));
  manager.plan(*path, header, left_bound, right_bound, planner_data);
  EXPECT_EQ(counts.debug_markers, 1);
  EXPECT_EQ(counts.virtual_walls, 2);
  EXPECT_TRUE(spinUntilReceived());
}
}  // namespace
