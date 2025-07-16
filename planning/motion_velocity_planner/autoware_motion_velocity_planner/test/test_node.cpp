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

#include "../src/node.hpp"

#include <autoware_motion_velocity_planner/srv/load_plugin.hpp>
#include <autoware_motion_velocity_planner/srv/unload_plugin.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using autoware::motion_velocity_planner::MotionVelocityPlannerNode;

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode() : Node("publisher_node")
  {
    trajectory_pub_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
      "/motion_velocity_planner/input/trajectory", rclcpp::QoS(1).transient_local());
    dynamic_objects_pub_ = create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
      "/motion_velocity_planner/input/dynamic_objects", rclcpp::QoS(1).transient_local());
    no_ground_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/motion_velocity_planner/input/no_ground_pointcloud",
      autoware_utils_rclcpp::single_depth_sensor_qos());
    vehicle_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/motion_velocity_planner/input/vehicle_odometry", rclcpp::QoS(1).transient_local());
    acceleration_pub_ = create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "/motion_velocity_planner/input/accel", rclcpp::QoS(1).transient_local());
    occupancy_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/motion_velocity_planner/input/occupancy_grid", rclcpp::QoS(1).transient_local());
    traffic_signals_pub_ = create_publisher<autoware_perception_msgs::msg::TrafficLightGroupArray>(
      "/motion_velocity_planner/input/traffic_signals", rclcpp::QoS(1).transient_local());
    lanelet_map_pub_ = create_publisher<autoware_map_msgs::msg::LaneletMapBin>(
      "/motion_velocity_planner/input/vector_map", rclcpp::QoS(1).transient_local());
    tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(1).transient_local());

    // construct topics
    test_data_file_ =
      ament_index_cpp::get_package_share_directory("autoware_motion_velocity_planner") +
      "/test_data/subscribed_topics.yaml";
    map_path_ = autoware::test_utils::get_absolute_path_to_lanelet_map(
      "autoware_motion_velocity_planner", "lanelet2_map.osm");
  }

  void construct_topics()
  {
    const auto config = YAML::LoadFile(test_data_file_);

    trajectory_ = convert_to_trajectory(
      autoware::test_utils::parse<autoware_internal_planning_msgs::msg::PathWithLaneId>(
        config["path_with_lane_id"]));
    dynamic_objects_ = autoware::test_utils::parse<autoware_perception_msgs::msg::PredictedObjects>(
      config["dynamic_object"]);
    route_ =
      autoware::test_utils::parse<autoware_planning_msgs::msg::LaneletRoute>(config["route"]);
    lanelet_map_ = autoware::test_utils::make_map_bin_msg(map_path_);
    traffic_signals_ =
      autoware::test_utils::parse<autoware_perception_msgs::msg::TrafficLightGroupArray>(
        config["traffic_signal"]);
    occupancy_grid_ = autoware::test_utils::makeCostMapMsg();
    self_odometry_ = autoware::test_utils::parse<nav_msgs::msg::Odometry>(config["self_odometry"]);
    self_acceleration_ =
      autoware::test_utils::parse<geometry_msgs::msg::AccelWithCovarianceStamped>(
        config["self_acceleration"]);
    no_ground_pointcloud_.header.stamp = this->now();
    no_ground_pointcloud_.header.frame_id = "base_link";
    no_ground_pointcloud_.height = 1;
    no_ground_pointcloud_.width = 0;  // Empty pointcloud
    tf2_ = autoware::test_utils::makeTFMsg(shared_from_this(), "base_link", "map");
  }

  autoware_planning_msgs::msg::Trajectory convert_to_trajectory(
    const autoware_internal_planning_msgs::msg::PathWithLaneId & path_with_lane_id)
  {
    autoware_planning_msgs::msg::Trajectory trajectory;
    trajectory.header = path_with_lane_id.header;
    for (const auto & point_with_lane_id : path_with_lane_id.points) {
      autoware_planning_msgs::msg::TrajectoryPoint trajectory_point;
      trajectory_point.pose = point_with_lane_id.point.pose;
      trajectory_point.longitudinal_velocity_mps =
        point_with_lane_id.point.longitudinal_velocity_mps;
      trajectory_point.lateral_velocity_mps = point_with_lane_id.point.lateral_velocity_mps;
      trajectory_point.heading_rate_rps = point_with_lane_id.point.heading_rate_rps;
      trajectory.points.push_back(trajectory_point);
    }
    return trajectory;
  }

  void publish_trajectory(const autoware_planning_msgs::msg::Trajectory & trajectory)
  {
    trajectory_pub_->publish(trajectory);
  }

  void publish_trajectory() { trajectory_pub_->publish(trajectory_); }

  void publish_dynamic_objects(
    const autoware_perception_msgs::msg::PredictedObjects & dynamic_objects)
  {
    dynamic_objects_pub_->publish(dynamic_objects);
  }

  void publish_dynamic_objects() { dynamic_objects_pub_->publish(dynamic_objects_); }

  void publish_no_ground_pointcloud(const sensor_msgs::msg::PointCloud2 & no_ground_pointcloud)
  {
    no_ground_pointcloud_pub_->publish(no_ground_pointcloud);
  }

  void publish_no_ground_pointcloud() { no_ground_pointcloud_pub_->publish(no_ground_pointcloud_); }

  void publish_vehicle_odometry(const nav_msgs::msg::Odometry & odometry)
  {
    vehicle_odometry_pub_->publish(odometry);
  }

  void publish_vehicle_odometry() { vehicle_odometry_pub_->publish(self_odometry_); }

  void publish_acceleration(const geometry_msgs::msg::AccelWithCovarianceStamped & acceleration)
  {
    acceleration_pub_->publish(acceleration);
  }

  void publish_acceleration() { acceleration_pub_->publish(self_acceleration_); }

  void publish_occupancy_grid(const nav_msgs::msg::OccupancyGrid & occupancy_grid)
  {
    occupancy_grid_pub_->publish(occupancy_grid);
  }

  void publish_occupancy_grid() { occupancy_grid_pub_->publish(occupancy_grid_); }

  void publish_traffic_signals(
    const autoware_perception_msgs::msg::TrafficLightGroupArray & traffic_signals)
  {
    traffic_signals_pub_->publish(traffic_signals);
  }

  void publish_traffic_signals() { traffic_signals_pub_->publish(traffic_signals_); }

  void publish_lanelet_map(const autoware_map_msgs::msg::LaneletMapBin & lanelet_map)
  {
    lanelet_map_pub_->publish(lanelet_map);
  }

  void publish_lanelet_map() { lanelet_map_pub_->publish(lanelet_map_); }

  void publish_tf(const tf2_msgs::msg::TFMessage & tf) { tf_pub_->publish(tf); }

  void publish_tf() { tf_pub_->publish(tf2_); }

  std::string test_data_file_;
  std::string map_path_;

  autoware_planning_msgs::msg::Trajectory trajectory_;
  autoware_perception_msgs::msg::PredictedObjects dynamic_objects_;
  autoware_planning_msgs::msg::LaneletRoute route_;
  autoware_map_msgs::msg::LaneletMapBin lanelet_map_;
  autoware_perception_msgs::msg::TrafficLightGroupArray traffic_signals_;
  nav_msgs::msg::OccupancyGrid occupancy_grid_;
  nav_msgs::msg::Odometry self_odometry_;
  geometry_msgs::msg::AccelWithCovarianceStamped self_acceleration_;
  sensor_msgs::msg::PointCloud2 no_ground_pointcloud_;
  tf2_msgs::msg::TFMessage tf2_;

  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr
    dynamic_objects_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr no_ground_pointcloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vehicle_odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acceleration_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::TrafficLightGroupArray>::SharedPtr
    traffic_signals_pub_;
  rclcpp::Publisher<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr lanelet_map_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
};

class ListenerNode : public rclcpp::Node
{
public:
  ListenerNode()
  : Node("topic_listener"),
    received_trajectory_(false),
    received_velocity_limit_(false),
    received_clear_velocity_limit_(false)
  {
    sub_trajectory_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
      "/motion_velocity_planner/output/trajectory", 1,
      std::bind(&ListenerNode::on_trajectory, this, std::placeholders::_1));

    sub_velocity_limit_ =
      this->create_subscription<autoware_internal_planning_msgs::msg::VelocityLimit>(
        "/motion_velocity_planner/output/velocity_limit", 1,
        std::bind(&ListenerNode::on_velocity_limit, this, std::placeholders::_1));

    sub_clear_velocity_limit_ =
      this->create_subscription<autoware_internal_planning_msgs::msg::VelocityLimitClearCommand>(
        "/motion_velocity_planner/output/clear_velocity_limit", 1,
        std::bind(&ListenerNode::on_clear_velocity_limit, this, std::placeholders::_1));
  }

  void on_trajectory(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    (void)msg;
    received_trajectory_ = true;
  }

  void on_velocity_limit(const autoware_internal_planning_msgs::msg::VelocityLimit::SharedPtr msg)
  {
    (void)msg;
    received_velocity_limit_ = true;
  }

  void on_clear_velocity_limit(
    const autoware_internal_planning_msgs::msg::VelocityLimitClearCommand::SharedPtr msg)
  {
    (void)msg;
    received_clear_velocity_limit_ = true;
  }

  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::VelocityLimit>::SharedPtr
    sub_velocity_limit_;
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::VelocityLimitClearCommand>::SharedPtr
    sub_clear_velocity_limit_;

  bool received_trajectory_;
  bool received_velocity_limit_;
  bool received_clear_velocity_limit_;
};

class TestMotionVelocityPlannerNode : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    std::vector<std::string> plugin_names = {
      "autoware::motion_velocity_planner::test_plugin",
      "autoware::motion_velocity_planner::ObstacleStopModule", ""};
    std::string launch_modules_param = "launch_modules:=[";
    for (const auto & plugin_name : plugin_names) {
      launch_modules_param += "'" + plugin_name + "',";
    }
    launch_modules_param += "]";

    const auto vehicle_info_config_file =
      ament_index_cpp::get_package_share_directory("autoware_motion_velocity_planner") +
      "/test_config/vehicle_info.param.yaml";
    const auto common_config_file =
      ament_index_cpp::get_package_share_directory("autoware_motion_velocity_planner") +
      "/test_config/common.param.yaml";
    const auto nearest_search_config_file =
      ament_index_cpp::get_package_share_directory("autoware_motion_velocity_planner") +
      "/test_config/nearest_search.param.yaml";
    const auto analytical_config_file =
      ament_index_cpp::get_package_share_directory("autoware_motion_velocity_planner") +
      "/test_config/autoware_velocity_smoother/Analytical.param.yaml";
    const auto velocity_smoother_config_file =
      ament_index_cpp::get_package_share_directory("autoware_motion_velocity_planner") +
      "/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml";
    const auto motion_velocity_planner_config_file =
      ament_index_cpp::get_package_share_directory("autoware_motion_velocity_planner") +
      "/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml";
    const auto obstacle_stop_config_file =
      ament_index_cpp::get_package_share_directory("autoware_motion_velocity_planner") +
      "/test_config/motion_velocity_planner/obstacle_stop.param.yaml";

    std::vector<std::string> args;
    args.push_back("--ros-args");
    args.push_back("--params-file");
    args.push_back(vehicle_info_config_file);
    args.push_back("--params-file");
    args.push_back(common_config_file);
    args.push_back("--params-file");
    args.push_back(nearest_search_config_file);
    args.push_back("--params-file");
    args.push_back(analytical_config_file);
    args.push_back("--params-file");
    args.push_back(velocity_smoother_config_file);
    args.push_back("--params-file");
    args.push_back(motion_velocity_planner_config_file);
    args.push_back("--params-file");
    args.push_back(obstacle_stop_config_file);
    args.push_back("-p");
    args.push_back(launch_modules_param);

    rclcpp::NodeOptions options;
    options.arguments(args);

    node_ = std::make_shared<MotionVelocityPlannerNode>(options);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    publisher_node_ = std::make_shared<PublisherNode>();
    publisher_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    publisher_executor_->add_node(publisher_node_);
    publisher_thread_ = std::thread([this]() { publisher_executor_->spin(); });

    listener_node_ = std::make_shared<ListenerNode>();
    listener_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    listener_executor_->add_node(listener_node_);
    listener_thread_ = std::thread([this]() { listener_executor_->spin(); });
  }

  bool request_load_plugin()
  {
    auto request = std::make_shared<autoware_motion_velocity_planner::srv::LoadPlugin::Request>();
    auto response = std::make_shared<autoware_motion_velocity_planner::srv::LoadPlugin::Response>();

    request->plugin_name = "test_plugin";

    auto client = node_->create_client<autoware_motion_velocity_planner::srv::LoadPlugin>(
      "/motion_velocity_planner/service/load_plugin");

    // auto future = client->async_send_request(request);
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      ADD_FAILURE() << "Service not available after waiting";
      return false;
    }

    std::promise<bool> response_received;
    std::future<bool> future = response_received.get_future();

    client->async_send_request(
      request, [&response_received](
                 rclcpp::Client<autoware_motion_velocity_planner::srv::LoadPlugin>::SharedFuture
                   future_result) {
        auto response = future_result.get();
        if (response != nullptr) {
          response_received.set_value(true);
        }
      });

    auto status = future.wait_for(std::chrono::seconds(1));
    EXPECT_EQ(status, std::future_status::ready);

    if (status != std::future_status::ready) {
      ADD_FAILURE() << "Service response not timeout";
      return false;
    }

    return true;
  }

  bool request_unload_plugin()
  {
    auto request = std::make_shared<autoware_motion_velocity_planner::srv::UnloadPlugin::Request>();
    auto response =
      std::make_shared<autoware_motion_velocity_planner::srv::UnloadPlugin::Response>();

    request->plugin_name = "test_plugin";

    auto client = node_->create_client<autoware_motion_velocity_planner::srv::UnloadPlugin>(
      "/motion_velocity_planner/service/unload_plugin");

    if (!client->wait_for_service(std::chrono::seconds(1))) {
      ADD_FAILURE() << "Service not available after waiting";
      return false;
    }

    std::promise<bool> response_received;
    std::future<bool> future = response_received.get_future();

    client->async_send_request(
      request, [&response_received](
                 rclcpp::Client<autoware_motion_velocity_planner::srv::UnloadPlugin>::SharedFuture
                   future_result) {
        auto response = future_result.get();
        if (response != nullptr) {
          response_received.set_value(true);
        }
      });

    auto status = future.wait_for(std::chrono::seconds(1));
    EXPECT_EQ(status, std::future_status::ready);

    if (status != std::future_status::ready) {
      ADD_FAILURE() << "Service response not timeout";
      return false;
    }

    return true;
  }

  void TearDown() override
  {
    publisher_executor_->cancel();
    publisher_executor_->remove_node(publisher_node_);
    if (publisher_thread_.joinable()) {
      publisher_thread_.join();
    }

    listener_executor_->cancel();
    listener_executor_->remove_node(listener_node_);
    if (listener_thread_.joinable()) {
      listener_thread_.join();
    }

    executor_->cancel();
    executor_->remove_node(node_);
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }

    executor_.reset();
    publisher_executor_.reset();
    listener_executor_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<MotionVelocityPlannerNode> node_;
  std::shared_ptr<PublisherNode> publisher_node_;
  std::shared_ptr<ListenerNode> listener_node_;

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> publisher_executor_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> listener_executor_;

  std::thread executor_thread_;
  std::thread publisher_thread_;
  std::thread listener_thread_;
};

TEST_F(TestMotionVelocityPlannerNode, set_param)
{
  EXPECT_TRUE(node_->get_parameter("smooth_velocity_before_planning").as_bool());
  node_->set_parameter(rclcpp::Parameter("smooth_velocity_before_planning", false));
  EXPECT_FALSE(node_->get_parameter("smooth_velocity_before_planning").as_bool());
}

TEST_F(TestMotionVelocityPlannerNode, request_load_plugin)
{
  EXPECT_TRUE(request_load_plugin());
}

TEST_F(TestMotionVelocityPlannerNode, request_unload_plugin)
{
  EXPECT_TRUE(request_unload_plugin());
}

TEST_F(TestMotionVelocityPlannerNode, trigger_main_logic)
{
  EXPECT_FALSE(
    listener_node_->received_trajectory_ && listener_node_->received_velocity_limit_ &&
    listener_node_->received_clear_velocity_limit_);

  // Publish necessary topics from publisher_node_
  publisher_node_->construct_topics();
  publisher_node_->publish_lanelet_map();
  publisher_node_->publish_dynamic_objects();
  publisher_node_->publish_no_ground_pointcloud();
  publisher_node_->publish_vehicle_odometry();
  publisher_node_->publish_acceleration();
  publisher_node_->publish_occupancy_grid();
  publisher_node_->publish_traffic_signals();
  publisher_node_->publish_tf();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  publisher_node_->publish_trajectory();

  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  EXPECT_TRUE(rclcpp::ok());
  EXPECT_TRUE(
    listener_node_->received_trajectory_ || listener_node_->received_velocity_limit_ ||
    listener_node_->received_clear_velocity_limit_);
}

TEST_F(TestMotionVelocityPlannerNode, trigger_traffic_light_logic)
{
  EXPECT_FALSE(
    listener_node_->received_trajectory_ && listener_node_->received_velocity_limit_ &&
    listener_node_->received_clear_velocity_limit_);

  publisher_node_->test_data_file_ =
    ament_index_cpp::get_package_share_directory("autoware_motion_velocity_planner") +
    "/test_data/test_traffic_light.yaml";
  publisher_node_->map_path_ = ament_index_cpp::get_package_share_directory("autoware_test_utils") +
                               "/test_map/intersection/lanelet2_map.osm";
  publisher_node_->construct_topics();

  publisher_node_->publish_lanelet_map();
  publisher_node_->publish_dynamic_objects();
  publisher_node_->publish_no_ground_pointcloud();
  publisher_node_->publish_vehicle_odometry();
  publisher_node_->publish_acceleration();
  publisher_node_->publish_occupancy_grid();
  publisher_node_->publish_traffic_signals();
  publisher_node_->publish_tf();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  publisher_node_->publish_trajectory();

  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  EXPECT_TRUE(rclcpp::ok());
  EXPECT_TRUE(
    listener_node_->received_trajectory_ || listener_node_->received_velocity_limit_ ||
    listener_node_->received_clear_velocity_limit_);
}
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
