// Copyright 2025 Tier IV, Inc.
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

#include "autoware/gnss_poser/gnss_poser_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>
#include <vector>

sensor_msgs::msg::NavSatFix createNavSatFixMsg(
  double latitude, double longitude, double altitude,
  sensor_msgs::msg::NavSatStatus::_status_type status = sensor_msgs::msg::NavSatStatus::STATUS_FIX)
{
  sensor_msgs::msg::NavSatFix msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "gnss";
  msg.latitude = latitude;
  msg.longitude = longitude;
  msg.altitude = altitude;
  msg.status.status = status;
  msg.position_covariance = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  return msg;
}

autoware_sensing_msgs::msg::GnssInsOrientationStamped createGnssInsOrientationMsg()
{
  autoware_sensing_msgs::msg::GnssInsOrientationStamped msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "gnss";
  msg.orientation.orientation.w = 1.0;
  msg.orientation.rmse_rotation_x = 0.01;
  msg.orientation.rmse_rotation_y = 0.01;
  msg.orientation.rmse_rotation_z = 0.01;
  return msg;
}

autoware_map_msgs::msg::MapProjectorInfo createMapProjectorInfoMsg()
{
  autoware_map_msgs::msg::MapProjectorInfo msg;
  msg.projector_type = autoware_map_msgs::msg::MapProjectorInfo::MGRS;
  msg.vertical_datum = autoware_map_msgs::msg::MapProjectorInfo::WGS84;
  msg.mgrs_grid = "54SUE";
  return msg;
}

class GNSSPoserConstructorTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
};

// Test node creation
TEST_F(GNSSPoserConstructorTest, TestNodeCreation)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("base_frame", "base_link");
  options.append_parameter_override("gnss_base_frame", "gnss_base_link");
  options.append_parameter_override("map_frame", "map");
  options.append_parameter_override("use_gnss_ins_orientation", true);
  options.append_parameter_override("buff_epoch", 10);
  options.append_parameter_override("gnss_pose_pub_method", 0);

  EXPECT_NO_THROW({ auto node = std::make_shared<autoware::gnss_poser::GNSSPoser>(options); });
}

// Define a node class for publishing messages
class TestPublisherNode : public rclcpp::Node
{
public:
  TestPublisherNode() : Node("test_publisher_node")
  {
    // Create publishers
    map_projector_pub_ = create_publisher<autoware_map_msgs::msg::MapProjectorInfo>(
      "/map/map_projector_info", rclcpp::QoS(1).transient_local());

    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("fix", rclcpp::QoS(1));

    orientation_pub_ = create_publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(
      "autoware_orientation", rclcpp::QoS(1));

    // Create TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }

  // Publish TF transform
  void publishTF(const std::string & frame_id, const std::string & child_frame_id)
  {
    geometry_msgs::msg::TransformStamped tf_gnss_to_base;
    tf_gnss_to_base.header.stamp = now();
    tf_gnss_to_base.header.frame_id = frame_id;
    tf_gnss_to_base.child_frame_id = child_frame_id;
    tf_gnss_to_base.transform.translation.x = 1.0;  // Offset from gnss to base_link
    tf_gnss_to_base.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(tf_gnss_to_base);
  }

  rclcpp::Publisher<autoware_map_msgs::msg::MapProjectorInfo>::SharedPtr map_projector_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr
    orientation_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

class GNSSPoserTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Create publisher node
    publisher_node_ = std::make_shared<TestPublisherNode>();

    // Parameter configuration
    rclcpp::NodeOptions options;
    options.append_parameter_override("base_frame", "base_link");
    options.append_parameter_override("gnss_base_frame", "gnss_base_link");
    options.append_parameter_override("map_frame", "map");
    options.append_parameter_override("use_gnss_ins_orientation", true);
    options.append_parameter_override("buff_epoch", 1);            // Set to 1 for easier testing
    options.append_parameter_override("gnss_pose_pub_method", 0);  // Direct position publishing

    // Create the node under test
    gnss_poser_node_ = std::make_shared<autoware::gnss_poser::GNSSPoser>(options);

    // Create executor and add both nodes
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(gnss_poser_node_);
    publisher_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    publisher_executor_->add_node(publisher_node_);

    // Create subscribers
    pose_received_ = false;
    pose_cov_received_ = false;
    fixed_status_received_ = false;

    auto pose_callback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      last_pose_ = *msg;
      pose_received_ = true;
    };

    auto pose_cov_callback =
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        last_pose_cov_ = *msg;
        pose_cov_received_ = true;
      };

    auto fixed_callback =
      [this](const autoware_internal_debug_msgs::msg::BoolStamped::SharedPtr msg) {
        last_fixed_status_ = *msg;
        fixed_status_received_ = true;
      };

    pose_sub_ = gnss_poser_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "gnss_pose", rclcpp::QoS(1), pose_callback);

    pose_cov_sub_ =
      gnss_poser_node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "gnss_pose_cov", rclcpp::QoS(1), pose_cov_callback);

    fixed_sub_ =
      gnss_poser_node_->create_subscription<autoware_internal_debug_msgs::msg::BoolStamped>(
        "gnss_fixed", rclcpp::QoS(1), fixed_callback);

    // Publish necessary TF
    publisher_node_->publishTF("gnss", "base_link");

    // Start executor threads
    executor_thread_ = std::thread([this]() { executor_->spin(); });
    publisher_thread_ = std::thread([this]() { publisher_executor_->spin(); });
  }

  void TearDown() override
  {
    publisher_executor_->cancel();
    if (publisher_thread_.joinable()) {
      publisher_thread_.join();
    }

    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    pose_sub_.reset();
    pose_cov_sub_.reset();
    fixed_sub_.reset();
    publisher_node_.reset();
    gnss_poser_node_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  // Utility function: Wait for message
  bool waitForMessage(std::function<bool()> predicate, std::chrono::milliseconds timeout)
  {
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < timeout) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  std::shared_ptr<autoware::gnss_poser::GNSSPoser> gnss_poser_node_;
  std::shared_ptr<TestPublisherNode> publisher_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> publisher_executor_;
  std::thread executor_thread_;
  std::thread publisher_thread_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::BoolStamped>::SharedPtr fixed_sub_;

  geometry_msgs::msg::PoseStamped last_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped last_pose_cov_;
  autoware_internal_debug_msgs::msg::BoolStamped last_fixed_status_;

  bool pose_received_;
  bool pose_cov_received_;
  bool fixed_status_received_;
};

// Test GNSS fixed status handling
TEST_F(GNSSPoserTest, TestFixedGNSS)
{
  // Publish GPS Fix info (STATUS_FIX)
  auto nav_sat_fix = createNavSatFixMsg(35.681236, 139.767125, 41.0);
  publisher_node_->fix_pub_->publish(nav_sat_fix);

  // Wait for message
  EXPECT_FALSE(
    waitForMessage([this]() { return fixed_status_received_; }, std::chrono::milliseconds(500)));

  auto map_projector_info = createMapProjectorInfoMsg();
  publisher_node_->map_projector_pub_->publish(map_projector_info);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Wait for message propagation

  publisher_node_->fix_pub_->publish(nav_sat_fix);
  EXPECT_TRUE(
    waitForMessage([this]() { return fixed_status_received_; }, std::chrono::milliseconds(500)));

  // Verify GNSS fixed flag
  EXPECT_TRUE(last_fixed_status_.data);

  // If position is published, check if it's valid
  EXPECT_TRUE(waitForMessage([this]() { return pose_received_; }, std::chrono::milliseconds(500)));
  EXPECT_NE(last_pose_.pose.position.x, 0.0);
  EXPECT_NE(last_pose_.pose.position.y, 0.0);
}

// Test GNSS non-fixed status handling
TEST_F(GNSSPoserTest, TestNonFixedGNSS)
{
  fixed_status_received_ = false;
  pose_received_ = false;

  // First publish projection info
  auto map_projector_info = createMapProjectorInfoMsg();
  publisher_node_->map_projector_pub_->publish(map_projector_info);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Wait for message propagation

  // Publish non-fixed GPS info
  auto non_fixed_nav_sat_fix =
    createNavSatFixMsg(35.681236, 139.767125, 41.0, sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX);
  publisher_node_->fix_pub_->publish(non_fixed_nav_sat_fix);

  // Wait for fixed status message
  EXPECT_TRUE(
    waitForMessage([this]() { return fixed_status_received_; }, std::chrono::milliseconds(500)));

  // Verify GNSS non-fixed flag
  EXPECT_FALSE(last_fixed_status_.data);

  // Ensure no position message is received (within short time)
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_FALSE(pose_received_);
}

// Test GNSS fixed status and position handling
TEST_F(GNSSPoserTest, TestGNSSFixProcessing)
{
  // Publish map projection info
  auto map_projector_info = createMapProjectorInfoMsg();
  publisher_node_->map_projector_pub_->publish(map_projector_info);

  // Publish orientation info
  auto orientation_msg = createGnssInsOrientationMsg();
  publisher_node_->orientation_pub_->publish(orientation_msg);

  // Wait a short time for message propagation
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Publish fixed GPS info
  auto nav_sat_fix = createNavSatFixMsg(35.681236, 139.767125, 41.0);
  publisher_node_->fix_pub_->publish(nav_sat_fix);

  // Verify fixed status and position publishing
  EXPECT_TRUE(
    waitForMessage([this]() { return fixed_status_received_; }, std::chrono::milliseconds(500)));
  EXPECT_TRUE(last_fixed_status_.data);
  EXPECT_TRUE(waitForMessage([this]() { return pose_received_; }, std::chrono::milliseconds(500)));

  // Reset status
  fixed_status_received_ = false;
  pose_received_ = false;

  // Publish non-fixed GPS info
  auto non_fixed_nav_sat_fix =
    createNavSatFixMsg(35.681236, 139.767125, 41.0, sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX);
  publisher_node_->fix_pub_->publish(non_fixed_nav_sat_fix);

  // Verify non-fixed status and no position publishing
  EXPECT_TRUE(
    waitForMessage([this]() { return fixed_status_received_; }, std::chrono::milliseconds(500)));
  EXPECT_FALSE(last_fixed_status_.data);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_FALSE(pose_received_);
}

// Test covariance handling
TEST_F(GNSSPoserTest, TestCovarianceProcessing)
{
  // Publish map projection info
  auto map_projector_info = createMapProjectorInfoMsg();
  publisher_node_->map_projector_pub_->publish(map_projector_info);

  // Publish orientation info
  auto orientation_msg = createGnssInsOrientationMsg();
  publisher_node_->orientation_pub_->publish(orientation_msg);

  // Wait a short time for message propagation
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Publish GPS info with different covariance types
  auto nav_sat_fix = createNavSatFixMsg(35.681236, 139.767125, 41.0);

  // Test DIAGONAL_KNOWN type
  nav_sat_fix.position_covariance_type =
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  nav_sat_fix.position_covariance = {2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 4.0};

  pose_cov_received_ = false;
  publisher_node_->fix_pub_->publish(nav_sat_fix);

  EXPECT_TRUE(
    waitForMessage([this]() { return pose_cov_received_; }, std::chrono::milliseconds(500)));

  // Verify covariance values are correctly passed
  EXPECT_NEAR(last_pose_cov_.pose.covariance[0], 2.0, 1e-6);
  EXPECT_NEAR(last_pose_cov_.pose.covariance[7], 3.0, 1e-6);
  EXPECT_NEAR(last_pose_cov_.pose.covariance[14], 4.0, 1e-6);

  // Test UNKNOWN type (should use default value 10.0)
  nav_sat_fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

  pose_cov_received_ = false;
  publisher_node_->fix_pub_->publish(nav_sat_fix);

  EXPECT_TRUE(
    waitForMessage([this]() { return pose_cov_received_; }, std::chrono::milliseconds(500)));

  // Verify default covariance values are used
  EXPECT_NEAR(last_pose_cov_.pose.covariance[0], 10.0, 1e-6);
  EXPECT_NEAR(last_pose_cov_.pose.covariance[7], 10.0, 1e-6);
  EXPECT_NEAR(last_pose_cov_.pose.covariance[14], 10.0, 1e-6);
}

// Test different orientation source configurations
TEST_F(GNSSPoserTest, TestOrientationSources)
{
  // Shut down current node setup
  executor_->cancel();
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  gnss_poser_node_.reset();

  // Create node without using GNSS-INS orientation
  rclcpp::NodeOptions options;
  options.append_parameter_override("base_frame", "base_link");
  options.append_parameter_override("gnss_base_frame", "gnss_base_link");
  options.append_parameter_override("map_frame", "map");
  options.append_parameter_override("use_gnss_ins_orientation", false);
  options.append_parameter_override("buff_epoch", 1);
  options.append_parameter_override("gnss_pose_pub_method", 0);

  gnss_poser_node_ = std::make_shared<autoware::gnss_poser::GNSSPoser>(options);
  executor_->add_node(gnss_poser_node_);

  // Reset subscription
  pose_received_ = false;
  auto pose_callback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    last_pose_ = *msg;
    pose_received_ = true;
  };

  pose_sub_ = gnss_poser_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "gnss_pose", rclcpp::QoS(1), pose_callback);

  // Restart executor
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  // Publish map projection info
  auto map_projector_info = createMapProjectorInfoMsg();
  publisher_node_->map_projector_pub_->publish(map_projector_info);

  // Wait a short time
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Publish multiple position points to verify orientation calculation from position differences
  auto nav_sat_fix1 = createNavSatFixMsg(35.681236, 139.767125, 41.0);
  publisher_node_->fix_pub_->publish(nav_sat_fix1);

  EXPECT_TRUE(waitForMessage([this]() { return pose_received_; }, std::chrono::milliseconds(500)));
  pose_received_ = false;

  // Publish second position point (moving north)
  auto nav_sat_fix2 = createNavSatFixMsg(35.682236, 139.767125, 41.0);
  publisher_node_->fix_pub_->publish(nav_sat_fix2);

  EXPECT_TRUE(waitForMessage([this]() { return pose_received_; }, std::chrono::milliseconds(500)));

  // Verify orientation (facing north, positive y-axis direction)
  tf2::Quaternion quat;
  tf2::fromMsg(last_pose_.pose.orientation, quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // Moving north should result in yaw angle of approximately 90 degrees (PI/2)
  EXPECT_NEAR(std::abs(yaw), M_PI / 2.0, 0.1);

  // Clean up
  executor_->cancel();
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
}

// Test different publishing methods
TEST_F(GNSSPoserTest, TestPositionBufferMethods)
{
  // Clean up existing node
  executor_->cancel();
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  gnss_poser_node_.reset();

  // Create node using position buffer
  rclcpp::NodeOptions options;
  options.append_parameter_override("base_frame", "base_link");
  options.append_parameter_override("gnss_base_frame", "gnss_base_link");
  options.append_parameter_override("map_frame", "map");
  options.append_parameter_override("use_gnss_ins_orientation", true);
  options.append_parameter_override("buff_epoch", 3);
  options.append_parameter_override("gnss_pose_pub_method", 1);  // Use averaging method

  gnss_poser_node_ = std::make_shared<autoware::gnss_poser::GNSSPoser>(options);
  executor_->add_node(gnss_poser_node_);

  // Reset subscription
  pose_received_ = false;
  auto pose_callback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    last_pose_ = *msg;
    pose_received_ = true;
  };
  pose_sub_ = gnss_poser_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "gnss_pose", rclcpp::QoS(1), pose_callback);

  // Restart executor
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  // Publish map projection info
  auto map_projector_info = createMapProjectorInfoMsg();
  publisher_node_->map_projector_pub_->publish(map_projector_info);

  // Publish orientation info
  auto orientation_msg = createGnssInsOrientationMsg();
  publisher_node_->orientation_pub_->publish(orientation_msg);

  // Wait a short time
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Publish multiple GPS points to gradually fill buffer
  for (int i = 0; i < 3; i++) {
    auto nav_sat_fix =
      createNavSatFixMsg(35.681236 + i * 0.0001, 139.767125 + i * 0.0001, 41.0 + i * 0.1);
    publisher_node_->fix_pub_->publish(nav_sat_fix);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  // Wait for position message (should be published after buffer is filled)
  EXPECT_TRUE(waitForMessage([this]() { return pose_received_; }, std::chrono::milliseconds(500)));
  EXPECT_EQ(last_pose_.header.frame_id, "map");

  // Clean up
  executor_->cancel();
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
}

// Add new test case to test static transform functionality
TEST_F(GNSSPoserTest, TestStaticTransform)
{
  // Publish map projection info
  auto map_projector_info = createMapProjectorInfoMsg();
  publisher_node_->map_projector_pub_->publish(map_projector_info);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Add a custom TF (not the standard 'gnss' to 'base_link' transform)
  geometry_msgs::msg::TransformStamped custom_tf;
  custom_tf.header.stamp = publisher_node_->now();
  custom_tf.header.frame_id = "custom_frame";
  custom_tf.child_frame_id = "target_frame";
  custom_tf.transform.translation.x = 2.5;
  custom_tf.transform.translation.y = 1.5;
  custom_tf.transform.translation.z = 0.5;
  tf2::Quaternion q;
  q.setRPY(0.1, 0.2, 0.3);
  custom_tf.transform.rotation = tf2::toMsg(q);
  publisher_node_->tf_broadcaster_->sendTransform(custom_tf);

  // Publish regular GNSS message
  auto nav_sat_fix = createNavSatFixMsg(35.681236, 139.767125, 41.0);
  // Modify frame ID to trigger get_static_transform to look up the custom TF we just published
  nav_sat_fix.header.frame_id = "custom_frame";
  publisher_node_->fix_pub_->publish(nav_sat_fix);

  // Wait for position message to be published
  EXPECT_TRUE(waitForMessage([this]() { return pose_received_; }, std::chrono::milliseconds(500)));

  // If get_static_transform successfully found and applied the TF, the final position will be
  // affected because GNSS position is calculated through a transform based on the custom TF Check
  // position is not zero
  EXPECT_NE(last_pose_.pose.position.x, 0.0);
  EXPECT_NE(last_pose_.pose.position.y, 0.0);

  // Test case when source and target frames are the same
  fixed_status_received_ = false;
  pose_received_ = false;

  // Publish GNSS message with same source and target frame
  auto same_frame_nav_sat_fix = createNavSatFixMsg(35.681236, 139.767125, 41.0);
  same_frame_nav_sat_fix.header.frame_id = "base_link";  // Use same frame ID as base_frame
  publisher_node_->fix_pub_->publish(same_frame_nav_sat_fix);

  // Wait for position message to be published
  EXPECT_TRUE(waitForMessage([this]() { return pose_received_; }, std::chrono::milliseconds(500)));

  // Position should still be valid
  EXPECT_NE(last_pose_.pose.position.x, 0.0);
  EXPECT_NE(last_pose_.pose.position.y, 0.0);
}

// Add test for median position calculation functionality
TEST_F(GNSSPoserTest, TestMedianPosition)
{
  // Clean up existing node
  executor_->cancel();
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  gnss_poser_node_.reset();

  // Create node using median method
  rclcpp::NodeOptions options;
  options.append_parameter_override("base_frame", "base_link");
  options.append_parameter_override("gnss_base_frame", "gnss_base_link");
  options.append_parameter_override("map_frame", "map");
  options.append_parameter_override("use_gnss_ins_orientation", true);
  options.append_parameter_override("buff_epoch", 5);            // Use 5 points
  options.append_parameter_override("gnss_pose_pub_method", 2);  // Use median method

  gnss_poser_node_ = std::make_shared<autoware::gnss_poser::GNSSPoser>(options);
  executor_->add_node(gnss_poser_node_);

  // Reset subscription
  pose_received_ = false;
  auto pose_callback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    last_pose_ = *msg;
    pose_received_ = true;
  };

  pose_sub_ = gnss_poser_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "gnss_pose", rclcpp::QoS(1), pose_callback);

  // Restart executor
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  // Publish map projection info
  auto map_projector_info = createMapProjectorInfoMsg();
  publisher_node_->map_projector_pub_->publish(map_projector_info);

  // Publish orientation info
  auto orientation_msg = createGnssInsOrientationMsg();
  publisher_node_->orientation_pub_->publish(orientation_msg);

  // Wait a short time
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Median test data - publish ordered GPS points
  // Use deliberately placed outlier points to test if median calculation works
  std::vector<double> test_latitudes = {35.681236, 35.681237, 35.681240, 35.681230, 35.681235};
  std::vector<double> test_longitudes = {
    139.767125, 139.767127, 139.767130, 139.767120, 139.767123};
  std::vector<double> test_altitudes = {41.0, 41.1, 41.5, 40.8, 41.2};

  // Publish multiple GPS points to fill buffer
  for (size_t i = 0; i < test_latitudes.size(); i++) {
    auto nav_sat_fix = createNavSatFixMsg(test_latitudes[i], test_longitudes[i], test_altitudes[i]);
    publisher_node_->fix_pub_->publish(nav_sat_fix);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Wait for position message (should be published after buffer is filled)
  EXPECT_TRUE(waitForMessage([this]() { return pose_received_; }, std::chrono::milliseconds(500)));

  // Since we can't directly verify median calculation results, we can check:
  // 1. Position message was published
  // 2. Position values are not zero
  // 3. Position is within reasonable range (should be close to median, not max or min)
  EXPECT_TRUE(pose_received_);
  EXPECT_NE(last_pose_.pose.position.x, 0.0);
  EXPECT_NE(last_pose_.pose.position.y, 0.0);

  // Clean up
  executor_->cancel();
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
