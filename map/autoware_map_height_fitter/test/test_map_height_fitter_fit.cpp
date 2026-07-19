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

#include "autoware/map_height_fitter/map_height_fitter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <thread>

namespace
{

using autoware::map_height_fitter::MapHeightFitter;
using geometry_msgs::msg::Point;

// map -> test_frame translation
constexpr double frame_tx = 100.0;
constexpr double frame_ty = 200.0;
constexpr double frame_tz = 30.0;

// query position in map coordinates and the ground height there
constexpr double query_map_x = 10.0;
constexpr double query_map_y = 20.0;
constexpr double ground_z = 5.0;

// ground height around the "mirrored" location (query - 2 * translation), where an
// inverted frame -> map transform would sample the ground instead
constexpr double mirrored_ground_z = 9.0;

void add_ground_patch(
  pcl::PointCloud<pcl::PointXYZ> & cloud, double center_x, double center_y, double z)
{
  for (double dx = -3.0; dx <= 3.0; dx += 0.5) {
    for (double dy = -3.0; dy <= 3.0; dy += 0.5) {
      cloud.push_back(
        pcl::PointXYZ(
          static_cast<float>(center_x + dx), static_cast<float>(center_y + dy),
          static_cast<float>(z)));
    }
  }
}

sensor_msgs::msg::PointCloud2 make_map_cloud()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  add_ground_patch(cloud, query_map_x, query_map_y, ground_z);
  add_ground_patch(
    cloud, query_map_x - 2.0 * frame_tx, query_map_y - 2.0 * frame_ty, mirrored_ground_z);
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "map";
  return msg;
}

}  // namespace

class MapHeightFitterFitTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions fitter_options;
    fitter_options.parameter_overrides(
      {{"map_height_fitter.target", "pointcloud_map"},
       {"map_height_fitter.map_loader_name", "/map_loader"}});
    fitter_node_ = std::make_shared<rclcpp::Node>("map_height_fitter_test", fitter_options);

    // stands in for pointcloud_map_loader: only its parameter service is used
    rclcpp::NodeOptions loader_options;
    loader_options.parameter_overrides({{"enable_partial_load", false}});
    loader_options.automatically_declare_parameters_from_overrides(true);
    loader_node_ = std::make_shared<rclcpp::Node>("map_loader", loader_options);

    executor_.add_node(fitter_node_);
    executor_.add_node(loader_node_);
    spin_thread_ = std::thread([this] { executor_.spin(); });

    fitter_ = std::make_unique<MapHeightFitter>(fitter_node_.get());

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(loader_node_);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "map";
    tf.child_frame_id = "test_frame";
    tf.transform.translation.x = frame_tx;
    tf.transform.translation.y = frame_ty;
    tf.transform.translation.z = frame_tz;
    tf.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf);

    map_pub_ = loader_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/map_height_fitter_test/pointcloud_map", rclcpp::QoS(1).transient_local());
    map_pub_->publish(make_map_cloud());
  }

  void TearDown() override
  {
    executor_.cancel();
    spin_thread_.join();
  }

  // The map subscription is created asynchronously by the parameter client callback and the
  // map / TF messages also arrive asynchronously. Probe with a map-frame query (independent of
  // the frame conversion under test) until the map is loaded and the ground height is served.
  bool wait_until_map_ready()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
      Point probe;
      probe.x = query_map_x;
      probe.y = query_map_y;
      probe.z = 1e9;  // fallback value, returned while the map is not fully processed yet
      const auto result = fitter_->fit(probe, "map");
      if (result && std::abs(result->z - ground_z) < 1e-3) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  }

  // retry only while fit() reports "not ready" (e.g. the TF is not received yet)
  std::optional<Point> fit_when_available(const Point & position, const std::string & frame)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
      const auto result = fitter_->fit(position, frame);
      if (result) {
        return result;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return std::nullopt;
  }

  rclcpp::Node::SharedPtr fitter_node_;
  rclcpp::Node::SharedPtr loader_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  std::unique_ptr<MapHeightFitter> fitter_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
};

TEST_F(MapHeightFitterFitTest, fit_in_map_frame_returns_ground_height)
{
  ASSERT_TRUE(wait_until_map_ready());

  Point position;
  position.x = query_map_x;
  position.y = query_map_y;
  position.z = 123.0;

  const auto result = fit_when_available(position, "map");

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->x, query_map_x, 1e-6);
  EXPECT_NEAR(result->y, query_map_y, 1e-6);
  EXPECT_NEAR(result->z, ground_z, 1e-3);
}

TEST_F(MapHeightFitterFitTest, fit_in_translated_frame_returns_height_in_request_frame)
{
  ASSERT_TRUE(wait_until_map_ready());

  // the query position expressed in test_frame
  Point position;
  position.x = query_map_x - frame_tx;
  position.y = query_map_y - frame_ty;
  position.z = 0.0;

  const auto result = fit_when_available(position, "test_frame");

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->x, position.x, 1e-6);
  EXPECT_NEAR(result->y, position.y, 1e-6);
  // the ground height at the query position is ground_z in map coordinates, which is
  // ground_z - frame_tz expressed in test_frame. With the frame -> map transform inverted
  // (regression: lookupTransform argument order), the ground would be sampled at the location
  // mirrored by twice the frame translation and the result offset the wrong way, yielding
  // mirrored_ground_z + frame_tz instead.
  EXPECT_NEAR(result->z, ground_z - frame_tz, 1e-3);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
