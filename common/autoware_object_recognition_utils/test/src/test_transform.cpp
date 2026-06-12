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

#include "autoware/object_recognition_utils/transform.hpp"

#include <Eigen/Core>
#include <rclcpp/clock.hpp>

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{
autoware_perception_msgs::msg::DetectedObject createObject(
  const double x, const double y, const double z, const double yaw)
{
  autoware_perception_msgs::msg::DetectedObject object;
  auto & pose = object.kinematics.pose_with_covariance.pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.orientation = tf2::toMsg(q);
  // identity covariance so transforms are easy to reason about
  auto & cov = object.kinematics.pose_with_covariance.covariance;
  for (size_t i = 0; i < 6; ++i) {
    cov.at(i * 6 + i) = 1.0;
  }
  return object;
}

tf2::Transform makeTransform(const double x, const double y, const double z, const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  tf2::Transform t;
  t.setOrigin(tf2::Vector3(x, y, z));
  t.setRotation(q);
  return t;
}

// The concrete feature-object wire type (tier4_perception_msgs::msg::DetectedObjectsWithFeature)
// lives outside Autoware Core, so it cannot be a Core dependency. However,
// transformObjectsWithFeature / detail::applyTransformToFeatureObjects are duck-typed function
// templates: they only touch msg.header, msg.feature_objects[].object.kinematics
// .pose_with_covariance.pose and msg.feature_objects[].feature.cluster (a PointCloud2). We can
// therefore exercise the feature path by instantiating the templates against a Core-local stand-in
// type that mirrors exactly those member-access paths, built from Core-available message types.
struct FeatureObjectStandIn
{
  autoware_perception_msgs::msg::DetectedObject object;
  struct Feature
  {
    sensor_msgs::msg::PointCloud2 cluster;
  } feature;
};

struct ObjectsWithFeatureStandIn
{
  std_msgs::msg::Header header;
  std::vector<FeatureObjectStandIn> feature_objects;
};

// Build a single-point cluster cloud at (px, py, pz) with the given source frame.
sensor_msgs::msg::PointCloud2 createCluster(
  const double px, const double py, const double pz, const std::string & frame_id)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(px, py, pz));
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id;
  return msg;
}

// Build a feature object whose object pose is at (x, y, z, yaw) and whose cluster holds a single
// point at (px, py, pz).
FeatureObjectStandIn createFeatureObject(
  const double x, const double y, const double z, const double yaw, const double px,
  const double py, const double pz)
{
  FeatureObjectStandIn feature_object;
  feature_object.object = createObject(x, y, z, yaw);
  feature_object.feature.cluster = createCluster(px, py, pz, "sensor");
  return feature_object;
}

// A pure-translation Eigen 4x4 transform matrix (matching makeTransform's translation component).
Eigen::Matrix4f makeTranslationMatrix(const float x, const float y, const float z)
{
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  mat(0, 3) = x;
  mat(1, 3) = y;
  mat(2, 3) = z;
  return mat;
}
}  // namespace

// transformObjects must pass the message through unchanged when the input frame already equals the
// target frame (identity passthrough branch) and report success.
TEST(transform, transformObjectsIdentityPassthrough)
{
  using autoware::object_recognition_utils::transformObjects;

  autoware_perception_msgs::msg::DetectedObjects input;
  input.header.frame_id = "map";
  input.objects.push_back(createObject(1.0, 2.0, 3.0, 0.0));

  tf2_ros::Buffer empty_buffer{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)};
  autoware_perception_msgs::msg::DetectedObjects output;

  // Same frame: no lookup needed, so the (empty) buffer is never queried.
  EXPECT_TRUE(transformObjects(input, "map", empty_buffer, output));
  ASSERT_EQ(output.objects.size(), 1U);
  EXPECT_EQ(output.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(output.objects.at(0).kinematics.pose_with_covariance.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(output.objects.at(0).kinematics.pose_with_covariance.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(output.objects.at(0).kinematics.pose_with_covariance.pose.position.z, 3.0);
}

// transformObjects returns false when the buffer cannot resolve the requested transform.
TEST(transform, transformObjectsMissingTransformReturnsFalse)
{
  using autoware::object_recognition_utils::transformObjects;

  autoware_perception_msgs::msg::DetectedObjects input;
  input.header.frame_id = "sensor";
  input.objects.push_back(createObject(1.0, 2.0, 3.0, 0.0));

  tf2_ros::Buffer empty_buffer{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)};
  autoware_perception_msgs::msg::DetectedObjects output;

  // Different frame + empty buffer => lookup fails => false.
  EXPECT_FALSE(transformObjects(input, "map", empty_buffer, output));
  // Failure-path contract (matches the original implementation): the target frame is written to the
  // output header before the lookup, so it stays at the requested target frame on a failed lookup,
  // while the (copied) objects are left untransformed.
  EXPECT_EQ(output.header.frame_id, "map");
  ASSERT_EQ(output.objects.size(), 1U);
  const auto & pose = output.objects.at(0).kinematics.pose_with_covariance.pose;
  EXPECT_DOUBLE_EQ(pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(pose.position.z, 3.0);
}

// applyTransformToObjects must compose a pure translation onto each object's pose and rewrite the
// header frame, leaving orientation and (translation-only) covariance untouched.
TEST(transform, applyTransformToObjectsTranslationOnly)
{
  using autoware::object_recognition_utils::detail::applyTransformToObjects;

  autoware_perception_msgs::msg::DetectedObjects input;
  input.header.frame_id = "sensor";
  input.objects.push_back(createObject(1.0, 2.0, 0.0, 0.0));

  const auto tf_target2world = makeTransform(10.0, 20.0, 0.0, 0.0);

  autoware_perception_msgs::msg::DetectedObjects output;
  applyTransformToObjects(input, "map", tf_target2world, output);

  ASSERT_EQ(output.objects.size(), 1U);
  EXPECT_EQ(output.header.frame_id, "map");
  const auto & pose = output.objects.at(0).kinematics.pose_with_covariance.pose;
  EXPECT_NEAR(pose.position.x, 11.0, 1e-9);
  EXPECT_NEAR(pose.position.y, 22.0, 1e-9);
  EXPECT_NEAR(pose.position.z, 0.0, 1e-9);
  // orientation unchanged for a pure translation
  EXPECT_NEAR(pose.orientation.z, 0.0, 1e-9);
  EXPECT_NEAR(pose.orientation.w, 1.0, 1e-9);
  // a pure translation does not rotate covariance: diagonal stays 1.0
  const auto & cov = output.objects.at(0).kinematics.pose_with_covariance.covariance;
  EXPECT_NEAR(cov.at(0), 1.0, 1e-9);
  EXPECT_NEAR(cov.at(7), 1.0, 1e-9);
}

// applyTransformToObjects must compose a 90-degree yaw rotation (about origin) onto each object's
// pose: a point at (1, 0) maps to (0, 1) and the orientation accumulates the rotation.
TEST(transform, applyTransformToObjectsRotation90)
{
  using autoware::object_recognition_utils::detail::applyTransformToObjects;

  autoware_perception_msgs::msg::DetectedObjects input;
  input.header.frame_id = "sensor";
  input.objects.push_back(createObject(1.0, 0.0, 0.0, 0.0));

  const double half_pi = M_PI / 2.0;
  const auto tf_target2world = makeTransform(0.0, 0.0, 0.0, half_pi);

  autoware_perception_msgs::msg::DetectedObjects output;
  applyTransformToObjects(input, "map", tf_target2world, output);

  ASSERT_EQ(output.objects.size(), 1U);
  const auto & pose = output.objects.at(0).kinematics.pose_with_covariance.pose;
  EXPECT_NEAR(pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(pose.position.y, 1.0, 1e-9);
  // orientation becomes a +90deg yaw: qz = sin(pi/4), qw = cos(pi/4)
  EXPECT_NEAR(pose.orientation.z, std::sin(half_pi / 2.0), 1e-9);
  EXPECT_NEAR(pose.orientation.w, std::cos(half_pi / 2.0), 1e-9);
}

// transformObjects wired through a live buffer with a known static transform must produce the same
// result as applyTransformToObjects with the equivalent tf2::Transform.
TEST(transform, transformObjectsAppliesBufferedTransform)
{
  using autoware::object_recognition_utils::transformObjects;

  // Static transform: target("map") <- source("sensor") = translate (10, 20, 0)
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped.header.frame_id = "map";
  tf_stamped.child_frame_id = "sensor";
  tf_stamped.transform.translation.x = 10.0;
  tf_stamped.transform.translation.y = 20.0;
  tf_stamped.transform.translation.z = 0.0;
  tf_stamped.transform.rotation.w = 1.0;

  tf2_ros::Buffer buffer{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)};
  buffer.setTransform(tf_stamped, "test_authority", true);

  autoware_perception_msgs::msg::DetectedObjects input;
  input.header.frame_id = "sensor";
  input.objects.push_back(createObject(1.0, 2.0, 0.0, 0.0));

  autoware_perception_msgs::msg::DetectedObjects output;
  ASSERT_TRUE(transformObjects(input, "map", buffer, output));
  ASSERT_EQ(output.objects.size(), 1U);
  EXPECT_EQ(output.header.frame_id, "map");
  const auto & pose = output.objects.at(0).kinematics.pose_with_covariance.pose;
  EXPECT_NEAR(pose.position.x, 11.0, 1e-9);
  EXPECT_NEAR(pose.position.y, 22.0, 1e-9);
  EXPECT_NEAR(pose.position.z, 0.0, 1e-9);
}

// applyTransformToObjects on an empty objects vector must rewrite the header frame and produce an
// empty output without throwing (empty-container degenerate case).
TEST(transform, applyTransformToObjectsEmptyObjects)
{
  using autoware::object_recognition_utils::detail::applyTransformToObjects;

  autoware_perception_msgs::msg::DetectedObjects input;
  input.header.frame_id = "sensor";

  const auto tf_target2world = makeTransform(10.0, 20.0, 0.0, 0.0);

  autoware_perception_msgs::msg::DetectedObjects output;
  applyTransformToObjects(input, "map", tf_target2world, output);

  EXPECT_EQ(output.header.frame_id, "map");
  EXPECT_TRUE(output.objects.empty());
}

// applyTransformToFeatureObjects must compose a pure translation onto the feature object's pose and
// rewrite both the message header frame and the per-cluster header frame to the target frame.
TEST(transform, applyTransformToFeatureObjectsTranslationOnly)
{
  using autoware::object_recognition_utils::detail::applyTransformToFeatureObjects;

  ObjectsWithFeatureStandIn input;
  input.header.frame_id = "sensor";
  input.feature_objects.push_back(createFeatureObject(1.0, 2.0, 0.0, 0.0, 0.5, 0.5, 0.0));

  const auto tf_target2world = makeTransform(10.0, 20.0, 0.0, 0.0);
  const auto tf_matrix = makeTranslationMatrix(10.0F, 20.0F, 0.0F);

  ObjectsWithFeatureStandIn output;
  applyTransformToFeatureObjects(input, "map", tf_target2world, tf_matrix, output);

  ASSERT_EQ(output.feature_objects.size(), 1U);
  EXPECT_EQ(output.header.frame_id, "map");

  // object pose is composed with the world transform: (1,2) + (10,20) = (11,22)
  const auto & pose = output.feature_objects.at(0).object.kinematics.pose_with_covariance.pose;
  EXPECT_NEAR(pose.position.x, 11.0, 1e-6);
  EXPECT_NEAR(pose.position.y, 22.0, 1e-6);
  EXPECT_NEAR(pose.position.z, 0.0, 1e-6);
  EXPECT_NEAR(pose.orientation.z, 0.0, 1e-6);
  EXPECT_NEAR(pose.orientation.w, 1.0, 1e-6);

  // cluster header frame is rewritten to the target frame (computed contract value)
  const auto & cluster = output.feature_objects.at(0).feature.cluster;
  EXPECT_EQ(cluster.header.frame_id, "map");

  // the single cluster point is translated by tf_matrix: (0.5,0.5) + (10,20) = (10.5,20.5)
  ASSERT_EQ(cluster.width * cluster.height, 1U);
  pcl::PointCloud<pcl::PointXYZ> transformed_points;
  pcl::fromROSMsg(cluster, transformed_points);
  ASSERT_EQ(transformed_points.size(), 1U);
  EXPECT_NEAR(transformed_points.at(0).x, 10.5, 1e-5);
  EXPECT_NEAR(transformed_points.at(0).y, 20.5, 1e-5);
  EXPECT_NEAR(transformed_points.at(0).z, 0.0, 1e-5);
}

// applyTransformToFeatureObjects on an empty feature_objects vector must rewrite the header frame
// and produce an empty output without throwing (empty-container degenerate case).
TEST(transform, applyTransformToFeatureObjectsEmptyFeatureObjects)
{
  using autoware::object_recognition_utils::detail::applyTransformToFeatureObjects;

  ObjectsWithFeatureStandIn input;
  input.header.frame_id = "sensor";

  const auto tf_target2world = makeTransform(10.0, 20.0, 0.0, 0.0);
  const auto tf_matrix = makeTranslationMatrix(10.0F, 20.0F, 0.0F);

  ObjectsWithFeatureStandIn output;
  applyTransformToFeatureObjects(input, "map", tf_target2world, tf_matrix, output);

  EXPECT_EQ(output.header.frame_id, "map");
  EXPECT_TRUE(output.feature_objects.empty());
}

// applyTransformToFeatureObjects must tolerate a feature object whose cluster has zero points: the
// pose is still composed and the (empty) cluster header is rewritten to the target frame.
TEST(transform, applyTransformToFeatureObjectsEmptyCluster)
{
  using autoware::object_recognition_utils::detail::applyTransformToFeatureObjects;

  ObjectsWithFeatureStandIn input;
  input.header.frame_id = "sensor";
  FeatureObjectStandIn feature_object;
  feature_object.object = createObject(1.0, 2.0, 0.0, 0.0);
  feature_object.feature.cluster.header.frame_id = "sensor";  // no points
  input.feature_objects.push_back(feature_object);

  const auto tf_target2world = makeTransform(10.0, 20.0, 0.0, 0.0);
  const auto tf_matrix = makeTranslationMatrix(10.0F, 20.0F, 0.0F);

  ObjectsWithFeatureStandIn output;
  applyTransformToFeatureObjects(input, "map", tf_target2world, tf_matrix, output);

  ASSERT_EQ(output.feature_objects.size(), 1U);
  EXPECT_EQ(output.header.frame_id, "map");
  const auto & pose = output.feature_objects.at(0).object.kinematics.pose_with_covariance.pose;
  EXPECT_NEAR(pose.position.x, 11.0, 1e-6);
  EXPECT_NEAR(pose.position.y, 22.0, 1e-6);

  const auto & cluster = output.feature_objects.at(0).feature.cluster;
  EXPECT_EQ(cluster.header.frame_id, "map");
  EXPECT_EQ(cluster.width * cluster.height, 0U);
}

// transformObjectsWithFeature wired through a live buffer with a known static transform must
// compose the object pose and rewrite each cluster header to the target frame, reporting success.
TEST(transform, transformObjectsWithFeatureAppliesBufferedTransform)
{
  using autoware::object_recognition_utils::transformObjectsWithFeature;

  // Static transform: target("map") <- source("sensor") = translate (10, 20, 0)
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped.header.frame_id = "map";
  tf_stamped.child_frame_id = "sensor";
  tf_stamped.transform.translation.x = 10.0;
  tf_stamped.transform.translation.y = 20.0;
  tf_stamped.transform.translation.z = 0.0;
  tf_stamped.transform.rotation.w = 1.0;

  tf2_ros::Buffer buffer{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)};
  buffer.setTransform(tf_stamped, "test_authority", true);

  ObjectsWithFeatureStandIn input;
  input.header.frame_id = "sensor";
  input.feature_objects.push_back(createFeatureObject(1.0, 2.0, 0.0, 0.0, 0.5, 0.5, 0.0));

  ObjectsWithFeatureStandIn output;
  ASSERT_TRUE(transformObjectsWithFeature(input, "map", buffer, output));
  ASSERT_EQ(output.feature_objects.size(), 1U);
  EXPECT_EQ(output.header.frame_id, "map");
  const auto & pose = output.feature_objects.at(0).object.kinematics.pose_with_covariance.pose;
  // object pose is composed with the buffered world transform: (1,2) + (10,20) = (11,22)
  EXPECT_NEAR(pose.position.x, 11.0, 1e-6);
  EXPECT_NEAR(pose.position.y, 22.0, 1e-6);
  // cluster header frame is rewritten to the target frame
  const auto & cluster = output.feature_objects.at(0).feature.cluster;
  EXPECT_EQ(cluster.header.frame_id, "map");
  // the single cluster point is translated by the buffered transform: (0.5,0.5) + (10,20) =
  // (10.5,20.5)
  ASSERT_EQ(cluster.width * cluster.height, 1U);
  pcl::PointCloud<pcl::PointXYZ> transformed_points;
  pcl::fromROSMsg(cluster, transformed_points);
  ASSERT_EQ(transformed_points.size(), 1U);
  EXPECT_NEAR(transformed_points.at(0).x, 10.5, 1e-5);
  EXPECT_NEAR(transformed_points.at(0).y, 20.5, 1e-5);
  EXPECT_NEAR(transformed_points.at(0).z, 0.0, 1e-5);
}

// transformObjectsWithFeature must pass the message through unchanged when the input frame already
// equals the target frame (identity passthrough branch) and report success.
TEST(transform, transformObjectsWithFeatureIdentityPassthrough)
{
  using autoware::object_recognition_utils::transformObjectsWithFeature;

  ObjectsWithFeatureStandIn input;
  input.header.frame_id = "map";
  input.feature_objects.push_back(createFeatureObject(1.0, 2.0, 3.0, 0.0, 0.5, 0.5, 0.0));

  tf2_ros::Buffer empty_buffer{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)};
  ObjectsWithFeatureStandIn output;

  // Same frame: no lookup needed, so the (empty) buffer is never queried.
  EXPECT_TRUE(transformObjectsWithFeature(input, "map", empty_buffer, output));
  ASSERT_EQ(output.feature_objects.size(), 1U);
  EXPECT_EQ(output.header.frame_id, "map");
  const auto & pose = output.feature_objects.at(0).object.kinematics.pose_with_covariance.pose;
  EXPECT_DOUBLE_EQ(pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(pose.position.z, 3.0);
}

// transformObjectsWithFeature returns false when the buffer cannot resolve the requested transform.
// Mirrors transformObjectsMissingTransformReturnsFalse for the feature path.
TEST(transform, transformObjectsWithFeatureMissingTransformReturnsFalse)
{
  using autoware::object_recognition_utils::transformObjectsWithFeature;

  ObjectsWithFeatureStandIn input;
  input.header.frame_id = "sensor";
  input.feature_objects.push_back(createFeatureObject(1.0, 2.0, 3.0, 0.0, 0.5, 0.5, 0.0));

  tf2_ros::Buffer empty_buffer{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)};
  ObjectsWithFeatureStandIn output;

  // Different frame + empty buffer => lookup fails => false.
  EXPECT_FALSE(transformObjectsWithFeature(input, "map", empty_buffer, output));
  // Failure-path contract: the target frame is written to the output header before the lookup, so
  // it stays at the requested target frame on a failed lookup, while the (copied) feature objects
  // are left untransformed.
  EXPECT_EQ(output.header.frame_id, "map");
  ASSERT_EQ(output.feature_objects.size(), 1U);
  const auto & pose = output.feature_objects.at(0).object.kinematics.pose_with_covariance.pose;
  EXPECT_DOUBLE_EQ(pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(pose.position.z, 3.0);
}
