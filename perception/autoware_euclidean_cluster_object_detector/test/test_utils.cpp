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

#include <autoware/euclidean_cluster_object_detector/utils.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <vector>
class UtilsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a test point cloud
    pointcloud_.header.frame_id = "base_link";
    pointcloud_.header.stamp.sec = 1;
    pointcloud_.header.stamp.nanosec = 0;

    // Setup fields
    pointcloud_.fields.resize(3);
    pointcloud_.fields[0].name = "x";
    pointcloud_.fields[1].name = "y";
    pointcloud_.fields[2].name = "z";
    pointcloud_.fields[0].offset = 0;
    pointcloud_.fields[1].offset = 4;
    pointcloud_.fields[2].offset = 8;
    pointcloud_.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud_.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud_.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud_.fields[0].count = 1;
    pointcloud_.fields[1].count = 1;
    pointcloud_.fields[2].count = 1;

    // Set size and content
    pointcloud_.height = 1;
    pointcloud_.width = 4;
    pointcloud_.point_step = 12;
    pointcloud_.row_step = pointcloud_.point_step * pointcloud_.width;
    pointcloud_.is_bigendian = false;
    pointcloud_.is_dense = true;
    pointcloud_.data.resize(pointcloud_.row_step);

    // Add points
    float points[4][3] = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}, {10.0, 11.0, 12.0}};

    for (size_t i = 0; i < 4; ++i) {
      memcpy(&pointcloud_.data[i * pointcloud_.point_step], &points[i][0], sizeof(float) * 3);
    }

    // Create PCL point clouds for cluster tests
    pcl::PointCloud<pcl::PointXYZ> cloud1;
    cloud1.width = 2;
    cloud1.height = 1;
    cloud1.points.resize(2);
    cloud1.points[0].x = 1.0;
    cloud1.points[0].y = 2.0;
    cloud1.points[0].z = 3.0;
    cloud1.points[1].x = 4.0;
    cloud1.points[1].y = 5.0;
    cloud1.points[1].z = 6.0;

    pcl::PointCloud<pcl::PointXYZ> cloud2;
    cloud2.width = 2;
    cloud2.height = 1;
    cloud2.points.resize(2);
    cloud2.points[0].x = 7.0;
    cloud2.points[0].y = 8.0;
    cloud2.points[0].z = 9.0;
    cloud2.points[1].x = 10.0;
    cloud2.points[1].y = 11.0;
    cloud2.points[1].z = 12.0;

    clusters_.push_back(cloud1);
    clusters_.push_back(cloud2);
  }

  sensor_msgs::msg::PointCloud2 pointcloud_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters_;
};

TEST_F(UtilsTest, TestGetCentroid)
{
  // Calculate centroid
  geometry_msgs::msg::Point centroid = autoware::euclidean_cluster::getCentroid(pointcloud_);

  EXPECT_FLOAT_EQ(centroid.x, 5.5f);
  EXPECT_FLOAT_EQ(centroid.y, 6.5f);
  EXPECT_FLOAT_EQ(centroid.z, 7.5f);
}

TEST_F(UtilsTest, TestConvertPointCloudClusters2Msg)
{
  // Create header for conversion
  std_msgs::msg::Header header;
  header.frame_id = "base_link";
  header.stamp.sec = 1;
  header.stamp.nanosec = 0;

  // Convert clusters to msg
  autoware_perception_msgs::msg::DetectedObjects objects;
  autoware::euclidean_cluster::convertPointCloudClusters2Msg(header, clusters_, objects);

  // Check results
  EXPECT_EQ(objects.header.frame_id, "base_link");
  EXPECT_EQ(objects.header.stamp.sec, 1);
  EXPECT_EQ(objects.header.stamp.nanosec, 0);
  EXPECT_EQ(objects.objects.size(), 2);

  // Check first object's centroid (average of cloud1 points)
  EXPECT_FLOAT_EQ(objects.objects[0].kinematics.pose_with_covariance.pose.position.x, 2.5f);
  EXPECT_FLOAT_EQ(objects.objects[0].kinematics.pose_with_covariance.pose.position.y, 3.5f);
  EXPECT_FLOAT_EQ(objects.objects[0].kinematics.pose_with_covariance.pose.position.z, 4.5f);

  // Check second object's centroid (average of cloud2 points)
  EXPECT_FLOAT_EQ(objects.objects[1].kinematics.pose_with_covariance.pose.position.x, 8.5f);
  EXPECT_FLOAT_EQ(objects.objects[1].kinematics.pose_with_covariance.pose.position.y, 9.5f);
  EXPECT_FLOAT_EQ(objects.objects[1].kinematics.pose_with_covariance.pose.position.z, 10.5f);

  // Check classification
  EXPECT_EQ(objects.objects[0].classification.size(), 1);
  EXPECT_EQ(
    objects.objects[0].classification[0].label,
    autoware_perception_msgs::msg::ObjectClassification::UNKNOWN);
  EXPECT_FLOAT_EQ(objects.objects[0].classification[0].probability, 1.0f);
}

TEST_F(UtilsTest, TestConvertClusters2SensorMsg)
{
  // Create header for conversion
  std_msgs::msg::Header header;
  header.frame_id = "base_link";
  header.stamp.sec = 1;
  header.stamp.nanosec = 0;

  // Convert clusters to sensor msg
  sensor_msgs::msg::PointCloud2 output;
  autoware::euclidean_cluster::convertClusters2SensorMsg(header, clusters_, output);

  // Check header
  EXPECT_EQ(output.header.frame_id, "base_link");
  EXPECT_EQ(output.header.stamp.sec, 1);
  EXPECT_EQ(output.header.stamp.nanosec, 0);

  // Check size
  EXPECT_EQ(output.width, 4);  // total number of points (2 per cluster)
  EXPECT_EQ(output.height, 1);
  EXPECT_FALSE(output.is_dense);

  // Check fields (should have x, y, z, rgb)
  EXPECT_EQ(output.fields.size(), 4);
  EXPECT_EQ(output.fields[0].name, "x");
  EXPECT_EQ(output.fields[1].name, "y");
  EXPECT_EQ(output.fields[2].name, "z");
  EXPECT_EQ(output.fields[3].name, "rgb");

  // Create iterators to check point data
  sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(output, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(output, "z");

  EXPECT_FLOAT_EQ(*iter_x, 1.0f);
  EXPECT_FLOAT_EQ(*iter_y, 2.0f);
  EXPECT_FLOAT_EQ(*iter_z, 3.0f);

  // Move to next point
  ++iter_x;
  ++iter_y;
  ++iter_z;

  EXPECT_FLOAT_EQ(*iter_x, 4.0f);
  EXPECT_FLOAT_EQ(*iter_y, 5.0f);
  EXPECT_FLOAT_EQ(*iter_z, 6.0f);

  // Move to third point
  ++iter_x;
  ++iter_y;
  ++iter_z;

  EXPECT_FLOAT_EQ(*iter_x, 7.0f);
  EXPECT_FLOAT_EQ(*iter_y, 8.0f);
  EXPECT_FLOAT_EQ(*iter_z, 9.0f);

  // Move to fourth point
  ++iter_x;
  ++iter_y;
  ++iter_z;

  EXPECT_FLOAT_EQ(*iter_x, 10.0f);
  EXPECT_FLOAT_EQ(*iter_y, 11.0f);
  EXPECT_FLOAT_EQ(*iter_z, 12.0f);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
