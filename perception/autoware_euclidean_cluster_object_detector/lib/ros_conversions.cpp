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

#include "ros_conversions.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <vector>

namespace autoware::euclidean_cluster
{

/**
* @brief Calculate centroid of a point cloud cluster.
*
* @param cluster Input point cloud cluster.

* @return geometry_msgs::msg::Point Centroid of this cluster.
*/
geometry_msgs::msg::Point get_centroid(const pcl::PointCloud<pcl::PointXYZ> & cluster)
{
  geometry_msgs::msg::Point centroid;
  centroid.x = 0.0;
  centroid.y = 0.0;
  centroid.z = 0.0;

  if (cluster.empty()) {
    return centroid;
  }

  for (const auto & p : cluster.points) {
    centroid.x += p.x;
    centroid.y += p.y;
    centroid.z += p.z;
  }

  const auto size = static_cast<float>(cluster.size());
  centroid.x /= size;
  centroid.y /= size;
  centroid.z /= size;

  return centroid;
}

/**
 * @brief Helper func to convert clusters this node to autoware_msgs detected_objects
 *
 * @param header Header for output message.
 * @param clusters Vector of point clouds, each representing a cluster.
 * @param output_msg Output message to populate with detected objects.
 */
void convert_clusters_to_detected_objects(
  const std_msgs::msg::Header & header,
  const std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters,
  autoware_perception_msgs::msg::DetectedObjects & output_msg)
{
  output_msg.header = header;
  output_msg.objects.reserve(clusters.size());

  for (const auto & cluster : clusters) {
    autoware_perception_msgs::msg::DetectedObject object;
    object.kinematics.pose_with_covariance.pose.position = get_centroid(cluster);

    autoware_perception_msgs::msg::ObjectClassification classification;
    classification.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
    classification.probability = 1.0f;
    object.classification.push_back(classification);

    output_msg.objects.push_back(object);
  }
}

/**
 * @brief Helper func to convert clusters this node to a single point cloud for debugging.
 *
 * @param header Header for output message.
 * @param clusters Vector of point clouds, each representing a cluster.
 * @param output_msg Output message to populate with a single point cloud for debugging.
 */
void convert_clusters_to_debug_point_cloud(
  const std_msgs::msg::Header & header,
  const std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters,
  sensor_msgs::msg::PointCloud2 & output_msg)
{
  output_msg.header = header;
  size_t total_points = 0;
  for (const auto & c : clusters) {
    total_points += c.size();
  }

  sensor_msgs::PointCloud2Modifier modifier(output_msg);
  modifier.setPointCloud2Fields(
    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(total_points);

  sensor_msgs::PointCloud2Iterator<float> iter_x(output_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(output_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(output_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(output_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(output_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(output_msg, "b");

  constexpr uint8_t colors[] = {200, 0,   0, 0,   200, 0,   0, 0,   200,
                                200, 200, 0, 200, 0,   200, 0, 200, 200};

  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto & cluster = clusters[i];
    uint8_t r = colors[3 * (i % 6) + 0];
    uint8_t g = colors[3 * (i % 6) + 1];
    uint8_t b = colors[3 * (i % 6) + 2];

    for (const auto & p : cluster.points) {
      *iter_x = p.x;
      *iter_y = p.y;
      *iter_z = p.z;
      *iter_r = r;
      *iter_g = g;
      *iter_b = b;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_r;
      ++iter_g;
      ++iter_b;
    }
  }
}

}  // namespace autoware::euclidean_cluster
