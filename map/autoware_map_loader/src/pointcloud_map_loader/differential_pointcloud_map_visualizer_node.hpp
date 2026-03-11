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

#ifndef POINTCLOUD_MAP_LOADER__DIFFERENTIAL_POINTCLOUD_MAP_VISUALIZER_NODE_HPP_
#define POINTCLOUD_MAP_LOADER__DIFFERENTIAL_POINTCLOUD_MAP_VISUALIZER_NODE_HPP_

#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <optional>
#include <string>
#include <unordered_map>
#include <mutex>
#include <atomic>

namespace autoware::map_loader
{
class DifferentialPointCloudMapVisualizerNode : public rclcpp::Node
{
public:
  explicit DifferentialPointCloudMapVisualizerNode(const rclcpp::NodeOptions & options);

private:
  using GetDifferentialPointCloudMap = autoware_map_msgs::srv::GetDifferentialPointCloudMap;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  void on_timer();
  void on_pose(const PoseWithCovarianceStamped::ConstSharedPtr msg);
  void on_service_response(rclcpp::Client<GetDifferentialPointCloudMap>::SharedFuture future);
  [[nodiscard]] sensor_msgs::msg::PointCloud2 create_merged_cloud() const;

  [[nodiscard]] static bool is_compatible_pointcloud_layout(
    const sensor_msgs::msg::PointCloud2 & lhs, const sensor_msgs::msg::PointCloud2 & rhs);

  rclcpp::Client<GetDifferentialPointCloudMap>::SharedPtr client_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string service_name_;
  std::string frame_id_;
  bool use_pose_;
  double center_x_;
  double center_y_;
  double radius_;

  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2> loaded_cells_;
  std::optional<geometry_msgs::msg::Point> latest_pose_;
  mutable std::mutex pose_mutex_;
  mutable std::mutex loaded_cells_mutex_;
  std::atomic_bool request_in_flight_{false};
};
}  // namespace autoware::map_loader

#endif  // POINTCLOUD_MAP_LOADER__DIFFERENTIAL_POINTCLOUD_MAP_VISUALIZER_NODE_HPP_
