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

#include "differential_pointcloud_map_visualizer_node.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace autoware::map_loader
{
DifferentialPointCloudMapVisualizerNode::DifferentialPointCloudMapVisualizerNode(
  const rclcpp::NodeOptions & options)
: Node("differential_pointcloud_map_visualizer", options), frame_id_("map")
{
  service_name_ =
    declare_parameter<std::string>("service_name", "/map/get_differential_pointcloud_map");
  const auto output_topic =
    declare_parameter<std::string>("output_topic", "output/differential_pointcloud_map");
  const auto pose_topic = declare_parameter<std::string>(
    "pose_topic", "/localization/pose_estimator/pose_with_covariance");
  const auto update_interval_sec = declare_parameter<double>("update_interval_sec", 1.0);
  center_x_ = declare_parameter<double>("center_x", 0.0);
  center_y_ = declare_parameter<double>("center_y", 0.0);
  radius_ = declare_parameter<double>("radius", 150.0);
  use_pose_ = declare_parameter<bool>("use_pose", false);

  const auto durable_qos = rclcpp::QoS{1}.transient_local();
  publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, durable_qos);
  client_ = create_client<GetDifferentialPointCloudMap>(service_name_);

  if (use_pose_) {
    pose_sub_ = create_subscription<PoseWithCovarianceStamped>(
      pose_topic, rclcpp::QoS{1},
      std::bind(&DifferentialPointCloudMapVisualizerNode::on_pose, this, std::placeholders::_1));
  }

  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::max(update_interval_sec, 0.1))),
    std::bind(&DifferentialPointCloudMapVisualizerNode::on_timer, this));
}

void DifferentialPointCloudMapVisualizerNode::on_timer()
{
  double query_center_x = center_x_;
  double query_center_y = center_y_;

  if (use_pose_) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!latest_pose_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for pose topic. Differential pointcloud map is not queried yet.");
      return;
    }
    query_center_x = latest_pose_->x;
    query_center_y = latest_pose_->y;
  }

  if (!client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000, "Waiting for service: %s", service_name_.c_str());
    return;
  }

  auto request = std::make_shared<GetDifferentialPointCloudMap::Request>();
  request->area.center_x = static_cast<float>(query_center_x);
  request->area.center_y = static_cast<float>(query_center_y);
  request->area.radius = static_cast<float>(radius_);
  {
    std::lock_guard<std::mutex> lock(loaded_cells_mutex_);
    request->cached_ids.reserve(loaded_cells_.size());
    for (const auto & [cell_id, cloud] : loaded_cells_) {
      static_cast<void>(cloud);
      request->cached_ids.push_back(cell_id);
    }
  }

  bool expected = false;
  if (!request_in_flight_.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Previous request is still in flight.");
    return;
  }

  client_->async_send_request(
    request,
    std::bind(
      &DifferentialPointCloudMapVisualizerNode::on_service_response, this, std::placeholders::_1));
}

void DifferentialPointCloudMapVisualizerNode::on_service_response(
  rclcpp::Client<GetDifferentialPointCloudMap>::SharedFuture future)
{
  request_in_flight_.store(false);

  GetDifferentialPointCloudMap::Response::SharedPtr response;
  try {
    response = future.get();
  } catch (const std::exception & e) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Failed to process response from differential pointcloud map service: " << e.what());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(loaded_cells_mutex_);
    for (const auto & id_to_remove : response->ids_to_remove) {
      loaded_cells_.erase(id_to_remove);
    }
    for (const auto & pcd_with_id : response->new_pointcloud_with_ids) {
      loaded_cells_[pcd_with_id.cell_id] = pcd_with_id.pointcloud;
    }
    if (!response->header.frame_id.empty()) {
      frame_id_ = response->header.frame_id;
    }
  }

  auto merged_cloud = create_merged_cloud();
  merged_cloud.header.stamp = now();
  merged_cloud.header.frame_id = frame_id_;
  publisher_->publish(merged_cloud);
}

void DifferentialPointCloudMapVisualizerNode::on_pose(
  const PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  latest_pose_ = msg->pose.pose.position;
}

sensor_msgs::msg::PointCloud2 DifferentialPointCloudMapVisualizerNode::create_merged_cloud() const
{
  sensor_msgs::msg::PointCloud2 merged;
  {
    std::lock_guard<std::mutex> lock(loaded_cells_mutex_);
    if (loaded_cells_.empty()) {
      return merged;
    }

    const auto first_itr = loaded_cells_.begin();
    merged = first_itr->second;
    std::vector<uint8_t> merged_data = merged.data;

    for (auto itr = std::next(loaded_cells_.begin()); itr != loaded_cells_.end(); ++itr) {
      const auto & cell_id = itr->first;
      const auto & cloud = itr->second;
      if (!is_compatible_pointcloud_layout(merged, cloud)) {
        RCLCPP_WARN_STREAM(
          get_logger(),
          "Skipped pointcloud map cell due to incompatible layout. cell_id: " << cell_id);
        continue;
      }
      merged_data.insert(merged_data.end(), cloud.data.begin(), cloud.data.end());
    }

    merged.data = std::move(merged_data);
  }
  merged.height = 1;
  if (merged.point_step == 0U) {
    merged.width = 0U;
    merged.row_step = 0U;
    return merged;
  }
  merged.width = static_cast<uint32_t>(merged.data.size() / merged.point_step);
  merged.row_step = merged.point_step * merged.width;
  merged.is_dense = false;
  return merged;
}

bool DifferentialPointCloudMapVisualizerNode::is_compatible_pointcloud_layout(
  const sensor_msgs::msg::PointCloud2 & lhs, const sensor_msgs::msg::PointCloud2 & rhs)
{
  return lhs.fields == rhs.fields && lhs.is_bigendian == rhs.is_bigendian &&
         lhs.point_step == rhs.point_step;
}
}  // namespace autoware::map_loader

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::map_loader::DifferentialPointCloudMapVisualizerNode)
