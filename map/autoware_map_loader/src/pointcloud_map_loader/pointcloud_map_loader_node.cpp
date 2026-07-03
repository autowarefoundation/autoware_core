// Copyright 2022 The Autoware Contributors
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

#include "pointcloud_map_loader_node.hpp"

#include "pointcloud_map_loader.hpp"

#include <boost/optional.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::map_loader
{
PointcloudMapLoaderModule::PointcloudMapLoaderModule(
  rclcpp::Node * node, const std::vector<std::string> & pcd_paths,
  const std::string & publisher_name, const bool use_downsample)
: PointcloudMapLoaderModule()
{
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  pub_pointcloud_map_ =
    node->create_publisher<sensor_msgs::msg::PointCloud2>(publisher_name, durable_qos);

  const auto leaf_size =
    use_downsample
      ? boost::make_optional(static_cast<float>(node->declare_parameter<float>("leaf_size")))
      : boost::none;
  const auto load_result = create_map_message(pcd_paths, leaf_size);
  if (!load_result) {
    for (const auto & msg : load_result.error().debug_messages) {
      RCLCPP_DEBUG_STREAM(node->get_logger(), msg);
    }
    RCLCPP_ERROR_STREAM(node->get_logger(), load_result.error().error_message);
    return;
  }
  for (const auto & msg : load_result->debug_messages) {
    RCLCPP_DEBUG_STREAM(node->get_logger(), msg);
  }
  if (load_result->loaded_pcd.width == 0) {
    RCLCPP_ERROR(node->get_logger(), "No PCD was loaded: pcd_paths.size() = %zu", pcd_paths.size());
    return;
  }
  pub_pointcloud_map_->publish(load_result->loaded_pcd);
}

PartialMapLoaderModule::PartialMapLoaderModule(
  rclcpp::Node * node, std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict)
: PartialMapLoaderModule(std::move(pcd_file_metadata_dict))
{
  get_partial_pcd_maps_service_ = node->create_service<GetPartialPointCloudMap>(
    "service/get_partial_pcd_map",
    std::bind(
      &PartialMapLoaderModule::on_service_get_partial_point_cloud_map, this, std::placeholders::_1,
      std::placeholders::_2));
}

DifferentialMapLoaderModule::DifferentialMapLoaderModule(
  rclcpp::Node * node, std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict)
: DifferentialMapLoaderModule(std::move(pcd_file_metadata_dict))
{
  get_differential_pcd_maps_service_ = node->create_service<GetDifferentialPointCloudMap>(
    "service/get_differential_pcd_map",
    std::bind(
      &DifferentialMapLoaderModule::on_service_get_differential_point_cloud_map, this,
      std::placeholders::_1, std::placeholders::_2));
}

SelectedMapLoaderModule::SelectedMapLoaderModule(
  rclcpp::Node * node, std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict)
: SelectedMapLoaderModule(std::move(pcd_file_metadata_dict))
{
  get_selected_pcd_maps_service_ = node->create_service<GetSelectedPointCloudMap>(
    "service/get_selected_pcd_map",
    std::bind(
      &SelectedMapLoaderModule::on_service_get_selected_point_cloud_map, this,
      std::placeholders::_1, std::placeholders::_2));

  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  pub_metadata_ = node->create_publisher<autoware_map_msgs::msg::PointCloudMapMetaData>(
    "output/pointcloud_map_metadata", durable_qos);
  pub_metadata_->publish(create_metadata(all_pcd_file_metadata_dict_));
}

PointCloudMapLoaderNode::PointCloudMapLoaderNode(const rclcpp::NodeOptions & options)
: Node("pointcloud_map_loader", options)
{
  const auto pcd_paths = resolve_pcd_paths(
    declare_parameter<std::vector<std::string>>("pcd_paths_or_directory"),
    [this](const std::string & msg) { RCLCPP_ERROR_STREAM(get_logger(), msg); });
  std::string pcd_metadata_path = declare_parameter<std::string>("pcd_metadata_path");
  bool enable_whole_load = declare_parameter<bool>("enable_whole_load");
  bool enable_downsample_whole_load = declare_parameter<bool>("enable_downsampled_whole_load");
  bool enable_partial_load = declare_parameter<bool>("enable_partial_load");
  bool enable_selected_load = declare_parameter<bool>("enable_selected_load");

  if (enable_whole_load) {
    pcd_map_loader_ = std::make_unique<PointcloudMapLoaderModule>();

    rclcpp::QoS durable_qos{1};
    durable_qos.transient_local();
    pub_pointcloud_map_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("output/pointcloud_map", durable_qos);

    const auto load_result = pcd_map_loader_->create_map_message(pcd_paths, boost::none);
    if (!load_result) {
      for (const auto & msg : load_result.error().debug_messages) {
        RCLCPP_DEBUG_STREAM(get_logger(), msg);
      }
      RCLCPP_ERROR_STREAM(get_logger(), load_result.error().error_message);
    } else {
      for (const auto & msg : load_result->debug_messages) {
        RCLCPP_DEBUG_STREAM(get_logger(), msg);
      }
      if (load_result->loaded_pcd.width == 0) {
        RCLCPP_ERROR(get_logger(), "No PCD was loaded: pcd_paths.size() = %zu", pcd_paths.size());
      } else {
        pub_pointcloud_map_->publish(load_result->loaded_pcd);
      }
    }
  }

  if (enable_downsample_whole_load) {
    downsampled_pcd_map_loader_ = std::make_unique<PointcloudMapLoaderModule>();

    rclcpp::QoS durable_qos{1};
    durable_qos.transient_local();
    pub_downsampled_pointcloud_map_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "output/debug/downsampled_pointcloud_map", durable_qos);

    const auto leaf_size =
      boost::make_optional(static_cast<float>(declare_parameter<float>("leaf_size")));
    const auto load_result = downsampled_pcd_map_loader_->create_map_message(pcd_paths, leaf_size);
    if (!load_result) {
      for (const auto & msg : load_result.error().debug_messages) {
        RCLCPP_DEBUG_STREAM(get_logger(), msg);
      }
      RCLCPP_ERROR_STREAM(get_logger(), load_result.error().error_message);
    } else {
      for (const auto & msg : load_result->debug_messages) {
        RCLCPP_DEBUG_STREAM(get_logger(), msg);
      }
      if (load_result->loaded_pcd.width == 0) {
        RCLCPP_ERROR(get_logger(), "No PCD was loaded: pcd_paths.size() = %zu", pcd_paths.size());
      } else {
        pub_downsampled_pointcloud_map_->publish(load_result->loaded_pcd);
      }
    }
  }

  // Parse the metadata file and get the map of (absolute pcd path, pcd file metadata)
  auto pcd_metadata_dict = build_pcd_metadata_dict(pcd_metadata_path, pcd_paths);

  if (enable_partial_load) {
    partial_map_loader_ = std::make_unique<PartialMapLoaderModule>(pcd_metadata_dict);
    get_partial_pcd_maps_service_ = create_service<GetPartialPointCloudMap>(
      "service/get_partial_pcd_map", [this](
                                       GetPartialPointCloudMap::Request::SharedPtr req,
                                       GetPartialPointCloudMap::Response::SharedPtr res) {
        return partial_map_loader_->create_response(req, res);
      });
  }

  differential_map_loader_ = std::make_unique<DifferentialMapLoaderModule>(pcd_metadata_dict);
  get_differential_pcd_maps_service_ = create_service<GetDifferentialPointCloudMap>(
    "service/get_differential_pcd_map", [this](
                                          GetDifferentialPointCloudMap::Request::SharedPtr req,
                                          GetDifferentialPointCloudMap::Response::SharedPtr res) {
      return differential_map_loader_->create_response(req, res);
    });

  if (enable_selected_load) {
    selected_map_loader_ = std::make_unique<SelectedMapLoaderModule>(pcd_metadata_dict);
    get_selected_pcd_maps_service_ = create_service<GetSelectedPointCloudMap>(
      "service/get_selected_pcd_map", [this](
                                        GetSelectedPointCloudMap::Request::SharedPtr req,
                                        GetSelectedPointCloudMap::Response::SharedPtr res) {
        return selected_map_loader_->create_response(req, res);
      });

    rclcpp::QoS durable_qos{1};
    durable_qos.transient_local();
    pub_metadata_ = create_publisher<autoware_map_msgs::msg::PointCloudMapMetaData>(
      "output/pointcloud_map_metadata", durable_qos);
    pub_metadata_->publish(create_metadata(pcd_metadata_dict));
  }
}
}  // namespace autoware::map_loader

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::map_loader::PointCloudMapLoaderNode)
