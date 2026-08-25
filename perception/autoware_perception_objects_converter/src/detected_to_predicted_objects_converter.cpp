// Copyright(c) 2025 AutoCore Technology (Nanjing) Co., Ltd. All rights reserved.
//
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

#include "detected_to_predicted_objects_converter.hpp"

#include "conversion.hpp"

#include <memory>
#include <utility>

namespace autoware::perception_objects_converter
{
DetectedToPredictedObjectsConverter::DetectedToPredictedObjectsConverter(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("detected_to_predicted_objects_converter", options)
{
  detected_objects_sub_ = create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
    "input/detected_objects", rclcpp::QoS{10},
    std::bind(
      &DetectedToPredictedObjectsConverter::detected_objects_callback, this,
      std::placeholders::_1));

  predicted_objects_pub_ = create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
    "output/predicted_objects", rclcpp::QoS{10});
}

void DetectedToPredictedObjectsConverter::detected_objects_callback(
  const autoware_perception_msgs::msg::DetectedObjects::SharedPtr detected_objects_msg)
{
  auto predicted_objects_msg = std::make_unique<autoware_perception_msgs::msg::PredictedObjects>(
    convert(*detected_objects_msg));

  predicted_objects_pub_->publish(std::move(predicted_objects_msg));
}
}  // namespace autoware::perception_objects_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception_objects_converter::DetectedToPredictedObjectsConverter)
