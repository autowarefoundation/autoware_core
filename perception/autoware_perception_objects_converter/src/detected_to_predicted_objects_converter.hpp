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

#ifndef DETECTED_TO_PREDICTED_OBJECTS_CONVERTER_HPP_
#define DETECTED_TO_PREDICTED_OBJECTS_CONVERTER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <string>

namespace autoware::perception_objects_converter
{
class DetectedToPredictedObjectsConverter : public rclcpp::Node
{
public:
  explicit DetectedToPredictedObjectsConverter(const rclcpp::NodeOptions & options);

private:
  void detected_objects_callback(
    const autoware_perception_msgs::msg::DetectedObjects::SharedPtr detected_objects_msg);

  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr
    detected_objects_sub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr
    predicted_objects_pub_;
};
}  // namespace autoware::perception_objects_converter

#endif  // DETECTED_TO_PREDICTED_OBJECTS_CONVERTER_HPP_
