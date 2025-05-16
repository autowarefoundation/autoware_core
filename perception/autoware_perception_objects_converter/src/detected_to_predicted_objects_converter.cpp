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

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <algorithm>
#include <memory>

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

// Convert Boost UUID to unique_identifier_msgs::msg::UUID
unique_identifier_msgs::msg::UUID generateUUIDMsg()
{
  boost::uuids::random_generator gen;
  boost::uuids::uuid uuid = gen();

  unique_identifier_msgs::msg::UUID uuid_msg;
  std::copy(uuid.begin(), uuid.end(), uuid_msg.uuid.begin());

  return uuid_msg;
}

void DetectedToPredictedObjectsConverter::detected_objects_callback(
  const autoware_perception_msgs::msg::DetectedObjects::SharedPtr detected_objects_msg)
{
  auto predicted_objects_msg = std::make_unique<autoware_perception_msgs::msg::PredictedObjects>();

  // Copy header
  predicted_objects_msg->header = detected_objects_msg->header;

  // Convert each detected object to predicted object
  for (const auto & detected_object : detected_objects_msg->objects) {
    autoware_perception_msgs::msg::PredictedObject predicted_object;

    // Generate UUID for the object using Boost
    predicted_object.object_id = generateUUIDMsg();

    // Copy fields from detected object
    predicted_object.existence_probability = detected_object.existence_probability;
    predicted_object.classification = detected_object.classification;
    predicted_object.shape = detected_object.shape;

    // Convert kinematics
    autoware_perception_msgs::msg::PredictedObjectKinematics predicted_kinematics;
    predicted_kinematics.initial_pose_with_covariance =
      detected_object.kinematics.pose_with_covariance;

    if (detected_object.kinematics.has_twist) {
      predicted_kinematics.initial_twist_with_covariance =
        detected_object.kinematics.twist_with_covariance;
    }

    // Note: Acceleration and predicted paths would typically be empty or set to default values
    // as they are not available in the DetectedObject message

    predicted_object.kinematics = predicted_kinematics;

    // Add to objects array
    predicted_objects_msg->objects.push_back(predicted_object);
  }

  // Publish the converted message
  predicted_objects_pub_->publish(*predicted_objects_msg);
}
}  // namespace autoware::perception_objects_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception_objects_converter::DetectedToPredictedObjectsConverter)
