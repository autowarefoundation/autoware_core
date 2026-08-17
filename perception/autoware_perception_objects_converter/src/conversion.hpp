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

#ifndef CONVERSION_HPP_
#define CONVERSION_HPP_

#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <functional>

namespace autoware::perception_objects_converter
{
/// \brief Type of the callable used to generate a fresh object id per converted object.
using UuidGenerator = std::function<unique_identifier_msgs::msg::UUID()>;

/// \brief Convert a single DetectedObject into a PredictedObject.
///
/// The mapping copies the existence probability, classification and shape, fills the
/// initial pose from the detected kinematics, and only copies the initial twist when the
/// detected kinematics report `has_twist`. Acceleration and predicted paths are left at their
/// message defaults because they are not available in a DetectedObject. The object id is
/// produced by the supplied \p generate_uuid callable.
inline autoware_perception_msgs::msg::PredictedObject convert_object(
  const autoware_perception_msgs::msg::DetectedObject & detected_object,
  const UuidGenerator & generate_uuid)
{
  autoware_perception_msgs::msg::PredictedObject predicted_object;

  predicted_object.object_id = generate_uuid();
  predicted_object.existence_probability = detected_object.existence_probability;
  predicted_object.classification = detected_object.classification;
  predicted_object.shape = detected_object.shape;

  predicted_object.kinematics.initial_pose_with_covariance =
    detected_object.kinematics.pose_with_covariance;
  if (detected_object.kinematics.has_twist) {
    predicted_object.kinematics.initial_twist_with_covariance =
      detected_object.kinematics.twist_with_covariance;
  }
  // Note: acceleration and predicted paths are left default; they are not available in a
  // DetectedObject.

  return predicted_object;
}

/// \brief Convert a DetectedObjects message into a PredictedObjects message.
///
/// The header is propagated unchanged and each detected object is converted via
/// convert_object(). The output vector is reserved up front to avoid per-object reallocations.
/// The \p generate_uuid callable is invoked once per output object to set its object id; whether
/// those ids are unique or non-zero is the generator's responsibility, not enforced here. It
/// defaults to autoware_utils_uuid::generate_uuid.
inline autoware_perception_msgs::msg::PredictedObjects convert(
  const autoware_perception_msgs::msg::DetectedObjects & detected_objects,
  const UuidGenerator & generate_uuid = &autoware_utils_uuid::generate_uuid)
{
  autoware_perception_msgs::msg::PredictedObjects predicted_objects;
  predicted_objects.header = detected_objects.header;
  predicted_objects.objects.reserve(detected_objects.objects.size());

  for (const auto & detected_object : detected_objects.objects) {
    predicted_objects.objects.emplace_back(convert_object(detected_object, generate_uuid));
  }

  return predicted_objects;
}
}  // namespace autoware::perception_objects_converter

#endif  // CONVERSION_HPP_
