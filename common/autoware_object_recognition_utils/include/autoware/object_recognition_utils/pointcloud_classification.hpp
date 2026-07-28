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

#ifndef AUTOWARE__OBJECT_RECOGNITION_UTILS__POINTCLOUD_CLASSIFICATION_HPP_
#define AUTOWARE__OBJECT_RECOGNITION_UTILS__POINTCLOUD_CLASSIFICATION_HPP_

#include <autoware/point_types/types.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <optional>

namespace autoware::object_recognition_utils
{

using autoware_perception_msgs::msg::ObjectClassification;
using ObjectLabel = ObjectClassification::_label_type;
using autoware::point_types::PointCloudClassification;

/**
 * @brief Convert a point cloud classification to an ObjectClassification label.
 * @param classification The point cloud classification to convert.
 * @return The ObjectClassification label value, or std::nullopt for non-object labels.
 */
inline std::optional<ObjectLabel> try_into_object(PointCloudClassification classification) noexcept
{
  switch (classification) {
    case PointCloudClassification::CAR:
      return ObjectClassification::CAR;
    case PointCloudClassification::TRUCK:
      return ObjectClassification::TRUCK;
    case PointCloudClassification::BUS:
      return ObjectClassification::BUS;
    case PointCloudClassification::MOTORCYCLE:
      return ObjectClassification::MOTORCYCLE;
    case PointCloudClassification::BICYCLE:
      return ObjectClassification::BICYCLE;
    case PointCloudClassification::PEDESTRIAN:
      return ObjectClassification::PEDESTRIAN;
    case PointCloudClassification::ANIMAL:
      return ObjectClassification::ANIMAL;
    case PointCloudClassification::HAZARD:
      return ObjectClassification::HAZARD;
    case PointCloudClassification::FLAT_SURFACE:
    case PointCloudClassification::STRUCTURE:
    case PointCloudClassification::VEGETATION:
    case PointCloudClassification::NOISE:
    case PointCloudClassification::INVALID:
      return std::nullopt;
    default:
      return std::nullopt;
  }
}

/**
 * @brief Convert an ObjectClassification label to its point cloud classification.
 * @param label The ObjectClassification label value.
 * @return The corresponding PointCloudClassification, or std::nullopt if not mapped.
 */
inline std::optional<PointCloudClassification> try_into_pointcloud(ObjectLabel label) noexcept
{
  switch (label) {
    case ObjectClassification::CAR:
      return PointCloudClassification::CAR;
    case ObjectClassification::TRUCK:
      return PointCloudClassification::TRUCK;
    case ObjectClassification::BUS:
      return PointCloudClassification::BUS;
    case ObjectClassification::TRAILER:
      return PointCloudClassification::TRUCK;
    case ObjectClassification::MOTORCYCLE:
      return PointCloudClassification::MOTORCYCLE;
    case ObjectClassification::BICYCLE:
      return PointCloudClassification::BICYCLE;
    case ObjectClassification::PEDESTRIAN:
      return PointCloudClassification::PEDESTRIAN;
    case ObjectClassification::ANIMAL:
      return PointCloudClassification::ANIMAL;
    case ObjectClassification::HAZARD:
      return PointCloudClassification::HAZARD;
    default:
      return std::nullopt;
  }
}

/**
 * @brief Check whether a point cloud classification is object-compatible.
 */
inline bool is_object_compatible(PointCloudClassification classification) noexcept
{
  return try_into_object(classification).has_value();
}

}  // namespace autoware::object_recognition_utils

#endif  // AUTOWARE__OBJECT_RECOGNITION_UTILS__POINTCLOUD_CLASSIFICATION_HPP_
