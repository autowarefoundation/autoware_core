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

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <cstdint>
#include <optional>
#include <string_view>

namespace autoware::object_recognition_utils
{

using autoware_perception_msgs::msg::ObjectClassification;
using ObjectLabel = ObjectClassification::_label_type;

/**
 * @brief Classification labels for point cloud segmentation.
 */
enum class PointCloudClassification : std::uint8_t {
  CAR = 0,
  TRUCK = 1,
  BUS = 2,
  MOTORCYCLE = 3,
  BICYCLE = 4,
  PEDESTRIAN = 5,
  ANIMAL = 6,
  HAZARD = 7,
  FLAT_SURFACE = 8,  ///< Flat surfaces that can be filtered out.
  STRUCTURE = 9,     ///< Non-drivable structures, such as buildings and walls.
  VEGETATION = 10,   ///< Vegetation, such as trees and bushes.
  NOISE = 11,        ///< Noise points and outliers.
};

/**
 * @brief Get the string representation of a point cloud classification.
 * @param classification The classification to convert.
 * @return String view of the classification name.
 */
constexpr std::string_view to_string(PointCloudClassification classification) noexcept
{
  switch (classification) {
    case PointCloudClassification::CAR:
      return "CAR";
    case PointCloudClassification::TRUCK:
      return "TRUCK";
    case PointCloudClassification::BUS:
      return "BUS";
    case PointCloudClassification::MOTORCYCLE:
      return "MOTORCYCLE";
    case PointCloudClassification::BICYCLE:
      return "BICYCLE";
    case PointCloudClassification::PEDESTRIAN:
      return "PEDESTRIAN";
    case PointCloudClassification::ANIMAL:
      return "ANIMAL";
    case PointCloudClassification::HAZARD:
      return "HAZARD";
    case PointCloudClassification::FLAT_SURFACE:
      return "FLAT_SURFACE";
    case PointCloudClassification::STRUCTURE:
      return "STRUCTURE";
    case PointCloudClassification::VEGETATION:
      return "VEGETATION";
    case PointCloudClassification::NOISE:
      return "NOISE";
    default:
      return "UNKNOWN";
  }
}

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
inline std::optional<PointCloudClassification> try_into_semantic(ObjectLabel label) noexcept
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
