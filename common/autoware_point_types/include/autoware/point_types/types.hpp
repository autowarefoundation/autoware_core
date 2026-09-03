// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__POINT_TYPES__TYPES_HPP_
#define AUTOWARE__POINT_TYPES__TYPES_HPP_

#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>

namespace autoware::point_types
{
template <class T>
bool float_eq(const T a, const T b, const T eps = 10e-6)
{
  return std::fabs(a - b) < eps;
}

inline bool float_eq_or_both_nan(const float a, const float b)
{
  return float_eq<float>(a, b) || (std::isnan(a) && std::isnan(b));
}

struct PointXYZI
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  friend bool operator==(const PointXYZI & p1, const PointXYZI & p2) noexcept
  {
    return float_eq<float>(p1.x, p2.x) && float_eq<float>(p1.y, p2.y) &&
           float_eq<float>(p1.z, p2.z) && float_eq<float>(p1.intensity, p2.intensity);
  }
};

enum ReturnType : uint8_t {
  INVALID = 0,
  SINGLE_STRONGEST,
  SINGLE_LAST,
  DUAL_STRONGEST_FIRST,
  DUAL_STRONGEST_LAST,
  DUAL_WEAK_FIRST,
  DUAL_WEAK_LAST,
  DUAL_ONLY,
};

struct PointXYZIRC
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  std::uint8_t intensity{0U};
  std::uint8_t return_type{0U};
  std::uint16_t channel{0U};

  friend bool operator==(const PointXYZIRC & p1, const PointXYZIRC & p2) noexcept
  {
    return float_eq<float>(p1.x, p2.x) && float_eq<float>(p1.y, p2.y) &&
           float_eq<float>(p1.z, p2.z) && p1.intensity == p2.intensity &&
           p1.return_type == p2.return_type && p1.channel == p2.channel;
  }
};

/// @brief PointXYZIRC extended by a per-point time stamp. Intended for point clouds whose points
/// no longer share a common sensor origin (e.g. the concatenation of several LiDARs), where the
/// polar fields of PointXYZIRCAEDT have lost their meaning but the acquisition time has not.
struct PointXYZIRCT
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  std::uint8_t intensity{0U};
  std::uint8_t return_type{0U};
  std::uint16_t channel{0U};
  /// Acquisition time of this point, as a non-negative offset in nanoseconds from the
  /// containing point cloud's `header.stamp`.
  std::uint32_t time_stamp{0U};

  friend bool operator==(const PointXYZIRCT & p1, const PointXYZIRCT & p2) noexcept
  {
    return float_eq<float>(p1.x, p2.x) && float_eq<float>(p1.y, p2.y) &&
           float_eq<float>(p1.z, p2.z) && p1.intensity == p2.intensity &&
           p1.return_type == p2.return_type && p1.channel == p2.channel &&
           p1.time_stamp == p2.time_stamp;
  }
};

struct PointXYZIRADRT
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  uint16_t ring{0U};
  float azimuth{0.0F};
  float distance{0.0F};
  uint8_t return_type{0U};
  double time_stamp{0.0};
  friend bool operator==(const PointXYZIRADRT & p1, const PointXYZIRADRT & p2) noexcept
  {
    return float_eq<float>(p1.x, p2.x) && float_eq<float>(p1.y, p2.y) &&
           float_eq<float>(p1.z, p2.z) && float_eq<float>(p1.intensity, p2.intensity) &&
           p1.ring == p2.ring && float_eq<float>(p1.azimuth, p2.azimuth) &&
           float_eq<float>(p1.distance, p2.distance) && p1.return_type == p2.return_type &&
           float_eq<float>(p1.time_stamp, p2.time_stamp);
  }
};

struct PointXYZIRCAEDT
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  std::uint8_t intensity{0U};
  std::uint8_t return_type{0U};
  std::uint16_t channel{0U};
  float azimuth{0.0F};
  float elevation{0.0F};
  float distance{0.0F};
  /// Acquisition time of this point, as a non-negative offset in nanoseconds from the
  /// containing point cloud's `header.stamp`.
  std::uint32_t time_stamp{0U};

  friend bool operator==(const PointXYZIRCAEDT & p1, const PointXYZIRCAEDT & p2) noexcept
  {
    return float_eq<float>(p1.x, p2.x) && float_eq<float>(p1.y, p2.y) &&
           float_eq<float>(p1.z, p2.z) && p1.intensity == p2.intensity &&
           p1.return_type == p2.return_type && p1.channel == p2.channel &&
           float_eq<float>(p1.azimuth, p2.azimuth) && float_eq<float>(p1.distance, p2.distance) &&
           p1.time_stamp == p2.time_stamp;
  }
};

enum class PointXYZIIndex { X, Y, Z, Intensity };
enum class PointXYZIRCIndex { X, Y, Z, Intensity, ReturnType, Channel };
enum class PointXYZIRCTIndex { X, Y, Z, Intensity, ReturnType, Channel, TimeStamp };
enum class PointXYZIRADRTIndex {
  X,
  Y,
  Z,
  Intensity,
  Ring,
  Azimuth,
  Distance,
  ReturnType,
  TimeStamp
};
enum class PointXYZIRCAEDTIndex {
  X,
  Y,
  Z,
  Intensity,
  ReturnType,
  Channel,
  Azimuth,
  Elevation,
  Distance,
  TimeStamp
};

LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(azimuth);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(elevation);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(distance);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(return_type);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(time_stamp);

LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(channel);

using PointXYZIRCGenerator = std::tuple<
  point_cloud_msg_wrapper::field_x_generator, point_cloud_msg_wrapper::field_y_generator,
  point_cloud_msg_wrapper::field_z_generator, point_cloud_msg_wrapper::field_intensity_generator,
  field_return_type_generator, field_channel_generator>;

using PointXYZIRCTGenerator = std::tuple<
  point_cloud_msg_wrapper::field_x_generator, point_cloud_msg_wrapper::field_y_generator,
  point_cloud_msg_wrapper::field_z_generator, point_cloud_msg_wrapper::field_intensity_generator,
  field_return_type_generator, field_channel_generator, field_time_stamp_generator>;

using PointXYZIRADRTGenerator = std::tuple<
  point_cloud_msg_wrapper::field_x_generator, point_cloud_msg_wrapper::field_y_generator,
  point_cloud_msg_wrapper::field_z_generator, point_cloud_msg_wrapper::field_intensity_generator,
  point_cloud_msg_wrapper::field_ring_generator, field_azimuth_generator, field_distance_generator,
  field_return_type_generator, field_time_stamp_generator>;

using PointXYZIRCAEDTGenerator = std::tuple<
  point_cloud_msg_wrapper::field_x_generator, point_cloud_msg_wrapper::field_y_generator,
  point_cloud_msg_wrapper::field_z_generator, point_cloud_msg_wrapper::field_intensity_generator,
  field_return_type_generator, field_channel_generator, field_azimuth_generator,
  field_elevation_generator, field_distance_generator, field_time_stamp_generator>;

/**
 * @brief Classification labels for point cloud segmentation, stored in PointXYZCPE::class_id.
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
  INVALID = 255,     ///< No classification assigned, e.g. a default-constructed point.
};

/**
 * @brief Get the string representation of a point cloud classification.
 * @param classification The classification to convert.
 * @return String view of the classification name.
 * @throws std::invalid_argument If the value does not correspond to any enumerator.
 */
constexpr std::string_view to_string(PointCloudClassification classification)
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
    case PointCloudClassification::INVALID:
      return "INVALID";
    default:
      throw std::invalid_argument("Unknown point cloud classification");
  }
}

/**
 * @brief Get the point cloud classification from its string representation.
 * @details The name is normalized to uppercase first, so matching is case-insensitive for ASCII
 * letters. Accepted names are exactly those produced by to_string(), including "INVALID".
 * @param name Classification name, e.g. "CAR" or "flat_surface".
 * @return Matching classification.
 * @throws std::invalid_argument If the name does not correspond to any enumerator.
 */
inline PointCloudClassification to_pointcloud_classification(std::string_view name)
{
  std::string uppercase(name);
  std::transform(uppercase.begin(), uppercase.end(), uppercase.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });

  // Matching against to_string() keeps both directions in sync: a new enumerator only has to be
  // added to to_string() and to this list.
  constexpr std::array<PointCloudClassification, 13> classifications{
    PointCloudClassification::CAR,          PointCloudClassification::TRUCK,
    PointCloudClassification::BUS,          PointCloudClassification::MOTORCYCLE,
    PointCloudClassification::BICYCLE,      PointCloudClassification::PEDESTRIAN,
    PointCloudClassification::ANIMAL,       PointCloudClassification::HAZARD,
    PointCloudClassification::FLAT_SURFACE, PointCloudClassification::STRUCTURE,
    PointCloudClassification::VEGETATION,   PointCloudClassification::NOISE,
    PointCloudClassification::INVALID};

  for (const auto classification : classifications) {
    if (uppercase == to_string(classification)) {
      return classification;
    }
  }

  throw std::invalid_argument("Unknown point cloud classification: '" + std::string(name) + "'");
}

struct PointXYZCPE
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  std::uint8_t class_id{static_cast<std::uint8_t>(PointCloudClassification::INVALID)};
  float probability{0.0F};
  float entropy{std::numeric_limits<float>::quiet_NaN()};

  friend bool operator==(const PointXYZCPE & p1, const PointXYZCPE & p2)
  {
    return autoware::point_types::float_eq<float>(p1.x, p2.x) &&
           autoware::point_types::float_eq<float>(p1.y, p2.y) &&
           autoware::point_types::float_eq<float>(p1.z, p2.z) && p1.class_id == p2.class_id &&
           autoware::point_types::float_eq<float>(p1.probability, p2.probability) &&
           autoware::point_types::float_eq_or_both_nan(p1.entropy, p2.entropy);
  }
};

enum class PointXYZCPEIndex { X, Y, Z, ClassId, Probability, Entropy };

LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(class_id);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(probability);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(entropy);

using PointXYZCPEFieldGenerator = std::tuple<
  point_cloud_msg_wrapper::field_x_generator, point_cloud_msg_wrapper::field_y_generator,
  point_cloud_msg_wrapper::field_z_generator, field_class_id_generator, field_probability_generator,
  field_entropy_generator>;

}  // namespace autoware::point_types

POINT_CLOUD_REGISTER_POINT_STRUCT(
  autoware::point_types::PointXYZIRC,
  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(
    std::uint8_t, return_type, return_type)(std::uint16_t, channel, channel))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  autoware::point_types::PointXYZIRCT,
  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(
    std::uint8_t, return_type,
    return_type)(std::uint16_t, channel, channel)(std::uint32_t, time_stamp, time_stamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  autoware::point_types::PointXYZIRADRT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(
    float, azimuth, azimuth)(float, distance, distance)(std::uint8_t, return_type, return_type)(
    double, time_stamp, time_stamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  autoware::point_types::PointXYZIRCAEDT,
  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(
    std::uint8_t, return_type,
    return_type)(std::uint16_t, channel, channel)(float, azimuth, azimuth)(
    float, elevation, elevation)(float, distance, distance)(std::uint32_t, time_stamp, time_stamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  autoware::point_types::PointXYZCPE,
  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, class_id, class_id)(
    float, probability, probability)(float, entropy, entropy))
#endif  // AUTOWARE__POINT_TYPES__TYPES_HPP_
