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

#ifndef AUTOWARE__POINT_TYPES__MEMORY_HPP_
#define AUTOWARE__POINT_TYPES__MEMORY_HPP_

#include "autoware/point_types/types.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <vector>

namespace autoware::point_types
{

inline bool is_data_layout_compatible_with_point_xyzi(
  const std::vector<sensor_msgs::msg::PointField> & fields)
{
  using PointIndex = autoware::point_types::PointXYZIIndex;
  using PointType = autoware::point_types::PointXYZI;

  if (fields.size() < 4) {
    return false;
  }
  bool same_layout = true;
  const auto & field_x = fields.at(static_cast<std::size_t>(PointIndex::X));
  same_layout &= field_x.name == "x";
  same_layout &= field_x.offset == offsetof(PointType, x);
  same_layout &= field_x.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_x.count == 1;
  const auto & field_y = fields.at(static_cast<std::size_t>(PointIndex::Y));
  same_layout &= field_y.name == "y";
  same_layout &= field_y.offset == offsetof(PointType, y);
  same_layout &= field_y.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_y.count == 1;
  const auto & field_z = fields.at(static_cast<std::size_t>(PointIndex::Z));
  same_layout &= field_z.name == "z";
  same_layout &= field_z.offset == offsetof(PointType, z);
  same_layout &= field_z.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_z.count == 1;
  const auto & field_intensity = fields.at(static_cast<std::size_t>(PointIndex::Intensity));
  same_layout &= field_intensity.name == "intensity";
  same_layout &= field_intensity.offset == offsetof(PointType, intensity);
  same_layout &= field_intensity.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_intensity.count == 1;
  return same_layout;
}

inline bool is_data_layout_compatible_with_point_xyzi(const sensor_msgs::msg::PointCloud2 & input)
{
  return is_data_layout_compatible_with_point_xyzi(input.fields);
}

inline bool is_data_layout_compatible_with_point_xyzirc(
  const std::vector<sensor_msgs::msg::PointField> & fields)
{
  using PointIndex = autoware::point_types::PointXYZIRCIndex;
  using PointType = autoware::point_types::PointXYZIRC;
  if (fields.size() < 6) {
    return false;
  }
  bool same_layout = true;
  const auto & field_x = fields.at(static_cast<std::size_t>(PointIndex::X));
  same_layout &= field_x.name == "x";
  same_layout &= field_x.offset == offsetof(PointType, x);
  same_layout &= field_x.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_x.count == 1;
  const auto & field_y = fields.at(static_cast<std::size_t>(PointIndex::Y));
  same_layout &= field_y.name == "y";
  same_layout &= field_y.offset == offsetof(PointType, y);
  same_layout &= field_y.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_y.count == 1;
  const auto & field_z = fields.at(static_cast<std::size_t>(PointIndex::Z));
  same_layout &= field_z.name == "z";
  same_layout &= field_z.offset == offsetof(PointType, z);
  same_layout &= field_z.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_z.count == 1;
  const auto & field_intensity = fields.at(static_cast<std::size_t>(PointIndex::Intensity));
  same_layout &= field_intensity.name == "intensity";
  same_layout &= field_intensity.offset == offsetof(PointType, intensity);
  same_layout &= field_intensity.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_intensity.count == 1;
  const auto & field_return_type = fields.at(static_cast<std::size_t>(PointIndex::ReturnType));
  same_layout &= field_return_type.name == "return_type";
  same_layout &= field_return_type.offset == offsetof(PointType, return_type);
  same_layout &= field_return_type.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_return_type.count == 1;
  const auto & field_ring = fields.at(static_cast<std::size_t>(PointIndex::Channel));
  same_layout &= field_ring.name == "channel";
  same_layout &= field_ring.offset == offsetof(PointType, channel);
  same_layout &= field_ring.datatype == sensor_msgs::msg::PointField::UINT16;
  same_layout &= field_ring.count == 1;

  return same_layout;
}

inline bool is_data_layout_compatible_with_point_xyzirc(const sensor_msgs::msg::PointCloud2 & input)
{
  return is_data_layout_compatible_with_point_xyzirc(input.fields);
}

inline bool is_data_layout_compatible_with_point_xyziradrt(
  const std::vector<sensor_msgs::msg::PointField> & fields)
{
  using PointIndex = autoware::point_types::PointXYZIRADRTIndex;
  using PointType = autoware::point_types::PointXYZIRADRT;

  if (fields.size() < 9) {
    return false;
  }
  bool same_layout = true;
  const auto & field_x = fields.at(static_cast<std::size_t>(PointIndex::X));
  same_layout &= field_x.name == "x";
  same_layout &= field_x.offset == offsetof(PointType, x);
  same_layout &= field_x.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_x.count == 1;
  const auto & field_y = fields.at(static_cast<std::size_t>(PointIndex::Y));
  same_layout &= field_y.name == "y";
  same_layout &= field_y.offset == offsetof(PointType, y);
  same_layout &= field_y.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_y.count == 1;
  const auto & field_z = fields.at(static_cast<std::size_t>(PointIndex::Z));
  same_layout &= field_z.name == "z";
  same_layout &= field_z.offset == offsetof(PointType, z);
  same_layout &= field_z.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_z.count == 1;
  const auto & field_intensity = fields.at(static_cast<std::size_t>(PointIndex::Intensity));
  same_layout &= field_intensity.name == "intensity";
  same_layout &= field_intensity.offset == offsetof(PointType, intensity);
  same_layout &= field_intensity.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_intensity.count == 1;
  const auto & field_ring = fields.at(static_cast<std::size_t>(PointIndex::Ring));
  same_layout &= field_ring.name == "ring";
  same_layout &= field_ring.offset == offsetof(PointType, ring);
  same_layout &= field_ring.datatype == sensor_msgs::msg::PointField::UINT16;
  same_layout &= field_ring.count == 1;
  const auto & field_azimuth = fields.at(static_cast<std::size_t>(PointIndex::Azimuth));
  same_layout &= field_azimuth.name == "azimuth";
  same_layout &= field_azimuth.offset == offsetof(PointType, azimuth);
  same_layout &= field_azimuth.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_azimuth.count == 1;
  const auto & field_distance = fields.at(static_cast<std::size_t>(PointIndex::Distance));
  same_layout &= field_distance.name == "distance";
  same_layout &= field_distance.offset == offsetof(PointType, distance);
  same_layout &= field_distance.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_distance.count == 1;
  const auto & field_return_type = fields.at(static_cast<std::size_t>(PointIndex::ReturnType));
  same_layout &= field_return_type.name == "return_type";
  same_layout &= field_return_type.offset == offsetof(PointType, return_type);
  same_layout &= field_return_type.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_return_type.count == 1;
  const auto & field_time_stamp = fields.at(static_cast<std::size_t>(PointIndex::TimeStamp));
  same_layout &= field_time_stamp.name == "time_stamp";
  same_layout &= field_time_stamp.offset == offsetof(PointType, time_stamp);
  same_layout &= field_time_stamp.datatype == sensor_msgs::msg::PointField::FLOAT64;
  same_layout &= field_time_stamp.count == 1;
  return same_layout;
}

inline bool is_data_layout_compatible_with_point_xyziradrt(
  const sensor_msgs::msg::PointCloud2 & input)
{
  return is_data_layout_compatible_with_point_xyziradrt(input.fields);
}

inline bool is_data_layout_compatible_with_point_xyzircaedt(
  const std::vector<sensor_msgs::msg::PointField> & fields)
{
  using PointIndex = autoware::point_types::PointXYZIRCAEDTIndex;
  using PointType = autoware::point_types::PointXYZIRCAEDT;

  if (fields.size() != 10) {
    return false;
  }
  bool same_layout = true;
  const auto & field_x = fields.at(static_cast<std::size_t>(PointIndex::X));
  same_layout &= field_x.name == "x";
  same_layout &= field_x.offset == offsetof(PointType, x);
  same_layout &= field_x.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_x.count == 1;
  const auto & field_y = fields.at(static_cast<std::size_t>(PointIndex::Y));
  same_layout &= field_y.name == "y";
  same_layout &= field_y.offset == offsetof(PointType, y);
  same_layout &= field_y.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_y.count == 1;
  const auto & field_z = fields.at(static_cast<std::size_t>(PointIndex::Z));
  same_layout &= field_z.name == "z";
  same_layout &= field_z.offset == offsetof(PointType, z);
  same_layout &= field_z.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_z.count == 1;
  const auto & field_intensity = fields.at(static_cast<std::size_t>(PointIndex::Intensity));
  same_layout &= field_intensity.name == "intensity";
  same_layout &= field_intensity.offset == offsetof(PointType, intensity);
  same_layout &= field_intensity.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_intensity.count == 1;
  const auto & field_return_type = fields.at(static_cast<std::size_t>(PointIndex::ReturnType));
  same_layout &= field_return_type.name == "return_type";
  same_layout &= field_return_type.offset == offsetof(PointType, return_type);
  same_layout &= field_return_type.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_return_type.count == 1;
  const auto & field_ring = fields.at(static_cast<std::size_t>(PointIndex::Channel));
  same_layout &= field_ring.name == "channel";
  same_layout &= field_ring.offset == offsetof(PointType, channel);
  same_layout &= field_ring.datatype == sensor_msgs::msg::PointField::UINT16;
  same_layout &= field_ring.count == 1;
  const auto & field_azimuth = fields.at(static_cast<std::size_t>(PointIndex::Azimuth));
  same_layout &= field_azimuth.name == "azimuth";
  same_layout &= field_azimuth.offset == offsetof(PointType, azimuth);
  same_layout &= field_azimuth.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_azimuth.count == 1;
  const auto & field_elevation = fields.at(static_cast<std::size_t>(PointIndex::Elevation));
  same_layout &= field_elevation.name == "elevation";
  same_layout &= field_elevation.offset == offsetof(PointType, elevation);
  same_layout &= field_elevation.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_elevation.count == 1;
  const auto & field_distance = fields.at(static_cast<std::size_t>(PointIndex::Distance));
  same_layout &= field_distance.name == "distance";
  same_layout &= field_distance.offset == offsetof(PointType, distance);
  same_layout &= field_distance.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_distance.count == 1;
  const auto & field_time_stamp = fields.at(static_cast<std::size_t>(PointIndex::TimeStamp));
  same_layout &= field_time_stamp.name == "time_stamp";
  same_layout &= field_time_stamp.offset == offsetof(PointType, time_stamp);
  same_layout &= field_time_stamp.datatype == sensor_msgs::msg::PointField::UINT32;
  same_layout &= field_time_stamp.count == 1;
  return same_layout;
}

inline bool is_data_layout_compatible_with_point_xyzircaedt(
  const sensor_msgs::msg::PointCloud2 & input)
{
  return is_data_layout_compatible_with_point_xyzircaedt(input.fields);
}

inline std::vector<sensor_msgs::msg::PointField> create_fields_point_xyzi()
{
  using PointIndex = autoware::point_types::PointXYZIIndex;
  using PointType = autoware::point_types::PointXYZI;
  std::vector<sensor_msgs::msg::PointField> fields;
  fields.resize(4);
  fields[static_cast<std::size_t>(PointIndex::X)].name = "x";
  fields[static_cast<std::size_t>(PointIndex::X)].offset = offsetof(PointType, x);
  fields[static_cast<std::size_t>(PointIndex::X)].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::X)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Y)].name = "y";
  fields[static_cast<std::size_t>(PointIndex::Y)].offset = offsetof(PointType, y);
  fields[static_cast<std::size_t>(PointIndex::Y)].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::Y)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Z)].name = "z";
  fields[static_cast<std::size_t>(PointIndex::Z)].offset = offsetof(PointType, z);
  fields[static_cast<std::size_t>(PointIndex::Z)].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::Z)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Intensity)].name = "intensity";
  fields[static_cast<std::size_t>(PointIndex::Intensity)].offset = offsetof(PointType, intensity);
  fields[static_cast<std::size_t>(PointIndex::Intensity)].datatype =
    sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::Intensity)].count = 1;

  return fields;
}

inline std::vector<sensor_msgs::msg::PointField> create_fields_point_xyzirc()
{
  using PointIndex = autoware::point_types::PointXYZIRCIndex;
  using PointType = autoware::point_types::PointXYZIRC;
  std::vector<sensor_msgs::msg::PointField> fields;
  fields.resize(6);
  fields[static_cast<std::size_t>(PointIndex::X)].name = "x";
  fields[static_cast<std::size_t>(PointIndex::X)].offset = offsetof(PointType, x);
  fields[static_cast<std::size_t>(PointIndex::X)].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::X)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Y)].name = "y";
  fields[static_cast<std::size_t>(PointIndex::Y)].offset = offsetof(PointType, y);
  fields[static_cast<std::size_t>(PointIndex::Y)].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::Y)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Z)].name = "z";
  fields[static_cast<std::size_t>(PointIndex::Z)].offset = offsetof(PointType, z);
  fields[static_cast<std::size_t>(PointIndex::Z)].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::Z)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Intensity)].name = "intensity";
  fields[static_cast<std::size_t>(PointIndex::Intensity)].offset = offsetof(PointType, intensity);
  fields[static_cast<std::size_t>(PointIndex::Intensity)].datatype =
    sensor_msgs::msg::PointField::UINT8;
  fields[static_cast<std::size_t>(PointIndex::Intensity)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::ReturnType)].name = "return_type";
  fields[static_cast<std::size_t>(PointIndex::ReturnType)].offset =
    offsetof(PointType, return_type);
  fields[static_cast<std::size_t>(PointIndex::ReturnType)].datatype =
    sensor_msgs::msg::PointField::UINT8;
  fields[static_cast<std::size_t>(PointIndex::ReturnType)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Channel)].name = "channel";
  fields[static_cast<std::size_t>(PointIndex::Channel)].offset = offsetof(PointType, channel);
  fields[static_cast<std::size_t>(PointIndex::Channel)].datatype =
    sensor_msgs::msg::PointField::UINT16;
  fields[static_cast<std::size_t>(PointIndex::Channel)].count = 1;

  return fields;
}

inline std::vector<sensor_msgs::msg::PointField> create_fields_point_xyziradrt()
{
  using PointIndex = autoware::point_types::PointXYZIRADRTIndex;
  using PointType = autoware::point_types::PointXYZIRADRT;
  std::vector<sensor_msgs::msg::PointField> fields;
  fields.resize(9);
  fields[static_cast<std::size_t>(PointIndex::X)].name = "x";
  fields[static_cast<std::size_t>(PointIndex::X)].offset = offsetof(PointType, x);
  fields[static_cast<std::size_t>(PointIndex::X)].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::X)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Y)].name = "y";
  fields[static_cast<std::size_t>(PointIndex::Y)].offset = offsetof(PointType, y);
  fields[static_cast<std::size_t>(PointIndex::Y)].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::Y)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Z)].name = "z";
  fields[static_cast<std::size_t>(PointIndex::Z)].offset = offsetof(PointType, z);
  fields[static_cast<std::size_t>(PointIndex::Z)].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::Z)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Intensity)].name = "intensity";
  fields[static_cast<std::size_t>(PointIndex::Intensity)].offset = offsetof(PointType, intensity);
  fields[static_cast<std::size_t>(PointIndex::Intensity)].datatype =
    sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::Intensity)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Ring)].name = "ring";
  fields[static_cast<std::size_t>(PointIndex::Ring)].offset = offsetof(PointType, ring);
  fields[static_cast<std::size_t>(PointIndex::Ring)].datatype =
    sensor_msgs::msg::PointField::UINT16;
  fields[static_cast<std::size_t>(PointIndex::Ring)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Azimuth)].name = "azimuth";
  fields[static_cast<std::size_t>(PointIndex::Azimuth)].offset = offsetof(PointType, azimuth);
  fields[static_cast<std::size_t>(PointIndex::Azimuth)].datatype =
    sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::Azimuth)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Distance)].name = "distance";
  fields[static_cast<std::size_t>(PointIndex::Distance)].offset = offsetof(PointType, distance);
  fields[static_cast<std::size_t>(PointIndex::Distance)].datatype =
    sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::Distance)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::ReturnType)].name = "return_type";
  fields[static_cast<std::size_t>(PointIndex::ReturnType)].offset =
    offsetof(PointType, return_type);
  fields[static_cast<std::size_t>(PointIndex::ReturnType)].datatype =
    sensor_msgs::msg::PointField::UINT8;
  fields[static_cast<std::size_t>(PointIndex::ReturnType)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::TimeStamp)].name = "time_stamp";
  fields[static_cast<std::size_t>(PointIndex::TimeStamp)].offset = offsetof(PointType, time_stamp);
  fields[static_cast<std::size_t>(PointIndex::TimeStamp)].datatype =
    sensor_msgs::msg::PointField::FLOAT64;
  fields[static_cast<std::size_t>(PointIndex::TimeStamp)].count = 1;

  return fields;
}

inline std::vector<sensor_msgs::msg::PointField> create_fields_point_xyzircaedt()
{
  using PointIndex = autoware::point_types::PointXYZIRCAEDTIndex;
  using PointType = autoware::point_types::PointXYZIRCAEDT;
  std::vector<sensor_msgs::msg::PointField> fields;
  fields.resize(10);
  fields[static_cast<std::size_t>(PointIndex::X)].name = "x";
  fields[static_cast<std::size_t>(PointIndex::X)].offset = offsetof(PointType, x);
  fields[static_cast<std::size_t>(PointIndex::X)].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::X)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Y)].name = "y";
  fields[static_cast<std::size_t>(PointIndex::Y)].offset = offsetof(PointType, y);
  fields[static_cast<std::size_t>(PointIndex::Y)].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::Y)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Z)].name = "z";
  fields[static_cast<std::size_t>(PointIndex::Z)].offset = offsetof(PointType, z);
  fields[static_cast<std::size_t>(PointIndex::Z)].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::Z)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Intensity)].name = "intensity";
  fields[static_cast<std::size_t>(PointIndex::Intensity)].offset = offsetof(PointType, intensity);
  fields[static_cast<std::size_t>(PointIndex::Intensity)].datatype =
    sensor_msgs::msg::PointField::UINT8;
  fields[static_cast<std::size_t>(PointIndex::Intensity)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::ReturnType)].name = "return_type";
  fields[static_cast<std::size_t>(PointIndex::ReturnType)].offset =
    offsetof(PointType, return_type);
  fields[static_cast<std::size_t>(PointIndex::ReturnType)].datatype =
    sensor_msgs::msg::PointField::UINT8;
  fields[static_cast<std::size_t>(PointIndex::ReturnType)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Channel)].name = "channel";
  fields[static_cast<std::size_t>(PointIndex::Channel)].offset = offsetof(PointType, channel);
  fields[static_cast<std::size_t>(PointIndex::Channel)].datatype =
    sensor_msgs::msg::PointField::UINT16;
  fields[static_cast<std::size_t>(PointIndex::Channel)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Azimuth)].name = "azimuth";
  fields[static_cast<std::size_t>(PointIndex::Azimuth)].offset = offsetof(PointType, azimuth);
  fields[static_cast<std::size_t>(PointIndex::Azimuth)].datatype =
    sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::Azimuth)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Elevation)].name = "elevation";
  fields[static_cast<std::size_t>(PointIndex::Elevation)].offset = offsetof(PointType, elevation);
  fields[static_cast<std::size_t>(PointIndex::Elevation)].datatype =
    sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::Elevation)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::Distance)].name = "distance";
  fields[static_cast<std::size_t>(PointIndex::Distance)].offset = offsetof(PointType, distance);
  fields[static_cast<std::size_t>(PointIndex::Distance)].datatype =
    sensor_msgs::msg::PointField::FLOAT32;
  fields[static_cast<std::size_t>(PointIndex::Distance)].count = 1;
  fields[static_cast<std::size_t>(PointIndex::TimeStamp)].name = "time_stamp";
  fields[static_cast<std::size_t>(PointIndex::TimeStamp)].offset = offsetof(PointType, time_stamp);
  fields[static_cast<std::size_t>(PointIndex::TimeStamp)].datatype =
    sensor_msgs::msg::PointField::UINT32;
  fields[static_cast<std::size_t>(PointIndex::TimeStamp)].count = 1;

  return fields;
}

}  // namespace autoware::point_types

#endif  // AUTOWARE__POINT_TYPES__MEMORY_HPP_
