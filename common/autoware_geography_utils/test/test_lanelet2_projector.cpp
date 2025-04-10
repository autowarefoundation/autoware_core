// Copyright 2024 TIER IV, Inc.
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

#include <autoware/geography_utils/lanelet2_projector.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/projection/transverse_mercator_projector.hpp>

#include <gtest/gtest.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_projection/UTM.h>

#include <memory>
#include <stdexcept>

TEST(GeographyUtilsLanelet2Projector, GetMGRSProjector)
{
  autoware_map_msgs::msg::MapProjectorInfo projector_info;
  projector_info.projector_type = autoware_map_msgs::msg::MapProjectorInfo::MGRS;
  projector_info.mgrs_grid = "54SUE";
  projector_info.vertical_datum = autoware_map_msgs::msg::MapProjectorInfo::WGS84;

  std::unique_ptr<lanelet::Projector> projector =
    autoware::geography_utils::get_lanelet2_projector(projector_info);

  // Check if the returned projector is of type MGRSProjector
  EXPECT_NE(dynamic_cast<lanelet::projection::MGRSProjector *>(projector.get()), nullptr);
}

TEST(GeographyUtilsLanelet2Projector, GetLocalCartesianUTMProjector)
{
  autoware_map_msgs::msg::MapProjectorInfo projector_info;
  projector_info.projector_type = autoware_map_msgs::msg::MapProjectorInfo::LOCAL_CARTESIAN_UTM;
  projector_info.vertical_datum = autoware_map_msgs::msg::MapProjectorInfo::WGS84;
  projector_info.map_origin.latitude = 35.62426;
  projector_info.map_origin.longitude = 139.74252;
  projector_info.map_origin.altitude = 0.0;

  std::unique_ptr<lanelet::Projector> projector =
    autoware::geography_utils::get_lanelet2_projector(projector_info);

  // Check if the returned projector is of type UtmProjector
  EXPECT_NE(dynamic_cast<lanelet::projection::UtmProjector *>(projector.get()), nullptr);
}

TEST(GeographyUtilsLanelet2Projector, GetLocalCartesianProjector)
{
  autoware_map_msgs::msg::MapProjectorInfo projector_info;
  projector_info.projector_type = autoware_map_msgs::msg::MapProjectorInfo::LOCAL_CARTESIAN;
  projector_info.vertical_datum = autoware_map_msgs::msg::MapProjectorInfo::WGS84;
  projector_info.map_origin.latitude = 35.62426;
  projector_info.map_origin.longitude = 139.74252;
  projector_info.map_origin.altitude = 0.0;

  std::unique_ptr<lanelet::Projector> projector =
    autoware::geography_utils::get_lanelet2_projector(projector_info);

  // Check if the returned projector is of type LocalCartesianProjector
  EXPECT_NE(dynamic_cast<lanelet::projection::LocalCartesianProjector *>(projector.get()), nullptr);
}

TEST(GeographyUtilsLanelet2Projector, GetTransverseMercatorProjector)
{
  autoware_map_msgs::msg::MapProjectorInfo projector_info;
  projector_info.projector_type = autoware_map_msgs::msg::MapProjectorInfo::TRANSVERSE_MERCATOR;
  projector_info.vertical_datum = autoware_map_msgs::msg::MapProjectorInfo::WGS84;
  projector_info.map_origin.latitude = 35.62426;
  projector_info.map_origin.longitude = 139.74252;
  projector_info.map_origin.altitude = 0.0;
  projector_info.scale_factor = 0.9996;

  std::unique_ptr<lanelet::Projector> projector =
    autoware::geography_utils::get_lanelet2_projector(projector_info);

  // Check if the returned projector is of type TransverseMercatorProjector
  EXPECT_NE(
    dynamic_cast<lanelet::projection::TransverseMercatorProjector *>(projector.get()), nullptr);
}

TEST(GeographyUtilsLanelet2Projector, InvalidProjectorType)
{
  autoware_map_msgs::msg::MapProjectorInfo projector_info;
  projector_info.projector_type = "INVALID_TYPE";
  projector_info.vertical_datum = autoware_map_msgs::msg::MapProjectorInfo::WGS84;

  // Check if the function throws an invalid_argument exception for invalid projector type
  EXPECT_THROW(
    autoware::geography_utils::get_lanelet2_projector(projector_info), std::invalid_argument);
}
