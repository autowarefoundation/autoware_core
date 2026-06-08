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

#include "autoware/map_projection_loader/map_projection_loader.hpp"

#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include <gmock/gmock.h>

#include <fstream>
#include <stdexcept>
#include <string>

namespace
{
void write_yaml(const std::string & content, const std::string & output_path)
{
  std::ofstream file(output_path);
  file << content;
  file.close();
}
}  // namespace

TEST(TestLoadInfoFromYaml, LoadMgrs)
{
  const std::string output_path = "/tmp/test_load_info_from_yaml_mgrs.yaml";
  write_yaml(
    "projector_type: MGRS\n"
    "vertical_datum: WGS84\n"
    "mgrs_grid: 54SUE\n",
    output_path);

  const auto msg = autoware::map_projection_loader::load_info_from_yaml(output_path);

  EXPECT_EQ(msg.projector_type, autoware_map_msgs::msg::MapProjectorInfo::MGRS);
  EXPECT_EQ(msg.vertical_datum, "WGS84");
  EXPECT_EQ(msg.mgrs_grid, "54SUE");
  // MGRS is overwritten with the UTM scale factor.
  EXPECT_FLOAT_EQ(msg.scale_factor, 0.9996F);
}

TEST(TestLoadInfoFromYaml, LoadLocalCartesianUtm)
{
  const std::string output_path = "/tmp/test_load_info_from_yaml_local_cartesian_utm.yaml";
  write_yaml(
    "projector_type: LocalCartesianUTM\n"
    "vertical_datum: WGS84\n"
    "map_origin:\n"
    "  latitude: 35.6762\n"
    "  longitude: 139.6503\n"
    "  altitude: 123.45\n",
    output_path);

  const auto msg = autoware::map_projection_loader::load_info_from_yaml(output_path);

  EXPECT_EQ(msg.projector_type, autoware_map_msgs::msg::MapProjectorInfo::LOCAL_CARTESIAN_UTM);
  EXPECT_EQ(msg.vertical_datum, "WGS84");
  EXPECT_DOUBLE_EQ(msg.map_origin.latitude, 35.6762);
  EXPECT_DOUBLE_EQ(msg.map_origin.longitude, 139.6503);
  // Altitude is always forced to 0.0 regardless of the YAML value.
  EXPECT_DOUBLE_EQ(msg.map_origin.altitude, 0.0);
  EXPECT_FLOAT_EQ(msg.scale_factor, 0.9996F);
}

TEST(TestLoadInfoFromYaml, LoadLocalCartesian)
{
  const std::string output_path = "/tmp/test_load_info_from_yaml_local_cartesian.yaml";
  write_yaml(
    "projector_type: LocalCartesian\n"
    "vertical_datum: WGS84\n"
    "map_origin:\n"
    "  latitude: 35.6762\n"
    "  longitude: 139.6503\n"
    "  altitude: 0.0\n",
    output_path);

  const auto msg = autoware::map_projection_loader::load_info_from_yaml(output_path);

  EXPECT_EQ(msg.projector_type, autoware_map_msgs::msg::MapProjectorInfo::LOCAL_CARTESIAN);
  EXPECT_EQ(msg.vertical_datum, "WGS84");
  EXPECT_DOUBLE_EQ(msg.map_origin.latitude, 35.6762);
  EXPECT_DOUBLE_EQ(msg.map_origin.longitude, 139.6503);
  EXPECT_DOUBLE_EQ(msg.map_origin.altitude, 0.0);
  // LocalCartesian is overwritten with the local scale factor.
  EXPECT_FLOAT_EQ(msg.scale_factor, 1.0F);
}

TEST(TestLoadInfoFromYaml, LoadLocal)
{
  const std::string output_path = "/tmp/test_load_info_from_yaml_local.yaml";
  write_yaml("projector_type: Local\n", output_path);

  const auto msg = autoware::map_projection_loader::load_info_from_yaml(output_path);

  EXPECT_EQ(msg.projector_type, autoware_map_msgs::msg::MapProjectorInfo::LOCAL);
  // Local only sets the projector type and the scale factor.
  EXPECT_EQ(msg.vertical_datum, "");
  EXPECT_FLOAT_EQ(msg.scale_factor, 1.0F);
}

TEST(TestLoadInfoFromYaml, LoadTransverseMercatorDefaultScaleFactor)
{
  const std::string output_path = "/tmp/test_load_info_from_yaml_tm_default.yaml";
  write_yaml(
    "projector_type: TransverseMercator\n"
    "vertical_datum: WGS84\n"
    "map_origin:\n"
    "  latitude: 35.6762\n"
    "  longitude: 139.6503\n"
    "  altitude: 0.0\n",
    output_path);

  const auto msg = autoware::map_projection_loader::load_info_from_yaml(output_path);

  EXPECT_EQ(msg.projector_type, autoware_map_msgs::msg::MapProjectorInfo::TRANSVERSE_MERCATOR);
  EXPECT_EQ(msg.vertical_datum, "WGS84");
  EXPECT_DOUBLE_EQ(msg.map_origin.latitude, 35.6762);
  EXPECT_DOUBLE_EQ(msg.map_origin.longitude, 139.6503);
  EXPECT_DOUBLE_EQ(msg.map_origin.altitude, 0.0);
  // No scale_factor key -> defaults to the UTM scale factor.
  EXPECT_FLOAT_EQ(msg.scale_factor, 0.9996F);
}

TEST(TestLoadInfoFromYaml, LoadTransverseMercatorExplicitScaleFactor)
{
  const std::string output_path = "/tmp/test_load_info_from_yaml_tm_explicit.yaml";
  write_yaml(
    "projector_type: TransverseMercator\n"
    "vertical_datum: WGS84\n"
    "scale_factor: 0.5\n"
    "map_origin:\n"
    "  latitude: 35.6762\n"
    "  longitude: 139.6503\n"
    "  altitude: 0.0\n",
    output_path);

  const auto msg = autoware::map_projection_loader::load_info_from_yaml(output_path);

  EXPECT_EQ(msg.projector_type, autoware_map_msgs::msg::MapProjectorInfo::TRANSVERSE_MERCATOR);
  // An explicit scale_factor key overrides the default.
  EXPECT_FLOAT_EQ(msg.scale_factor, 0.5F);
}

TEST(TestLoadInfoFromYaml, LoadDeprecatedLowercaseLocal)
{
  const std::string output_path = "/tmp/test_load_info_from_yaml_deprecated_local.yaml";
  write_yaml("projector_type: local\n", output_path);

  const auto msg = autoware::map_projection_loader::load_info_from_yaml(output_path);

  // Deprecated lowercase "local" is remapped to the canonical "Local".
  EXPECT_EQ(msg.projector_type, autoware_map_msgs::msg::MapProjectorInfo::LOCAL);
  EXPECT_FLOAT_EQ(msg.scale_factor, 1.0F);
}

TEST(TestLoadInfoFromYaml, InvalidProjectorTypeThrows)
{
  const std::string output_path = "/tmp/test_load_info_from_yaml_invalid_type.yaml";
  write_yaml("projector_type: NotARealProjector\n", output_path);

  EXPECT_THROW(
    autoware::map_projection_loader::load_info_from_yaml(output_path), std::runtime_error);
}

TEST(TestLoadInfoFromYaml, NonPositiveScaleFactorThrows)
{
  const std::string output_path = "/tmp/test_load_info_from_yaml_bad_scale.yaml";
  write_yaml(
    "projector_type: TransverseMercator\n"
    "vertical_datum: WGS84\n"
    "scale_factor: 0.0\n"
    "map_origin:\n"
    "  latitude: 35.6762\n"
    "  longitude: 139.6503\n"
    "  altitude: 0.0\n",
    output_path);

  EXPECT_THROW(
    autoware::map_projection_loader::load_info_from_yaml(output_path), std::runtime_error);
}

TEST(TestLoadMapProjectorInfo, YamlTakesPrecedenceOverLanelet2)
{
  const std::string yaml_path = "/tmp/test_load_map_projector_info_precedence.yaml";
  write_yaml(
    "projector_type: MGRS\n"
    "vertical_datum: WGS84\n"
    "mgrs_grid: 54SUE\n",
    yaml_path);

  // The lanelet2 path also exists but the yaml must win.
  const std::string lanelet2_path = "/tmp/test_load_map_projector_info_precedence.osm";
  write_yaml(
    "<?xml version=\"1.0\"?>\n"
    "<osm version=\"0.6\" generator=\"lanelet2\">\n"
    "  <node id=\"1\" lat=\"\" lon=\"\"/>\n"
    "</osm>",
    lanelet2_path);

  const auto msg =
    autoware::map_projection_loader::load_map_projector_info(yaml_path, lanelet2_path);

  EXPECT_EQ(msg.projector_type, autoware_map_msgs::msg::MapProjectorInfo::MGRS);
  EXPECT_EQ(msg.mgrs_grid, "54SUE");
}

TEST(TestLoadMapProjectorInfo, NoFilesFoundThrows)
{
  EXPECT_THROW(
    autoware::map_projection_loader::load_map_projector_info(
      "/tmp/does_not_exist_projector.yaml", "/tmp/does_not_exist_lanelet2.osm"),
    std::runtime_error);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
