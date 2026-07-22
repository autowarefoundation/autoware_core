// Copyright 2024 The Autoware Contributors
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

#include "../src/lanelet2_map_loader/lanelet2_map_cell_metadata.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>

using autoware::map_loader::Lanelet2FileMetaData;
using autoware::map_loader::utils::compute_cell_metadata;
using autoware::map_loader::utils::load_cell_metadata_from_yaml;

// ─── compute_cell_metadata ────────────────────────────────────────────────────

TEST(ComputeCellMetadataTest, ReturnsCorrectBbox)
{
  lanelet::Point3d p1(1, 1.0, 2.0, 0.0);
  lanelet::Point3d p2(2, 5.0, 3.0, 0.0);
  lanelet::Point3d p3(3, 3.0, 7.0, 0.0);
  lanelet::LineString3d ls(10, {p1, p2, p3});
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(ls);

  const Lanelet2FileMetaData meta = compute_cell_metadata("cell_a", *map);

  EXPECT_EQ(meta.id, "cell_a");
  EXPECT_DOUBLE_EQ(meta.min_x, 1.0);
  EXPECT_DOUBLE_EQ(meta.min_y, 2.0);
  EXPECT_DOUBLE_EQ(meta.max_x, 5.0);
  EXPECT_DOUBLE_EQ(meta.max_y, 7.0);
}

TEST(ComputeCellMetadataTest, SinglePointMap)
{
  lanelet::Point3d p(1, 4.0, 9.0, 0.0);
  lanelet::LineString3d ls(10, {p});
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(ls);

  const Lanelet2FileMetaData meta = compute_cell_metadata("single", *map);

  EXPECT_DOUBLE_EQ(meta.min_x, 4.0);
  EXPECT_DOUBLE_EQ(meta.min_y, 9.0);
  EXPECT_DOUBLE_EQ(meta.max_x, 4.0);
  EXPECT_DOUBLE_EQ(meta.max_y, 9.0);
}

TEST(ComputeCellMetadataTest, EmptyMapYieldsZeros)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  const Lanelet2FileMetaData meta = compute_cell_metadata("empty", *map);

  EXPECT_EQ(meta.id, "empty");
  EXPECT_DOUBLE_EQ(meta.min_x, 0.0);
  EXPECT_DOUBLE_EQ(meta.min_y, 0.0);
  EXPECT_DOUBLE_EQ(meta.max_x, 0.0);
  EXPECT_DOUBLE_EQ(meta.max_y, 0.0);
}

TEST(ComputeCellMetadataTest, NegativeCoordinates)
{
  lanelet::Point3d p1(1, -10.0, -5.0, 0.0);
  lanelet::Point3d p2(2, -1.0, -0.5, 0.0);
  lanelet::LineString3d ls(10, {p1, p2});
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(ls);

  const Lanelet2FileMetaData meta = compute_cell_metadata("neg", *map);

  EXPECT_DOUBLE_EQ(meta.min_x, -10.0);
  EXPECT_DOUBLE_EQ(meta.min_y, -5.0);
  EXPECT_DOUBLE_EQ(meta.max_x, -1.0);
  EXPECT_DOUBLE_EQ(meta.max_y, -0.5);
}

// ─── load_cell_metadata_from_yaml ────────────────────────────────────────────

class LoadCellMetadataFromYamlTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() / "test_cell_metadata_yaml";
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

  std::string write_yaml(const std::string & content, const std::string & name = "metadata.yaml")
  {
    const auto path = tmp_dir_ / name;
    std::ofstream(path) << content;
    return path.string();
  }

  std::filesystem::path tmp_dir_;
};

TEST_F(LoadCellMetadataFromYamlTest, NonExistentFileReturnsNullopt)
{
  const auto result = load_cell_metadata_from_yaml("/nonexistent/metadata.yaml");
  EXPECT_FALSE(result.has_value());
}

TEST_F(LoadCellMetadataFromYamlTest, EmptyPathReturnsNullopt)
{
  const auto result = load_cell_metadata_from_yaml("");
  EXPECT_FALSE(result.has_value());
}

TEST_F(LoadCellMetadataFromYamlTest, ValidYamlReturnsCells)
{
  // Create dummy osm files so absolute paths exist
  std::ofstream((tmp_dir_ / "1.osm")).close();
  std::ofstream((tmp_dir_ / "2.osm")).close();

  const std::string yaml =
    "x_resolution: 100.0\n"
    "y_resolution: 200.0\n"
    "1.osm: [1000.0, 2000.0]\n"
    "2.osm: [1100.0, 2200.0]\n";
  const auto yaml_path = write_yaml(yaml);

  const auto result = load_cell_metadata_from_yaml(yaml_path);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 2u);

  const std::string abs1 = (tmp_dir_ / "1.osm").string();
  const std::string abs2 = (tmp_dir_ / "2.osm").string();

  ASSERT_TRUE(result->count(abs1));
  ASSERT_TRUE(result->count(abs2));

  const auto & m1 = result->at(abs1);
  EXPECT_EQ(m1.id, abs1);
  EXPECT_DOUBLE_EQ(m1.min_x, 1000.0);
  EXPECT_DOUBLE_EQ(m1.min_y, 2000.0);
  EXPECT_DOUBLE_EQ(m1.max_x, 1100.0);  // 1000 + x_resolution
  EXPECT_DOUBLE_EQ(m1.max_y, 2200.0);  // 2000 + y_resolution

  const auto & m2 = result->at(abs2);
  EXPECT_EQ(m2.id, abs2);
  EXPECT_DOUBLE_EQ(m2.min_x, 1100.0);
  EXPECT_DOUBLE_EQ(m2.min_y, 2200.0);
  EXPECT_DOUBLE_EQ(m2.max_x, 1200.0);
  EXPECT_DOUBLE_EQ(m2.max_y, 2400.0);
}

TEST_F(LoadCellMetadataFromYamlTest, RelativeFilenamesResolvedAgainstYamlDir)
{
  // The YAML and the osm file live in sub-directory
  const auto sub = tmp_dir_ / "sub";
  std::filesystem::create_directories(sub);
  std::ofstream((sub / "map.osm")).close();

  const std::string yaml =
    "x_resolution: 50.0\n"
    "y_resolution: 50.0\n"
    "map.osm: [0.0, 0.0]\n";
  const auto yaml_path = (sub / "metadata.yaml").string();
  std::ofstream(yaml_path) << yaml;

  const auto result = load_cell_metadata_from_yaml(yaml_path);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 1u);

  const std::string expected_key = (sub / "map.osm").string();
  EXPECT_TRUE(result->count(expected_key));
}

TEST_F(LoadCellMetadataFromYamlTest, OnlyResolutionKeysYieldsEmptyMap)
{
  const std::string yaml =
    "x_resolution: 100.0\n"
    "y_resolution: 100.0\n";
  const auto yaml_path = write_yaml(yaml);

  const auto result = load_cell_metadata_from_yaml(yaml_path);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
