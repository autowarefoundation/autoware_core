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

#include "../src/lanelet2_map_loader/lanelet2_map_loader_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using autoware::map_loader::utils::get_lanelet2_paths;
using autoware::map_loader::utils::is_osm_file;
using autoware::map_loader::utils::merge_lanelet2_maps;

namespace
{
std::string test_map_path()
{
  return ament_index_cpp::get_package_share_directory("autoware_map_loader") +
         "/test/data/test_map.osm";
}
}  // namespace

// ─── is_osm_file ─────────────────────────────────────────────────────────────

TEST(IsOsmFileTest, ReturnsTrueForOsmExtension)
{
  EXPECT_TRUE(is_osm_file(test_map_path()));
}

TEST(IsOsmFileTest, ReturnsFalseForNonOsmExtension)
{
  EXPECT_FALSE(is_osm_file("map.txt"));
  EXPECT_FALSE(is_osm_file("map.pcd"));
  EXPECT_FALSE(is_osm_file("map.yaml"));
}

TEST(IsOsmFileTest, ReturnsFalseForNoExtension)
{
  EXPECT_FALSE(is_osm_file("map"));
}

TEST(IsOsmFileTest, ReturnsFalseForDirectory)
{
  // Use the system temp directory which is guaranteed to exist
  EXPECT_FALSE(is_osm_file("/tmp"));
}

// ─── get_lanelet2_paths ───────────────────────────────────────────────────────

class GetLanelet2PathsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() / "test_get_lanelet2_paths";
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

  // Creates an empty file at tmp_dir_/<name> and returns its path string
  std::string make_file(const std::string & name)
  {
    auto path = tmp_dir_ / name;
    std::ofstream(path).close();
    return path.string();
  }

  std::filesystem::path tmp_dir_;
};

TEST_F(GetLanelet2PathsTest, SingleOsmFile)
{
  const auto file = make_file("map.osm");
  const auto result = get_lanelet2_paths(file);
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0], file);
}

TEST_F(GetLanelet2PathsTest, NonExistentPath)
{
  const auto result = get_lanelet2_paths("/nonexistent/path/map.osm");
  EXPECT_TRUE(result.empty());
}

TEST_F(GetLanelet2PathsTest, NonOsmFile)
{
  const auto file = make_file("map.txt");
  const auto result = get_lanelet2_paths(file);
  EXPECT_TRUE(result.empty());
}

TEST_F(GetLanelet2PathsTest, DirectoryWithOsmFiles)
{
  make_file("b.osm");
  make_file("a.osm");
  make_file("c.osm");
  make_file("other.txt");

  const auto result = get_lanelet2_paths(tmp_dir_.string());
  ASSERT_EQ(result.size(), 3u);
  // Results must be sorted
  EXPECT_EQ(std::filesystem::path(result[0]).filename(), "a.osm");
  EXPECT_EQ(std::filesystem::path(result[1]).filename(), "b.osm");
  EXPECT_EQ(std::filesystem::path(result[2]).filename(), "c.osm");
}

TEST_F(GetLanelet2PathsTest, EmptyDirectory)
{
  const auto result = get_lanelet2_paths(tmp_dir_.string());
  EXPECT_TRUE(result.empty());
}

TEST_F(GetLanelet2PathsTest, DirectoryWithNoOsmFiles)
{
  make_file("map.pcd");
  make_file("metadata.yaml");

  const auto result = get_lanelet2_paths(tmp_dir_.string());
  EXPECT_TRUE(result.empty());
}

// ─── merge_lanelet2_maps ─────────────────────────────────────────────────────

TEST(MergeLanelet2MapsTest, MergeEmptyMaps)
{
  auto target = std::make_shared<lanelet::LaneletMap>();
  auto source = std::make_shared<lanelet::LaneletMap>();

  merge_lanelet2_maps(*target, *source);

  EXPECT_EQ(target->laneletLayer.size(), 0u);
  EXPECT_EQ(target->lineStringLayer.size(), 0u);
  EXPECT_EQ(target->pointLayer.size(), 0u);
}

TEST(MergeLanelet2MapsTest, MergeDistinctLanelets)
{
  // Map A: one lanelet with 4 unique points
  lanelet::Point3d p1(1, 0.0, 0.0, 0.0);
  lanelet::Point3d p2(2, 1.0, 0.0, 0.0);
  lanelet::Point3d p3(3, 0.0, 1.0, 0.0);
  lanelet::Point3d p4(4, 1.0, 1.0, 0.0);
  lanelet::LineString3d ls1(10, {p1, p2});
  lanelet::LineString3d ls2(11, {p3, p4});
  lanelet::Lanelet ll_a(100, ls1, ls2);
  auto map_a = std::make_shared<lanelet::LaneletMap>();
  map_a->add(ll_a);

  // Map B: one lanelet with 4 different points (no shared IDs)
  lanelet::Point3d p5(5, 2.0, 0.0, 0.0);
  lanelet::Point3d p6(6, 3.0, 0.0, 0.0);
  lanelet::Point3d p7(7, 2.0, 1.0, 0.0);
  lanelet::Point3d p8(8, 3.0, 1.0, 0.0);
  lanelet::LineString3d ls3(12, {p5, p6});
  lanelet::LineString3d ls4(13, {p7, p8});
  lanelet::Lanelet ll_b(101, ls3, ls4);
  auto map_b = std::make_shared<lanelet::LaneletMap>();
  map_b->add(ll_b);

  auto merged = std::make_shared<lanelet::LaneletMap>();
  merge_lanelet2_maps(*merged, *map_a);
  merge_lanelet2_maps(*merged, *map_b);

  EXPECT_EQ(merged->laneletLayer.size(), 2u);
  EXPECT_EQ(merged->lineStringLayer.size(), 4u);
  EXPECT_EQ(merged->pointLayer.size(), 8u);
}

TEST(MergeLanelet2MapsTest, MergeLaneletsWithSharedPoints)
{
  // P2 and P4 are shared between map_a and map_b (same C++ objects, same IDs)
  lanelet::Point3d p1(1, 0.0, 0.0, 0.0);
  lanelet::Point3d p2(2, 1.0, 0.0, 0.0);  // shared
  lanelet::Point3d p3(3, 0.0, 1.0, 0.0);
  lanelet::Point3d p4(4, 1.0, 1.0, 0.0);  // shared

  auto map_a = std::make_shared<lanelet::LaneletMap>();
  {
    lanelet::LineString3d ls1(10, {p1, p2});
    lanelet::LineString3d ls2(11, {p3, p4});
    lanelet::Lanelet ll(100, ls1, ls2);
    map_a->add(ll);
  }

  // Map B reuses p2 and p4 (shared boundary points) with two new endpoints
  lanelet::Point3d p5(5, 2.0, 0.0, 0.0);
  lanelet::Point3d p6(6, 2.0, 1.0, 0.0);

  auto map_b = std::make_shared<lanelet::LaneletMap>();
  {
    lanelet::LineString3d ls3(12, {p2, p5});
    lanelet::LineString3d ls4(13, {p4, p6});
    lanelet::Lanelet ll(101, ls3, ls4);
    map_b->add(ll);
  }

  auto merged = std::make_shared<lanelet::LaneletMap>();
  merge_lanelet2_maps(*merged, *map_a);
  merge_lanelet2_maps(*merged, *map_b);

  EXPECT_EQ(merged->laneletLayer.size(), 2u);
  // p1, p2, p3, p4, p5, p6 — shared points must not be duplicated
  EXPECT_EQ(merged->pointLayer.size(), 6u);
}

TEST(MergeLanelet2MapsTest, MergeArea)
{
  lanelet::Point3d p1(1, 0.0, 0.0, 0.0);
  lanelet::Point3d p2(2, 1.0, 0.0, 0.0);
  lanelet::Point3d p3(3, 0.5, 1.0, 0.0);
  lanelet::LineString3d boundary(10, {p1, p2, p3, p1});
  lanelet::Area area(100, {boundary});

  auto source = std::make_shared<lanelet::LaneletMap>();
  source->add(area);

  auto target = std::make_shared<lanelet::LaneletMap>();
  merge_lanelet2_maps(*target, *source);

  EXPECT_EQ(target->areaLayer.size(), 1u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
