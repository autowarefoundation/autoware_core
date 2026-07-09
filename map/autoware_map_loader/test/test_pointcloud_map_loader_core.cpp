// Copyright 2026 The Autoware Contributors
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

#include "../src/pointcloud_map_loader/pointcloud_map_loader.hpp"

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace autoware::map_loader
{
namespace
{
std::string create_test_dir(const std::string & dir_name)
{
  const std::filesystem::path dir = std::filesystem::temp_directory_path() / dir_name;
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir.string();
}

std::string create_pcd(const std::string & path, const std::vector<pcl::PointXYZ> & points)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.points.assign(points.begin(), points.end());
  pcl::io::savePCDFileASCII(path, cloud);
  return path;
}

TEST(PointcloudMapLoaderCore, ResolvePcdPathsCollectsFilesAndSkipsInvalidPath)
{
  const auto test_dir = create_test_dir("test_pointcloud_map_loader_core_resolve");
  const std::string direct_pcd = create_pcd(
    test_dir + "/direct.pcd", {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(1.0F, 1.0F, 1.0F)});
  const std::string dir_pcd = create_pcd(
    test_dir + "/from_dir.PCD",
    {pcl::PointXYZ(-1.0F, 2.0F, 0.5F), pcl::PointXYZ(3.0F, 0.0F, -2.0F)});
  const std::string non_pcd = test_dir + "/ignore.txt";
  {
    std::ofstream ofs(non_pcd);
    ofs << "not a pcd";
  }

  std::vector<std::string> logged_errors;
  const auto error_log = [&logged_errors](const std::string & msg) {
    logged_errors.push_back(msg);
  };

  const std::vector<std::string> inputs = {
    direct_pcd,
    test_dir,
    test_dir + "/does_not_exist",
  };

  auto resolved = resolve_pcd_paths(inputs, error_log);
  std::sort(resolved.begin(), resolved.end());

  EXPECT_EQ(resolved, (std::vector<std::string>{direct_pcd, direct_pcd, dir_pcd}));

  std::filesystem::remove_all(test_dir);
}

TEST(PointcloudMapLoaderCore, BuildMetadataFromSinglePcdWithoutMetadataFile)
{
  const auto test_dir = create_test_dir("test_pointcloud_map_loader_core_metadata_single");
  const std::string pcd_path = create_pcd(
    test_dir + "/single.pcd", {
                                pcl::PointXYZ(-2.0F, 3.0F, -1.0F),
                                pcl::PointXYZ(5.0F, -4.0F, 7.0F),
                                pcl::PointXYZ(1.0F, 2.0F, 0.0F),
                              });

  const auto metadata = build_pcd_metadata_dict(test_dir + "/missing_metadata.yaml", {pcd_path});

  ASSERT_EQ(metadata.size(), 1U);
  const auto it = metadata.find(pcd_path);
  ASSERT_NE(it, metadata.end());
  EXPECT_FLOAT_EQ(it->second.min.x, -2.0F);
  EXPECT_FLOAT_EQ(it->second.min.y, -4.0F);
  EXPECT_FLOAT_EQ(it->second.min.z, -1.0F);
  EXPECT_FLOAT_EQ(it->second.max.x, 5.0F);
  EXPECT_FLOAT_EQ(it->second.max.y, 3.0F);
  EXPECT_FLOAT_EQ(it->second.max.z, 7.0F);

  std::filesystem::remove_all(test_dir);
}

TEST(PointcloudMapLoaderCore, DownsamplePointcloudPreservesHeaderAndReducesPointCount)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_input;
  pcl_input.width = 4;
  pcl_input.height = 1;
  pcl_input.is_dense = true;
  pcl_input.points = {
    pcl::PointXYZ(0.00F, 0.00F, 0.00F),
    pcl::PointXYZ(0.01F, 0.01F, 0.01F),
    pcl::PointXYZ(0.02F, 0.02F, 0.02F),
    pcl::PointXYZ(1.00F, 1.00F, 1.00F),
  };

  sensor_msgs::msg::PointCloud2 msg_input;
  pcl::toROSMsg(pcl_input, msg_input);
  msg_input.header.frame_id = "input_frame";

  const auto msg_output = downsample_pointcloud(msg_input, 0.5F);

  pcl::PointCloud<pcl::PointXYZ> pcl_output;
  pcl::fromROSMsg(msg_output, pcl_output);
  EXPECT_EQ(msg_output.header.frame_id, "input_frame");
  EXPECT_EQ(pcl_output.size(), 2U);
}

TEST(PointcloudMapLoaderCore, LoadPointcloudMapMergesFilesSetsMapFrameAndReportsProgress)
{
  const auto test_dir = create_test_dir("test_pointcloud_map_loader_core_load_merge");
  const std::string pcd_a = create_pcd(
    test_dir + "/a.pcd", {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(1.0F, 0.0F, 0.0F)});
  const std::string pcd_b = create_pcd(
    test_dir + "/b.pcd", {pcl::PointXYZ(0.0F, 1.0F, 0.0F), pcl::PointXYZ(1.0F, 1.0F, 0.0F)});

  size_t progress_count = 0;
  size_t last_processed = 0;
  size_t last_total = 0;
  std::string last_path;
  std::vector<std::string> errors;

  const auto progress = [&progress_count, &last_processed, &last_total, &last_path](
                          size_t processed, size_t total, const std::string & path) {
    ++progress_count;
    last_processed = processed;
    last_total = total;
    last_path = path;
  };
  const auto error_log = [&errors](const std::string & msg) { errors.push_back(msg); };

  const auto merged = load_pointcloud_map({pcd_a, pcd_b}, boost::none, progress, error_log);

  pcl::PointCloud<pcl::PointXYZ> merged_pcl;
  pcl::fromROSMsg(merged, merged_pcl);

  EXPECT_EQ(merged.header.frame_id, "map");
  EXPECT_EQ(merged_pcl.size(), 4U);
  EXPECT_TRUE(errors.empty());
  EXPECT_EQ(progress_count, 1U);
  EXPECT_EQ(last_processed, 1U);
  EXPECT_EQ(last_total, 2U);
  EXPECT_EQ(last_path, pcd_a);

  std::filesystem::remove_all(test_dir);
}

TEST(PointcloudMapLoaderCore, LoadPointcloudMapCallsErrorCallbackOnMissingFile)
{
  std::vector<std::string> errors;
  const auto error_log = [&errors](const std::string & msg) { errors.push_back(msg); };

  const auto merged = load_pointcloud_map(
    {"/tmp/pointcloud_map_loader_missing_input_file.pcd"}, boost::none, {}, error_log);

  EXPECT_EQ(merged.header.frame_id, "map");
}

TEST(PointcloudMapLoaderCore, ThrowsWhenMetadataFileMissingForMultiplePcds)
{
  const std::vector<std::string> pcd_paths = {"/tmp/a.pcd", "/tmp/b.pcd"};

  EXPECT_THROW(
    (void)build_pcd_metadata_dict("/tmp/metadata_that_does_not_exist.yaml", pcd_paths),
    std::runtime_error);
}
}  // namespace
}  // namespace autoware::map_loader

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
