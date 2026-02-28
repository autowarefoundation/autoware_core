// Copyright 2026 Autoware Foundation
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

#include "visualize_point_score.hpp"

#include <autoware/localization_util/util_func.hpp>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace
{
using autoware::localization_util::exchange_color_crc;
using autoware::ndt_scan_matcher::detail::colorize_point_scores;

TEST(VisualizePointScoreTest, ConvertsIntensityToColor)
{
  pcl::PointCloud<pcl::PointXYZI> scores;
  scores.push_back(pcl::PointXYZI{1.0F, 2.0F, 3.0F, 1.0F});
  scores.push_back(pcl::PointXYZI{-1.0F, -2.0F, -3.0F, 2.0F});
  scores.push_back(pcl::PointXYZI{0.5F, 0.0F, -0.5F, 3.0F});

  constexpr float lower_nvs = 1.0F;
  constexpr float upper_nvs = 3.0F;

  const auto colored = colorize_point_scores(scores, lower_nvs, upper_nvs);

  ASSERT_EQ(colored->size(), scores.size());
  const float range = upper_nvs - lower_nvs;
  for (std::size_t i = 0; i < scores.size(); ++i) {
    EXPECT_FLOAT_EQ(colored->points[i].x, scores.points[i].x);
    EXPECT_FLOAT_EQ(colored->points[i].y, scores.points[i].y);
    EXPECT_FLOAT_EQ(colored->points[i].z, scores.points[i].z);

    const auto expected_color =
      exchange_color_crc((scores.points[i].intensity - lower_nvs) / range);
    EXPECT_EQ(colored->points[i].r, static_cast<uint8_t>(expected_color.r * 255));
    EXPECT_EQ(colored->points[i].g, static_cast<uint8_t>(expected_color.g * 255));
    EXPECT_EQ(colored->points[i].b, static_cast<uint8_t>(expected_color.b * 255));
  }
}

TEST(VisualizePointScoreTest, HandlesEmptyInput)
{
  pcl::PointCloud<pcl::PointXYZI> empty_scores;

  const auto colored = colorize_point_scores(empty_scores, 0.0F, 1.0F);

  EXPECT_TRUE(colored->empty());
}
}  // namespace
