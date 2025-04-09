// Copyright 2025 The Autoware Contributors
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

#include "autoware/localization_util/covariance_ellipse.hpp"

#include <gtest/gtest.h>

namespace autoware::localization_util
{
class CovarianceEllipseTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    pose_.pose.position.x = 1.0;
    pose_.pose.position.y = 2.0;
    pose_.pose.position.z = 3.0;
    pose_.pose.orientation.x = 0.0;
    pose_.pose.orientation.y = 0.0;
    pose_.pose.orientation.z = 0.0;
    pose_.pose.orientation.w = 1.0;
    pose_.covariance = {
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // NOLINT
      0.0, 1.0, 0.0, 0.0, 0.0, 0.0,  // NOLINT
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0,  // NOLINT
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  // NOLINT
      0.0, 0.0, 0.0, 0.0, 1.0, 0.0,  // NOLINT
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0   // NOLINT
    };

    header_.stamp.sec = 0;
    header_.stamp.nanosec = 0;
    header_.frame_id = "map";
  }

  geometry_msgs::msg::PoseWithCovariance pose_;
  std_msgs::msg::Header header_;
};

TEST_F(CovarianceEllipseTest, calculate_xyz_ellipse_basic)
{
  // test case 1: basic test
  Ellipse ellipse = calculate_xy_ellipse(pose_, 1.0);
  EXPECT_NEAR(ellipse.long_radius, 1.0, 1e-6);
  EXPECT_NEAR(ellipse.short_radius, 1.0, 1e-6);
  EXPECT_NEAR(ellipse.yaw, M_PI_2, 1e-6);
  EXPECT_NEAR(ellipse.size_lateral_direction, 1.0, 1e-6);
}

TEST_F(CovarianceEllipseTest, calculate_xyz_ellipse_change_position)
{
  // test case 2: change the position may not influence the result
  pose_.pose.position.x = 1.0;
  pose_.pose.position.y = 1.0;
  pose_.pose.position.z = 1.0;

  Ellipse ellipse = calculate_xy_ellipse(pose_, 1.0);
  EXPECT_NEAR(ellipse.long_radius, 1.0, 1e-6);
  EXPECT_NEAR(ellipse.short_radius, 1.0, 1e-6);
  EXPECT_NEAR(ellipse.yaw, M_PI_2, 1e-6);
  EXPECT_NEAR(ellipse.size_lateral_direction, 1.0, 1e-6);
}

TEST_F(CovarianceEllipseTest, calculate_xy_ellipse_change_covariance)
{
  // test case 3: change the covariance may influence the result
  pose_.covariance[0 * 6 + 0] = 0.25;
  pose_.covariance[1 * 6 + 1] = 0.25;
  Ellipse ellipse = calculate_xy_ellipse(pose_, 1.0);
  EXPECT_NEAR(ellipse.long_radius, 0.5, 1e-6);
  EXPECT_NEAR(ellipse.short_radius, 0.5, 1e-6);
  EXPECT_NEAR(ellipse.yaw, M_PI_2, 1e-6);
  EXPECT_NEAR(ellipse.size_lateral_direction, 0.5, 1e-20);
}

TEST_F(CovarianceEllipseTest, calculate_xy_ellipse_change_orientation)
{
  // test case 4: change the orientation may influence the result
  pose_.pose.orientation.x = 0.0;
  pose_.pose.orientation.y = 0.0;
  pose_.pose.orientation.z = 0.850511;
  pose_.pose.orientation.w = 0.525957;

  Ellipse ellipse = calculate_xy_ellipse(pose_, 1.0);
  EXPECT_NEAR(ellipse.long_radius, 1.0, 1e-6);
  EXPECT_NEAR(ellipse.short_radius, 1.0, 1e-6);
  EXPECT_NEAR(ellipse.yaw, M_PI_2, 1e-6);
}

TEST_F(CovarianceEllipseTest, calculate_xy_ellipse)
{
  // test case 5: change the both the covariance and orientation may influence the result
  pose_.covariance[0 * 6 + 0] = 0.49;
  pose_.covariance[0 * 6 + 1] = 0.16;
  pose_.covariance[1 * 6 + 0] = 0.16;
  pose_.covariance[1 * 6 + 1] = 0.25;
  pose_.pose.orientation.x = 0.0;
  pose_.pose.orientation.y = 0.0;
  pose_.pose.orientation.z = 0.850511;
  pose_.pose.orientation.w = 0.525957;

  Ellipse ellipse = calculate_xy_ellipse(pose_, 1.0);
  EXPECT_NE(ellipse.long_radius, 0.7);
  EXPECT_NE(ellipse.short_radius, 0.5);
}

TEST_F(CovarianceEllipseTest, create_ellipse_marker)
{
  Ellipse ellipse = calculate_xy_ellipse(pose_, 1.0);
  EXPECT_NO_THROW(create_ellipse_marker(ellipse, header_, pose_));
}
}  // namespace autoware::localization_util
