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

#include "test_fixture.hpp"
#include "test_util.hpp"

#include <autoware/ndt_scan_matcher/ndt_scan_matcher_core.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

namespace autoware::ndt_scan_matcher
{
class NDTScanMatcherTestHelper
{
public:
  explicit NDTScanMatcherTestHelper(const std::shared_ptr<NDTScanMatcher> & node) : node_(node) {}

  std::optional<NDTScanMatcher::PreprocessResult> preprocess(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) const
  {
    return node_->preprocess_sensor_points(msg, std::chrono::system_clock::now());
  }

private:
  std::shared_ptr<NDTScanMatcher> node_;
};
}  // namespace autoware::ndt_scan_matcher

class PreprocessSensorPointsTest : public TestNDTScanMatcher
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    TestNDTScanMatcher::SetUp();
    helper_ = std::make_unique<autoware::ndt_scan_matcher::NDTScanMatcherTestHelper>(node_);
  }

  std::unique_ptr<autoware::ndt_scan_matcher::NDTScanMatcherTestHelper> helper_;
};

TEST_F(PreprocessSensorPointsTest, ReturnsNulloptWhenPointCloudEmpty)
{
  auto empty_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  empty_cloud->header.frame_id = "sensor_frame";
  empty_cloud->header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME).to_builtin_time();
  empty_cloud->width = 0;
  empty_cloud->height = 0;

  const auto result = helper_->preprocess(empty_cloud);

  EXPECT_FALSE(result);  // Early exit on empty cloud
}

TEST_F(PreprocessSensorPointsTest, SucceedsWithValidPointCloud)
{
  auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(make_default_sensor_pcd());

  const auto result = helper_->preprocess(cloud_msg);

  ASSERT_TRUE(result);
  EXPECT_EQ(result->sensor_ros_time, cloud_msg->header.stamp);
  ASSERT_NE(result->sensor_points_in_baselink_frame, nullptr);
  EXPECT_GT(result->sensor_points_in_baselink_frame->size(), 0U);
}
