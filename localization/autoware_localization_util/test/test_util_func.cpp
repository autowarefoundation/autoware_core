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

#include "autoware/localization_util/matrix_type.hpp"
#include "autoware/localization_util/util_func.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

namespace autoware::localization_util
{
TEST(UtilFuncTest, ExchangeColorCRC)
{
  {
    auto color = exchange_color_crc(0.1);
    EXPECT_FLOAT_EQ(color.r, 0.0f);
    EXPECT_GT(color.g, 0.0f);
    EXPECT_FLOAT_EQ(color.b, 1.0f);
    EXPECT_FLOAT_EQ(color.a, 0.999f);
  }

  {
    auto color = exchange_color_crc(0.4);
    EXPECT_FLOAT_EQ(color.r, 0.0f);
    EXPECT_FLOAT_EQ(color.g, 1.0f);
    EXPECT_GT(color.b, 0.0f);
    EXPECT_FLOAT_EQ(color.a, 0.999f);
  }

  {
    auto color = exchange_color_crc(0.6);
    EXPECT_GT(color.r, 0.0f);
    EXPECT_FLOAT_EQ(color.g, 1.0f);
    EXPECT_FLOAT_EQ(color.b, 0.0f);
    EXPECT_FLOAT_EQ(color.a, 0.999f);
  }

  {
    auto color = exchange_color_crc(0.8);
    EXPECT_FLOAT_EQ(color.r, 1.0f);
    EXPECT_GT(color.g, 0.0f);
    EXPECT_FLOAT_EQ(color.b, 0.0f);
    EXPECT_FLOAT_EQ(color.a, 0.999f);
  }

  EXPECT_NO_THROW(exchange_color_crc(-1.0));
  EXPECT_NO_THROW(exchange_color_crc(2.0));
}

TEST(UtilFuncTest, CalcDiffForRadian)
{
  EXPECT_NEAR(calc_diff_for_radian(1.0, 0.5), 0.5, 1e-6);
  EXPECT_NEAR(calc_diff_for_radian(6.0, 0.0), -0.28318530718, 1e-6);
  EXPECT_NEAR(calc_diff_for_radian(0.0, 6.0), 0.28318530718, 1e-6);
}

TEST(UtilFuncTest, MakeEigenCovariance)
{
  std::array<double, 36> covariance;
  for (int i = 0; i < 36; ++i) {
    if (i % 7 == 0) {
      covariance[i] = i + 1.0;
    } else {
      covariance[i] = 0.1;
    }
  }
  EXPECT_EQ(covariance.size(), 36);
}

TEST(UtilFuncTest, GetRPY)
{
  geometry_msgs::msg::Pose pose;
  tf2::Quaternion q;
  q.setRPY(0.1, 0.2, 0.3);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  auto rpy = get_rpy(pose);
  EXPECT_NEAR(rpy.x, 0.1, 1e-6);
  EXPECT_NEAR(rpy.y, 0.2, 1e-6);
  EXPECT_NEAR(rpy.z, 0.3, 1e-6);

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.pose = pose;
  rpy = get_rpy(pose_stamped);
  EXPECT_NEAR(rpy.x, 0.1, 1e-6);
  EXPECT_NEAR(rpy.y, 0.2, 1e-6);
  EXPECT_NEAR(rpy.z, 0.3, 1e-6);

  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_cov_stamped;
  pose_with_cov_stamped.pose.pose = pose;
  rpy = get_rpy(pose_with_cov_stamped);
  EXPECT_NEAR(rpy.x, 0.1, 1e-6);
  EXPECT_NEAR(rpy.y, 0.2, 1e-6);
  EXPECT_NEAR(rpy.z, 0.3, 1e-6);
}

TEST(UtilFuncTest, RPYToQuaternion)
{
  const double r_rad = 0.1;
  const double p_rad = 0.2;
  const double y_rad = 0.3;
  auto quat = rpy_rad_to_quaternion(r_rad, p_rad, y_rad);

  tf2::Quaternion tf_quat;
  tf_quat.setX(quat.x);
  tf_quat.setY(quat.y);
  tf_quat.setZ(quat.z);
  tf_quat.setW(quat.w);

  double r, p, y;
  tf2::Matrix3x3(tf_quat).getRPY(r, p, y);

  EXPECT_NEAR(r, r_rad, 1e-6);
  EXPECT_NEAR(p, p_rad, 1e-6);
  EXPECT_NEAR(y, y_rad, 1e-6);

  const double r_deg = 10.0;
  const double p_deg = 20.0;
  const double y_deg = 30.0;
  quat = rpy_deg_to_quaternion(r_deg, p_deg, y_deg);

  tf_quat.setX(quat.x);
  tf_quat.setY(quat.y);
  tf_quat.setZ(quat.z);
  tf_quat.setW(quat.w);

  tf2::Matrix3x3(tf_quat).getRPY(r, p, y);

  EXPECT_NEAR(r, r_deg * M_PI / 180.0, 1e-6);
  EXPECT_NEAR(p, p_deg * M_PI / 180.0, 1e-6);
  EXPECT_NEAR(y, y_deg * M_PI / 180.0, 1e-6);
}

TEST(UtilFuncTest, CalcTwist)
{
  geometry_msgs::msg::PoseStamped pose_a, pose_b;

  pose_a.header.stamp.sec = 10;
  pose_a.header.stamp.nanosec = 0;
  pose_b.header.stamp.sec = 11;
  pose_b.header.stamp.nanosec = 0;

  pose_a.pose.position.x = 1.0;
  pose_a.pose.position.y = 2.0;
  pose_a.pose.position.z = 3.0;

  pose_b.pose.position.x = 2.0;
  pose_b.pose.position.y = 3.0;
  pose_b.pose.position.z = 3.5;

  tf2::Quaternion q_a, q_b;
  q_a.setRPY(0.1, 0.2, 0.3);
  q_b.setRPY(0.2, 0.3, 0.5);

  pose_a.pose.orientation.x = q_a.x();
  pose_a.pose.orientation.y = q_a.y();
  pose_a.pose.orientation.z = q_a.z();
  pose_a.pose.orientation.w = q_a.w();

  pose_b.pose.orientation.x = q_b.x();
  pose_b.pose.orientation.y = q_b.y();
  pose_b.pose.orientation.z = q_b.z();
  pose_b.pose.orientation.w = q_b.w();

  auto twist = calc_twist(pose_a, pose_b);

  EXPECT_NEAR(twist.linear.x, 1.0, 1e-6);
  EXPECT_NEAR(twist.linear.y, 1.0, 1e-6);
  EXPECT_NEAR(twist.linear.z, 0.5, 1e-6);

  EXPECT_NEAR(twist.angular.x, 0.1, 1e-6);
  EXPECT_NEAR(twist.angular.y, 0.1, 1e-6);
  EXPECT_NEAR(twist.angular.z, 0.2, 1e-6);

  pose_b.header.stamp = pose_a.header.stamp;
  twist = calc_twist(pose_a, pose_b);
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(twist.linear.z, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.y, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.0);
}

TEST(UtilFuncTest, InterpolatePose)
{
  geometry_msgs::msg::PoseStamped pose_a, pose_b;

  rclcpp::Time time_a(10, 0, RCL_ROS_TIME);
  rclcpp::Time time_b(12, 0, RCL_ROS_TIME);
  pose_a.header.stamp = time_a;
  pose_b.header.stamp = time_b;

  pose_a.pose.position.x = 1.0;
  pose_a.pose.position.y = 2.0;
  pose_a.pose.position.z = 3.0;

  pose_b.pose.position.x = 3.0;
  pose_b.pose.position.y = 4.0;
  pose_b.pose.position.z = 5.0;

  tf2::Quaternion q_a, q_b;
  q_a.setRPY(0.0, 0.0, 0.0);
  q_b.setRPY(0.2, 0.2, 0.2);

  pose_a.pose.orientation.x = q_a.x();
  pose_a.pose.orientation.y = q_a.y();
  pose_a.pose.orientation.z = q_a.z();
  pose_a.pose.orientation.w = q_a.w();

  pose_b.pose.orientation.x = q_b.x();
  pose_b.pose.orientation.y = q_b.y();
  pose_b.pose.orientation.z = q_b.z();
  pose_b.pose.orientation.w = q_b.w();

  rclcpp::Time interp_time(11, 0, RCL_ROS_TIME);
  auto interp_pose = interpolate_pose(pose_a, pose_b, interp_time);

  EXPECT_NEAR(interp_pose.pose.position.x, 2.0, 1e-6);
  EXPECT_NEAR(interp_pose.pose.position.y, 3.0, 1e-6);
  EXPECT_NEAR(interp_pose.pose.position.z, 4.0, 1e-6);

  auto rpy = get_rpy(interp_pose);
  EXPECT_NEAR(rpy.x, 0.1, 1e-6);
  EXPECT_NEAR(rpy.y, 0.1, 1e-6);
  EXPECT_NEAR(rpy.z, 0.1, 1e-6);

  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_a, pose_cov_b;
  pose_cov_a.header = pose_a.header;
  pose_cov_a.pose.pose = pose_a.pose;

  pose_cov_b.header = pose_b.header;
  pose_cov_b.pose.pose = pose_b.pose;

  auto interp_pose2 = interpolate_pose(pose_cov_a, pose_cov_b, interp_time);

  EXPECT_NEAR(interp_pose2.pose.position.x, interp_pose.pose.position.x, 1e-6);
  EXPECT_NEAR(interp_pose2.pose.position.y, interp_pose.pose.position.y, 1e-6);
  EXPECT_NEAR(interp_pose2.pose.position.z, interp_pose.pose.position.z, 1e-6);

  rclcpp::Time zero_time(0, 0, RCL_ROS_TIME);
  pose_a.header.stamp = zero_time;
  auto zero_result = interpolate_pose(pose_a, pose_b, interp_time);
  EXPECT_DOUBLE_EQ(zero_result.header.stamp.sec, 0);
  EXPECT_DOUBLE_EQ(zero_result.header.stamp.nanosec, 0);
}

TEST(UtilFuncTest, PoseConversion)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 3.0;

  tf2::Quaternion q;
  q.setRPY(0.1, 0.2, 0.3);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  auto affine = pose_to_affine3d(pose);
  EXPECT_NEAR(affine.translation().x(), 1.0, 1e-6);
  EXPECT_NEAR(affine.translation().y(), 2.0, 1e-6);
  EXPECT_NEAR(affine.translation().z(), 3.0, 1e-6);

  auto matrix = pose_to_matrix4f(pose);
  EXPECT_NEAR(matrix(0, 3), 1.0, 1e-6);
  EXPECT_NEAR(matrix(1, 3), 2.0, 1e-6);
  EXPECT_NEAR(matrix(2, 3), 3.0, 1e-6);

  auto vec = point_to_vector3d(pose.position);
  EXPECT_NEAR(vec.x(), 1.0, 1e-6);
  EXPECT_NEAR(vec.y(), 2.0, 1e-6);
  EXPECT_NEAR(vec.z(), 3.0, 1e-6);

  auto converted_pose = matrix4f_to_pose(matrix);
  EXPECT_NEAR(converted_pose.position.x, pose.position.x, 1e-6);
  EXPECT_NEAR(converted_pose.position.y, pose.position.y, 1e-6);
  EXPECT_NEAR(converted_pose.position.z, pose.position.z, 1e-6);
  EXPECT_NEAR(converted_pose.orientation.x, pose.orientation.x, 1e-6);
  EXPECT_NEAR(converted_pose.orientation.y, pose.orientation.y, 1e-6);
  EXPECT_NEAR(converted_pose.orientation.z, pose.orientation.z, 1e-6);
  EXPECT_NEAR(converted_pose.orientation.w, pose.orientation.w, 1e-6);
}

TEST(UtilFuncTest, NormCalculation)
{
  geometry_msgs::msg::Point p1, p2;
  p1.x = 1.0;
  p1.y = 2.0;
  p1.z = 3.0;

  p2.x = 4.0;
  p2.y = 6.0;
  p2.z = 8.0;

  double expected =
    std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));

  EXPECT_DOUBLE_EQ(norm(p1, p2), expected);
}

TEST(UtilFuncTest, OutputPoseToCovLog)
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");

  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_cov;

  pose_with_cov.pose.pose.position.x = 1.0;
  pose_with_cov.pose.pose.position.y = 2.0;
  pose_with_cov.pose.pose.position.z = 3.0;

  tf2::Quaternion q;
  q.setRPY(0.1, 0.2, 0.3);
  pose_with_cov.pose.pose.orientation.x = q.x();
  pose_with_cov.pose.pose.orientation.y = q.y();
  pose_with_cov.pose.pose.orientation.z = q.z();
  pose_with_cov.pose.pose.orientation.w = q.w();

  for (int i = 0; i < 36; ++i) {
    if (i % 7 == 0) {
      pose_with_cov.pose.covariance[i] = 1.0;
    } else {
      pose_with_cov.pose.covariance[i] = 0.1;
    }
  }

  EXPECT_NO_THROW(output_pose_with_cov_to_log(logger, "TEST", pose_with_cov));

  EXPECT_EQ(pose_with_cov.pose.covariance.size(), 36);
  EXPECT_DOUBLE_EQ(pose_with_cov.pose.covariance[0], 1.0);
  EXPECT_DOUBLE_EQ(pose_with_cov.pose.covariance[7], 1.0);
  EXPECT_DOUBLE_EQ(pose_with_cov.pose.covariance[14], 1.0);
  EXPECT_DOUBLE_EQ(pose_with_cov.pose.covariance[21], 1.0);
  EXPECT_DOUBLE_EQ(pose_with_cov.pose.covariance[28], 1.0);
  EXPECT_DOUBLE_EQ(pose_with_cov.pose.covariance[35], 1.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
}  // namespace autoware::localization_util
