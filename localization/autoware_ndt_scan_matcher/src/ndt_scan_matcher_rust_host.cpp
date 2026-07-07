// Copyright 2015-2019 Autoware Foundation
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

#include "ndt_scan_matcher_helper.hpp"

#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/ndt_scan_matcher_core.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{
using autoware::localization_util::exchange_color_crc;
using autoware::localization_util::pose_to_matrix4f;

autoware_internal_debug_msgs::msg::Float32Stamped make_float32_stamped(
  const builtin_interfaces::msg::Time & stamp, float data);
autoware_internal_debug_msgs::msg::Int32Stamped make_int32_stamped(
  const builtin_interfaces::msg::Time & stamp, int32_t data);

struct NdtRustHostAccess
{
  static int64_t host_now_ns(void * ctx);
  static void host_log(void * ctx, int32_t level, const std::uint8_t * msg, std::size_t msg_len);
  static bool host_lookup_transform(
    void * ctx, AwStr target, AwStr source, float * out_matrix4x4_row_major);
  static void host_publish_pose(
    void * ctx, AwPoseTopic topic, int64_t stamp_ns, const AwPose * pose, const double * cov);
  static void host_publish_pose_array(
    void * ctx, AwPoseArrayTopic topic, int64_t stamp_ns, const AwPose * poses, std::size_t n);
  static void host_publish_marker(
    void * ctx, int64_t stamp_ns, const AwPose * poses, std::size_t n, int32_t max_iterations);
  static void host_publish_float32(void * ctx, AwFloat32Topic topic, int64_t stamp_ns, float value);
  static void host_publish_int32(void * ctx, AwInt32Topic topic, int64_t stamp_ns, int32_t value);
  static void host_publish_tf(void * ctx, int64_t stamp_ns, const AwPose * pose);
  static void host_publish_initial_to_result(
    void * ctx, int64_t stamp_ns, const AwPose * result, const AwPose * initial,
    const double * old_pos, const double * new_pos);
  static bool host_pointcloud_has_subscribers(void * ctx, AwPointCloudTopic topic);
  static void host_publish_pointcloud_xyz(
    void * ctx, AwPointCloudTopic topic, int64_t stamp_ns, AwPoint3fSlice points);
  static void host_publish_voxel_score_points(
    void * ctx, int64_t stamp_ns, AwPoint3fSlice points, const float * scores,
    std::size_t scores_len);
};

// The AwHost side-effects vtable trampolines (ctx == this) + make_host. Side-effects only
// (clock / logging / TF); node state stays Rust-owned on the handle.
int64_t NdtRustHostAccess::host_now_ns(void * ctx)
{
  return static_cast<NDTScanMatcher *>(ctx)->now().nanoseconds();
}
void NdtRustHostAccess::host_log(
  void * ctx, int32_t level, const std::uint8_t * msg, std::size_t msg_len)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  const std::string text(reinterpret_cast<const char *>(msg), msg_len);
  if (level >= 2) {
    RCLCPP_ERROR_STREAM_THROTTLE(self->get_logger(), *self->get_clock(), 1000, text);
  } else {
    RCLCPP_WARN_STREAM_THROTTLE(self->get_logger(), *self->get_clock(), 1000, text);
  }
}
bool NdtRustHostAccess::host_lookup_transform(
  void * ctx, AwStr target, AwStr source, float * out_matrix4x4_row_major)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  const std::string target_frame(reinterpret_cast<const char *>(target.ptr), target.len);
  const std::string source_frame(reinterpret_cast<const char *>(source.ptr), source.len);
  try {
    const geometry_msgs::msg::TransformStamped transform =
      self->tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    const geometry_msgs::msg::PoseStamped pose_stamped =
      autoware_utils_geometry::transform2pose(transform);
    const Eigen::Matrix4f matrix = pose_to_matrix4f(pose_stamped.pose);
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        out_matrix4x4_row_major[(row * 4) + col] = matrix(row, col);
      }
    }
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}
// AwPose (position + [x,y,z,w] quaternion) → geometry_msgs::Pose, for the publish trampolines.
static geometry_msgs::msg::Pose aw_pose_to_msg(const AwPose & p)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = p.position[0];
  pose.position.y = p.position[1];
  pose.position.z = p.position[2];
  pose.orientation.x = p.orientation[0];
  pose.orientation.y = p.orientation[1];
  pose.orientation.z = p.orientation[2];
  pose.orientation.w = p.orientation[3];
  return pose;
}
void NdtRustHostAccess::host_publish_pose(
  void * ctx, AwPoseTopic topic, int64_t stamp_ns, const AwPose * pose, const double * cov)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    const rclcpp::Time stamp(stamp_ns);
    const geometry_msgs::msg::Pose pose_msg = aw_pose_to_msg(*pose);
    if (topic == AwPoseTopic::NdtPose) {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = self->param_.frame.map_frame;
      msg.pose = pose_msg;
      self->ndt_pose_pub_->publish(msg);
    } else {
      geometry_msgs::msg::PoseWithCovarianceStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = self->param_.frame.map_frame;
      msg.pose.pose = pose_msg;
      if (cov != nullptr) {
        std::copy_n(cov, 36, msg.pose.covariance.begin());
      }
      if (topic == AwPoseTopic::NdtPoseWithCovariance) {
        self->ndt_pose_with_covariance_pub_->publish(msg);
      } else {
        self->initial_pose_with_covariance_pub_->publish(msg);
      }
    }
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_pose failed");
  }
}
void NdtRustHostAccess::host_publish_pose_array(
  void * ctx, AwPoseArrayTopic topic, int64_t stamp_ns, const AwPose * poses, std::size_t n)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = rclcpp::Time(stamp_ns);
    msg.header.frame_id = self->param_.frame.map_frame;
    for (std::size_t i = 0; i < n; ++i) {
      msg.poses.push_back(aw_pose_to_msg(poses[i]));  // NOLINT
    }
    if (topic == AwPoseArrayTopic::MultiNdtPose) {
      self->multi_ndt_pose_pub_->publish(msg);
    } else {
      self->multi_initial_pose_pub_->publish(msg);
    }
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_pose_array failed");
  }
}
void NdtRustHostAccess::host_publish_marker(
  void * ctx, int64_t stamp_ns, const AwPose * poses, std::size_t n, int32_t max_iterations)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    std::vector<geometry_msgs::msg::Pose> pose_array;
    pose_array.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      pose_array.push_back(aw_pose_to_msg(poses[i]));  // NOLINT
    }
    self->publish_marker(rclcpp::Time(stamp_ns), pose_array, max_iterations);
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_marker failed");
  }
}
void NdtRustHostAccess::host_publish_float32(
  void * ctx, AwFloat32Topic topic, int64_t stamp_ns, float value)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    const auto msg = make_float32_stamped(rclcpp::Time(stamp_ns), value);
    if (topic == AwFloat32Topic::TransformProbability) {
      self->transform_probability_pub_->publish(msg);
    } else if (topic == AwFloat32Topic::NearestVoxelTransformationLikelihood) {
      self->nearest_voxel_transformation_likelihood_pub_->publish(msg);
    } else if (topic == AwFloat32Topic::NoGroundTransformProbability) {
      self->no_ground_transform_probability_pub_->publish(msg);
    } else {
      self->no_ground_nearest_voxel_transformation_likelihood_pub_->publish(msg);
    }
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_float32 failed");
  }
}
void NdtRustHostAccess::host_publish_int32(
  void * ctx, AwInt32Topic /*topic*/, int64_t stamp_ns, int32_t value)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    self->iteration_num_pub_->publish(make_int32_stamped(rclcpp::Time(stamp_ns), value));
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_int32 failed");
  }
}
void NdtRustHostAccess::host_publish_tf(void * ctx, int64_t stamp_ns, const AwPose * pose)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    self->publish_tf(rclcpp::Time(stamp_ns), aw_pose_to_msg(*pose));
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_tf failed");
  }
}
void NdtRustHostAccess::host_publish_initial_to_result(
  void * ctx, int64_t stamp_ns, const AwPose * result, const AwPose * initial,
  const double * old_pos, const double * new_pos)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    geometry_msgs::msg::PoseWithCovarianceStamped initial_msg;
    initial_msg.pose.pose = aw_pose_to_msg(*initial);
    geometry_msgs::msg::PoseWithCovarianceStamped old_msg;
    old_msg.pose.pose.position.x = old_pos[0];
    old_msg.pose.pose.position.y = old_pos[1];
    old_msg.pose.pose.position.z = old_pos[2];
    geometry_msgs::msg::PoseWithCovarianceStamped new_msg;
    new_msg.pose.pose.position.x = new_pos[0];
    new_msg.pose.pose.position.y = new_pos[1];
    new_msg.pose.pose.position.z = new_pos[2];
    self->publish_initial_to_result(
      rclcpp::Time(stamp_ns), aw_pose_to_msg(*result), initial_msg, old_msg, new_msg);
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_initial_to_result failed");
  }
}

bool NdtRustHostAccess::host_pointcloud_has_subscribers(void * ctx, AwPointCloudTopic topic)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  if (topic == AwPointCloudTopic::VoxelScorePoints) {
    return self->voxel_score_points_pub_->get_subscription_count() > 0;
  }
  return true;
}

static pcl::PointCloud<pcl::PointXYZ> aw_xyz_slice_to_cloud(AwPoint3fSlice points)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (points.ptr == nullptr && points.len > 0) {
    return cloud;
  }
  cloud.points.reserve(points.len);
  for (std::size_t i = 0; i < points.len; ++i) {
    const std::size_t base = i * 3;
    cloud.points.emplace_back(points.ptr[base], points.ptr[base + 1], points.ptr[base + 2]);
  }
  cloud.width = static_cast<std::uint32_t>(cloud.points.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

void NdtRustHostAccess::host_publish_pointcloud_xyz(
  void * ctx, AwPointCloudTopic topic, int64_t stamp_ns, AwPoint3fSlice points)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    const pcl::PointCloud<pcl::PointXYZ> cloud = aw_xyz_slice_to_cloud(points);
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = rclcpp::Time(stamp_ns);
    msg.header.frame_id = self->param_.frame.map_frame;
    if (topic == AwPointCloudTopic::PointsAlignedNoGround) {
      self->no_ground_points_aligned_pose_pub_->publish(msg);
    } else {
      self->sensor_aligned_pose_pub_->publish(msg);
    }
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_pointcloud_xyz failed");
  }
}

void NdtRustHostAccess::host_publish_voxel_score_points(
  void * ctx, int64_t stamp_ns, AwPoint3fSlice points, const float * scores, std::size_t scores_len)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    if ((points.ptr == nullptr && points.len > 0) || scores == nullptr || scores_len != points.len) {
      return;
    }
    constexpr float lower_nvs = 1.0f;
    constexpr float upper_nvs = 3.5f;
    constexpr float range = upper_nvs - lower_nvs;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.points.reserve(points.len);
    for (std::size_t i = 0; i < points.len; ++i) {
      const std::size_t base = i * 3;
      pcl::PointXYZRGB point;
      point.x = points.ptr[base];
      point.y = points.ptr[base + 1];
      point.z = points.ptr[base + 2];
      const std_msgs::msg::ColorRGBA color = exchange_color_crc((scores[i] - lower_nvs) / range);
      point.r = static_cast<std::uint8_t>(color.r * 255);
      point.g = static_cast<std::uint8_t>(color.g * 255);
      point.b = static_cast<std::uint8_t>(color.b * 255);
      cloud.points.push_back(point);
    }
    cloud.width = static_cast<std::uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = true;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = rclcpp::Time(stamp_ns);
    msg.header.frame_id = self->param_.frame.map_frame;
    self->voxel_score_points_pub_->publish(msg);
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_voxel_score_points failed");
  }
}
AwHost NDTScanMatcher::make_host()
{
  return AwHost{
    this,
    &NdtRustHostAccess::host_now_ns,
    &NdtRustHostAccess::host_log,
    &NdtRustHostAccess::host_lookup_transform,
    &NdtRustHostAccess::host_publish_pose,
    &NdtRustHostAccess::host_publish_pose_array,
    &NdtRustHostAccess::host_publish_marker,
    &NdtRustHostAccess::host_publish_float32,
    &NdtRustHostAccess::host_publish_int32,
    &NdtRustHostAccess::host_publish_tf,
    &NdtRustHostAccess::host_publish_initial_to_result,
    &NdtRustHostAccess::host_pointcloud_has_subscribers,
    &NdtRustHostAccess::host_publish_pointcloud_xyz,
    &NdtRustHostAccess::host_publish_voxel_score_points};
}

}  // namespace autoware::ndt_scan_matcher
