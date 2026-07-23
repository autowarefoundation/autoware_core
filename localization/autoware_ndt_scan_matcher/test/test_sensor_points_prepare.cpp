// Copyright 2024 Autoware Foundation
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

// Differential test: the Rust sensor-callback prologue
// (on_sensor_points_prepare — decode + transform-to-base_link via the AwHost TF) must reproduce the
// C++ pcl path (pcl::fromROSMsg + pcl::transformPointCloud) bit-close, and its validation gates
// (empty / TF-fail / too-close) must return the matching status. The TF is supplied by a mock
// AwHost returning a fixed matrix, so the test is self-contained (no live tf2).

#include "autoware/ndt_scan_matcher/ndt_scan_matcher_rs.hpp"

#include <Eigen/Core>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <array>
#include <cstdint>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{
// A mock AwHost: `lookup_transform` returns a fixed row-major 4x4 (or fails when `tf_ok` is false).
struct HostCtx
{
  std::array<float, 16> matrix;
  bool tf_ok;
};
extern "C" std::int64_t h_now(void *)
{
  return 0;
}
extern "C" void h_log(void *, std::int32_t, const std::uint8_t *, std::size_t)
{
}
extern "C" bool h_lookup(void * ctx, AwStr /*target*/, AwStr /*source*/, float * out)
{
  auto * c = static_cast<HostCtx *>(ctx);
  if (!c->tf_ok) {
    return false;
  }
  for (int i = 0; i < 16; ++i) {
    out[i] = c->matrix[static_cast<std::size_t>(i)];
  }
  return true;
}
extern "C" void h_pub_pose(void *, AwPoseTopic, std::int64_t, const AwPose *, const double *)
{
}
extern "C" void h_pub_pose_array(
  void *, AwPoseArrayTopic, std::int64_t, const AwPose *, std::size_t)
{
}
extern "C" void h_pub_marker(void *, std::int64_t, const AwPose *, std::size_t, std::int32_t)
{
}
extern "C" void h_pub_float32(void *, AwFloat32Topic, std::int64_t, float)
{
}
extern "C" void h_pub_int32(void *, AwInt32Topic, std::int64_t, std::int32_t)
{
}
extern "C" void h_pub_tf(void *, std::int64_t, const AwPose *)
{
}
extern "C" void h_pub_itr(
  void *, std::int64_t, const AwPose *, const AwPose *, const double *, const double *)
{
}
extern "C" bool h_pc_has(void *, AwPointCloudTopic)
{
  return false;
}
extern "C" void h_pub_cloud(void *, AwPointCloudTopic, std::int64_t, AwPoint3fSlice)
{
}
extern "C" void h_pub_score_cloud(void *, std::int64_t, AwPoint3fSlice, const float *, std::size_t)
{
}
AwHost mock_host(HostCtx & c)
{
  return AwHost{&c,           h_now,
                h_log,        h_lookup,
                h_pub_pose,   h_pub_pose_array,
                h_pub_marker, h_pub_float32,
                h_pub_int32,  h_pub_tf,
                h_pub_itr,    h_pc_has,
                h_pub_cloud,  h_pub_score_cloud};
}

// A no-op diagnostics vtable.
extern "C" void d_clear(void *)
{
}
extern "C" void d_bool(void *, const std::uint8_t *, std::size_t, bool)
{
}
extern "C" void d_i64(void *, const std::uint8_t *, std::size_t, std::int64_t)
{
}
extern "C" void d_f64(void *, const std::uint8_t *, std::size_t, double)
{
}
extern "C" void d_str(void *, const std::uint8_t *, std::size_t, const std::uint8_t *, std::size_t)
{
}
extern "C" void d_level(void *, std::int8_t, const std::uint8_t *, std::size_t)
{
}
extern "C" void d_publish(void *, std::int64_t)
{
}
AwDiagnostics noop_diag()
{
  return AwDiagnostics{nullptr, d_clear, d_bool, d_i64, d_f64, d_str, d_level, d_publish};
}

AwNdtScanMatcher * make_handle(double required_distance)
{
  AwNdtParams p{};
  p.resolution = 1.0;
  p.min_points = 6;
  p.num_threads = 1;
  const std::string base = "base_link";
  p.base_frame = reinterpret_cast<const std::uint8_t *>(base.data());
  p.base_frame_len = base.size();
  p.sensor_points_timeout_sec = 1e9;  // never "delayed" in the test
  p.sensor_points_required_distance = required_distance;
  return autoware_ndt_scan_matcher_rs_new(&p);  // `base` outlives this call (Rust copies it)
}

// Row-major 4x4 → Eigen (column-major storage, same element layout).
Eigen::Matrix4f to_eigen(const std::array<float, 16> & m)
{
  Eigen::Matrix4f e;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      e(r, c) = m[static_cast<std::size_t>((r * 4) + c)];
    }
  }
  return e;
}

sensor_msgs::msg::PointCloud2 make_cloud_msg(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "lidar";
  return msg;
}
}  // namespace

TEST(SensorPointsPrepare, MatchesPclTransform)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(1.0F, 0.0F, 0.0F));
  cloud.push_back(pcl::PointXYZ(0.0F, 2.0F, 0.5F));
  cloud.push_back(pcl::PointXYZ(-3.0F, 4.0F, -1.0F));
  cloud.push_back(pcl::PointXYZ(10.0F, -5.0F, 2.0F));
  const sensor_msgs::msg::PointCloud2 msg = make_cloud_msg(cloud);

  // base_link <- lidar: +90 deg about Z then translate (1, 2, 3).
  const std::array<float, 16> m{0.0F, -1.0F, 0.0F, 1.0F, 1.0F, 0.0F, 0.0F, 2.0F,
                                0.0F, 0.0F,  1.0F, 3.0F, 0.0F, 0.0F, 0.0F, 1.0F};
  HostCtx ctx{m, true};
  const AwHost host = mock_host(ctx);
  const AwDiagnostics diag = noop_diag();
  AwNdtScanMatcher * h = make_handle(0.0);
  ASSERT_NE(h, nullptr);

  const AwPointCloud2View view = make_pointcloud2_view(msg);
  std::vector<float> out(cloud.size() * 3);
  std::size_t count = 0;
  const int status = autoware_ndt_scan_matcher_rs_node_on_sensor_points_prepare(
    h, &host, &diag, &view, out.data(), out.size(), &count);
  EXPECT_EQ(status, 0);  // SP_PREPARED
  ASSERT_EQ(count, cloud.size());

  // C++ reference: pcl::transformPointCloud with the same matrix.
  pcl::PointCloud<pcl::PointXYZ> ref;
  pcl::transformPointCloud(cloud, ref, to_eigen(m));
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    EXPECT_NEAR(out[(i * 3) + 0], ref.points[i].x, 1e-4) << "point " << i;
    EXPECT_NEAR(out[(i * 3) + 1], ref.points[i].y, 1e-4) << "point " << i;
    EXPECT_NEAR(out[(i * 3) + 2], ref.points[i].z, 1e-4) << "point " << i;
  }
  autoware_ndt_scan_matcher_rs_free(h);
}

TEST(SensorPointsPrepare, EmptyCloudReturnsEmptyStatus)  // NOLINT
{
  const pcl::PointCloud<pcl::PointXYZ> empty;
  const sensor_msgs::msg::PointCloud2 msg = make_cloud_msg(empty);
  const std::array<float, 16> identity{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  HostCtx ctx{identity, true};
  const AwHost host = mock_host(ctx);
  const AwDiagnostics diag = noop_diag();
  AwNdtScanMatcher * h = make_handle(0.0);
  const AwPointCloud2View view = make_pointcloud2_view(msg);
  std::vector<float> out(3);
  std::size_t count = 0;
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_node_on_sensor_points_prepare(
      h, &host, &diag, &view, out.data(), out.size(), &count),
    1);  // SP_EMPTY
  autoware_ndt_scan_matcher_rs_free(h);
}

TEST(SensorPointsPrepare, TfFailureReturnsTfStatus)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(1.0F, 1.0F, 1.0F));
  const sensor_msgs::msg::PointCloud2 msg = make_cloud_msg(cloud);
  const std::array<float, 16> identity{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  HostCtx ctx{identity, false};  // TF unavailable
  const AwHost host = mock_host(ctx);
  const AwDiagnostics diag = noop_diag();
  AwNdtScanMatcher * h = make_handle(0.0);
  const AwPointCloud2View view = make_pointcloud2_view(msg);
  std::vector<float> out(3);
  std::size_t count = 0;
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_node_on_sensor_points_prepare(
      h, &host, &diag, &view, out.data(), out.size(), &count),
    2);  // SP_TF_FAILED
  autoware_ndt_scan_matcher_rs_free(h);
}

TEST(SensorPointsPrepare, TooCloseReturnsTooCloseStatus)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(1.0F, 1.0F, 1.0F));
  const sensor_msgs::msg::PointCloud2 msg = make_cloud_msg(cloud);
  const std::array<float, 16> identity{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  HostCtx ctx{identity, true};
  const AwHost host = mock_host(ctx);
  const AwDiagnostics diag = noop_diag();
  AwNdtScanMatcher * h = make_handle(1e9);  // required_distance huge → all points too close
  const AwPointCloud2View view = make_pointcloud2_view(msg);
  std::vector<float> out(3);
  std::size_t count = 0;
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_node_on_sensor_points_prepare(
      h, &host, &diag, &view, out.data(), out.size(), &count),
    3);  // SP_TOO_CLOSE
  autoware_ndt_scan_matcher_rs_free(h);
}

}  // namespace autoware::ndt_scan_matcher
