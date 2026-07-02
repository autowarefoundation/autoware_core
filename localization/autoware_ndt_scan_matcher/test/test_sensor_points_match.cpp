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

// Differential test (Phase 5 sub-slices 2 + 3): the sensor-callback *middle* orchestrator
// (on_sensor_points_match) fuses the gates → align → convergence → covariance into one Rust call and
// (sub-slice 3) requests the POD publishers through the AwHost publish ops. This test drives it on a
// synthetic engine + seeded handle with a **recording mock AwHost** that captures every publish op,
// and asserts: the align values it publishes match the decomposed run_align on the same engine (exact,
// identity-orientation guess); the FIXED covariance == rotate_covariance(output_cov, rot(result_pose))
// (tolerance); ndt_pose / ndt_pose_with_covariance fire only on convergence; and the gate status codes.

#include "../src/ndt_scan_matcher_helper.hpp"

#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/ndt_rust_adapter.hpp>

#include "autoware_ndt_scan_matcher_rs.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{
using Adapter = autoware::ndt_scan_matcher::NdtRustAdapter;

constexpr int kTp = 0;  // ConvergedParamType::TRANSFORM_PROBABILITY
// A distinct diagonal 6x6 (row-major) so the covariance rotation is observable.
constexpr std::array<double, 36> kOutputCov = {
  4.0, 0.0, 0.0, 0.0, 0.0, 0.0,   0.0, 9.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 16.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.2, 0.0,   0.0, 0.0, 0.0, 0.0, 0.0, 0.3};

// --- engine (adapter) + source, mirroring test_node_run_align ------------------------------------
void make_tile(float cx, float cy, float cz, std::mt19937 & rng, pcl::PointCloud<pcl::PointXYZ> & c)
{
  std::normal_distribution<float> jitter(0.0F, 0.3F);
  for (int k = 0; k < 60; ++k) {
    c.push_back(pcl::PointXYZ(cx + jitter(rng), cy + jitter(rng), cz + jitter(rng)));
  }
  c.is_dense = true;
}

pclomp::NdtParams make_ndt_params()
{
  pclomp::NdtParams p{};
  p.trans_epsilon = 0.01;
  p.step_size = 0.1;
  p.resolution = 2.0F;
  p.max_iterations = 30;
  p.search_method = pclomp::KDTREE;
  p.num_threads = 1;
  p.regularization_scale_factor = 0.0F;
  p.use_line_search = false;
  return p;
}

Adapter make_driven_adapter(pcl::PointCloud<pcl::PointXYZ>::Ptr & source_out)
{
  std::mt19937 rng(13);
  auto tile0 = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto tile1 = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  make_tile(0.0F, 0.0F, 0.0F, rng, *tile0);
  make_tile(8.0F, 4.0F, 0.0F, rng, *tile1);
  const std::array<float, 3> t_true = {0.2F, -0.15F, 0.1F};
  source_out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (const auto & tile : {tile0, tile1}) {
    for (const auto & p : *tile) {
      source_out->push_back(pcl::PointXYZ(p.x + t_true[0], p.y + t_true[1], p.z + t_true[2]));
    }
  }
  Adapter rs;
  rs.setParams(make_ndt_params());
  rs.addTarget(tile0, "0");
  rs.addTarget(tile1, "1");
  rs.createVoxelKdtree();
  return rs;
}

std::vector<float> flatten(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  std::vector<float> flat;
  flat.reserve(cloud.size() * 3);
  for (const auto & p : cloud.points) {
    flat.push_back(p.x);
    flat.push_back(p.y);
    flat.push_back(p.z);
  }
  return flat;
}

// --- recording mock AwHost: capture every publish op ---------------------------------------------
struct PoseCall
{
  int topic;
  AwPose pose;
  bool has_cov;
  std::array<double, 36> cov;
};
struct Captured
{
  std::vector<PoseCall> poses;                 // publish_pose
  std::vector<std::pair<int, float>> float32s;  // publish_float32 (topic, value)
  std::vector<int32_t> iterations;              // publish_int32 (IterationNum only)
  std::vector<std::pair<int, std::size_t>> pose_arrays;  // publish_pose_array (topic, count)
  bool marker_called = false;
  std::size_t marker_count = 0;
  int32_t marker_max_iterations = 0;
  bool tf_called = false;
  bool itr_called = false;

  int count_pose_topic(int topic) const
  {
    int n = 0;
    for (const auto & p : poses) {
      if (p.topic == topic) {
        ++n;
      }
    }
    return n;
  }
};

extern "C" std::int64_t h_now(void *) { return 0; }
extern "C" void h_log(void *, std::int32_t, const std::uint8_t *, std::size_t) {}
extern "C" bool h_lookup(void *, AwStr, AwStr, float *) { return false; }
extern "C" void h_pub_pose(
  void * ctx, AwPoseTopic topic, std::int64_t, const AwPose * pose, const double * cov)
{
  auto * c = static_cast<Captured *>(ctx);
  PoseCall call{static_cast<int>(topic), *pose, cov != nullptr, {}};
  if (cov != nullptr) {
    std::copy_n(cov, 36, call.cov.begin());
  }
  c->poses.push_back(call);
}
extern "C" void h_pub_pose_array(
  void * ctx, AwPoseArrayTopic topic, std::int64_t, const AwPose *, std::size_t n)
{
  static_cast<Captured *>(ctx)->pose_arrays.emplace_back(static_cast<int>(topic), n);
}
extern "C" void h_pub_marker(
  void * ctx, std::int64_t, const AwPose *, std::size_t n, std::int32_t max_iterations)
{
  auto * c = static_cast<Captured *>(ctx);
  c->marker_called = true;
  c->marker_count = n;
  c->marker_max_iterations = max_iterations;
}
extern "C" void h_pub_float32(void * ctx, AwFloat32Topic topic, std::int64_t, float value)
{
  static_cast<Captured *>(ctx)->float32s.emplace_back(static_cast<int>(topic), value);
}
extern "C" void h_pub_int32(void * ctx, AwInt32Topic, std::int64_t, std::int32_t value)
{
  static_cast<Captured *>(ctx)->iterations.push_back(value);
}
extern "C" void h_pub_tf(void * ctx, std::int64_t, const AwPose *)
{
  static_cast<Captured *>(ctx)->tf_called = true;
}
extern "C" void h_pub_itr(
  void * ctx, std::int64_t, const AwPose *, const AwPose *, const double *, const double *)
{
  static_cast<Captured *>(ctx)->itr_called = true;
}
AwHost recording_host(Captured & c)
{
  return AwHost{
    &c,           h_now,        h_log,         h_lookup,   h_pub_pose, h_pub_pose_array,
    h_pub_marker, h_pub_float32, h_pub_int32, h_pub_tf, h_pub_itr};
}

// --- no-op diagnostics ---------------------------------------------------------------------------
extern "C" void d_clear(void *) {}
extern "C" void d_bool(void *, const std::uint8_t *, std::size_t, bool) {}
extern "C" void d_i64(void *, const std::uint8_t *, std::size_t, std::int64_t) {}
extern "C" void d_f64(void *, const std::uint8_t *, std::size_t, double) {}
extern "C" void d_str(void *, const std::uint8_t *, std::size_t, const std::uint8_t *, std::size_t) {}
extern "C" void d_level(void *, std::int8_t, const std::uint8_t *, std::size_t) {}
extern "C" void d_publish(void *, std::int64_t) {}
AwDiagnostics noop_diag()
{
  return AwDiagnostics{nullptr, d_clear, d_bool, d_i64, d_f64, d_str, d_level, d_publish};
}

// --- node handle (FIXED covariance, regularization off) ------------------------------------------
AwNdtScanMatcher * make_handle(
  const std::string & map_frame, const std::string & base_frame, double converged_tp)
{
  AwNdtParams p{};
  p.resolution = 2.0;
  p.min_points = 6;
  p.num_threads = 1;
  p.converged_param_type = kTp;
  p.converged_param_transform_probability = converged_tp;
  p.converged_param_nearest_voxel_transformation_likelihood = 0.0;
  p.covariance_estimation_type = 0;  // FIXED_VALUE
  p.covariance_scale_factor = 1.0;
  p.covariance_temperature = 0.05;
  std::copy(kOutputCov.begin(), kOutputCov.end(), p.output_pose_covariance);
  p.regularization_enable = false;
  p.map_frame = reinterpret_cast<const std::uint8_t *>(map_frame.data());
  p.map_frame_len = map_frame.size();
  p.initial_pose_timeout_sec = 1e9;
  p.initial_pose_distance_tolerance_m = 1e9;
  p.base_frame = reinterpret_cast<const std::uint8_t *>(base_frame.data());
  p.base_frame_len = base_frame.size();
  p.sensor_points_timeout_sec = 1e9;
  p.sensor_points_required_distance = 0.0;
  return autoware_ndt_scan_matcher_rs_new(&p);
}

// Activate + push two identity-orientation, origin-position initial poses bracketing `stamp`.
void activate_and_seed(AwNdtScanMatcher * h, std::int64_t stamp, const std::string & map_frame)
{
  const AwDiagnostics d = noop_diag();
  autoware_ndt_scan_matcher_rs_node_on_trigger(h, &d, true, 0);
  const auto push = [&](std::int64_t t) {
    AwPoseWithCovarianceStampedView v{};
    v.stamp_ns = t;
    v.orientation[3] = 1.0;
    v.frame_id = reinterpret_cast<const std::uint8_t *>(map_frame.data());
    v.frame_id_len = map_frame.size();
    autoware_ndt_scan_matcher_rs_node_on_initial_pose(h, &d, &v);
  };
  push(stamp - 100'000'000);
  push(stamp + 100'000'000);
}

Eigen::Matrix4f to_eigen(const float * m)
{
  Eigen::Matrix4f e;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      e(r, c) = m[(r * 4) + c];
    }
  }
  return e;
}
}  // namespace

// The orchestrator publishes the align values the decomposed run_align yields (exact), the FIXED
// covariance rotated from the result pose (tolerance), and fires ndt_pose only on convergence.
TEST(SensorPointsMatch, PublishesDecomposedAlignAndCovariance)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source;
  Adapter rs = make_driven_adapter(source);
  const std::vector<float> src = flatten(*source);
  const std::int64_t stamp = 1'000'000'000;
  const std::string map_frame = "map";

  AwNdtScanMatcher * h = make_handle(map_frame, "base_link", 0.0);
  ASSERT_NE(h, nullptr);
  activate_and_seed(h, stamp, map_frame);

  Captured cap;
  const AwHost host = recording_host(cap);
  const AwDiagnostics diag = noop_diag();
  const AwSensorPointsMatchParams mp{50.0, 1e9, 1e9, 0.0};
  AwSensorPointsMatchOutput out{};

  const std::int32_t status = autoware_ndt_scan_matcher_rs_node_on_sensor_points_match(
    h, rs.raw_handle(), &host, &diag, &mp, stamp, src.data(), source->size(), &out);
  ASSERT_EQ(status, 0);  // SM_MATCHED
  ASSERT_TRUE(out.is_converged);

  // Reference: align the same engine from the identity guess (== the interpolated identity pose).
  const std::array<float, 16> guess16 = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  const AwAlignParams align_params{kTp, 0.0, 0.0};
  AwAlignOutcome ref{};
  autoware_ndt_scan_matcher_rs_node_run_align(
    rs.raw_handle(), guess16.data(), src.data(), source->size(), &align_params, &ref);

  // scalars published (exact vs the decomposed align).
  ASSERT_EQ(cap.float32s.size(), 2U);
  ASSERT_EQ(cap.iterations.size(), 1U);
  EXPECT_EQ(cap.iterations[0], ref.iteration_num);
  for (const auto & [topic, value] : cap.float32s) {
    if (topic == 0) {  // TransformProbability
      EXPECT_FLOAT_EQ(value, ref.transform_probability);
    } else {  // NearestVoxelTransformationLikelihood
      EXPECT_FLOAT_EQ(value, ref.nearest_voxel_likelihood);
    }
  }

  // tf + initial_to_result + marker each fire once; marker got one pose per iteration.
  EXPECT_TRUE(cap.tf_called);
  EXPECT_TRUE(cap.itr_called);
  EXPECT_TRUE(cap.marker_called);
  EXPECT_GT(cap.marker_count, 0U);
  EXPECT_EQ(cap.marker_max_iterations, rs.getMaximumIterations());

  // pose publishes: InitialPoseWithCovariance always; NdtPose + NdtPoseWithCovariance on convergence.
  EXPECT_EQ(cap.count_pose_topic(2), 1);  // InitialPoseWithCovariance
  EXPECT_EQ(cap.count_pose_topic(0), 1);  // NdtPose (converged)
  EXPECT_EQ(cap.count_pose_topic(1), 1);  // NdtPoseWithCovariance (converged)

  // ndt_pose pose == the decomposed result; ndt_pose_with_covariance carries the FIXED rotated cov.
  const geometry_msgs::msg::Pose ref_pose_msg =
    autoware::localization_util::matrix4f_to_pose(to_eigen(ref.pose));
  const Eigen::Quaterniond q(
    ref_pose_msg.orientation.w, ref_pose_msg.orientation.x, ref_pose_msg.orientation.y,
    ref_pose_msg.orientation.z);
  const std::array<double, 36> expected_cov =
    rotate_covariance(kOutputCov, q.normalized().toRotationMatrix());
  for (const auto & call : cap.poses) {
    if (call.topic == 1) {  // NdtPoseWithCovariance
      EXPECT_NEAR(call.pose.position[0], ref_pose_msg.position.x, 1e-5);
      EXPECT_NEAR(call.pose.position[1], ref_pose_msg.position.y, 1e-5);
      EXPECT_NEAR(call.pose.position[2], ref_pose_msg.position.z, 1e-5);
      ASSERT_TRUE(call.has_cov);
      for (int i = 0; i < 36; ++i) {
        EXPECT_NEAR(call.cov[static_cast<std::size_t>(i)], expected_cov[static_cast<std::size_t>(i)], 1e-5)
          << "cov[" << i << "]";
      }
    } else if (call.topic == 2) {  // InitialPoseWithCovariance == the seeded identity/origin pose
      EXPECT_NEAR(call.pose.position[0], 0.0, 1e-9);
      EXPECT_NEAR(call.pose.orientation[3], 1.0, 1e-9);
    }
  }

  // FIXED estimation publishes no covariance debug pose arrays.
  EXPECT_TRUE(cap.pose_arrays.empty());

  autoware_ndt_scan_matcher_rs_free(h);
}

// A below-threshold score → not converged → ndt_pose / ndt_pose_with_covariance are NOT published.
TEST(SensorPointsMatch, NotConvergedSkipsNdtPose)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source;
  Adapter rs = make_driven_adapter(source);
  const std::vector<float> src = flatten(*source);
  const std::int64_t stamp = 1'000'000'000;
  const std::string map_frame = "map";

  AwNdtScanMatcher * h = make_handle(map_frame, "base_link", 1e9);  // impossible TP threshold
  activate_and_seed(h, stamp, map_frame);
  Captured cap;
  const AwHost host = recording_host(cap);
  const AwDiagnostics diag = noop_diag();
  const AwSensorPointsMatchParams mp{50.0, 1e9, 1e9, 0.0};
  AwSensorPointsMatchOutput out{};

  ASSERT_EQ(
    autoware_ndt_scan_matcher_rs_node_on_sensor_points_match(
      h, rs.raw_handle(), &host, &diag, &mp, stamp, src.data(), source->size(), &out),
    0);  // SM_MATCHED (a below-threshold score still matches; it just isn't converged)
  EXPECT_FALSE(out.is_converged);
  EXPECT_EQ(cap.count_pose_topic(0), 0);  // no NdtPose
  EXPECT_EQ(cap.count_pose_topic(1), 0);  // no NdtPoseWithCovariance
  EXPECT_EQ(cap.count_pose_topic(2), 1);  // InitialPoseWithCovariance still published
  autoware_ndt_scan_matcher_rs_free(h);
}

// Gate: an inactive node returns SM_NOT_ACTIVATED and publishes nothing.
TEST(SensorPointsMatch, NotActivatedGate)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source;
  Adapter rs = make_driven_adapter(source);
  const std::vector<float> src = flatten(*source);
  AwNdtScanMatcher * h = make_handle("map", "base_link", 0.0);  // never activated
  Captured cap;
  const AwHost host = recording_host(cap);
  const AwDiagnostics diag = noop_diag();
  const AwSensorPointsMatchParams mp{50.0, 1e9, 1e9, 0.0};
  AwSensorPointsMatchOutput out{};
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_node_on_sensor_points_match(
      h, rs.raw_handle(), &host, &diag, &mp, 1'000'000'000, src.data(), source->size(), &out),
    1);  // SM_NOT_ACTIVATED
  EXPECT_TRUE(cap.poses.empty());
  EXPECT_TRUE(cap.float32s.empty());
  autoware_ndt_scan_matcher_rs_free(h);
}

// Gate: activated but no initial poses to interpolate returns SM_INTERPOLATE_FAILED.
TEST(SensorPointsMatch, InterpolateFailedGate)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source;
  Adapter rs = make_driven_adapter(source);
  const std::vector<float> src = flatten(*source);
  AwNdtScanMatcher * h = make_handle("map", "base_link", 0.0);
  const AwDiagnostics d = noop_diag();
  autoware_ndt_scan_matcher_rs_node_on_trigger(h, &d, true, 0);  // activate, seed nothing
  Captured cap;
  const AwHost host = recording_host(cap);
  const AwSensorPointsMatchParams mp{50.0, 1e9, 1e9, 0.0};
  AwSensorPointsMatchOutput out{};
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_node_on_sensor_points_match(
      h, rs.raw_handle(), &host, &d, &mp, 1'000'000'000, src.data(), source->size(), &out),
    2);  // SM_INTERPOLATE_FAILED
  autoware_ndt_scan_matcher_rs_free(h);
}

// Gate: an engine with no loaded map returns SM_MAP_NOT_SET (after a successful interpolation).
TEST(SensorPointsMatch, MapNotSetGate)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source;
  Adapter driven = make_driven_adapter(source);  // only for a non-empty source cloud
  const std::vector<float> src = flatten(*source);
  Adapter empty;
  empty.setParams(make_ndt_params());  // no addTarget → hasTarget() == false

  const std::int64_t stamp = 1'000'000'000;
  const std::string map_frame = "map";
  AwNdtScanMatcher * h = make_handle(map_frame, "base_link", 0.0);
  activate_and_seed(h, stamp, map_frame);
  Captured cap;
  const AwHost host = recording_host(cap);
  const AwDiagnostics diag = noop_diag();
  const AwSensorPointsMatchParams mp{50.0, 1e9, 1e9, 0.0};
  AwSensorPointsMatchOutput out{};
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_node_on_sensor_points_match(
      h, empty.raw_handle(), &host, &diag, &mp, stamp, src.data(), source->size(), &out),
    3);  // SM_MAP_NOT_SET
  autoware_ndt_scan_matcher_rs_free(h);
}

}  // namespace autoware::ndt_scan_matcher
