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

// Differential test (Phase 5 sub-slice 2): the sensor-callback *middle* orchestrator
// (on_sensor_points_match) fuses the activation / interpolate / map gates, align + convergence, and
// the covariance block into one Rust call. It must reproduce the decomposed FFI path it replaces —
// on the SAME engine, from a guess built from the SAME interpolated initial pose — bit-for-bit for
// the align outputs (deterministic engine + identity-orientation guess ⇒ identical guess) and within
// tolerance for the covariance (whose base-cov rotation is derived from the result pose). It also
// pins the gate status codes. The engine-vs-pclomp accuracy, the convergence formula, and the
// covariance math are each already differential-tested elsewhere; this pins the orchestration wiring.

#include "../src/ndt_scan_matcher_helper.hpp"

#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/ndt_rust_adapter.hpp>

#include "autoware_ndt_scan_matcher_rs.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <cstdint>
#include <random>
#include <string>
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

// --- engine (adapter) + source, mirroring test_node_run_align -------------------------------------
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

// --- mock host + no-op diagnostics ----------------------------------------------------------------
extern "C" std::int64_t h_now(void *) { return 0; }
extern "C" void h_log(void *, std::int32_t, const std::uint8_t *, std::size_t) {}
extern "C" bool h_lookup(void *, AwStr, AwStr, float *) { return false; }
AwHost mock_host() { return AwHost{nullptr, h_now, h_log, h_lookup}; }

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

// --- node handle ----------------------------------------------------------------------------------
// FIXED-value covariance keeps the covariance path deterministic (rotate the configured 6x6 only) and
// avoids needing offset models; regularization off keeps the align free of the reg buffer.
AwNdtScanMatcher * make_handle(const std::string & map_frame, const std::string & base_frame)
{
  AwNdtParams p{};
  p.resolution = 2.0;
  p.min_points = 6;
  p.num_threads = 1;
  p.converged_param_type = kTp;
  p.converged_param_transform_probability = 0.0;
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
  return autoware_ndt_scan_matcher_rs_new(&p);  // `map_frame`/`base_frame` outlive this call
}

// Activate + push two identity-orientation, origin-position initial poses bracketing `stamp`, so the
// interpolation at `stamp` yields an identity guess (bit-identical across nalgebra/Eigen).
void activate_and_seed(AwNdtScanMatcher * h, std::int64_t stamp, const std::string & map_frame)
{
  const AwDiagnostics d = noop_diag();
  autoware_ndt_scan_matcher_rs_node_on_trigger(h, &d, true, 0);
  const auto push = [&](std::int64_t t) {
    AwPoseWithCovarianceStampedView v{};
    v.stamp_ns = t;
    v.position[0] = 0.0;
    v.position[1] = 0.0;
    v.position[2] = 0.0;
    v.orientation[0] = 0.0;
    v.orientation[1] = 0.0;
    v.orientation[2] = 0.0;
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

AwSensorPointsMatchOutput make_out(
  std::vector<float> & tbuf, std::vector<float> & crp, std::vector<float> & cip, uint32_t pose_cap)
{
  AwSensorPointsMatchOutput out{};
  out.transformation_array = tbuf.data();
  out.transformation_cap = static_cast<uint32_t>(tbuf.size() / 16);
  out.multi_ndt_result_poses = crp.data();
  out.multi_initial_poses = cip.data();
  out.pose_cap = pose_cap;
  return out;
}
}  // namespace

// The orchestrator's align outputs == the decomposed run_align on the same engine + identity guess
// (exact), and its FIXED covariance == rotate_covariance(output_cov, rot(result_pose)) (tolerance,
// the rotation is nalgebra-derived here vs Eigen-derived in the reference).
TEST(SensorPointsMatch, MatchesDecomposedAlignAndCovariance)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source;
  Adapter rs = make_driven_adapter(source);
  const std::vector<float> src = flatten(*source);
  const std::int64_t stamp = 1'000'000'000;
  const std::string map_frame = "map";

  AwNdtScanMatcher * h = make_handle(map_frame, "base_link");
  ASSERT_NE(h, nullptr);
  activate_and_seed(h, stamp, map_frame);

  const AwHost host = mock_host();
  const AwDiagnostics diag = noop_diag();
  const AwSensorPointsMatchParams mp{50.0, 1e9, 1e9, 0.0};

  std::vector<float> tbuf(static_cast<std::size_t>(256) * 16);
  std::vector<float> crp(16);
  std::vector<float> cip(16);
  AwSensorPointsMatchOutput out = make_out(tbuf, crp, cip, 1);

  const std::int32_t status = autoware_ndt_scan_matcher_rs_node_on_sensor_points_match(
    h, rs.raw_handle(), &host, &diag, &mp, stamp, src.data(), source->size(), &out);
  ASSERT_EQ(status, 0);  // SM_MATCHED

  // Reference: align the same engine from the identity guess (== the interpolated identity pose).
  const std::array<float, 16> guess16 = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  const AwAlignParams align_params{kTp, 0.0, 0.0};
  AwAlignOutcome ref{};
  autoware_ndt_scan_matcher_rs_node_run_align(
    rs.raw_handle(), guess16.data(), src.data(), source->size(), &align_params, &ref);

  EXPECT_EQ(out.iteration_num, ref.iteration_num);
  EXPECT_FLOAT_EQ(out.transform_probability, ref.transform_probability);
  EXPECT_FLOAT_EQ(out.nearest_voxel_transformation_likelihood, ref.nearest_voxel_likelihood);
  EXPECT_EQ(out.is_converged, ref.verdict.is_converged);
  for (int i = 0; i < 16; ++i) {
    EXPECT_FLOAT_EQ(out.result_pose[i], ref.pose[i]) << "result_pose[" << i << "]";
  }
  // marker trajectory: one pose per iteration, same as the reference align.
  EXPECT_GT(out.transformation_count, 0U);

  // interpolated initial pose == the seeded identity pose at the origin.
  EXPECT_NEAR(out.initial_position[0], 0.0, 1e-9);
  EXPECT_NEAR(out.initial_position[1], 0.0, 1e-9);
  EXPECT_NEAR(out.initial_position[2], 0.0, 1e-9);
  EXPECT_NEAR(out.initial_orientation[3], 1.0, 1e-9);

  // FIXED covariance: the configured 6x6 rotated into the map frame by the result pose's rotation.
  const geometry_msgs::msg::Pose result_pose_msg =
    autoware::localization_util::matrix4f_to_pose(to_eigen(out.result_pose));
  const Eigen::Quaterniond q(
    result_pose_msg.orientation.w, result_pose_msg.orientation.x, result_pose_msg.orientation.y,
    result_pose_msg.orientation.z);
  const std::array<double, 36> expected =
    rotate_covariance(kOutputCov, q.normalized().toRotationMatrix());
  for (int i = 0; i < 36; ++i) {
    EXPECT_NEAR(out.ndt_covariance[i], expected[i], 1e-5) << "ndt_covariance[" << i << "]";
  }
  EXPECT_EQ(out.publish_kind, 0);  // FIXED publishes no debug pose arrays
  EXPECT_EQ(out.multi_ndt_result_count, 0U);
  EXPECT_EQ(out.multi_initial_count, 0U);

  autoware_ndt_scan_matcher_rs_free(h);
}

// Gate: an inactive node returns SM_NOT_ACTIVATED (no align).
TEST(SensorPointsMatch, NotActivatedGate)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source;
  Adapter rs = make_driven_adapter(source);
  const std::vector<float> src = flatten(*source);
  AwNdtScanMatcher * h = make_handle("map", "base_link");  // never activated
  const AwHost host = mock_host();
  const AwDiagnostics diag = noop_diag();
  const AwSensorPointsMatchParams mp{50.0, 1e9, 1e9, 0.0};
  std::vector<float> tbuf(16);
  std::vector<float> crp(16);
  std::vector<float> cip(16);
  AwSensorPointsMatchOutput out = make_out(tbuf, crp, cip, 1);
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_node_on_sensor_points_match(
      h, rs.raw_handle(), &host, &diag, &mp, 1'000'000'000, src.data(), source->size(), &out),
    1);  // SM_NOT_ACTIVATED
  autoware_ndt_scan_matcher_rs_free(h);
}

// Gate: activated but no initial poses to interpolate returns SM_INTERPOLATE_FAILED.
TEST(SensorPointsMatch, InterpolateFailedGate)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source;
  Adapter rs = make_driven_adapter(source);
  const std::vector<float> src = flatten(*source);
  AwNdtScanMatcher * h = make_handle("map", "base_link");
  const AwDiagnostics d = noop_diag();
  autoware_ndt_scan_matcher_rs_node_on_trigger(h, &d, true, 0);  // activate, but seed nothing
  const AwHost host = mock_host();
  const AwSensorPointsMatchParams mp{50.0, 1e9, 1e9, 0.0};
  std::vector<float> tbuf(16);
  std::vector<float> crp(16);
  std::vector<float> cip(16);
  AwSensorPointsMatchOutput out = make_out(tbuf, crp, cip, 1);
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
  Adapter driven = make_driven_adapter(source);  // only used for a non-empty source cloud
  const std::vector<float> src = flatten(*source);
  Adapter empty;
  empty.setParams(make_ndt_params());  // no addTarget → hasTarget() == false

  const std::int64_t stamp = 1'000'000'000;
  const std::string map_frame = "map";
  AwNdtScanMatcher * h = make_handle(map_frame, "base_link");
  activate_and_seed(h, stamp, map_frame);
  const AwHost host = mock_host();
  const AwDiagnostics diag = noop_diag();
  const AwSensorPointsMatchParams mp{50.0, 1e9, 1e9, 0.0};
  std::vector<float> tbuf(16);
  std::vector<float> crp(16);
  std::vector<float> cip(16);
  AwSensorPointsMatchOutput out = make_out(tbuf, crp, cip, 1);
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_node_on_sensor_points_match(
      h, empty.raw_handle(), &host, &diag, &mp, stamp, src.data(), source->size(), &out),
    3);  // SM_MAP_NOT_SET
  autoware_ndt_scan_matcher_rs_free(h);
}

}  // namespace autoware::ndt_scan_matcher
