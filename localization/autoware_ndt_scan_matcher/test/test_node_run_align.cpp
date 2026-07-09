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

// Test: the sensor-callback align orchestrator
// (autoware_ndt_scan_matcher_rs_node_run_align) drives a live Rust engine directly through the C
// ABI. It must align EXACTLY like the bare engine align FFI on the same deterministic Rust engine +
// guess and return a self-consistent convergence verdict. This pins the engine-handle +
// orchestrator wiring; engine-vs-pclomp accuracy is already covered by test_ndt_engine/test_align,
// and the convergence-formula vs C++ by test_convergence_verdict.

#include "autoware_ndt_scan_matcher_rs.h"

#include <Eigen/Core>
#include <autoware/ndt_scan_matcher/ndt_omp/ndt_struct.hpp>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <cmath>
#include <random>
#include <vector>

namespace
{
void make_tile(float cx, float cy, float cz, std::mt19937 & rng, pcl::PointCloud<pcl::PointXYZ> & c)
{
  std::normal_distribution<float> jitter(0.0F, 0.3F);
  for (int k = 0; k < 60; ++k) {
    c.push_back(pcl::PointXYZ(cx + jitter(rng), cy + jitter(rng), cz + jitter(rng)));
  }
  c.is_dense = true;
}

pclomp::NdtParams make_params()
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

constexpr int kTp = 0;  // ConvergedParamType::TRANSFORM_PROBABILITY

class RustEngine
{
public:
  RustEngine() : handle_(autoware_ndt_scan_matcher_rs_ndt_engine_new(2.0, 6, 0.01)) {}
  ~RustEngine() { autoware_ndt_scan_matcher_rs_ndt_engine_free(handle_); }

  RustEngine(const RustEngine &) = delete;
  RustEngine & operator=(const RustEngine &) = delete;

  RustEngine(RustEngine && other) noexcept : handle_(other.handle_) { other.handle_ = nullptr; }
  RustEngine & operator=(RustEngine && other) noexcept
  {
    if (this != &other) {
      autoware_ndt_scan_matcher_rs_ndt_engine_free(handle_);
      handle_ = other.handle_;
      other.handle_ = nullptr;
    }
    return *this;
  }

  [[nodiscard]] const AwNdtEngine * raw() const { return handle_; }

private:
  AwNdtEngine * handle_{nullptr};
};

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

void add_target(const RustEngine & engine, const pcl::PointCloud<pcl::PointXYZ> & tile, uint64_t id)
{
  const std::vector<float> flat = flatten(tile);
  autoware_ndt_scan_matcher_rs_ndt_engine_add_target(engine.raw(), flat.data(), tile.size(), id);
}

// Build a 2-tile Rust engine + a source cloud shifted by a small known translation.
RustEngine make_driven_engine(pcl::PointCloud<pcl::PointXYZ>::Ptr & source_out)
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
  RustEngine engine;
  const pclomp::NdtParams params = make_params();
  autoware_ndt_scan_matcher_rs_ndt_engine_set_params(
    engine.raw(), params.trans_epsilon, params.step_size, params.resolution, params.max_iterations,
    0.55, params.num_threads);
  add_target(engine, *tile0, 0);
  add_target(engine, *tile1, 1);
  autoware_ndt_scan_matcher_rs_ndt_engine_create_kdtree(engine.raw());
  return engine;
}

AwNdtAlignServiceSearchInput make_search_input(
  const pcl::PointCloud<pcl::PointXYZ> & source, const std::vector<float> & flat,
  const int64_t particles_num, const int64_t startup_trials)
{
  AwNdtAlignServiceSearchInput input{};
  input.position[0] = 0.0;
  input.position[1] = 0.0;
  input.position[2] = 0.0;
  input.orientation[0] = 0.0;
  input.orientation[1] = 0.0;
  input.orientation[2] = 0.0;
  input.orientation[3] = 1.0;
  input.covariance[0] = 0.25;
  input.covariance[7] = 0.25;
  input.covariance[14] = 0.01;
  input.covariance[21] = 0.01;
  input.covariance[28] = 0.01;
  input.particles_num = particles_num;
  input.n_startup_trials = startup_trials;
  input.reliable_score_threshold = 0.0;
  input.source_points = flat.data();
  input.source_points_len = source.size();
  return input;
}

AwNdtAlignServiceSearchOutput make_search_output(
  std::vector<AwPose> & initial_poses, std::vector<AwPose> & result_poses,
  std::vector<double> & scores, std::vector<std::int32_t> & iterations)
{
  AwNdtAlignServiceSearchOutput output{};
  output.particles_capacity = initial_poses.size();
  output.initial_poses = initial_poses.data();
  output.result_poses = result_poses.data();
  output.scores = scores.data();
  output.iterations = iterations.data();
  return output;
}
}  // namespace

// run_align via a direct engine handle == the bare engine align FFI on the same engine + same guess
// + a self-consistent verdict. Exact equality (deterministic Rust engine), no tolerance.
TEST(NodeRunAlign, MatchesEngineAlignAndVerdictIsConsistent)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source;
  RustEngine engine = make_driven_engine(source);
  const std::array<float, 16> guess16 = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  const std::vector<float> src = flatten(*source);

  // Reference: align the same Rust engine via the bare engine FFI + read its result (the slimmed
  // direct Rust engine exposes no C++ getResult wrapper). run_align below re-aligns the same
  // engine from the same guess, so the align outputs match exactly (deterministic).
  autoware_ndt_scan_matcher_rs_ndt_engine_align(
    engine.raw(), guess16.data(), src.data(), source->size());
  std::array<float, 16> ref_pose{};
  std::int32_t ref_iter = 0;
  float ref_tp = 0.0F;
  float ref_nvl = 0.0F;
  AwNdtAlignOutput ref_out{};
  ref_out.pose = ref_pose.data();
  ref_out.iteration_num = &ref_iter;
  ref_out.transform_probability = &ref_tp;
  ref_out.nearest_voxel_likelihood = &ref_nvl;
  autoware_ndt_scan_matcher_rs_ndt_engine_get_result(engine.raw(), &ref_out);

  // Orchestrator: re-align the same engine from the same guess via run_align.
  const AwAlignParams params{kTp, 0.0, 0.0};
  AwAlignOutcome outcome{};
  autoware_ndt_scan_matcher_rs_node_run_align(
    engine.raw(), guess16.data(), src.data(), source->size(), &params, &outcome);

  EXPECT_EQ(outcome.iteration_num, ref_iter);
  EXPECT_EQ(
    outcome.max_iterations, autoware_ndt_scan_matcher_rs_ndt_engine_max_iterations(engine.raw()));
  EXPECT_FLOAT_EQ(outcome.transform_probability, ref_tp);
  EXPECT_FLOAT_EQ(outcome.nearest_voxel_likelihood, ref_nvl);
  for (int i = 0; i < 16; ++i) {
    EXPECT_FLOAT_EQ(outcome.pose[i], ref_pose[i]) << "pose[" << i << "]";
  }

  // Verdict self-consistency (the formula is differential-tested against C++ in
  // test_convergence_verdict).
  EXPECT_TRUE(outcome.verdict.valid_param_type);
  EXPECT_GE(outcome.oscillation_num, 0);
  const bool expect_converged = (outcome.verdict.is_ok_iteration_num ||
                                 outcome.verdict.is_local_optimal_solution_oscillation) &&
                                outcome.verdict.is_ok_score;
  EXPECT_EQ(outcome.verdict.is_converged, expect_converged);
  // With zero thresholds and a good alignment, the score gate passes and it converges.
  EXPECT_TRUE(outcome.verdict.is_ok_score);
  EXPECT_TRUE(outcome.verdict.is_converged);
}

// An unknown converged_param_type yields valid_param_type == false (C++ would emit ERROR + abort).
TEST(NodeRunAlign, UnknownParamTypeIsInvalid)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source;
  RustEngine engine = make_driven_engine(source);
  const std::array<float, 16> guess16 = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  const std::vector<float> src = flatten(*source);
  const AwAlignParams params{2 /* unknown */, 0.0, 0.0};
  AwAlignOutcome outcome{};
  autoware_ndt_scan_matcher_rs_node_run_align(
    engine.raw(), guess16.data(), src.data(), source->size(), &params, &outcome);
  EXPECT_FALSE(outcome.verdict.valid_param_type);
}

// A null engine is a no-op (no crash, no write).
TEST(NodeRunAlign, NullEngineIsNoop)  // NOLINT
{
  const std::array<float, 16> guess16 = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  const std::array<float, 3> pt = {0.0F, 0.0F, 0.0F};
  const AwAlignParams params{kTp, 0.0, 0.0};
  AwAlignOutcome outcome{};
  outcome.iteration_num = 123;
  autoware_ndt_scan_matcher_rs_node_run_align(
    nullptr, guess16.data(), pt.data(), 1, &params, &outcome);
  EXPECT_EQ(outcome.iteration_num, 123);
}

TEST(NodeRunAlign, AlignServiceSearchWritesParticlesAndBestPose)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source;
  RustEngine engine = make_driven_engine(source);
  const std::vector<float> flat = flatten(*source);
  constexpr std::size_t kParticles = 6U;
  std::vector<AwPose> initial_poses(kParticles);
  std::vector<AwPose> result_poses(kParticles);
  std::vector<double> scores(kParticles);
  std::vector<std::int32_t> iterations(kParticles);
  AwNdtAlignServiceSearchInput input =
    make_search_input(*source, flat, static_cast<int64_t>(kParticles), 3);
  AwNdtAlignServiceSearchOutput output =
    make_search_output(initial_poses, result_poses, scores, iterations);

  const std::int32_t status =
    autoware_ndt_scan_matcher_rs_node_run_align_service_search(engine.raw(), &input, &output);

  EXPECT_EQ(status, NDT_ALIGN_SERVICE_STATUS_ALIGNED);
  EXPECT_EQ(output.status, NDT_ALIGN_SERVICE_STATUS_ALIGNED);
  EXPECT_EQ(output.valid, 1U);
  EXPECT_EQ(output.particles_len, kParticles);
  EXPECT_EQ(output.particles_requested, static_cast<int64_t>(kParticles));
  EXPECT_EQ(output.particles_evaluated, static_cast<int64_t>(kParticles));
  EXPECT_EQ(output.cloud_publish_count, static_cast<int64_t>(kParticles));
  EXPECT_GE(output.marker_publish_count, 1);
  EXPECT_TRUE(std::isfinite(output.best_score));
  EXPECT_GE(output.best_iteration, 0);
  for (std::size_t i = 0; i < kParticles; ++i) {
    EXPECT_TRUE(std::isfinite(initial_poses[i].position[0]));
    EXPECT_TRUE(std::isfinite(result_poses[i].position[0]));
    EXPECT_TRUE(std::isfinite(scores[i]));
    EXPECT_GE(iterations[i], 0);
  }
}

TEST(NodeRunAlign, AlignServiceSearchRejectsTooSmallOutputCapacity)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source;
  RustEngine engine = make_driven_engine(source);
  const std::vector<float> flat = flatten(*source);
  constexpr std::size_t kCapacity = 2U;
  std::vector<AwPose> initial_poses(kCapacity);
  std::vector<AwPose> result_poses(kCapacity);
  std::vector<double> scores(kCapacity, 42.0);
  std::vector<std::int32_t> iterations(kCapacity, 7);
  AwNdtAlignServiceSearchInput input = make_search_input(*source, flat, 3, 1);
  AwNdtAlignServiceSearchOutput output =
    make_search_output(initial_poses, result_poses, scores, iterations);

  const std::int32_t status =
    autoware_ndt_scan_matcher_rs_node_run_align_service_search(engine.raw(), &input, &output);

  EXPECT_EQ(status, NDT_ALIGN_SERVICE_STATUS_INVALID_INPUT);
  EXPECT_EQ(output.status, NDT_ALIGN_SERVICE_STATUS_INVALID_INPUT);
  EXPECT_EQ(output.valid, 0U);
  EXPECT_EQ(output.particles_len, 0U);
  EXPECT_EQ(scores[0], 42.0);
  EXPECT_EQ(iterations[0], 7);
}
