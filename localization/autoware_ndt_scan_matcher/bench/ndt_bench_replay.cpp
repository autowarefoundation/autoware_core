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

// L3 offline replay benchmark. A single executable drives BOTH NDT engines
// on identical inputs: the C++ `MultiGridNormalDistributionsTransform` and the Rust port over the C
// ABI. The target map + kd-tree is built ONCE per engine; only the repeated `align` loop is timed
// (steady_clock), so this measures the align kernel, not map construction. Both engines see the same
// synthetic geometry from the same buffers, so the comparison is apples-to-apples by construction.
//
// Output: a JSON file with per-align latency samples (ms) + iteration_num + run metadata, consumed by
// bench/gen_report.py to render a self-contained HTML report.
//
// Usage: ndt_bench_replay [iters=200] [warmup=20] [out.json=ndt_bench.json] [interval=0.2]
//   `interval` sets the synthetic point spacing → point count (default 0.2 m ⇒ ~30,603 pts).

#include <autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h>

#include "autoware_ndt_scan_matcher_rs.h"

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace
{
using Ndt = pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
using Clock = std::chrono::steady_clock;

// Three orthogonal planes over [0, length] at `interval` spacing — the standard-sequence fixture
// geometry. Fills both a PCL cloud (C++ engine) and the flat xyz buffer (Rust FFI) from identical
// points.
void make_half_cubic(
  float length, float interval, pcl::PointCloud<pcl::PointXYZ> & cloud, std::vector<float> & flat)
{
  const int n = static_cast<int>(length / interval) + 1;
  const auto add = [&](float x, float y, float z) {
    pcl::PointXYZ p;
    p.x = x;
    p.y = y;
    p.z = z;
    cloud.push_back(p);
    flat.push_back(x);
    flat.push_back(y);
    flat.push_back(z);
  };
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      const float u = interval * static_cast<float>(j);
      const float v = interval * static_cast<float>(i);
      add(u, v, 0.0F);
      add(0.0F, u, v);
      add(u, 0.0F, v);
    }
  }
  cloud.is_dense = true;
}

double ms_since(Clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

void write_samples(FILE * f, const std::vector<double> & xs)
{
  std::fputc('[', f);
  for (size_t i = 0; i < xs.size(); ++i) {
    std::fprintf(f, "%s%.6f", (i == 0 ? "" : ","), xs[i]);
  }
  std::fputc(']', f);
}
}  // namespace

int main(int argc, char ** argv)
{
  const int iters = (argc > 1) ? std::atoi(argv[1]) : 200;
  const int warmup = (argc > 2) ? std::atoi(argv[2]) : 20;
  const std::string out_path = (argc > 3) ? argv[3] : "ndt_bench.json";
  const float interval = (argc > 4) ? static_cast<float>(std::atof(argv[4])) : 0.2F;
  const float length = 20.0F;

  // Target map + source = target translated by a known offset (identity guess should recover it).
  auto target = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::vector<float> target_flat;
  make_half_cubic(length, interval, *target, target_flat);

  const std::array<float, 3> offset = {0.2F, -0.15F, 0.1F};
  auto source = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::vector<float> source_flat;
  source_flat.reserve(target_flat.size());
  for (const auto & p : *target) {
    pcl::PointXYZ q;
    q.x = p.x + offset[0];
    q.y = p.y + offset[1];
    q.z = p.z + offset[2];
    source->push_back(q);
    source_flat.push_back(q.x);
    source_flat.push_back(q.y);
    source_flat.push_back(q.z);
  }
  const size_t n_pts = target->size();

  pclomp::NdtParams params{};
  params.trans_epsilon = 0.01;
  params.step_size = 0.1;
  params.resolution = 2.0F;
  params.max_iterations = 30;
  params.search_method = pclomp::KDTREE;
  params.num_threads = 1;  // serial-vs-serial baseline
  params.regularization_scale_factor = 0.0F;
  params.use_line_search = false;

  const Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  std::array<float, 16> guess16{};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      guess16[(r * 4) + c] = guess(r, c);
    }
  }

  // ---- C++ engine: build map once, time the align loop ----
  Ndt ndt;
  ndt.setParams(params);
  ndt.setInputTarget(target);
  auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int i = 0; i < warmup; ++i) {
    ndt.align(*aligned, guess, source);
  }
  std::vector<double> cpp_ms;
  cpp_ms.reserve(static_cast<size_t>(iters));
  for (int i = 0; i < iters; ++i) {
    const auto t0 = Clock::now();
    ndt.align(*aligned, guess, source);
    cpp_ms.push_back(ms_since(t0));
  }
  const int cpp_iter = ndt.getResult().iteration_num;

  // ---- Rust engine (over the C ABI): build map once, time the align loop ----
  struct AwNdtEngine * eng = autoware_ndt_scan_matcher_rs_ndt_engine_new(2.0, 6, 0.01);
  autoware_ndt_scan_matcher_rs_ndt_engine_set_params(eng, 0.01, 0.1, 2.0, 30, 0.55, 1);
  autoware_ndt_scan_matcher_rs_ndt_engine_add_target(eng, target_flat.data(), n_pts, 0);
  autoware_ndt_scan_matcher_rs_ndt_engine_create_kdtree(eng);
  for (int i = 0; i < warmup; ++i) {
    autoware_ndt_scan_matcher_rs_ndt_engine_align(eng, guess16.data(), source_flat.data(), n_pts);
  }
  std::vector<double> rs_ms;
  rs_ms.reserve(static_cast<size_t>(iters));
  for (int i = 0; i < iters; ++i) {
    const auto t0 = Clock::now();
    autoware_ndt_scan_matcher_rs_ndt_engine_align(eng, guess16.data(), source_flat.data(), n_pts);
    rs_ms.push_back(ms_since(t0));
  }
  int32_t rs_iter = 0;
  AwNdtAlignOutput rout{};
  rout.iteration_num = &rs_iter;
  autoware_ndt_scan_matcher_rs_ndt_engine_get_result(eng, &rout);
  autoware_ndt_scan_matcher_rs_ndt_engine_free(eng);

  // ---- emit JSON ----
  FILE * f = std::fopen(out_path.c_str(), "w");
  if (f == nullptr) {
    std::fprintf(stderr, "ndt_bench_replay: cannot open %s for writing\n", out_path.c_str());
    return 1;
  }
  std::fprintf(f, "{\n");
  std::fprintf(f, "  \"benchmark\": \"L3 offline align replay (synthetic half-cubic)\",\n");
  std::fprintf(f, "  \"meta\": {\n");
  std::fprintf(f, "    \"n_points\": %zu,\n", n_pts);
  std::fprintf(f, "    \"iters\": %d,\n", iters);
  std::fprintf(f, "    \"warmup\": %d,\n", warmup);
  std::fprintf(f, "    \"resolution\": 2.0,\n");
  std::fprintf(f, "    \"max_iterations\": 30,\n");
  std::fprintf(f, "    \"num_threads\": 1,\n");
  std::fprintf(f, "    \"clock\": \"steady_clock\",\n");
  std::fprintf(f, "    \"unit\": \"ms\",\n");
  std::fprintf(f, "    \"note\": \"align loop only; map+kdtree built once per engine\"\n");
  std::fprintf(f, "  },\n");
  std::fprintf(f, "  \"engines\": {\n");
  std::fprintf(f, "    \"cpp\": { \"label\": \"C++ (multigrid_ndt_omp)\", \"iteration_num\": %d, \"samples_ms\": ", cpp_iter);
  write_samples(f, cpp_ms);
  std::fprintf(f, " },\n");
  std::fprintf(f, "    \"rust\": { \"label\": \"Rust (autoware_ndt_scan_matcher_rs)\", \"iteration_num\": %d, \"samples_ms\": ", rs_iter);
  write_samples(f, rs_ms);
  std::fprintf(f, " }\n");
  std::fprintf(f, "  }\n");
  std::fprintf(f, "}\n");
  std::fclose(f);

  std::fprintf(
    stderr, "ndt_bench_replay: %zu pts, %d aligns/engine (+%d warmup) -> %s\n", n_pts, iters, warmup,
    out_path.c_str());
  return 0;
}
