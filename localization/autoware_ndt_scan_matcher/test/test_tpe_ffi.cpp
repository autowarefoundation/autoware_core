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

#include "autoware_ndt_scan_matcher_rs.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct TpeHandle
{
  AwTpe * ptr{nullptr};
  TpeHandle() = default;
  ~TpeHandle() { autoware_ndt_scan_matcher_rs_tpe_free(ptr); }
  TpeHandle(const TpeHandle &) = delete;
  TpeHandle & operator=(const TpeHandle &) = delete;
  TpeHandle(TpeHandle && other) noexcept : ptr(other.ptr) { other.ptr = nullptr; }
  TpeHandle & operator=(TpeHandle && other) noexcept
  {
    if (this != &other) {
      autoware_ndt_scan_matcher_rs_tpe_free(ptr);
      ptr = other.ptr;
      other.ptr = nullptr;
    }
    return *this;
  }
};

TpeHandle make_tpe(const uint64_t seed = 0)
{
  constexpr std::array<double, 5> mean{0.0, 0.0, 0.0, 0.0, 0.0};
  constexpr std::array<double, 5> stddev{1.0, 1.0, 0.1, 0.1, 0.1};
  TpeHandle handle{};
  handle.ptr = autoware_ndt_scan_matcher_rs_tpe_new(
    AW_TPE_DIRECTION_MAXIMIZE, 5, mean.data(), mean.size(), stddev.data(), stddev.size(), seed);
  return handle;
}

void expect_finite_candidate(const std::array<double, 6> & input)
{
  for (const double value : input) {
    EXPECT_TRUE(std::isfinite(value));
  }
  EXPECT_GE(input[5], -kPi);
  EXPECT_LT(input[5], kPi);
}
}  // namespace

TEST(TpeFfi, ConstructorRejectsInvalidInputs)  // NOLINT
{
  constexpr std::array<double, 5> mean{0.0, 0.0, 0.0, 0.0, 0.0};
  constexpr std::array<double, 5> stddev{1.0, 1.0, 0.1, 0.1, 0.1};
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_tpe_new(
      AW_TPE_DIRECTION_MAXIMIZE, 5, mean.data(), 4, stddev.data(), stddev.size(), 0),
    nullptr);
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_tpe_new(
      99, 5, mean.data(), mean.size(), stddev.data(), stddev.size(), 0),
    nullptr);
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_tpe_new(
      AW_TPE_DIRECTION_MAXIMIZE, -1, mean.data(), mean.size(), stddev.data(), stddev.size(), 0),
    nullptr);
}

TEST(TpeFfi, GeneratesFiniteCandidates)  // NOLINT
{
  TpeHandle handle = make_tpe(11);
  ASSERT_NE(handle.ptr, nullptr);
  std::array<double, 6> input{};
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_tpe_get_next_input(handle.ptr, input.data(), input.size()),
    AW_TPE_STATUS_OK);
  expect_finite_candidate(input);
}

TEST(TpeFfi, FixedSeedReproducesSequence)  // NOLINT
{
  TpeHandle a = make_tpe(123);
  TpeHandle b = make_tpe(123);
  ASSERT_NE(a.ptr, nullptr);
  ASSERT_NE(b.ptr, nullptr);
  for (std::size_t i = 0; i < 8; ++i) {
    std::array<double, 6> ai{};
    std::array<double, 6> bi{};
    EXPECT_EQ(
      autoware_ndt_scan_matcher_rs_tpe_get_next_input(a.ptr, ai.data(), ai.size()),
      AW_TPE_STATUS_OK);
    EXPECT_EQ(
      autoware_ndt_scan_matcher_rs_tpe_get_next_input(b.ptr, bi.data(), bi.size()),
      AW_TPE_STATUS_OK);
    EXPECT_EQ(ai, bi);
  }
}

TEST(TpeFfi, AddTrialUpdatesCountsAndOptimizationStillSamples)  // NOLINT
{
  TpeHandle handle = make_tpe(44);
  ASSERT_NE(handle.ptr, nullptr);
  for (std::size_t i = 0; i < 10; ++i) {
    const double score = static_cast<double>(i);
    const std::array<double, 6> trial{score, -score, 0.0, 0.0, 0.0, 0.0};
    EXPECT_EQ(
      autoware_ndt_scan_matcher_rs_tpe_add_trial(handle.ptr, trial.data(), trial.size(), score),
      AW_TPE_STATUS_OK);
  }
  EXPECT_EQ(autoware_ndt_scan_matcher_rs_tpe_trials_len(handle.ptr), 10U);
  EXPECT_EQ(autoware_ndt_scan_matcher_rs_tpe_above_num(handle.ptr), 1U);

  std::array<double, 6> input{};
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_tpe_get_next_input(handle.ptr, input.data(), input.size()),
    AW_TPE_STATUS_OK);
  expect_finite_candidate(input);
}

TEST(TpeFfi, RejectsNullAndWrongLengthBuffers)  // NOLINT
{
  TpeHandle handle = make_tpe(55);
  ASSERT_NE(handle.ptr, nullptr);
  std::array<double, 6> input{};
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_tpe_get_next_input(nullptr, input.data(), input.size()),
    AW_TPE_STATUS_NULL_POINTER);
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_tpe_get_next_input(handle.ptr, nullptr, input.size()),
    AW_TPE_STATUS_NULL_POINTER);
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_tpe_get_next_input(handle.ptr, input.data(), 5),
    AW_TPE_STATUS_INVALID_LENGTH);
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_tpe_add_trial(handle.ptr, nullptr, input.size(), 1.0),
    AW_TPE_STATUS_NULL_POINTER);
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_tpe_add_trial(handle.ptr, input.data(), 5, 1.0),
    AW_TPE_STATUS_INVALID_LENGTH);
}
