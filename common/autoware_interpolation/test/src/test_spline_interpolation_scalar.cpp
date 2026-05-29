// Copyright 2026 Tier IV, Inc.
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

#include "autoware/interpolation/spline_interpolation.hpp"

#include <gtest/gtest.h>

#include <vector>

using autoware::interpolation::SplineInterpolation;

// The scalar overloads must return exactly what the vector overloads return for the
// same query key. They are the per-knot allocation-free evaluation seam that the
// SplineInterpolationPoints2d per-knot loops route through.
TEST(spline_interpolation_scalar, value_matches_vector_overload)
{
  const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
  const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
  const std::vector<double> query_keys{-1.5, 0.0, 8.0, 12.0, 18.0, 20.0};

  const SplineInterpolation s(base_keys, base_values);

  const auto vector_values = s.getSplineInterpolatedValues(query_keys);
  for (size_t i = 0; i < query_keys.size(); ++i) {
    EXPECT_DOUBLE_EQ(s.getSplineInterpolatedValue(query_keys.at(i)), vector_values.at(i));
  }
}

TEST(spline_interpolation_scalar, diff_value_matches_vector_overload)
{
  const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
  const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
  const std::vector<double> query_keys{-1.5, 0.0, 8.0, 12.0, 18.0, 20.0};

  const SplineInterpolation s(base_keys, base_values);

  const auto vector_values = s.getSplineInterpolatedDiffValues(query_keys);
  for (size_t i = 0; i < query_keys.size(); ++i) {
    EXPECT_DOUBLE_EQ(s.getSplineInterpolatedDiffValue(query_keys.at(i)), vector_values.at(i));
  }
}

TEST(spline_interpolation_scalar, quad_diff_value_matches_vector_overload)
{
  const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
  const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
  const std::vector<double> query_keys{-1.5, 0.0, 8.0, 12.0, 18.0, 20.0};

  const SplineInterpolation s(base_keys, base_values);

  const auto vector_values = s.getSplineInterpolatedQuadDiffValues(query_keys);
  for (size_t i = 0; i < query_keys.size(); ++i) {
    EXPECT_DOUBLE_EQ(s.getSplineInterpolatedQuadDiffValue(query_keys.at(i)), vector_values.at(i));
  }
}

// Single-segment (n == 2) evaluation: the scalar overload clamps the segment index via
// get_index() and evaluates the cubic in place, matching the vector overload's per-key math.
TEST(spline_interpolation_scalar, value_at_two_knot_segment)
{
  const std::vector<double> base_keys{0.0, 2.0};
  const std::vector<double> base_values{0.0, 4.0};
  const SplineInterpolation s(base_keys, base_values);

  // n == 2 path: c_[0] = (4-0)/(2-0) = 2, d_[0] = 0 -> value = 2 * dx
  EXPECT_DOUBLE_EQ(s.getSplineInterpolatedValue(0.0), 0.0);
  EXPECT_DOUBLE_EQ(s.getSplineInterpolatedValue(1.0), 2.0);
  EXPECT_DOUBLE_EQ(s.getSplineInterpolatedValue(2.0), 4.0);
  // first derivative is the constant slope, second derivative is zero on this segment
  EXPECT_DOUBLE_EQ(s.getSplineInterpolatedDiffValue(1.0), 2.0);
  EXPECT_DOUBLE_EQ(s.getSplineInterpolatedQuadDiffValue(1.0), 0.0);
}
