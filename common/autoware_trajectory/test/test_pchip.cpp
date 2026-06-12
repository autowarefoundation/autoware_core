// Copyright 2026 TIER IV, Inc.
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

// cspell:ignore Fritsch

#include "autoware/trajectory/interpolator/pchip.hpp"

#include <gtest/gtest.h>

#include <vector>

// Characterization tests for the PCHIP (Piecewise Cubic Hermite Interpolating
// Polynomial) interpolator. The expected values below were derived from the
// algorithm exactly as implemented in src/interpolator/pchip.cpp and
// independently cross-checked against scipy.interpolate.PchipInterpolator,
// which uses the same Fritsch-Carlson construction. They pin the current
// observable behavior of the shipped interpolator: interpolation through the
// data points, the harmonic-mean interior knot derivatives, the one-sided
// endpoint-derivative estimate (with sign guard and 3*delta limiting), the
// monotonicity / no-overshoot shape preservation, the n == 2 linear special
// case, and the build-time rejection / duplicate-removal branches.

namespace
{
using autoware::experimental::trajectory::interpolator::Pchip;

constexpr double k_tol = 1e-12;
}  // namespace

// The interpolated curve must pass through every data point exactly, and the interior
// knot derivatives must equal the Fritsch-Carlson weighted-harmonic-mean
// slopes. Dataset: y = x^2 sampled on [0, 4].
TEST(TestPchip, InterpolatesThroughDataPointsAndKnotDerivatives)
{
  const std::vector<double> bases = {0.0, 1.0, 2.0, 3.0, 4.0};
  const std::vector<double> values = {0.0, 1.0, 4.0, 9.0, 16.0};

  const auto result = Pchip::Builder{}.set_bases(bases).set_values(values).build();
  ASSERT_TRUE(result.has_value());
  const auto & pchip = result.value();

  // Passes through the data points.
  for (size_t i = 0; i < bases.size(); ++i) {
    EXPECT_NEAR(pchip.compute(bases[i]), values[i], k_tol);
  }

  // Interior knot first derivatives (harmonic-mean slopes) and the
  // sign-guarded endpoint derivative at x = 0 (clamped to 0).
  EXPECT_NEAR(pchip.compute_first_derivative(0.0), 0.0, k_tol);
  EXPECT_NEAR(pchip.compute_first_derivative(1.0), 1.5, k_tol);
  EXPECT_NEAR(pchip.compute_first_derivative(2.0), 3.75, k_tol);
  EXPECT_NEAR(pchip.compute_first_derivative(3.0), 5.833333333333334, k_tol);
  EXPECT_NEAR(pchip.compute_first_derivative(4.0), 8.0, k_tol);

  // Mid-interval values.
  EXPECT_NEAR(pchip.compute(0.5), 0.3125, k_tol);
  EXPECT_NEAR(pchip.compute(1.5), 2.21875, k_tol);
  EXPECT_NEAR(pchip.compute(2.5), 6.239583333333333, k_tol);
  EXPECT_NEAR(pchip.compute(3.5), 12.229166666666666, k_tol);
}

// On strictly increasing data the interpolated curve must stay monotone increasing
// between knots (no spurious dips).
TEST(TestPchip, PreservesMonotonicity)
{
  const std::vector<double> bases = {0.0, 1.0, 2.0, 3.0, 4.0};
  const std::vector<double> values = {0.0, 1.0, 4.0, 9.0, 16.0};

  const auto result = Pchip::Builder{}.set_bases(bases).set_values(values).build();
  ASSERT_TRUE(result.has_value());
  const auto & pchip = result.value();

  // Small absolute tolerance to absorb floating-point roundoff (tiny negative
  // steps even when the interpolated curve is monotone in theory). It is many orders
  // of magnitude below the data scale (values up to 16, secant slopes up to 8
  // over a step of 0.01), so a genuine monotonicity violation is still caught.
  constexpr double k_monotonicity_tol = 1e-9;
  double previous = pchip.compute(0.0);
  for (int i = 1; i <= 400; ++i) {
    const double s = 4.0 * static_cast<double>(i) / 400.0;
    const double current = pchip.compute(s);
    EXPECT_GE(current, previous - k_monotonicity_tol) << "monotonicity violated at s = " << s;
    previous = current;
  }
}

// At a local maximum the interior derivative must be zeroed (the
// delta_prev > 0 && delta_next < 0 sign-change branch), and the interpolated curve
// must not overshoot the data (shape preservation). Dataset has a peak at
// x = 2.
TEST(TestPchip, ZeroesDerivativeAtLocalExtremumAndDoesNotOvershoot)
{
  const std::vector<double> bases = {0.0, 1.0, 2.0, 3.0, 4.0};
  const std::vector<double> values = {0.0, 2.0, 3.0, 2.0, 0.0};

  const auto result = Pchip::Builder{}.set_bases(bases).set_values(values).build();
  ASSERT_TRUE(result.has_value());
  const auto & pchip = result.value();

  // Local-extremum derivative is exactly zero.
  EXPECT_NEAR(pchip.compute_first_derivative(2.0), 0.0, k_tol);

  // Endpoint and neighbouring interior knot derivatives.
  EXPECT_NEAR(pchip.compute_first_derivative(0.0), 2.5, k_tol);
  EXPECT_NEAR(pchip.compute_first_derivative(1.0), 1.3333333333333333, k_tol);
  EXPECT_NEAR(pchip.compute_first_derivative(3.0), -1.3333333333333333, k_tol);
  EXPECT_NEAR(pchip.compute_first_derivative(4.0), -2.5, k_tol);

  // Pinned mid-interval values.
  EXPECT_NEAR(pchip.compute(0.5), 1.1458333333333333, k_tol);
  EXPECT_NEAR(pchip.compute(1.5), 2.6666666666666665, k_tol);
  EXPECT_NEAR(pchip.compute(2.5), 2.6666666666666665, k_tol);
  EXPECT_NEAR(pchip.compute(3.5), 1.1458333333333333, k_tol);

  // No overshoot above the data maximum (3.0) over the whole range.
  for (int i = 0; i <= 400; ++i) {
    const double s = 4.0 * static_cast<double>(i) / 400.0;
    EXPECT_LE(pchip.compute(s), 3.0 + k_tol) << "overshoot at s = " << s;
    EXPECT_GE(pchip.compute(s), 0.0 - k_tol) << "undershoot at s = " << s;
  }
}

// The endpoint derivative uses a one-sided estimate that is limited to
// 3 * delta when the two end segments have opposite slope signs and the
// estimate is too steep. Here delta0 = 1, delta1 = -10 gives a raw estimate of
// 6.5, which must be clamped to 3 * delta0 = 3.0.
TEST(TestPchip, LimitsEndpointDerivative)
{
  const std::vector<double> bases = {0.0, 1.0, 2.0, 3.0};
  const std::vector<double> values = {0.0, 1.0, -9.0, -9.5};

  const auto result = Pchip::Builder{}.set_bases(bases).set_values(values).build();
  ASSERT_TRUE(result.has_value());
  const auto & pchip = result.value();

  EXPECT_NEAR(pchip.compute_first_derivative(0.0), 3.0, k_tol);
}

// With exactly two points PCHIP degenerates to the straight line through them:
// both knot derivatives equal the secant slope, so the cubic coefficients
// c and d vanish.
TEST(TestPchip, TwoPointsAreLinear)
{
  const std::vector<double> bases = {0.0, 2.0};
  const std::vector<double> values = {1.0, 5.0};

  const auto result = Pchip::Builder{}.set_bases(bases).set_values(values).build();
  ASSERT_TRUE(result.has_value());
  const auto & pchip = result.value();

  EXPECT_NEAR(pchip.compute(0.0), 1.0, k_tol);
  EXPECT_NEAR(pchip.compute(1.0), 3.0, k_tol);
  EXPECT_NEAR(pchip.compute(2.0), 5.0, k_tol);

  // Constant slope, zero curvature.
  EXPECT_NEAR(pchip.compute_first_derivative(0.5), 2.0, k_tol);
  EXPECT_NEAR(pchip.compute_first_derivative(1.5), 2.0, k_tol);
  EXPECT_NEAR(pchip.compute_second_derivative(1.0), 0.0, k_tol);
}

// Building with fewer than two distinct points must fail.
TEST(TestPchip, RejectsTooFewPoints)
{
  const std::vector<double> bases = {0.0};
  const std::vector<double> values = {1.0};

  const auto result = Pchip::Builder{}.set_bases(bases).set_values(values).build();
  EXPECT_FALSE(result.has_value());
}

// Out-of-order bases are silently filtered rather than rejected: the
// duplicate-removal pass keeps a base only when it exceeds the previous kept
// base by more than epsilon, so a point that steps backwards is dropped. With
// bases {0, 2, 1} the trailing 1.0 is discarded, leaving the strictly
// increasing subsequence {0, 2}, and the build succeeds. This pins the current
// (somewhat surprising) behavior that the strictly-increasing guard is
// subsumed by duplicate removal.
TEST(TestPchip, DropsOutOfOrderBasesAndStillBuilds)
{
  const std::vector<double> bases = {0.0, 2.0, 1.0};
  const std::vector<double> values = {0.0, 4.0, 999.0};

  const auto result = Pchip::Builder{}.set_bases(bases).set_values(values).build();
  ASSERT_TRUE(result.has_value());
  const auto & pchip = result.value();

  // Only the {0, 2} subsequence survives, so the fit is the line through
  // (0, 0) and (2, 4); the dropped backward point (value 999.0) has no effect.
  EXPECT_NEAR(pchip.compute(0.0), 0.0, k_tol);
  EXPECT_NEAR(pchip.compute(2.0), 4.0, k_tol);
  EXPECT_NEAR(pchip.compute(1.0), 2.0, k_tol);
}

// Strictly decreasing bases collapse under duplicate removal to a single kept
// point (every later base is below the first), which is below the minimum of
// two, so the build is rejected.
TEST(TestPchip, RejectsStrictlyDecreasingBases)
{
  const std::vector<double> bases = {3.0, 2.0, 1.0};
  const std::vector<double> values = {0.0, 1.0, 2.0};

  const auto result = Pchip::Builder{}.set_bases(bases).set_values(values).build();
  EXPECT_FALSE(result.has_value());
}

// Near-duplicate bases (difference below epsilon) are dropped before fitting.
// Passing a large epsilon collapses the middle point, leaving the two distinct
// endpoints, which then interpolate as a straight line.
TEST(TestPchip, RemovesDuplicateBases)
{
  const std::vector<double> bases = {0.0, 1.0, 2.0};
  const std::vector<double> values = {0.0, 100.0, 4.0};

  // epsilon = 1.5 collapses base 1.0 into base 0.0 (1.0 - 0.0 <= 1.5),
  // keeps base 2.0 (2.0 - 0.0 > 1.5). Result is the line from (0,0) to (2,4).
  const auto result = Pchip::Builder{}.set_bases(bases).set_values(values).build(1.5);
  ASSERT_TRUE(result.has_value());
  const auto & pchip = result.value();

  EXPECT_NEAR(pchip.compute(0.0), 0.0, k_tol);
  EXPECT_NEAR(pchip.compute(2.0), 4.0, k_tol);
  // The dropped middle value (100.0) has no influence: pure line, slope 2.
  EXPECT_NEAR(pchip.compute(1.0), 2.0, k_tol);
}

// After collapsing duplicates the remaining distinct points may fall below the
// minimum, in which case the build must fail.
TEST(TestPchip, RejectsWhenDuplicateRemovalLeavesTooFewPoints)
{
  const std::vector<double> bases = {0.0, 0.5, 1.0};
  const std::vector<double> values = {0.0, 1.0, 2.0};

  // epsilon = 2.0 collapses every base into the first one, leaving a single
  // distinct point, which is below the minimum of 2.
  const auto result = Pchip::Builder{}.set_bases(bases).set_values(values).build(2.0);
  EXPECT_FALSE(result.has_value());
}
