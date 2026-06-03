// Copyright 2024 Tier IV, Inc.
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

#include "grid.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

// Test fixture befriended by autoware::ground_filter::Grid so the otherwise
// private index math (getRadialIdx / getAzimuthGridIdx / getGridIdx /
// getRadialAzimuthIdxFromCellIdx) can be exercised directly. These are
// characterization tests pinning the current behavior of the polar grid.
class GridTest : public ::testing::Test
{
protected:
  // Virtual-lidar origin matching the default node configuration.
  static constexpr float kOriginX = 1.4f;
  static constexpr float kOriginY = 0.0f;
  static constexpr float kOriginZ = 1.9f;

  void SetUp() override
  {
    grid_ = std::make_unique<autoware::ground_filter::Grid>(kOriginX, kOriginY, kOriginZ);
    // grid_dist_size, grid_azimuth_size, grid_linearity_switch_radius
    grid_->initialize(0.5f, 0.01f, 20.0f);
  }

  // Thin accessors that forward to Grid's private members via friendship.
  int getRadialIdx(const float radius) const { return grid_->getRadialIdx(radius); }

  int getAzimuthGridIdx(const int radial_idx, const float azimuth) const
  {
    return grid_->getAzimuthGridIdx(radial_idx, azimuth);
  }

  int getGridIdx(const float radius, const float azimuth) const
  {
    return grid_->getGridIdx(radius, azimuth);
  }

  int getGridIdx(const int radial_idx, const int azimuth_idx) const
  {
    return grid_->getGridIdx(radial_idx, azimuth_idx);
  }

  void getRadialAzimuthIdxFromCellIdx(const int cell_id, int & radial_idx, int & azimuth_idx) const
  {
    grid_->getRadialAzimuthIdxFromCellIdx(cell_id, radial_idx, azimuth_idx);
  }

  std::unique_ptr<autoware::ground_filter::Grid> grid_;
};

namespace detail = autoware::ground_filter::detail;

// ---------------------------------------------------------------------------
// pseudoArcTan2 characterization: exact special-case branches.
// ---------------------------------------------------------------------------
TEST(GroundFilterGridTrig, PseudoArcTan2SpecialCases)
{
  // y == 0 branch
  EXPECT_FLOAT_EQ(detail::pseudoArcTan2(0.0f, 1.0f), 0.0f);
  EXPECT_FLOAT_EQ(detail::pseudoArcTan2(0.0f, -1.0f), M_PIf);

  // x == 0 branch: both axis directions stay within the [0, 2pi) convention so
  // the negative-Y axis bins consistently instead of falling out of range.
  EXPECT_FLOAT_EQ(detail::pseudoArcTan2(1.0f, 0.0f), M_PI_2f);
  EXPECT_FLOAT_EQ(detail::pseudoArcTan2(-1.0f, 0.0f), 3.0f * M_PI_2f);
}

// ---------------------------------------------------------------------------
// pseudoArcTan2 accuracy: tracks std::atan2 mapped into [0, 2pi) within the
// approximation tolerance of the 8-zone linear fit.
// ---------------------------------------------------------------------------
TEST(GroundFilterGridTrig, PseudoArcTan2TracksStdAtan2)
{
  // The piecewise-linear 8-zone approximation peaks near the middle of each
  // zone; its measured worst-case error over the full circle is ~0.071 rad
  // (~4.05 deg). 0.08 rad bounds it and pins the approximation quality.
  constexpr float kTol = 0.08f;

  for (int deg = 1; deg < 360; ++deg) {
    if (deg % 90 == 0) {
      continue;  // axis cases covered by the special-case test above
    }
    const float angle = static_cast<float>(deg) * M_PIf / 180.0f;
    const float x = std::cos(angle);
    const float y = std::sin(angle);

    float expected = std::atan2(y, x);
    if (expected < 0.0f) {
      expected += 2.0f * M_PIf;  // map std::atan2 (-pi, pi] into [0, 2pi)
    }

    const float actual = detail::pseudoArcTan2(y, x);
    EXPECT_NEAR(actual, expected, kTol) << "deg=" << deg;
  }
}

// ---------------------------------------------------------------------------
// pseudoTan characterization: zero and small-angle linearization.
// ---------------------------------------------------------------------------
TEST(GroundFilterGridTrig, PseudoTanSpecialCases)
{
  EXPECT_FLOAT_EQ(detail::pseudoTan(0.0f), 0.0f);

  // For |theta| <= 1 rad the helper uses theta / (pi/4).
  EXPECT_FLOAT_EQ(detail::pseudoTan(0.5f), 0.5f / M_PI_4f);
  EXPECT_FLOAT_EQ(detail::pseudoTan(-0.5f), -0.5f / M_PI_4f);

  // Result keeps the sign of the input across the full domain.
  EXPECT_GT(detail::pseudoTan(1.3f), 0.0f);
  EXPECT_LT(detail::pseudoTan(-1.3f), 0.0f);
}

// ---------------------------------------------------------------------------
// Grid round-trip: every cell index maps back to (radial, azimuth) that
// re-composes to the same cell index. Locks the inverse mapping.
// ---------------------------------------------------------------------------
TEST_F(GridTest, CellIdxRoundTrip)
{
  const auto grid_size = static_cast<int>(grid_->getGridSize());
  ASSERT_GT(grid_size, 0);

  for (int cell_id = 0; cell_id < grid_size; ++cell_id) {
    int radial_idx = -1;
    int azimuth_idx = -1;
    getRadialAzimuthIdxFromCellIdx(cell_id, radial_idx, azimuth_idx);

    EXPECT_GE(radial_idx, 0) << "cell_id=" << cell_id;
    EXPECT_GE(azimuth_idx, 0) << "cell_id=" << cell_id;

    const int recomposed = getGridIdx(radial_idx, azimuth_idx);
    EXPECT_EQ(recomposed, cell_id) << "cell_id=" << cell_id;
  }
}

// ---------------------------------------------------------------------------
// getRadialIdx out-of-range handling: negative radius and beyond the radial
// limit return -1; in-range radii return monotonically non-decreasing indices.
// ---------------------------------------------------------------------------
TEST_F(GridTest, RadialIdxOutOfRange)
{
  // Negative radius -> invalid.
  EXPECT_EQ(getRadialIdx(-1.0f), -1);

  // Far beyond the configured radial limit (200 m) -> invalid.
  EXPECT_EQ(getRadialIdx(1.0e4f), -1);

  // Exactly at the configured radial limit (grid_radial_limit_ == 200 m): the
  // limit is exclusive (radius < grid_radial_limit_ in both branches), so the
  // boundary value falls through to -1 just like out-of-range radii.
  EXPECT_EQ(getRadialIdx(200.0f), -1);

  // Just below the limit -> valid (non-negative) index, pinning the open
  // boundary on the in-range side.
  EXPECT_GE(getRadialIdx(199.9f), 0);

  // Within the constant-distance region radial index is floor(radius / size).
  EXPECT_EQ(getRadialIdx(0.0f), 0);
  EXPECT_EQ(getRadialIdx(0.4f), 0);  // < grid_dist_size (0.5)
  EXPECT_EQ(getRadialIdx(0.6f), 1);  // one cell out
  EXPECT_EQ(getRadialIdx(1.1f), 2);

  // Monotonic non-decreasing across increasing in-range radii.
  int prev = getRadialIdx(0.0f);
  for (float r = 0.0f; r < 150.0f; r += 0.5f) {
    const int idx = getRadialIdx(r);
    ASSERT_GE(idx, 0) << "r=" << r;
    EXPECT_GE(idx, prev) << "r=" << r;
    prev = idx;
  }
}

// ---------------------------------------------------------------------------
// getGridIdx(radius, azimuth) returns a valid in-bounds cell for in-range
// inputs and -1 when the radius is out of range, regardless of azimuth.
// ---------------------------------------------------------------------------
TEST_F(GridTest, GridIdxFromRadiusAzimuth)
{
  const auto grid_size = static_cast<int>(grid_->getGridSize());

  // In-range point: must land in a valid cell.
  const int idx = getGridIdx(5.0f, 0.5f);
  EXPECT_GE(idx, 0);
  EXPECT_LT(idx, grid_size);

  // Negative-Y axis (azimuth 3*pi/2 from pseudoArcTan2(-1, 0)) is in range and
  // must bin into a valid cell rather than being rejected as out-of-range.
  const int neg_y_axis_idx = getGridIdx(5.0f, detail::pseudoArcTan2(-1.0f, 0.0f));
  EXPECT_GE(neg_y_axis_idx, 0);
  EXPECT_LT(neg_y_axis_idx, grid_size);

  // Out-of-range radius: -1 irrespective of azimuth.
  EXPECT_EQ(getGridIdx(1.0e4f, 0.5f), -1);
  EXPECT_EQ(getGridIdx(-1.0f, 0.5f), -1);

  // Innermost ring (radial index 0) has a single azimuth grid spanning the full
  // circle (azimuth_grid_num == 1, azimuth_interval == 2*pi). Azimuth 0 lands in
  // the only bin, index 0.
  EXPECT_EQ(getAzimuthGridIdx(0, 0.0f), 0);

  // Wrap-around branch in getAzimuthGridIdx: an azimuth of exactly 2*pi yields
  // floor(2*pi / 2*pi) == 1, which equals azimuth_grid_num for this ring and is
  // reset to bin 0 by the loop-back clause. This exercises the
  // (azimuth_grid_idx == azimuth_grid_num) reset, not just the trivial bin-0 case.
  EXPECT_EQ(getAzimuthGridIdx(0, 2.0f * M_PIf), 0);
}

// ---------------------------------------------------------------------------
// Grid::addPoint on the origin's negative-Y axis: this is the external
// postcondition of the pseudoArcTan2 negative-Y-axis fix. A point with
// x_fixed == 0 and y_fixed < 0 yields azimuth 3*pi/2 (was -pi/2), so it now
// bins into a valid cell and is KEPT.
//
// Prior behavior (regression guard): pseudoArcTan2 returned -pi/2 for this axis,
// the only negative azimuth the function produced. getAzimuthGridIdx floored it
// into a negative bin, getGridIdx returned -1, and addPoint DROPPED the point
// (the destination cell stayed empty). This test pins that the point is now
// retained in a non-empty cell.
// ---------------------------------------------------------------------------
TEST_F(GridTest, AddPointOnNegativeYAxisIsKept)
{
  // Point exactly on the origin's negative-Y axis: x == origin_x_ (x_fixed == 0)
  // and y < origin_y_ (y_fixed < 0). radius == 5 m is inside the switch radius.
  const float x = kOriginX;
  const float y = kOriginY - 5.0f;
  const float z = 0.0f;
  const size_t point_idx = 7;

  const float x_fixed = x - kOriginX;
  const float y_fixed = y - kOriginY;
  const float radius = std::sqrt(x_fixed * x_fixed + y_fixed * y_fixed);
  const float azimuth = detail::pseudoArcTan2(y_fixed, x_fixed);

  // Sanity: this branch is the negative-Y axis with the fixed [0, 2*pi) value.
  ASSERT_FLOAT_EQ(x_fixed, 0.0f);
  EXPECT_FLOAT_EQ(azimuth, 3.0f * M_PI_2f);

  // The destination cell index is valid (>= 0); under the old -pi/2 value this
  // would have been -1 and the point dropped.
  const int grid_idx = getGridIdx(radius, azimuth);
  ASSERT_GE(grid_idx, 0);

  // Adding the point lands it in that cell, which becomes non-empty.
  grid_->addPoint(x, y, z, point_idx);

  auto & cell = grid_->getCell(grid_idx);
  EXPECT_FALSE(cell.isEmpty());
  ASSERT_EQ(cell.point_list_.size(), 1u);
  EXPECT_EQ(cell.point_list_.front().index, point_idx);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
