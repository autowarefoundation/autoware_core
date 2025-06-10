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

#include "autoware/trajectory/interpolator/akima_spline.hpp"
#include "autoware/trajectory/interpolator/cubic_spline.hpp"
#include "autoware/trajectory/interpolator/lane_ids_interpolator.hpp"
#include "autoware/trajectory/interpolator/linear.hpp"
#include "autoware/trajectory/interpolator/nearest_neighbor.hpp"
#include "autoware/trajectory/interpolator/spherical_linear.hpp"
#include "autoware/trajectory/interpolator/stairstep.hpp"

#include <geometry_msgs/msg/quaternion.hpp>

#include <gtest/gtest.h>

#include <optional>
#include <random>
#include <vector>

template <class Interpolator>
class TestInterpolator : public ::testing::Test
{
public:
  std::optional<Interpolator> interpolator;
  std::vector<double> bases;
  std::vector<double> values;

  void SetUp() override
  {
    // generate random values -1 to 1
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1, 1);
    bases.resize(10);
    values.resize(10);
    for (size_t i = 0; i < bases.size(); ++i) {
      bases[i] = static_cast<double>(i);
      values[i] = dis(gen);
    }
  }
};

using Interpolators = testing::Types<
  autoware::experimental::trajectory::interpolator::CubicSpline,
  autoware::experimental::trajectory::interpolator::AkimaSpline,
  autoware::experimental::trajectory::interpolator::Linear,
  autoware::experimental::trajectory::interpolator::NearestNeighbor<double>,
  autoware::experimental::trajectory::interpolator::Stairstep<double>>;

TYPED_TEST_SUITE(TestInterpolator, Interpolators, );

TYPED_TEST(TestInterpolator, compute)
{
  this->interpolator =
    typename TypeParam::Builder().set_bases(this->bases).set_values(this->values).build().value();
  for (size_t i = 0; i < this->bases.size(); ++i) {
    EXPECT_NEAR(this->values[i], this->interpolator->compute(this->bases[i]), 1e-6);
  }
}

// Instantiate test cases for all interpolators
template class TestInterpolator<autoware::experimental::trajectory::interpolator::CubicSpline>;
template class TestInterpolator<autoware::experimental::trajectory::interpolator::AkimaSpline>;
template class TestInterpolator<autoware::experimental::trajectory::interpolator::Linear>;
template class TestInterpolator<
  autoware::experimental::trajectory::interpolator::NearestNeighbor<double>>;
template class TestInterpolator<
  autoware::experimental::trajectory::interpolator::Stairstep<double>>;

/*
 * Test LaneIds interpolator
 */

TEST(TestLaneIdsInterpolator, compute)
{
  using autoware::experimental::trajectory::interpolator::LaneIdsInterpolator;

  // Test data from user's example
  std::vector<double> bases = {0.0, 1.0, 3.0, 3.5, 4.0, 6.0, 7.0, 8.0, 9.0};
  std::vector<std::vector<int64_t>> values = {{1}, {1}, {1}, {1}, {1, 2}, {2}, {2}, {2}, {2}};

  auto interpolator = LaneIdsInterpolator::Builder().set_bases(bases).set_values(values).build();

  if (!interpolator) {
    FAIL();
  }

  // Test domain knowledge: at base=3.2, should return [1] (prefers single lane IDs)
  auto result_left = interpolator->compute(3.2);
  EXPECT_EQ(result_left.size(), 1);
  EXPECT_EQ(result_left[0], 1);

  auto result_right = interpolator->compute(3.75);
  EXPECT_EQ(result_right.size(), 1);
  EXPECT_EQ(result_right[0], 1);

  auto result_right_after_boundary = interpolator->compute(5.0);
  EXPECT_EQ(result_right_after_boundary.size(), 1);
  EXPECT_EQ(result_right_after_boundary[0], 2);
  // Test exact boundary point
  auto boundary_result = interpolator->compute(4.0);
  EXPECT_EQ(boundary_result.size(), 2);
  EXPECT_EQ(boundary_result[0], 1);
  EXPECT_EQ(boundary_result[1], 2);

  // Test that same input generates same output
  for (size_t i = 0; i < bases.size(); ++i) {
    auto result = interpolator->compute(bases[i]);
    EXPECT_EQ(result, values[i]);
  }
}

/*
 * Test SphericalLinear interpolator
 */

geometry_msgs::msg::Quaternion create_quaternion(double w, double x, double y, double z)
{
  geometry_msgs::msg::Quaternion q;
  q.w = w;
  q.x = x;
  q.y = y;
  q.z = z;
  return q;
}

TEST(TestSphericalLinearInterpolator, compute)
{
  using autoware::experimental::trajectory::interpolator::SphericalLinear;

  std::vector<double> bases = {0.0, 1.0};
  std::vector<geometry_msgs::msg::Quaternion> quaternions = {
    create_quaternion(1.0, 0.0, 0.0, 0.0), create_quaternion(0.0, 1.0, 0.0, 0.0)};

  auto interpolator = autoware::experimental::trajectory::interpolator::SphericalLinear::Builder()
                        .set_bases(bases)
                        .set_values(quaternions)
                        .build();

  if (!interpolator) {
    FAIL();
  }

  double s = 0.5;
  geometry_msgs::msg::Quaternion result = interpolator->compute(s);

  // Expected values (from SLERP calculation)
  double expected_w = std::sqrt(2.0) / 2.0;
  double expected_x = std::sqrt(2.0) / 2.0;
  double expected_y = 0.0;
  double expected_z = 0.0;

  EXPECT_NEAR(result.w, expected_w, 1e-6);
  EXPECT_NEAR(result.x, expected_x, 1e-6);
  EXPECT_NEAR(result.y, expected_y, 1e-6);
  EXPECT_NEAR(result.z, expected_z, 1e-6);

  const std::vector<double> ss = {0.5, 0.75};
  const auto results = interpolator->compute(ss);
  EXPECT_NEAR(results[0].w, expected_w, 1e-6);
  EXPECT_NEAR(results[0].x, expected_x, 1e-6);
  EXPECT_NEAR(results[0].y, expected_y, 1e-6);
  EXPECT_NEAR(results[0].z, expected_z, 1e-6);
}
