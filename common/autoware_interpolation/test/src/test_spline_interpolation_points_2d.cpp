// Copyright 2023 TIER IV, Inc.
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
#include "autoware/interpolation/spline_interpolation_points_2d.hpp"

#include <Eigen/Core>
#include <autoware_utils_geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

constexpr double epsilon = 1e-6;

using autoware::interpolation::SplineInterpolationPoints2d;

TEST(spline_interpolation, splineYawFromPoints)
{
  using autoware_utils_geometry::create_point;

  {  // straight
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(create_point(0.0, 0.0, 0.0));
    points.push_back(create_point(1.0, 1.5, 0.0));
    points.push_back(create_point(2.0, 3.0, 0.0));
    points.push_back(create_point(3.0, 4.5, 0.0));
    points.push_back(create_point(4.0, 6.0, 0.0));

    const std::vector<double> ans{0.9827937, 0.9827937, 0.9827937, 0.9827937, 0.9827937};

    const auto yaws = autoware::interpolation::splineYawFromPoints(points);
    for (size_t i = 0; i < yaws.size(); ++i) {
      EXPECT_NEAR(yaws.at(i), ans.at(i), epsilon);
    }
  }

  {  // curve
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(create_point(-2.0, -10.0, 0.0));
    points.push_back(create_point(2.0, 1.5, 0.0));
    points.push_back(create_point(3.0, 3.0, 0.0));
    points.push_back(create_point(5.0, 10.0, 0.0));
    points.push_back(create_point(10.0, 12.5, 0.0));

    const std::vector<double> ans{1.368174, 0.961318, 1.086098, 0.938357, 0.278594};
    const auto yaws = autoware::interpolation::splineYawFromPoints(points);
    for (size_t i = 0; i < yaws.size(); ++i) {
      EXPECT_NEAR(yaws.at(i), ans.at(i), epsilon);
    }
  }

  {  // size of base_keys is 1 (infeasible to interpolate)
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(create_point(1.0, 0.0, 0.0));

    EXPECT_THROW(autoware::interpolation::splineYawFromPoints(points), std::logic_error);
  }

  {  // straight: size of base_keys is 2 (edge case in the implementation)
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(create_point(1.0, 0.0, 0.0));
    points.push_back(create_point(2.0, 1.5, 0.0));

    const std::vector<double> ans{0.9827937, 0.9827937};

    const auto yaws = autoware::interpolation::splineYawFromPoints(points);
    for (size_t i = 0; i < yaws.size(); ++i) {
      EXPECT_NEAR(yaws.at(i), ans.at(i), epsilon);
    }
  }

  {  // straight: size of base_keys is 3 (edge case in the implementation)
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(create_point(1.0, 0.0, 0.0));
    points.push_back(create_point(2.0, 1.5, 0.0));
    points.push_back(create_point(3.0, 3.0, 0.0));

    const std::vector<double> ans{0.9827937, 0.9827937, 0.9827937};

    const auto yaws = autoware::interpolation::splineYawFromPoints(points);
    for (size_t i = 0; i < yaws.size(); ++i) {
      EXPECT_NEAR(yaws.at(i), ans.at(i), epsilon);
    }
  }

  {  // points very close together
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(create_point(1.0, 0.0, 0.0));
    points.push_back(create_point(1.0 + 1e-9, 0.0 + 1.5e-9, 0.0));
    points.push_back(create_point(2.0, 1.5, 0.0));
    points.push_back(create_point(3.0, 3.0, 0.0));

    const std::vector<double> ans{0.9827937, 0.9827937, 0.9827937, 0.9827937};

    const auto yaws = autoware::interpolation::splineYawFromPoints(points);
    for (size_t i = 0; i < yaws.size(); ++i) {
      EXPECT_NEAR(yaws.at(i), ans.at(i), epsilon);
    }
  }
}

TEST(spline_interpolation, SplineInterpolationPoints2d)
{
  using autoware_utils_geometry::create_point;

  // curve
  std::vector<geometry_msgs::msg::Point> points;
  points.push_back(create_point(-2.0, -10.0, 0.0));
  points.push_back(create_point(2.0, 1.5, 0.0));
  points.push_back(create_point(3.0, 3.0, 0.0));
  points.push_back(create_point(5.0, 10.0, 0.0));
  points.push_back(create_point(10.0, 12.5, 0.0));

  SplineInterpolationPoints2d s(points);

  {  // point
    // front
    const auto front_point = s.getSplineInterpolatedPoint(0, 0.0);
    EXPECT_NEAR(front_point.x, -2.0, epsilon);
    EXPECT_NEAR(front_point.y, -10.0, epsilon);

    // back
    const auto back_point = s.getSplineInterpolatedPoint(4, 0.0);
    EXPECT_NEAR(back_point.x, 10.0, epsilon);
    EXPECT_NEAR(back_point.y, 12.5, epsilon);

    // random
    const auto random_point = s.getSplineInterpolatedPoint(3, 0.5);
    EXPECT_NEAR(random_point.x, 5.28974, epsilon);
    EXPECT_NEAR(random_point.y, 10.3450319, epsilon);

    // out of range of total length
    const auto front_out_point = s.getSplineInterpolatedPoint(0.0, -0.1);
    EXPECT_NEAR(front_out_point.x, -2.0, epsilon);
    EXPECT_NEAR(front_out_point.y, -10.0, epsilon);

    const auto back_out_point = s.getSplineInterpolatedPoint(4.0, 0.1);
    EXPECT_NEAR(back_out_point.x, 10.0, epsilon);
    EXPECT_NEAR(back_out_point.y, 12.5, epsilon);

    // out of range of index
    EXPECT_THROW(s.getSplineInterpolatedPoint(-1, 0.0), std::out_of_range);
    EXPECT_THROW(s.getSplineInterpolatedPoint(5, 0.0), std::out_of_range);
  }

  {  // yaw
    // front
    EXPECT_NEAR(s.getSplineInterpolatedYaw(0, 0.0), 1.368174, epsilon);

    // back
    EXPECT_NEAR(s.getSplineInterpolatedYaw(4, 0.0), 0.278594, epsilon);

    // random
    EXPECT_NEAR(s.getSplineInterpolatedYaw(3, 0.5), 0.808580, epsilon);

    // out of range of total length
    EXPECT_NEAR(s.getSplineInterpolatedYaw(0.0, -0.1), 1.368174, epsilon);
    EXPECT_NEAR(s.getSplineInterpolatedYaw(4, 0.1), 0.278594, epsilon);

    // out of range of index
    EXPECT_THROW(s.getSplineInterpolatedYaw(-1, 0.0), std::out_of_range);
    EXPECT_THROW(s.getSplineInterpolatedYaw(5, 0.0), std::out_of_range);
  }

  {  // curvature
    // front
    EXPECT_NEAR(s.getSplineInterpolatedCurvature(0, 0.0), 0.0, epsilon);

    // back
    EXPECT_NEAR(s.getSplineInterpolatedCurvature(4, 0.0), 0.0, epsilon);

    // random
    EXPECT_NEAR(s.getSplineInterpolatedCurvature(3, 0.5), -0.271073, epsilon);

    // out of range of total length
    EXPECT_NEAR(s.getSplineInterpolatedCurvature(0.0, -0.1), 0.0, epsilon);
    EXPECT_NEAR(s.getSplineInterpolatedCurvature(4, 0.1), 0.0, epsilon);

    // out of range of index
    EXPECT_THROW(s.getSplineInterpolatedCurvature(-1, 0.0), std::out_of_range);
    EXPECT_THROW(s.getSplineInterpolatedCurvature(5, 0.0), std::out_of_range);
  }

  {  // accumulated distance
    // front
    EXPECT_NEAR(s.getAccumulatedLength(0), 0.0, epsilon);

    // back
    EXPECT_NEAR(s.getAccumulatedLength(4), 26.8488511, epsilon);

    // random
    EXPECT_NEAR(s.getAccumulatedLength(3), 21.2586811, epsilon);

    // out of range of index
    EXPECT_THROW(s.getAccumulatedLength(-1), std::out_of_range);
    EXPECT_THROW(s.getAccumulatedLength(5), std::out_of_range);
  }

  // size of base_keys is 1 (infeasible to interpolate)
  std::vector<geometry_msgs::msg::Point> single_points;
  single_points.push_back(create_point(1.0, 0.0, 0.0));
  EXPECT_THROW(SplineInterpolationPoints2d{single_points}, std::logic_error);
}

TEST(spline_interpolation, SplineInterpolationPoints2dPolymorphism)
{
  using autoware_planning_msgs::msg::TrajectoryPoint;
  using autoware_utils_geometry::create_point;

  std::vector<geometry_msgs::msg::Point> points;
  points.push_back(create_point(-2.0, -10.0, 0.0));
  points.push_back(create_point(2.0, 1.5, 0.0));
  points.push_back(create_point(3.0, 3.0, 0.0));

  std::vector<TrajectoryPoint> trajectory_points;
  for (const auto & p : points) {
    TrajectoryPoint tp;
    tp.pose.position = p;
    trajectory_points.push_back(tp);
  }

  SplineInterpolationPoints2d s_point(points);
  s_point.getSplineInterpolatedPoint(0, 0.);

  SplineInterpolationPoints2d s_traj_point(trajectory_points);
  s_traj_point.getSplineInterpolatedPoint(0, 0.);
}

TEST(spline_interpolation, getSplineInterpolatedPointAt)
{
  using autoware_utils_geometry::create_point;

  // Create a simple spline
  std::vector<geometry_msgs::msg::Point> points;
  points.push_back(create_point(0.0, 0.0, 0.0));
  points.push_back(create_point(1.0, 1.0, 0.0));
  points.push_back(create_point(2.0, 2.0, 0.0));
  points.push_back(create_point(3.0, 3.0, 0.0));

  SplineInterpolationPoints2d s(points);

  // Test at start (s = 0)
  const auto point_at_start = s.getSplineInterpolatedPointAt(0.0);
  EXPECT_NEAR(point_at_start.x, 0.0, epsilon);
  EXPECT_NEAR(point_at_start.y, 0.0, epsilon);
  EXPECT_NEAR(point_at_start.z, 0.0, epsilon);

  // Test at middle point
  const double s_mid = s.getAccumulatedLength(1);
  const auto point_at_mid = s.getSplineInterpolatedPointAt(s_mid);
  EXPECT_NEAR(point_at_mid.x, 1.0, epsilon);
  EXPECT_NEAR(point_at_mid.y, 1.0, epsilon);
  EXPECT_NEAR(point_at_mid.z, 0.0, epsilon);

  // Test at end
  const double s_end = s.getAccumulatedLength(s.getSize() - 1);
  const auto point_at_end = s.getSplineInterpolatedPointAt(s_end);
  EXPECT_NEAR(point_at_end.x, 3.0, epsilon);
  EXPECT_NEAR(point_at_end.y, 3.0, epsilon);
  EXPECT_NEAR(point_at_end.z, 0.0, epsilon);

  // Test interpolation between points
  const double s_interp = s.getAccumulatedLength(0) + 0.5;
  const auto point_interp = s.getSplineInterpolatedPointAt(s_interp);
  // Should be between (0,0) and (1,1)
  EXPECT_GT(point_interp.x, 0.0);
  EXPECT_LT(point_interp.x, 1.0);
  EXPECT_GT(point_interp.y, 0.0);
  EXPECT_LT(point_interp.y, 1.0);
}

TEST(spline_interpolation, getSplineCoefficients)
{
  using autoware_utils_geometry::create_point;

  // Create a simple spline
  std::vector<geometry_msgs::msg::Point> points;
  points.push_back(create_point(0.0, 0.0, 0.0));
  points.push_back(create_point(1.0, 1.0, 0.0));
  points.push_back(create_point(2.0, 2.0, 0.0));
  points.push_back(create_point(3.0, 3.0, 0.0));

  SplineInterpolationPoints2d s(points);

  // Test X coefficients
  const Eigen::VectorXd coeff_x = s.getSplineCoefficientsX();
  EXPECT_GT(coeff_x.size(), 0);
  for (Eigen::Index i = 0; i < coeff_x.size(); ++i) {
    EXPECT_TRUE(std::isfinite(coeff_x(i)));
  }

  // Test Y coefficients
  const Eigen::VectorXd coeff_y = s.getSplineCoefficientsY();
  EXPECT_GT(coeff_y.size(), 0);
  EXPECT_EQ(coeff_y.size(), coeff_x.size());  // Should have same number of segments
  for (Eigen::Index i = 0; i < coeff_y.size(); ++i) {
    EXPECT_TRUE(std::isfinite(coeff_y(i)));
  }

  // Test curvature coefficients (should be initialized after updateCurvatureSpline)
  s.updateCurvatureSpline();
  const Eigen::VectorXd coeff_curvature = s.getSplineCoefficientsCurvature();
  EXPECT_GT(coeff_curvature.size(), 0);
  for (Eigen::Index i = 0; i < coeff_curvature.size(); ++i) {
    EXPECT_TRUE(std::isfinite(coeff_curvature(i)));
  }
}

TEST(spline_interpolation, getSplineKnots)
{
  using autoware_utils_geometry::create_point;

  {
    // Test basic functionality
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(create_point(0.0, 0.0, 0.0));
    points.push_back(create_point(1.0, 1.0, 0.0));
    points.push_back(create_point(2.0, 2.0, 0.0));
    points.push_back(create_point(3.0, 3.0, 0.0));

    SplineInterpolationPoints2d s(points);
    const std::vector<double> knots = s.getSplineKnots();

    // Compute expected base_s_vec_ (accumulated arc lengths) from points
    std::vector<double> expected_base_s{0.0};
    for (size_t i = 1; i < points.size(); ++i) {
      const double dx = points[i].x - points[i - 1].x;
      const double dy = points[i].y - points[i - 1].y;
      const double dist = std::hypot(dx, dy);
      expected_base_s.push_back(expected_base_s.back() + dist);
    }

    EXPECT_EQ(knots.size(), expected_base_s.size());
    for (size_t i = 0; i < knots.size(); ++i) {
      EXPECT_NEAR(knots.at(i), expected_base_s.at(i), epsilon);
    }
  }

  {
    // Test with different values
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(create_point(-2.0, -10.0, 0.0));
    points.push_back(create_point(2.0, 1.5, 0.0));
    points.push_back(create_point(3.0, 3.0, 0.0));
    points.push_back(create_point(5.0, 10.0, 0.0));
    points.push_back(create_point(10.0, 12.5, 0.0));

    SplineInterpolationPoints2d s(points);
    const std::vector<double> knots = s.getSplineKnots();

    // Compute expected base_s_vec_ (accumulated arc lengths) from points
    std::vector<double> expected_base_s{0.0};
    for (size_t i = 1; i < points.size(); ++i) {
      const double dx = points[i].x - points[i - 1].x;
      const double dy = points[i].y - points[i - 1].y;
      const double dist = std::hypot(dx, dy);
      expected_base_s.push_back(expected_base_s.back() + dist);
    }

    EXPECT_EQ(knots.size(), expected_base_s.size());
    for (size_t i = 0; i < knots.size(); ++i) {
      EXPECT_NEAR(knots.at(i), expected_base_s.at(i), epsilon);
    }
  }

  {
    // Test with minimum size
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(create_point(0.0, 0.0, 0.0));
    points.push_back(create_point(1.0, 1.5, 0.0));

    SplineInterpolationPoints2d s(points);
    const std::vector<double> knots = s.getSplineKnots();

    // Compute expected base_s_vec_ (accumulated arc lengths) from points
    std::vector<double> expected_base_s{0.0};
    for (size_t i = 1; i < points.size(); ++i) {
      const double dx = points[i].x - points[i - 1].x;
      const double dy = points[i].y - points[i - 1].y;
      const double dist = std::hypot(dx, dy);
      expected_base_s.push_back(expected_base_s.back() + dist);
    }

    EXPECT_EQ(knots.size(), expected_base_s.size());
    for (size_t i = 0; i < knots.size(); ++i) {
      EXPECT_NEAR(knots.at(i), expected_base_s.at(i), epsilon);
    }
  }
}

TEST(spline_interpolation, updateCurvatureSpline)
{
  using autoware_utils_geometry::create_point;

  // Create a curved spline
  std::vector<geometry_msgs::msg::Point> points;
  points.push_back(create_point(-2.0, -10.0, 0.0));
  points.push_back(create_point(2.0, 1.5, 0.0));
  points.push_back(create_point(3.0, 3.0, 0.0));
  points.push_back(create_point(5.0, 10.0, 0.0));
  points.push_back(create_point(10.0, 12.5, 0.0));

  SplineInterpolationPoints2d s(points);

  // Update curvature spline
  s.updateCurvatureSpline();

  // Verify curvature spline is initialized
  const Eigen::VectorXd coeff_curvature = s.getSplineCoefficientsCurvature();
  EXPECT_GT(coeff_curvature.size(), 0);

  // Verify curvature values can be retrieved
  const auto curvatures = s.getSplineInterpolatedCurvatures();
  EXPECT_EQ(curvatures.size(), s.getSize());

  // Test with straight line (should have zero curvature)
  std::vector<geometry_msgs::msg::Point> straight_points;
  straight_points.push_back(create_point(0.0, 0.0, 0.0));
  straight_points.push_back(create_point(1.0, 1.0, 0.0));
  straight_points.push_back(create_point(2.0, 2.0, 0.0));

  SplineInterpolationPoints2d s_straight(straight_points);
  s_straight.updateCurvatureSpline();

  const auto straight_curvatures = s_straight.getSplineInterpolatedCurvatures();
  for (const auto & curvature : straight_curvatures) {
    EXPECT_NEAR(curvature, 0.0, epsilon);
  }
}

TEST(spline_interpolation, SplineInterpolationPoints2dResize)
{
  using autoware_utils_geometry::create_point;

  // Create initial spline
  std::vector<geometry_msgs::msg::Point> points;
  points.push_back(create_point(0.0, 0.0, 0.0));
  points.push_back(create_point(1.0, 1.0, 0.0));
  points.push_back(create_point(2.0, 2.0, 0.0));
  points.push_back(create_point(3.0, 3.0, 0.0));
  points.push_back(create_point(4.0, 4.0, 0.0));

  SplineInterpolationPoints2d s(points);
  const size_t original_size = s.getSize();
  EXPECT_EQ(original_size, 5);

  // Save original knots before resizing
  const std::vector<double> original_knots = s.getSplineKnots();

  // Test extending: resize to larger size
  s.resize(7);
  EXPECT_EQ(s.getSize(), 7);
  const std::vector<double> knots_extended = s.getSplineKnots();
  EXPECT_EQ(knots_extended.size(), 7);
  // First 5 knots should match original
  for (size_t i = 0; i < original_size; ++i) {
    EXPECT_NEAR(knots_extended.at(i), original_knots.at(i), epsilon);
  }

  // Test clipping: resize to smaller size
  s.resize(3);
  EXPECT_EQ(s.getSize(), 3);
  const std::vector<double> knots_clipped = s.getSplineKnots();
  EXPECT_EQ(knots_clipped.size(), 3);
  // First 3 knots should match original first 3
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(knots_clipped.at(i), original_knots.at(i), epsilon);
  }

  // Test no-op: resize to same size
  const size_t current_size = s.getSize();
  const std::vector<double> knots_before = s.getSplineKnots();
  s.resize(current_size);
  EXPECT_EQ(s.getSize(), current_size);
  const std::vector<double> knots_after = s.getSplineKnots();
  EXPECT_EQ(knots_before.size(), knots_after.size());
  for (size_t i = 0; i < knots_before.size(); ++i) {
    EXPECT_NEAR(knots_before.at(i), knots_after.at(i), epsilon);
  }
}

TEST(spline_interpolation, extendLinearlyForward)
{
  using autoware_utils_geometry::create_point;

  // Create initial spline
  std::vector<geometry_msgs::msg::Point> points;
  points.push_back(create_point(0.0, 0.0, 0.0));
  points.push_back(create_point(1.0, 1.0, 0.0));
  points.push_back(create_point(2.0, 2.0, 0.0));

  SplineInterpolationPoints2d s(points);
  const size_t original_size = s.getSize();
  const double original_length = s.getAccumulatedLength(original_size - 1);

  // Save original accumulated lengths
  std::vector<double> original_accumulated_lengths;
  for (size_t i = 0; i < original_size; ++i) {
    original_accumulated_lengths.push_back(s.getAccumulatedLength(i));
  }

  // Extend to 5 knots with delta_s = 0.5
  const size_t target_n_knots = 5;
  const double delta_s = 0.5;
  s.extendLinearlyForward(target_n_knots, delta_s);

  EXPECT_EQ(s.getSize(), target_n_knots);

  // Verify extended knots
  const std::vector<double> knots = s.getSplineKnots();
  EXPECT_EQ(knots.size(), target_n_knots);

  // Original knots should be preserved
  for (size_t i = 0; i < original_size; ++i) {
    EXPECT_NEAR(knots.at(i), original_accumulated_lengths.at(i), epsilon);
  }

  // New knots should be spaced by delta_s
  for (size_t i = original_size; i < target_n_knots; ++i) {
    const double expected_s = original_length + (i - original_size + 1) * delta_s;
    EXPECT_NEAR(knots.at(i), expected_s, epsilon);
  }

  // Verify extended points follow linear extrapolation
  const auto point_at_end = s.getSplineInterpolatedPointAt(original_length);
  const auto point_extended = s.getSplineInterpolatedPointAt(knots.back());
  // Extended point should be further along the direction
  const double dx = point_extended.x - point_at_end.x;
  const double dy = point_extended.y - point_at_end.y;
  EXPECT_GT(std::hypot(dx, dy), 0.0);

  // Test extending when already at target size (should be no-op)
  const size_t size_before = s.getSize();
  s.extendLinearlyForward(size_before, delta_s);
  EXPECT_EQ(s.getSize(), size_before);

  // Test extending when target is smaller (should be no-op)
  s.extendLinearlyForward(size_before - 1, delta_s);
  EXPECT_EQ(s.getSize(), size_before);
}

TEST(spline_interpolation, projectPointOntoSpline)
{
  using autoware_utils_geometry::create_point;

  // Create a simple spline
  std::vector<geometry_msgs::msg::Point> points;
  points.push_back(create_point(0.0, 0.0, 0.0));
  points.push_back(create_point(1.0, 1.0, 0.0));
  points.push_back(create_point(2.0, 2.0, 0.0));
  points.push_back(create_point(3.0, 3.0, 0.0));

  SplineInterpolationPoints2d s(points);

  // Test projecting a point on the spline
  const double s_on_spline = s.getAccumulatedLength(1);
  const auto point_on_spline = s.getSplineInterpolatedPointAt(s_on_spline);
  const auto [s_proj1, eY1] = s.projectPointOntoSpline(point_on_spline.x, point_on_spline.y);
  EXPECT_NEAR(s_proj1, s_on_spline, 0.1);  // Should project close to original s
  EXPECT_NEAR(eY1, 0.0, 0.1);              // Cross-track error should be small

  // Test projecting a point near the spline
  const auto [s_proj2, eY2] = s.projectPointOntoSpline(1.0, 1.1);
  EXPECT_GE(s_proj2, 0.0);
  EXPECT_LE(s_proj2, s.getAccumulatedLength(s.getSize() - 1));
  // Cross-track error should be positive (point is to the right of spline)
  EXPECT_GT(eY2, -1.0);
  EXPECT_LT(eY2, 1.0);

  // Test projecting a point far from the spline
  const auto [s_proj3, eY3] = s.projectPointOntoSpline(10.0, 10.0);
  EXPECT_GE(s_proj3, 0.0);
  EXPECT_LE(s_proj3, s.getAccumulatedLength(s.getSize() - 1));
  // Should project to the end of the spline
  EXPECT_NEAR(s_proj3, s.getAccumulatedLength(s.getSize() - 1), 0.5);

  // Test with custom initial guess
  const auto [s_proj4, eY4] = s.projectPointOntoSpline(1.5, 1.5, 1.0, 1e-6, 20);
  EXPECT_GE(s_proj4, 0.0);
  EXPECT_LE(s_proj4, s.getAccumulatedLength(s.getSize() - 1));

  // Test with point at start
  const auto [s_proj5, eY5] = s.projectPointOntoSpline(0.0, 0.0);
  EXPECT_NEAR(s_proj5, 0.0, 0.1);
  EXPECT_NEAR(eY5, 0.0, 0.1);

  // Test with point at end
  const double s_end = s.getAccumulatedLength(s.getSize() - 1);
  const auto point_at_end = s.getSplineInterpolatedPointAt(s_end);
  const auto [s_proj6, eY6] = s.projectPointOntoSpline(point_at_end.x, point_at_end.y);
  EXPECT_NEAR(s_proj6, s_end, 0.1);
  EXPECT_NEAR(eY6, 0.0, 0.1);
}
