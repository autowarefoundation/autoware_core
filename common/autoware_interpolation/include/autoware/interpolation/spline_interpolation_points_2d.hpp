// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__INTERPOLATION__SPLINE_INTERPOLATION_POINTS_2D_HPP_
#define AUTOWARE__INTERPOLATION__SPLINE_INTERPOLATION_POINTS_2D_HPP_

#include "autoware/interpolation/spline_interpolation.hpp"

#include <limits>
#include <utility>
#include <vector>

namespace autoware::interpolation
{

template <typename T>
std::vector<double> splineYawFromPoints(const std::vector<T> & points);

// non-static points spline interpolation
// NOTE: We can calculate yaw from the x and y by interpolation derivatives.
//
// Usage:
// ```
// SplineInterpolationPoints2d spline;
// // memorize pre-interpolation result internally
// spline.calcSplineCoefficients(base_keys, base_values);
// const auto interpolation_result1 = spline.getSplineInterpolatedPoint(
//   base_keys, query_keys1);
// const auto interpolation_result2 = spline.getSplineInterpolatedPoint(
//   base_keys, query_keys2);
// const auto yaw_interpolation_result = spline.getSplineInterpolatedYaw(
//   base_keys, query_keys1);
// ```
class SplineInterpolationPoints2d
{
public:
  SplineInterpolationPoints2d() = default;
  template <typename T>
  explicit SplineInterpolationPoints2d(const std::vector<T> & points)
  {
    std::vector<geometry_msgs::msg::Point> points_inner;
    for (const auto & p : points) {
      points_inner.push_back(autoware_utils_geometry::get_point(p));
    }
    calcSplineCoefficientsInner(points_inner);
  }

  // TODO(murooka) implement these functions
  // std::vector<geometry_msgs::msg::Point> getSplineInterpolatedPoints(const double width);
  // std::vector<geometry_msgs::msg::Pose> getSplineInterpolatedPoses(const double width);

  geometry_msgs::msg::Point getSplineInterpolatedPointAt(const double s) const
  {
    geometry_msgs::msg::Point point;
    point.x = spline_x_.getSplineInterpolatedValues({s}).at(0);
    point.y = spline_y_.getSplineInterpolatedValues({s}).at(0);
    point.z = spline_z_.getSplineInterpolatedValues({s}).at(0);
    return point;
  }

  // pose (= getSplineInterpolatedPoint + getSplineInterpolatedYaw)
  geometry_msgs::msg::Pose getSplineInterpolatedPose(const size_t idx, const double s) const;

  // point
  std::vector<geometry_msgs::msg::Point> getSplineInterpolatedPoints() const;
  geometry_msgs::msg::Point getSplineInterpolatedPoint(const size_t idx, const double s) const;

  // yaw
  double getSplineInterpolatedYaw(const size_t idx, const double s) const;
  std::vector<double> getSplineInterpolatedYaws() const;

  // curvature
  double getSplineInterpolatedCurvature(const size_t idx, const double s) const;
  std::vector<double> getSplineInterpolatedCurvatures() const;

  // Debug methods to expose spline coefficients
  const Eigen::VectorXd getSplineCoefficientsX() const { return spline_x_.getCoefficients(); }
  const Eigen::VectorXd getSplineCoefficientsY() const { return spline_y_.getCoefficients(); }
  const Eigen::VectorXd getSplineCoefficientsCurvature() const
  {
    return spline_curvature_.getCoefficients();
  }
  const std::vector<double> getSplineKnots() const { return spline_x_.getKnots(); }

  size_t getSize() const { return base_s_vec_.size(); }
  size_t getOffsetIndex(const size_t idx, const double offset) const;
  double getAccumulatedLength(const size_t idx) const;
  void updateCurvatureSpline();
  void resize(const size_t size)
  {
    if (size > base_s_vec_.size()) {
      // Extending - just resize (new elements will be uninitialized, caller should fill them)
      base_s_vec_.resize(size);
      spline_x_.resize(size);
      spline_y_.resize(size);
      spline_z_.resize(size);
      spline_curvature_.resize(size);
    } else if (size < base_s_vec_.size()) {
      // Clipping - explicitly copy first N elements to preserve data
      base_s_vec_.resize(size);
      spline_x_.resize(size);
      spline_y_.resize(size);
      spline_z_.resize(size);
      spline_curvature_.resize(size);
    }
  }

  /**
   * @brief Linearly extend the spline forward to reach target_n_knots knots
   * @param target_n_knots Target number of knots (must be >= current size)
   * @param delta_s Arc length step for extension segments
   * @details Extends the x and y splines linearly forward, then curvature is automatically
   *          recalculated from the extended splines when getSplineInterpolatedCurvatures() is
   * called
   */
  void extendLinearlyForward(const size_t target_n_knots, const double delta_s);

  /**
   * @brief Project a 2D point onto a spline (x(s), y(s)) and return s-coordinate and cross-track
   * error.
   * @param x_i x-coordinate of the point
   * @param y_i y-coordinate of the point
   * @param s_init initial guess for s (optional)
   * @param tol Newton convergence tolerance
   * @param max_iter maximum Newton iterations
   * @return std::pair<double, double> -> (s_projected, eY)
   */
  std::pair<double, double> projectPointOntoSpline(
    const double x_i, const double y_i, double s_init = 0.0, const double tol = 1e-6,
    const int max_iter = 20) const;

private:
  void calcSplineCoefficientsInner(const std::vector<geometry_msgs::msg::Point> & points);
  SplineInterpolation spline_x_;
  SplineInterpolation spline_y_;
  SplineInterpolation spline_z_;
  SplineInterpolation spline_curvature_;

  std::vector<double> base_s_vec_;
};
}  // namespace autoware::interpolation

#endif  // AUTOWARE__INTERPOLATION__SPLINE_INTERPOLATION_POINTS_2D_HPP_
