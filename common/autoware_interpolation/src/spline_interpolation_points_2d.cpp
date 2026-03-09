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

#include "autoware/interpolation/spline_interpolation_points_2d.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <limits>
#include <utility>
#include <vector>

namespace autoware::interpolation
{
std::vector<double> calcEuclidDist(const std::vector<double> & x, const std::vector<double> & y)
{
  if (x.size() != y.size()) {
    return std::vector<double>{};
  }

  std::vector<double> dist_v;
  dist_v.push_back(0.0);
  for (size_t i = 0; i < x.size() - 1; ++i) {
    const double dx = x.at(i + 1) - x.at(i);
    const double dy = y.at(i + 1) - y.at(i);
    dist_v.push_back(dist_v.at(i) + std::hypot(dx, dy));
  }

  return dist_v;
}

std::array<std::vector<double>, 4> getBaseValues(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  // calculate x, y
  std::vector<double> base_x;
  std::vector<double> base_y;
  std::vector<double> base_z;
  for (size_t i = 0; i < points.size(); i++) {
    const auto & current_pos = points.at(i);
    if (i > 0) {
      const auto & prev_pos = points.at(i - 1);
      if (
        std::fabs(current_pos.x - prev_pos.x) < 1e-6 &&
        std::fabs(current_pos.y - prev_pos.y) < 1e-6) {
        continue;
      }
    }
    base_x.push_back(current_pos.x);
    base_y.push_back(current_pos.y);
    base_z.push_back(current_pos.z);
  }

  // calculate base_keys, base_values
  if (base_x.size() < 2 || base_y.size() < 2 || base_z.size() < 2) {
    throw std::logic_error("The number of unique points is not enough.");
  }

  const std::vector<double> base_s = calcEuclidDist(base_x, base_y);

  return {base_s, base_x, base_y, base_z};
}

template <typename T>
std::vector<double> splineYawFromPoints(const std::vector<T> & points)
{
  // calculate spline coefficients
  SplineInterpolationPoints2d interpolator(points);

  if (interpolator.getSize() == points.size()) {
    return interpolator.getSplineInterpolatedYaws();
  }

  const auto interpolated_points = interpolator.getSplineInterpolatedPoints();
  std::vector<double> yaw_vec;
  yaw_vec.reserve(points.size());
  size_t interpolated_idx = 0;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto distance =
      autoware_utils_geometry::calc_distance2d(points[i], interpolated_points[interpolated_idx]);
    const auto same_point = interpolated_idx < interpolated_points.size() && distance < 1e-9;
    yaw_vec.push_back(interpolator.getSplineInterpolatedYaw(interpolated_idx, distance));
    if (same_point) {
      interpolated_idx++;
    }
  }
  return yaw_vec;
}

template std::vector<double> splineYawFromPoints(
  const std::vector<geometry_msgs::msg::Point> & points);
template std::vector<double> splineYawFromPoints(
  const std::vector<geometry_msgs::msg::Pose> & points);
template std::vector<double> splineYawFromPoints(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points);
template std::vector<double> splineYawFromPoints(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points);
template std::vector<double> splineYawFromPoints(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points);

geometry_msgs::msg::Pose SplineInterpolationPoints2d::getSplineInterpolatedPose(
  const size_t idx, const double s) const
{
  geometry_msgs::msg::Pose pose;
  pose.position = getSplineInterpolatedPoint(idx, s);
  pose.orientation =
    autoware_utils_geometry::create_quaternion_from_yaw(getSplineInterpolatedYaw(idx, s));
  return pose;
}

std::vector<geometry_msgs::msg::Point> SplineInterpolationPoints2d::getSplineInterpolatedPoints()
  const
{
  std::vector<geometry_msgs::msg::Point> geom_points;
  geom_points.reserve(getSize());
  const auto xs = spline_x_.getSplineInterpolatedValues(base_s_vec_);
  const auto ys = spline_y_.getSplineInterpolatedValues(base_s_vec_);
  const auto zs = spline_z_.getSplineInterpolatedValues(base_s_vec_);
  geometry_msgs::msg::Point geom_point;
  for (auto i = 0UL; i < getSize(); ++i) {
    geom_point.x = xs[i];
    geom_point.y = ys[i];
    geom_point.z = zs[i];
    geom_points.push_back(geom_point);
  }
  return geom_points;
}

geometry_msgs::msg::Point SplineInterpolationPoints2d::getSplineInterpolatedPoint(
  const size_t idx, const double s) const
{
  if (base_s_vec_.size() <= idx) {
    throw std::out_of_range("idx is out of range.");
  }

  double whole_s = base_s_vec_.at(idx) + s;
  if (whole_s < base_s_vec_.front()) {
    whole_s = base_s_vec_.front();
  }
  if (whole_s > base_s_vec_.back()) {
    whole_s = base_s_vec_.back();
  }

  const double x = spline_x_.getSplineInterpolatedValues({whole_s}).at(0);
  const double y = spline_y_.getSplineInterpolatedValues({whole_s}).at(0);
  const double z = spline_z_.getSplineInterpolatedValues({whole_s}).at(0);

  geometry_msgs::msg::Point geom_point;
  geom_point.x = x;
  geom_point.y = y;
  geom_point.z = z;
  return geom_point;
}

double SplineInterpolationPoints2d::getSplineInterpolatedYaw(const size_t idx, const double s) const
{
  if (base_s_vec_.size() <= idx) {
    throw std::out_of_range("idx is out of range.");
  }

  const double whole_s =
    std::clamp(base_s_vec_.at(idx) + s, base_s_vec_.front(), base_s_vec_.back());

  const double diff_x = spline_x_.getSplineInterpolatedDiffValues({whole_s}).at(0);
  const double diff_y = spline_y_.getSplineInterpolatedDiffValues({whole_s}).at(0);

  return std::atan2(diff_y, diff_x);
}

std::vector<double> SplineInterpolationPoints2d::getSplineInterpolatedYaws() const
{
  std::vector<double> yaw_vec;
  for (size_t i = 0; i < spline_x_.getSize(); ++i) {
    const double yaw = getSplineInterpolatedYaw(i, 0.0);
    yaw_vec.push_back(yaw);
  }
  return yaw_vec;
}

double SplineInterpolationPoints2d::getSplineInterpolatedCurvature(
  const size_t idx, const double s) const
{
  if (base_s_vec_.size() <= idx) {
    throw std::out_of_range("idx is out of range.");
  }

  const double whole_s =
    std::clamp(base_s_vec_.at(idx) + s, base_s_vec_.front(), base_s_vec_.back());

  const double diff_x = spline_x_.getSplineInterpolatedDiffValues({whole_s}).at(0);
  const double diff_y = spline_y_.getSplineInterpolatedDiffValues({whole_s}).at(0);

  const double quad_diff_x = spline_x_.getSplineInterpolatedQuadDiffValues({whole_s}).at(0);
  const double quad_diff_y = spline_y_.getSplineInterpolatedQuadDiffValues({whole_s}).at(0);

  return (diff_x * quad_diff_y - quad_diff_x * diff_y) /
         std::pow(std::pow(diff_x, 2) + std::pow(diff_y, 2), 1.5);
}

std::vector<double> SplineInterpolationPoints2d::getSplineInterpolatedCurvatures() const
{
  std::vector<double> curvature_vec;
  curvature_vec.reserve(spline_x_.getSize());
  for (size_t i = 0; i < spline_x_.getSize(); ++i) {
    const double curvature = getSplineInterpolatedCurvature(i, 0.0);
    curvature_vec.push_back(curvature);
  }
  return curvature_vec;
}

size_t SplineInterpolationPoints2d::getOffsetIndex(const size_t idx, const double offset) const
{
  const double whole_s = base_s_vec_.at(idx) + offset;
  for (size_t s_idx = 0; s_idx < base_s_vec_.size(); ++s_idx) {
    if (whole_s < base_s_vec_.at(s_idx)) {
      return s_idx;
    }
  }
  return base_s_vec_.size() - 1;
}

double SplineInterpolationPoints2d::getAccumulatedLength(const size_t idx) const
{
  if (base_s_vec_.size() <= idx) {
    throw std::out_of_range("idx is out of range.");
  }
  return base_s_vec_.at(idx);
}

void SplineInterpolationPoints2d::calcSplineCoefficientsInner(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  const auto base = getBaseValues(points);

  base_s_vec_ = base.at(0);
  const auto & base_x_vec = base.at(1);
  const auto & base_y_vec = base.at(2);
  const auto & base_z_vec = base.at(3);

  // calculate spline coefficients
  spline_x_ = SplineInterpolation(base_s_vec_, base_x_vec);
  spline_y_ = SplineInterpolation(base_s_vec_, base_y_vec);
  spline_z_ = SplineInterpolation(base_s_vec_, base_z_vec);
}

void SplineInterpolationPoints2d::updateCurvatureSpline()
{
  if (base_s_vec_.size() < 2 || spline_x_.getSize() < 2 || spline_y_.getSize() < 2) {
    // Initialize with zero curvature if not enough points
    std::vector<double> zero_curvatures(base_s_vec_.size(), 0.0);
    if (base_s_vec_.size() >= 2) {
      spline_curvature_ = SplineInterpolation(base_s_vec_, zero_curvatures);
    }
    return;
  }

  // Compute curvature values at each knot
  std::vector<double> curvature_values;
  curvature_values.reserve(base_s_vec_.size());

  for (const auto & s : base_s_vec_) {
    const double diff_x = spline_x_.getSplineInterpolatedDiffValues({s}).at(0);
    const double diff_y = spline_y_.getSplineInterpolatedDiffValues({s}).at(0);

    const double quad_diff_x = spline_x_.getSplineInterpolatedQuadDiffValues({s}).at(0);
    const double quad_diff_y = spline_y_.getSplineInterpolatedQuadDiffValues({s}).at(0);

    const double denom = std::pow(diff_x, 2) + std::pow(diff_y, 2);
    if (denom < 1e-10) {
      curvature_values.push_back(0.0);  // Straight line
    } else {
      const double curvature = (diff_x * quad_diff_y - quad_diff_x * diff_y) / std::pow(denom, 1.5);
      curvature_values.push_back(curvature);
    }
  }

  // Fit a spline to the curvature values
  spline_curvature_ = SplineInterpolation(base_s_vec_, curvature_values);
}

void SplineInterpolationPoints2d::extendLinearlyForward(
  const size_t target_n_knots, const double delta_s)
{
  if (target_n_knots <= base_s_vec_.size()) {
    return;
  }

  const size_t n_missing = target_n_knots - base_s_vec_.size();

  // Get the end value and derivative from the last segment
  const double s_end = base_s_vec_.back();
  const double x_end = spline_x_.getSplineInterpolatedValues({s_end}).at(0);
  const double y_end = spline_y_.getSplineInterpolatedValues({s_end}).at(0);
  const double z_end = spline_z_.getSplineInterpolatedValues({s_end}).at(0);

  const double dx_ds = spline_x_.getSplineInterpolatedDiffValues({s_end}).at(0);
  const double dy_ds = spline_y_.getSplineInterpolatedDiffValues({s_end}).at(0);
  const double dz_ds = spline_z_.getSplineInterpolatedDiffValues({s_end}).at(0);

  // Build extended knots and values
  std::vector<double> extended_s = base_s_vec_;
  std::vector<double> extended_x;
  std::vector<double> extended_y;
  std::vector<double> extended_z;

  // Get original values at existing knots
  for (const auto & s : base_s_vec_) {
    extended_x.push_back(spline_x_.getSplineInterpolatedValues({s}).at(0));
    extended_y.push_back(spline_y_.getSplineInterpolatedValues({s}).at(0));
    extended_z.push_back(spline_z_.getSplineInterpolatedValues({s}).at(0));
  }

  // Add extended knots and linearly extrapolated values
  for (size_t i = 0; i < n_missing; ++i) {
    const double new_s = s_end + (i + 1) * delta_s;
    extended_s.push_back(new_s);

    // Linear extrapolation: value = end_value + derivative * (new_s - s_end)
    extended_x.push_back(x_end + dx_ds * (new_s - s_end));
    extended_y.push_back(y_end + dy_ds * (new_s - s_end));
    extended_z.push_back(z_end + dz_ds * (new_s - s_end));
  }

  // Recalculate splines with extended knots and values
  // SplineInterpolation will automatically create appropriate segments
  base_s_vec_ = extended_s;
  spline_x_ = SplineInterpolation(extended_s, extended_x);
  spline_y_ = SplineInterpolation(extended_s, extended_y);
  spline_z_ = SplineInterpolation(extended_s, extended_z);
}

std::pair<double, double> SplineInterpolationPoints2d::projectPointOntoSpline(
  const double x_i, const double y_i, double s_init, const double tol, const int max_iter) const
{
  double s = s_init;

  // Coarse search: iterate over spline knots to find closest s
  const auto & knots = spline_x_.getKnots();
  double min_dist = std::numeric_limits<double>::max();
  for (const auto & s_knot : knots) {
    double dx = spline_x_.getSplineInterpolatedValues({s_knot}).at(0) - x_i;
    double dy = spline_y_.getSplineInterpolatedValues({s_knot}).at(0) - y_i;
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      s = s_knot;
    }
  }

  // Newton iteration with proper second derivative
  for (int iter = 0; iter < max_iter; ++iter) {
    const double x_s = spline_x_.getSplineInterpolatedValues({s}).at(0);
    const double y_s = spline_y_.getSplineInterpolatedValues({s}).at(0);

    const double dx_ds = spline_x_.getSplineInterpolatedDiffValues({s}).at(0);
    const double dy_ds = spline_y_.getSplineInterpolatedDiffValues({s}).at(0);

    const double d2x_ds2 = spline_x_.getSplineInterpolatedQuadDiffValues({s}).at(0);
    const double d2y_ds2 = spline_y_.getSplineInterpolatedQuadDiffValues({s}).at(0);

    // f'(s) = d/ds[||p(s) - p_i||^2] = 2[(x_s - x_i)·dx_ds + (y_s - y_i)·dy_ds]
    const double df_ds = (x_s - x_i) * dx_ds + (y_s - y_i) * dy_ds;

    // f''(s) = d²/ds²[||p(s) - p_i||^2] = 2[dx_ds² + dy_ds² + (x_s - x_i)·d²x_ds² + (y_s -
    // y_i)·d²y_ds²]
    const double d2f_ds2 =
      dx_ds * dx_ds + dy_ds * dy_ds + (x_s - x_i) * d2x_ds2 + (y_s - y_i) * d2y_ds2;

    if (std::fabs(d2f_ds2) < 1e-12) break;  // avoid divide by zero

    double ds = df_ds / d2f_ds2;

    // Bind s between min s and max s
    if (s - ds < base_s_vec_.front()) {
      ds = s - base_s_vec_.front();
    } else if (s - ds > base_s_vec_.back()) {
      ds = s - base_s_vec_.back();
    }
    s -= ds;

    if (std::fabs(ds) < tol) {
      break;  // converged
    }
  }

  // Compute cross-track error eY
  double x_s = spline_x_.getSplineInterpolatedValues({s}).at(0);
  double y_s = spline_y_.getSplineInterpolatedValues({s}).at(0);
  double psi_s = getSplineInterpolatedYaw(0, s);

  double eY = -(x_i - x_s) * std::sin(psi_s) + (y_i - y_s) * std::cos(psi_s);

  return {s, eY};
}

}  // namespace autoware::interpolation
