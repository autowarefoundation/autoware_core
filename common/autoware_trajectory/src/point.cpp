// Copyright 2024 TIER IV, Inc.
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

#include "autoware/trajectory/point.hpp"

#include "autoware/trajectory/detail/helpers.hpp"
#include "autoware/trajectory/interpolator/cubic_spline.hpp"
#include "autoware/trajectory/interpolator/linear.hpp"
#include "autoware/trajectory/threshold.hpp"
#include "autoware_utils_geometry/geometry.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{

using PointType = geometry_msgs::msg::Point;

Trajectory<PointType>::Trajectory()
{
  Builder::defaults(this);
}

Trajectory<PointType>::Trajectory(const Trajectory & rhs)
: x_interpolator_(rhs.x_interpolator_->clone()),
  y_interpolator_(rhs.y_interpolator_->clone()),
  z_interpolator_(rhs.z_interpolator_->clone()),
  bases_(rhs.bases_),
  start_(rhs.start_),
  end_(rhs.end_)
{
}

Trajectory<PointType> & Trajectory<PointType>::operator=(const Trajectory & rhs)
{
  if (this != &rhs) {
    x_interpolator_ = rhs.x_interpolator_->clone();
    y_interpolator_ = rhs.y_interpolator_->clone();
    z_interpolator_ = rhs.z_interpolator_->clone();
    bases_ = rhs.bases_;
    start_ = rhs.start_;
    end_ = rhs.end_;
  }
  return *this;
}

interpolator::InterpolationResult Trajectory<PointType>::build(
  const std::vector<PointType> & points)
{
  if (points.empty()) {
    return tl::unexpected(interpolator::InterpolationFailure{"cannot interpolate 0 size points"});
  }
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;

  bases_.clear();
  bases_.reserve(points.size() + 1);
  xs.reserve(points.size() + 1);
  ys.reserve(points.size() + 1);
  zs.reserve(points.size() + 1);

  bases_.emplace_back(0.0);
  xs.emplace_back(points[0].x);
  ys.emplace_back(points[0].y);
  zs.emplace_back(points[0].z);

  for (size_t i = 1; i < points.size(); ++i) {
    /**
       NOTE:
       this sanitization is essential for avoiding zero division and NaN in later interpolation.

       if there are 100 points with the interval of 1nm, then they are treated as 100 points with
       the interval of k_points_minimum_dist_threshold and interpolation is continued.
    */
    const auto dist = std::max<double>(
      autoware_utils_geometry::calc_distance3d(points[i], points[i - 1]),
      k_points_minimum_dist_threshold);
    bases_.emplace_back(bases_.back() + dist);
    xs.emplace_back(points[i].x);
    ys.emplace_back(points[i].y);
    zs.emplace_back(points[i].z);
  }

  start_ = bases_.front();
  end_ = bases_.back();

  if (const auto result = x_interpolator_->build(bases_, std::move(xs)); !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"failed to interpolate Point::x"} + result.error());
  }
  if (const auto result = y_interpolator_->build(bases_, std::move(ys)); !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"failed to interpolate Point::y"} + result.error());
  }
  if (const auto result = z_interpolator_->build(bases_, std::move(zs)); !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"failed to interpolate Point::z"} + result.error());
  }
  return interpolator::InterpolationSuccess{};
}

double Trajectory<PointType>::clamp(const double s, bool show_warning) const
{
  constexpr double eps = 1e-5;
  if (show_warning && (s < -eps || s > length() + eps)) {
    RCLCPP_WARN(
      rclcpp::get_logger("Trajectory"), "The arc length %f is out of the trajectory length %f", s,
      length());
  }
  return std::clamp(s, 0.0, length()) + start_;
}

std::vector<double> Trajectory<PointType>::get_internal_bases() const
{
  return get_underlying_bases();
}

std::vector<double> Trajectory<PointType>::get_underlying_bases() const
{
  auto bases = detail::crop_bases(bases_, start_, end_);
  std::transform(
    bases.begin(), bases.end(), bases.begin(), [this](const double s) { return s - start_; });
  return bases;
}

void Trajectory<PointType>::update_bases(const double s)
{
  const auto it = std::lower_bound(bases_.begin(), bases_.end(), s);
  if (it == bases_.end()) {
    // NOTE(soblin): the extension of base(or extrapolation) will be supported by other API.
    return;
  }
  if (*it == s) {
    // already inserted
    return;
  }
  bases_.insert(it, s);
}

double Trajectory<PointType>::length() const
{
  return end_ - start_;
}

PointType Trajectory<PointType>::compute(const double s) const
{
  const auto s_clamp = clamp(s, true);
  PointType result;
  result.x = x_interpolator_->compute(s_clamp);
  result.y = y_interpolator_->compute(s_clamp);
  result.z = z_interpolator_->compute(s_clamp);
  return result;
}

std::vector<PointType> Trajectory<PointType>::compute(const std::vector<double> & ss) const
{
  std::vector<PointType> points;
  points.reserve(ss.size());
  for (const auto s : ss) {
    points.emplace_back(compute(s));
  }
  return points;
}

double Trajectory<PointType>::azimuth(const double s) const
{
  const auto s_clamp = clamp(s, true);
  const double dx = x_interpolator_->compute_first_derivative(s_clamp);
  const double dy = y_interpolator_->compute_first_derivative(s_clamp);
  return std::atan2(dy, dx);
}

std::vector<double> Trajectory<PointType>::azimuth(const std::vector<double> & ss) const
{
  std::vector<double> a;
  a.reserve(ss.size());
  for (const auto s : ss) {
    a.push_back(azimuth(s));
  }
  return a;
}

double Trajectory<PointType>::elevation(const double s) const
{
  const auto s_clamp = clamp(s, true);
  const double dz = z_interpolator_->compute_first_derivative(s_clamp);
  return std::atan2(dz, 1.0);
}

double Trajectory<PointType>::curvature(const double s) const
{
  const auto s_clamp = clamp(s, true);
  const double dx = x_interpolator_->compute_first_derivative(s_clamp);
  const double ddx = x_interpolator_->compute_second_derivative(s_clamp);
  const double dy = y_interpolator_->compute_first_derivative(s_clamp);
  const double ddy = y_interpolator_->compute_second_derivative(s_clamp);
  return (dx * ddy - dy * ddx) / std::pow(dx * dx + dy * dy, 1.5);
}

std::vector<double> Trajectory<PointType>::curvature(const std::vector<double> & ss) const
{
  std::vector<double> ks;
  ks.reserve(ss.size());
  for (const auto s : ss) {
    ks.push_back(curvature(s));
  }
  return ks;
}

std::vector<PointType> Trajectory<PointType>::restore(const size_t min_points) const
{
  std::vector<double> sanitized_bases{};
  {
    const auto bases = detail::fill_bases(get_underlying_bases(), min_points);
    std::vector<PointType> points;

    points.reserve(bases.size());
    for (const auto & s : bases) {
      const auto point = compute(s);
      if (points.empty() || !is_almost_same(point, points.back())) {
        points.push_back(point);
        sanitized_bases.push_back(s);
      }
    }
    if (points.size() >= min_points) {
      return points;
    }
  }

  // retry to satisfy min_point requirement as much as possible
  const auto bases = detail::fill_bases(sanitized_bases, min_points);
  std::vector<PointType> points;
  points.reserve(bases.size());
  for (const auto & s : bases) {
    const auto point = compute(s);
    if (points.empty() || !is_almost_same(point, points.back())) {
      points.push_back(point);
    }
  }
  return points;
}

void Trajectory<PointType>::crop(const double start, const double length)
{
  start_ = std::clamp(start_ + start, start_, end_);
  end_ = std::clamp(start_ + length, start_, end_);
}

Trajectory<PointType>::Builder::Builder() : trajectory_(std::make_unique<Trajectory<PointType>>())
{
  defaults(trajectory_.get());
}

void Trajectory<PointType>::Builder::defaults(Trajectory<PointType> * trajectory)
{
  trajectory->x_interpolator_ = std::make_shared<interpolator::CubicSpline>();
  trajectory->y_interpolator_ = std::make_shared<interpolator::CubicSpline>();
  trajectory->z_interpolator_ = std::make_shared<interpolator::Linear>();
}

tl::expected<Trajectory<PointType>, interpolator::InterpolationFailure>
Trajectory<PointType>::Builder::build(const std::vector<PointType> & points)
{
  auto trajectory_result = trajectory_->build(points);
  if (trajectory_result) {
    auto result = Trajectory(std::move(*trajectory_));
    trajectory_.reset();
    return result;
  }
  return tl::unexpected(trajectory_result.error());
}

}  // namespace autoware::experimental::trajectory
