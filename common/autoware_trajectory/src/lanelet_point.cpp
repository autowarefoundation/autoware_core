// Copyright 2025 TIER IV, Inc.
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

#include "autoware/trajectory/lanelet_point.hpp"

#include "autoware/trajectory/detail/helpers.hpp"
#include "autoware/trajectory/interpolator/cubic_spline.hpp"
#include "autoware/trajectory/interpolator/linear.hpp"
#include "autoware/trajectory/threshold.hpp"

#include <memory>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{

using PointType = lanelet::ConstPoint3d;

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
  xs.emplace_back(points[0].x());
  ys.emplace_back(points[0].y());
  zs.emplace_back(points[0].z());

  for (size_t i = 1; i < points.size(); ++i) {
    /**
       NOTE:
       this sanitization is essential for avoiding zero division and NaN in later interpolation.

       if there are 100 points with the interval of 1nm, then they are treated as 100 points with
       the interval of k_points_minimum_dist_threshold and interpolation is continued.
    */
    const auto dist = std::max<double>(
      std::hypot(
        points[i].x() - points[i - 1].x(), points[i].y() - points[i - 1].y(),
        points[i].z() - points[i - 1].z()),
      k_points_minimum_dist_threshold);
    bases_.emplace_back(bases_.back() + dist);
    xs.emplace_back(points[i].x());
    ys.emplace_back(points[i].y());
    zs.emplace_back(points[i].z());
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

std::vector<double> Trajectory<PointType>::get_underlying_bases() const
{
  auto bases = detail::crop_bases(bases_, start_, end_);
  std::transform(
    bases.begin(), bases.end(), bases.begin(), [this](const double s) { return s - start_; });
  return bases;
}

PointType Trajectory<PointType>::compute(const double s) const
{
  const auto s_clamp = clamp(s, true);
  lanelet::Point3d result;
  result.setId(lanelet::InvalId);
  result.x() = x_interpolator_->compute(s_clamp);
  result.y() = y_interpolator_->compute(s_clamp);
  result.z() = z_interpolator_->compute(s_clamp);
  return result;
}

double Trajectory<PointType>::azimuth(const double s) const
{
  const auto s_clamp = clamp(s, true);
  const double dx = x_interpolator_->compute_first_derivative(s_clamp);
  const double dy = y_interpolator_->compute_first_derivative(s_clamp);
  return std::atan2(dy, dx);
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
