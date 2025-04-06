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

#include "autoware/trajectory/path_point.hpp"

#include "autoware/trajectory/detail/helpers.hpp"
#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/interpolator/stairstep.hpp"
#include "autoware/trajectory/pose.hpp"

#include <autoware_planning_msgs/msg/path_point.hpp>

#include <memory>
#include <utility>
#include <vector>
namespace autoware::trajectory
{

using PointType = autoware_planning_msgs::msg::PathPoint;

Trajectory<PointType>::Trajectory()
{
  Builder::defaults(this);
}

Trajectory<PointType>::Trajectory(const Trajectory & rhs)
: BaseClass(rhs),
  longitudinal_velocity_mps_(
    std::make_shared<detail::InterpolatedArray<double>>(*rhs.longitudinal_velocity_mps_)),
  lateral_velocity_mps_(
    std::make_shared<detail::InterpolatedArray<double>>(*rhs.lateral_velocity_mps_)),
  heading_rate_rps_(std::make_shared<detail::InterpolatedArray<double>>(*rhs.heading_rate_rps_))
{
}

Trajectory<PointType> & Trajectory<PointType>::operator=(const Trajectory & rhs)
{
  if (this != &rhs) {
    BaseClass::operator=(rhs);
    *longitudinal_velocity_mps_ = *rhs.longitudinal_velocity_mps_;
    *lateral_velocity_mps_ = *rhs.lateral_velocity_mps_;
    *heading_rate_rps_ = *rhs.heading_rate_rps_;
  }
  return *this;
}

interpolator::InterpolationResult Trajectory<PointType>::build(
  const std::vector<PointType> & points)
{
  std::vector<geometry_msgs::msg::Pose> poses;
  std::vector<double> longitudinal_velocity_mps_values;
  std::vector<double> lateral_velocity_mps_values;
  std::vector<double> heading_rate_rps_values;

  for (const auto & point : points) {
    poses.emplace_back(point.pose);
    longitudinal_velocity_mps_values.emplace_back(point.longitudinal_velocity_mps);
    lateral_velocity_mps_values.emplace_back(point.lateral_velocity_mps);
    heading_rate_rps_values.emplace_back(point.heading_rate_rps);
  }

  if (const auto result = Trajectory<geometry_msgs::msg::Pose>::build(poses); !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"failed to interpolate PathPoint::pose"} + result.error());
  }
  if (const auto result = this->longitudinal_velocity_mps().build(
        bases_, std::move(longitudinal_velocity_mps_values));
      !result) {
    return tl::unexpected(interpolator::InterpolationFailure{
      "failed to interpolate PathPoint::longitudinal_velocity_mps"});
  }
  if (const auto result =
        this->lateral_velocity_mps().build(bases_, std::move(lateral_velocity_mps_values));
      !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"failed to interpolate PathPoint::lateral_velocity_mps"});
  }
  if (const auto result =
        this->heading_rate_rps().build(bases_, std::move(heading_rate_rps_values));
      !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"failed to interpolate PathPoint::heading_rate_rps"});
  }

  return interpolator::InterpolationSuccess{};
}

std::vector<double> Trajectory<PointType>::get_internal_bases() const
{
  return get_underlying_bases();
}

std::vector<double> Trajectory<PointType>::get_underlying_bases() const
{
  auto get_bases = [](const auto & interpolated_array) {
    auto [bases, values] = interpolated_array.get_data();
    return bases;
  };

  auto bases = detail::merge_vectors(
    bases_, get_bases(this->longitudinal_velocity_mps()), get_bases(this->lateral_velocity_mps()),
    get_bases(this->heading_rate_rps()));

  bases = detail::crop_bases(bases, start_, end_);
  std::transform(
    bases.begin(), bases.end(), bases.begin(), [this](const double & s) { return s - start_; });
  return bases;
}

PointType Trajectory<PointType>::compute(const double s) const
{
  PointType result;
  result.pose = Trajectory<geometry_msgs::msg::Pose>::compute(s);
  const auto s_clamp = clamp(s);
  result.longitudinal_velocity_mps =
    static_cast<float>(this->longitudinal_velocity_mps().compute(s_clamp));
  result.lateral_velocity_mps = static_cast<float>(this->lateral_velocity_mps().compute(s_clamp));
  result.heading_rate_rps = static_cast<float>(this->heading_rate_rps().compute(s_clamp));
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

std::vector<PointType> Trajectory<PointType>::restore(const size_t min_points) const
{
  std::vector<double> bases = get_underlying_bases();
  bases = detail::fill_bases(bases, min_points);

  std::vector<PointType> points;
  points.reserve(bases.size());
  for (const auto & s : bases) {
    points.emplace_back(compute(s));
  }
  return points;
}

Trajectory<PointType>::Builder::Builder() : trajectory_(std::make_unique<Trajectory<PointType>>())
{
  defaults(trajectory_.get());
}

void Trajectory<PointType>::Builder::defaults(Trajectory<PointType> * trajectory)
{
  BaseClass::Builder::defaults(trajectory);
  trajectory->longitudinal_velocity_mps_ = std::make_shared<detail::InterpolatedArray<double>>(
    std::make_shared<interpolator::Stairstep<double>>());
  trajectory->lateral_velocity_mps_ = std::make_shared<detail::InterpolatedArray<double>>(
    std::make_shared<interpolator::Stairstep<double>>());
  trajectory->heading_rate_rps_ = std::make_shared<detail::InterpolatedArray<double>>(
    std::make_shared<interpolator::Stairstep<double>>());
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

}  // namespace autoware::trajectory
