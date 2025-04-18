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

#include "autoware/trajectory/trajectory_point.hpp"

#include "autoware/trajectory/detail/helpers.hpp"
#include "autoware/trajectory/detail/interpolated_array.hpp"
#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/interpolator/stairstep.hpp"
#include "autoware/trajectory/pose.hpp"
#include "autoware/trajectory/threshold.hpp"

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{

using PointType = autoware_planning_msgs::msg::TrajectoryPoint;

void Trajectory<PointType>::add_base_addition_callback()
{
  longitudinal_velocity_mps_->connect_base_addition_callback(
    [&](const double s) { return this->update_bases(s); });
  lateral_velocity_mps_->connect_base_addition_callback(
    [&](const double s) { return this->update_bases(s); });
  heading_rate_rps_->connect_base_addition_callback(
    [&](const double s) { return this->update_bases(s); });
  acceleration_mps2_->connect_base_addition_callback(
    [&](const double s) { return this->update_bases(s); });
  front_wheel_angle_rad_->connect_base_addition_callback(
    [&](const double s) { return this->update_bases(s); });
  rear_wheel_angle_rad_->connect_base_addition_callback(
    [&](const double s) { return this->update_bases(s); });
}

Trajectory<PointType>::Trajectory()
{
  Builder::defaults(this);
  add_base_addition_callback();
}

Trajectory<PointType>::Trajectory(const Trajectory & rhs)
: BaseClass(rhs),
  longitudinal_velocity_mps_(
    std::make_shared<detail::InterpolatedArray<double>>(*rhs.longitudinal_velocity_mps_)),
  lateral_velocity_mps_(
    std::make_shared<detail::InterpolatedArray<double>>(*rhs.lateral_velocity_mps_)),
  heading_rate_rps_(std::make_shared<detail::InterpolatedArray<double>>(*rhs.heading_rate_rps_)),
  acceleration_mps2_(std::make_shared<detail::InterpolatedArray<double>>(*rhs.acceleration_mps2_)),
  front_wheel_angle_rad_(
    std::make_shared<detail::InterpolatedArray<double>>(*rhs.front_wheel_angle_rad_)),
  rear_wheel_angle_rad_(
    std::make_shared<detail::InterpolatedArray<double>>(*rhs.rear_wheel_angle_rad_))
{
  add_base_addition_callback();
}

Trajectory<PointType>::Trajectory(Trajectory && rhs) noexcept
: BaseClass(std::forward<Trajectory>(rhs)),
  longitudinal_velocity_mps_(std::move(rhs.longitudinal_velocity_mps_)),
  lateral_velocity_mps_(std::move(rhs.lateral_velocity_mps_)),
  heading_rate_rps_(std::move(rhs.heading_rate_rps_)),
  acceleration_mps2_(std::move(rhs.acceleration_mps2_)),
  front_wheel_angle_rad_(std::move(rhs.front_wheel_angle_rad_)),
  rear_wheel_angle_rad_(std::move(rhs.rear_wheel_angle_rad_))
{
  add_base_addition_callback();
}

Trajectory<PointType> & Trajectory<PointType>::operator=(const Trajectory & rhs)
{
  if (this != &rhs) {
    BaseClass::operator=(rhs);
    *longitudinal_velocity_mps_ = *rhs.longitudinal_velocity_mps_;
    *lateral_velocity_mps_ = *rhs.lateral_velocity_mps_;
    *heading_rate_rps_ = *rhs.heading_rate_rps_;
    *acceleration_mps2_ = *rhs.acceleration_mps2_;
    *front_wheel_angle_rad_ = *rhs.front_wheel_angle_rad_;
    *rear_wheel_angle_rad_ = *rhs.rear_wheel_angle_rad_;
    add_base_addition_callback();
  }
  return *this;
}

Trajectory<PointType> & Trajectory<PointType>::operator=(Trajectory && rhs) noexcept
{
  if (this != &rhs) {
    BaseClass::operator=(std::forward<Trajectory>(rhs));
    // cppcheck-suppress accessForwarded
    longitudinal_velocity_mps_ = std::move(rhs.longitudinal_velocity_mps_);
    // cppcheck-suppress accessForwarded
    lateral_velocity_mps_ = std::move(rhs.lateral_velocity_mps_);
    // cppcheck-suppress accessForwarded
    heading_rate_rps_ = std::move(rhs.heading_rate_rps_);
    // cppcheck-suppress accessForwarded
    acceleration_mps2_ = std::move(rhs.acceleration_mps2_);
    // cppcheck-suppress accessForwarded
    front_wheel_angle_rad_ = std::move(rhs.front_wheel_angle_rad_);
    // cppcheck-suppress accessForwarded
    rear_wheel_angle_rad_ = std::move(rhs.rear_wheel_angle_rad_);
    add_base_addition_callback();
  }
  return *this;
}

interpolator::InterpolationResult Trajectory<PointType>::build(
  const std::vector<PointType> & points)
{
  if (
    !longitudinal_velocity_mps_ || !lateral_velocity_mps_ || !heading_rate_rps_ ||
    !acceleration_mps2_ || !front_wheel_angle_rad_ || !rear_wheel_angle_rad_) {
    return tl::unexpected(interpolator::InterpolationFailure{
      "longitudinal_velocity_mps/lateral_velocity_mps/heading_rate_rps/acceleration_mps2/"
      "front_wheel_angle_rad/rear_wheel_angle_rad interpolator are nullptr! check Builder usage"});
  }

  std::vector<geometry_msgs::msg::Pose> poses;
  std::vector<double> longitudinal_velocity_mps_values;
  std::vector<double> lateral_velocity_mps_values;
  std::vector<double> heading_rate_rps_values;
  std::vector<double> acceleration_mps2_values;
  std::vector<double> front_wheel_angle_rad_values;
  std::vector<double> rear_wheel_angle_rad_values;

  for (const auto & point : points) {
    poses.emplace_back(point.pose);
    longitudinal_velocity_mps_values.emplace_back(point.longitudinal_velocity_mps);
    lateral_velocity_mps_values.emplace_back(point.lateral_velocity_mps);
    heading_rate_rps_values.emplace_back(point.heading_rate_rps);
    acceleration_mps2_values.emplace_back(point.acceleration_mps2);
    front_wheel_angle_rad_values.emplace_back(point.front_wheel_angle_rad);
    rear_wheel_angle_rad_values.emplace_back(point.rear_wheel_angle_rad);
  }

  if (const auto result = Trajectory<geometry_msgs::msg::Pose>::build(poses); !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"failed to interpolate TrajectoryPoint::pose"} +
      result.error());
  }
  if (const auto result = this->longitudinal_velocity_mps().build(
        bases_, std::move(longitudinal_velocity_mps_values));
      !result) {
    return tl::unexpected(interpolator::InterpolationFailure{
      "failed to interpolate TrajectoryPoint::longitudinal_velocity_mps"});
  }
  if (const auto result =
        this->lateral_velocity_mps().build(bases_, std::move(lateral_velocity_mps_values));
      !result) {
    return tl::unexpected(interpolator::InterpolationFailure{
      "failed to interpolate TrajectoryPoint::lateral_velocity_mps"});
  }
  if (const auto result =
        this->heading_rate_rps().build(bases_, std::move(heading_rate_rps_values));
      !result) {
    return tl::unexpected(interpolator::InterpolationFailure{
      "failed to interpolate TrajectoryPoint::heading_rate_rps"});
  }
  if (const auto result =
        this->acceleration_mps2().build(bases_, std::move(acceleration_mps2_values));
      !result) {
    return tl::unexpected(interpolator::InterpolationFailure{
      "failed to interpolate TrajectoryPoint::acceleration_mps2"});
  }
  if (const auto result =
        this->front_wheel_angle_rad().build(bases_, std::move(front_wheel_angle_rad_values));
      !result) {
    return tl::unexpected(interpolator::InterpolationFailure{
      "failed to interpolate TrajectoryPoint::front_wheel_angle_rad"});
  }
  if (const auto result =
        this->rear_wheel_angle_rad().build(bases_, std::move(rear_wheel_angle_rad_values));
      !result) {
    return tl::unexpected(interpolator::InterpolationFailure{
      "failed to interpolate TrajectoryPoint::rear_wheel_angle_rad"});
  }

  return interpolator::InterpolationSuccess{};
}

std::vector<double> Trajectory<PointType>::get_internal_bases() const
{
  return get_underlying_bases();
}

std::vector<double> Trajectory<PointType>::get_underlying_bases() const
{
  auto bases = detail::crop_bases(bases_, start_, end_);
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
  result.acceleration_mps2 = static_cast<float>(this->acceleration_mps2().compute(s_clamp));
  result.front_wheel_angle_rad = static_cast<float>(this->front_wheel_angle_rad().compute(s_clamp));
  result.rear_wheel_angle_rad = static_cast<float>(this->rear_wheel_angle_rad().compute(s_clamp));
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

Trajectory<PointType>::Builder::Builder() : trajectory_(std::make_unique<Trajectory>())
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
  trajectory->acceleration_mps2_ = std::make_shared<detail::InterpolatedArray<double>>(
    std::make_shared<interpolator::Stairstep<double>>());
  trajectory->front_wheel_angle_rad_ = std::make_shared<detail::InterpolatedArray<double>>(
    std::make_shared<interpolator::Stairstep<double>>());
  trajectory->rear_wheel_angle_rad_ = std::make_shared<detail::InterpolatedArray<double>>(
    std::make_shared<interpolator::Stairstep<double>>());
}

tl::expected<Trajectory<PointType>, interpolator::InterpolationFailure>
Trajectory<PointType>::Builder::build(const std::vector<PointType> & points)
{
  auto trajectory_result = trajectory_->build(points);
  if (trajectory_result) {
    return std::move(*trajectory_);
  }
  return tl::unexpected(trajectory_result.error());
}

}  // namespace autoware::experimental::trajectory
