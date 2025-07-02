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

#include "autoware/trajectory/path_point_with_lane_id.hpp"

#include "autoware/trajectory/detail/helpers.hpp"
#include "autoware/trajectory/interpolator/lane_ids_interpolator.hpp"
#include "autoware/trajectory/threshold.hpp"

#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{

using PointType = autoware_internal_planning_msgs::msg::PathPointWithLaneId;

void Trajectory<PointType>::add_base_addition_callback()
{
  BaseClass::add_base_addition_callback();
  lane_ids_->connect_base_addition_callback([&](const double s) { return this->update_bases(s); });
}

Trajectory<PointType>::Trajectory()
{
  Builder::defaults(this);
  add_base_addition_callback();
}

Trajectory<PointType>::Trajectory(const Trajectory & rhs)
: BaseClass(rhs), lane_ids_(std::make_shared<detail::InterpolatedArray<LaneIdType>>(*rhs.lane_ids_))
{
  add_base_addition_callback();
}

Trajectory<PointType>::Trajectory(Trajectory && rhs) noexcept
: BaseClass(std::forward<Trajectory>(rhs)), lane_ids_(std::move(rhs.lane_ids_))
{
  add_base_addition_callback();
}

Trajectory<PointType> & Trajectory<PointType>::operator=(const Trajectory & rhs)
{
  if (this != &rhs) {
    BaseClass::operator=(rhs);
    lane_ids_ = std::make_shared<detail::InterpolatedArray<LaneIdType>>(this->lane_ids());
    add_base_addition_callback();
  }
  return *this;
}

Trajectory<PointType> & Trajectory<PointType>::operator=(Trajectory && rhs) noexcept
{
  if (this != &rhs) {
    BaseClass::operator=(std::forward<Trajectory>(rhs));
    // cppcheck-suppress accessForwarded
    lane_ids_ = std::move(rhs.lane_ids_);
    add_base_addition_callback();
  }
  return *this;
}

interpolator::InterpolationResult Trajectory<PointType>::build(
  const std::vector<PointType> & points)
{
  std::vector<autoware_planning_msgs::msg::PathPoint> path_points;
  std::vector<std::vector<int64_t>> lane_ids_values;

  for (const auto & point : points) {
    path_points.emplace_back(point.point);
    lane_ids_values.emplace_back(point.lane_ids);
  }

  if (const auto result = BaseClass::build(path_points); !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"failed to interpolate PathPointWithLaneId::point"} +
      result.error());
  }
  if (const auto result = lane_ids().build(bases_, std::move(lane_ids_values)); !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"failed to interpolate PathPointWithLaneId::lane_id"});
  }

  return interpolator::InterpolationSuccess{};
}

std::vector<int64_t> Trajectory<PointType>::get_contained_lane_ids() const
{
  std::vector<int64_t> contained_lane_ids;
  const auto & [bases, values] = lane_ids_->get_data();

  for (size_t i = 0; i < bases.size(); i++) {
    if (start_ <= bases[i] && bases[i] <= end_) {
      for (const auto & lane_id : values[i]) {
        if (contained_lane_ids.empty()) {
          contained_lane_ids.emplace_back(lane_id);
        } else {
          int64_t last_lane_id = contained_lane_ids.back();

          if (last_lane_id != lane_id) {
            contained_lane_ids.emplace_back(lane_id);
          }
        }
      }
    }
  }
  return contained_lane_ids;
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
  result.point = BaseClass::compute(s);
  const auto s_clamp = clamp(s);
  result.lane_ids = lane_ids().compute(s_clamp);
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

Trajectory<PointType>::Builder::Builder() : trajectory_(std::make_unique<Trajectory<PointType>>())
{
  defaults(trajectory_.get());
}

void Trajectory<PointType>::Builder::defaults(Trajectory<PointType> * trajectory)
{
  BaseClass::Builder::defaults(trajectory);
  trajectory->lane_ids_ = std::make_shared<detail::InterpolatedArray<LaneIdType>>(
    std::make_shared<interpolator::LaneIdsInterpolator>());
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
