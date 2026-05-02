// Copyright 2026 TIER IV, Inc.
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

#include "autoware/trajectory/temporal_trajectory.hpp"

#include "autoware/trajectory/detail/validate_range.hpp"
#include "autoware/trajectory/interpolator/pchip.hpp"
#include "autoware/trajectory/threshold.hpp"

#include <range/v3/to_container.hpp>
#include <range/v3/view/transform.hpp>

#include <cmath>
#include <memory>
#include <utility>
#include <vector>
namespace autoware::experimental::trajectory
{
namespace
{
double to_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return rclcpp::Duration(duration).seconds();
}

builtin_interfaces::msg::Duration to_duration_msg(const double seconds)
{
  return rclcpp::Duration::from_seconds(seconds);
}

}  // namespace

TemporalTrajectory::TemporalTrajectory()
{
  Builder::defaults(this);
}

interpolator::InterpolationResult TemporalTrajectory::build(const std::vector<PointType> & points)
{
  if (const auto result = spatial_trajectory_.build(points); !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"failed to interpolate temporal spatial trajectory"} +
      result.error());
  }

  auto time_bases =
    points |
    ranges::views::transform([](const auto & point) { return to_seconds(point.time_from_start); }) |
    ranges::to<std::vector<double>>();

  time_distance_mapping_.set_time_offset(0.0);
  distance_offset_ = 0.0;
  return time_distance_mapping_.build(time_bases, spatial_trajectory_.get_underlying_bases());
}

double TemporalTrajectory::length() const
{
  return spatial_trajectory_.length();
}

double TemporalTrajectory::duration() const
{
  return time_distance_mapping_.duration();
}

double TemporalTrajectory::start_time() const
{
  return time_distance_mapping_.start_time();
}

double TemporalTrajectory::end_time() const
{
  return time_distance_mapping_.end_time();
}

double TemporalTrajectory::time_offset() const
{
  return time_distance_mapping_.time_offset();
}

std::vector<double> TemporalTrajectory::get_underlying_time_bases() const
{
  return time_distance_mapping_.cropped_time_bases();
}

TemporalTrajectory::PointType TemporalTrajectory::compute_from_time(const double t) const
{
  detail::throw_if_out_of_range(t, start_time(), end_time(), "time");
  auto point =
    spatial_trajectory_.compute(time_distance_mapping_.distance_at(t) - distance_offset_);
  point.time_from_start = to_duration_msg(t);
  return point;
}

TemporalTrajectory::PointType TemporalTrajectory::compute_from_distance(const double s) const
{
  detail::throw_if_out_of_range(s, 0.0, length(), "distance");
  auto point = spatial_trajectory_.compute(s);
  const auto t = distance_to_time(s);
  point.time_from_start = to_duration_msg(t);
  return point;
}

double TemporalTrajectory::azimuth_from_time(const double t) const
{
  return spatial_trajectory_.azimuth(time_to_distance(t));
}

double TemporalTrajectory::azimuth_from_distance(const double s) const
{
  detail::throw_if_out_of_range(s, 0.0, length(), "distance");
  return spatial_trajectory_.azimuth(s);
}

double TemporalTrajectory::elevation_from_time(const double t) const
{
  return spatial_trajectory_.elevation(time_to_distance(t));
}

double TemporalTrajectory::elevation_from_distance(const double s) const
{
  detail::throw_if_out_of_range(s, 0.0, length(), "distance");
  return spatial_trajectory_.elevation(s);
}

double TemporalTrajectory::curvature_from_time(const double t) const
{
  return spatial_trajectory_.curvature(time_to_distance(t));
}

double TemporalTrajectory::curvature_from_distance(const double s) const
{
  detail::throw_if_out_of_range(s, 0.0, length(), "distance");
  return spatial_trajectory_.curvature(s);
}

double TemporalTrajectory::time_to_distance(const double t) const
{
  detail::throw_if_out_of_range(t, start_time(), end_time(), "time");
  return time_distance_mapping_.distance_at(t) - distance_offset_;
}

double TemporalTrajectory::distance_to_time(const double s, const bool return_end_time) const
{
  detail::throw_if_out_of_range(s, 0.0, length(), "distance");
  const auto absolute_distance = s + distance_offset_;
  return time_distance_mapping_.time_at_distance(absolute_distance, return_end_time);
}

std::vector<TemporalTrajectory::PointType> TemporalTrajectory::restore() const
{
  const auto time_bases = get_underlying_time_bases();
  return time_bases |
         ranges::views::transform([this](const double t) { return compute_from_time(t); }) |
         ranges::to<std::vector>();
}

const TemporalTrajectory::SpatialTrajectory & TemporalTrajectory::spatial_trajectory() const
{
  return spatial_trajectory_;
}

TemporalTrajectory::Builder::Builder() : trajectory_(std::make_unique<TemporalTrajectory>())
{
  defaults(trajectory_.get());
}

void TemporalTrajectory::Builder::defaults(TemporalTrajectory * trajectory)
{
  trajectory->time_distance_mapping_.set_interpolator(
    std::make_shared<interpolator::Pchip>(k_epsilon_time));
}

tl::expected<TemporalTrajectory, interpolator::InterpolationFailure>
TemporalTrajectory::Builder::build(const std::vector<PointType> & points)
{
  const auto spatial_trajectory_result = spatial_trajectory_builder_.build(points);
  if (!spatial_trajectory_result) {
    return tl::unexpected(spatial_trajectory_result.error());
  }

  trajectory_->spatial_trajectory_ = spatial_trajectory_result.value();
  if (const auto result = trajectory_->build(points); !result) {
    return tl::unexpected(result.error());
  }
  auto result = TemporalTrajectory(std::move(*trajectory_));
  trajectory_.reset();
  return result;
}

}  // namespace autoware::experimental::trajectory
