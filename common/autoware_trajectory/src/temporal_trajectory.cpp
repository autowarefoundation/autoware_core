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

#include "autoware/trajectory/interpolator/linear.hpp"
#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/utils/temporal/find_stop_points.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
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

  std::vector<double> time_bases;
  time_bases.reserve(points.size());
  for (const auto & point : points) {
    time_bases.emplace_back(to_seconds(point.time_from_start));
  }

  time_distance_mapping_.set_time_offset(0.0);
  distance_offset_ = 0.0;
  return set_time_distance_bases(time_bases, spatial_trajectory_.get_underlying_bases());
}

interpolator::InterpolationResult TemporalTrajectory::set_time_distance_bases(
  const std::vector<double> & time_bases, const std::vector<double> & distance_bases)
{
  if (const auto result = time_distance_mapping_.build(time_bases, distance_bases); !result) {
    return result;
  }

  return interpolator::InterpolationSuccess{};
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

void TemporalTrajectory::set_time_offset(const double offset)
{
  time_distance_mapping_.set_time_offset(offset);
}

std::vector<double> TemporalTrajectory::get_underlying_time_bases() const
{
  return time_distance_mapping_.cropped_time_bases();
}

TemporalTrajectory::PointType TemporalTrajectory::compute_from_time(const double t) const
{
  const auto t_clamped = clamp_time(t, true);
  auto point =
    spatial_trajectory_.compute(time_distance_mapping_.distance_at(t_clamped) - distance_offset_);
  point.time_from_start = to_duration_msg(t_clamped);
  return point;
}

std::vector<TemporalTrajectory::PointType> TemporalTrajectory::compute_from_time(
  const std::vector<double> & ts) const
{
  std::vector<PointType> points;
  points.reserve(ts.size());
  for (const auto t : ts) {
    points.emplace_back(compute_from_time(t));
  }
  return points;
}

TemporalTrajectory::PointType TemporalTrajectory::compute_from_distance(const double s) const
{
  auto point = spatial_trajectory_.compute(s);
  const auto t = distance_to_time(s);
  if (t.has_value()) {
    point.time_from_start = to_duration_msg(*t);
  }
  return point;
}

std::vector<TemporalTrajectory::PointType> TemporalTrajectory::compute_from_distance(
  const std::vector<double> & ss) const
{
  std::vector<PointType> points;
  points.reserve(ss.size());
  for (const auto s : ss) {
    points.emplace_back(compute_from_distance(s));
  }
  return points;
}

double TemporalTrajectory::time_to_distance(const double t) const
{
  return time_distance_mapping_.distance_at(clamp_time(t, true)) - distance_offset_;
}

std::optional<double> TemporalTrajectory::distance_to_time(const double s) const
{
  const auto s_clamped = std::clamp(s, 0.0, length());
  const auto absolute_distance = s_clamped + distance_offset_;

  for (const auto & stop_point : find_stop_points(*this)) {
    if (std::abs(s_clamped - stop_point.distance) <= k_same_time_threshold) {
      return stop_point.start_time;
    }
  }
  return time_distance_mapping_.time_at_distance(absolute_distance);
}

std::vector<TemporalTrajectory::PointType> TemporalTrajectory::restore() const
{
  return compute_from_time(get_underlying_time_bases());
}

void TemporalTrajectory::crop_time(const double start_time, const double duration)
{
  const auto clamped_start_time = clamp_time(start_time, false);
  const auto clamped_end_time = clamp_time(start_time + duration, false);
  if (clamped_end_time - clamped_start_time <= k_same_time_threshold) {
    return;
  }

  const auto absolute_start_distance = time_distance_mapping_.distance_at(clamped_start_time);
  const auto absolute_end_distance = time_distance_mapping_.distance_at(clamped_end_time);
  spatial_trajectory_.crop(
    absolute_start_distance - distance_offset_, absolute_end_distance - absolute_start_distance);
  time_distance_mapping_.set_time_range(clamped_start_time, clamped_end_time);
  distance_offset_ = absolute_start_distance;
}

void TemporalTrajectory::set_stopline(const double arc_length)
{
  const auto stop_time = distance_to_time(arc_length);
  if (!stop_time.has_value()) {
    return;
  }

  const auto stop_distance = std::clamp(arc_length, 0.0, length());
  spatial_trajectory_.crop(0.0, stop_distance);
  spatial_trajectory_.longitudinal_velocity_mps().at(spatial_trajectory_.length()).set(0.0);

  if (const auto result = time_distance_mapping_.set_distance_range(
        *stop_time, time_distance_mapping_.end_time(), stop_distance + distance_offset_);
      !result) {
    RCLCPP_WARN(
      rclcpp::get_logger("TemporalTrajectory"), "Failed to update stopline time-distance bases");
  }
}

void TemporalTrajectory::set_stopline(const double arc_length, const double duration)
{
  const auto stop_time = distance_to_time(arc_length);
  if (!stop_time.has_value()) {
    return;
  }

  const auto stop_duration = std::max(duration, 0.0);

  const auto stop_distance = std::clamp(arc_length, 0.0, length());
  spatial_trajectory_.longitudinal_velocity_mps().at(stop_distance).set(0.0);
  if (const auto result = time_distance_mapping_.extend_time_at(*stop_time, stop_duration);
      !result) {
    RCLCPP_WARN(
      rclcpp::get_logger("TemporalTrajectory"),
      "Failed to extend stopline time-distance bases with duration");
    return;
  }
}

const TemporalTrajectory::SpatialTrajectory & TemporalTrajectory::spatial_trajectory() const
{
  return spatial_trajectory_;
}

double TemporalTrajectory::clamp_time(const double t, const bool show_warning) const
{
  return time_distance_mapping_.clamp_time(t, show_warning);
}

TemporalTrajectory::Builder::Builder() : trajectory_(std::make_unique<TemporalTrajectory>())
{
  defaults(trajectory_.get());
}

void TemporalTrajectory::Builder::defaults(TemporalTrajectory * trajectory)
{
  trajectory->time_distance_mapping_.set_interpolator(std::make_shared<interpolator::Linear>());
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
