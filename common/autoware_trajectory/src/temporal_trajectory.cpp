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

#include "autoware/trajectory/detail/helpers.hpp"
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

bool has_same_time(const double lhs, const double rhs)
{
  return std::abs(lhs - rhs) <= k_same_time_threshold;
}

size_t find_time_index(const std::vector<double> & time_bases, const double time)
{
  return static_cast<size_t>(std::distance(
    time_bases.begin(), std::lower_bound(time_bases.begin(), time_bases.end(), time)));
}

void insert_or_replace_base_at_time(
  std::vector<double> & time_bases, std::vector<double> & values, const double time,
  const double value)
{
  const auto insert_index = find_time_index(time_bases, time);
  if (insert_index == time_bases.size() || !has_same_time(time_bases.at(insert_index), time)) {
    time_bases.insert(time_bases.begin() + static_cast<std::ptrdiff_t>(insert_index), time);
    values.insert(values.begin() + static_cast<std::ptrdiff_t>(insert_index), value);
    return;
  }

  time_bases.at(insert_index) = time;
  values.at(insert_index) = value;
}
}  // namespace

TemporalTrajectory::TimeDistanceMapping::TimeDistanceMapping(
  const TemporalTrajectory::TimeDistanceMapping & rhs)
: interpolator_(rhs.interpolator_ ? rhs.interpolator_->clone() : nullptr),
  time_bases_(rhs.time_bases_),
  distance_bases_(rhs.distance_bases_)
{
}

TemporalTrajectory::TimeDistanceMapping & TemporalTrajectory::TimeDistanceMapping::operator=(
  const TemporalTrajectory::TimeDistanceMapping & rhs)
{
  if (this != &rhs) {
    interpolator_ = rhs.interpolator_ ? rhs.interpolator_->clone() : nullptr;
    time_bases_ = rhs.time_bases_;
    distance_bases_ = rhs.distance_bases_;
  }
  return *this;
}

void TemporalTrajectory::TimeDistanceMapping::set_interpolator(
  std::shared_ptr<TemporalTrajectory::InterpolatorInterface> interpolator)
{
  interpolator_ = std::move(interpolator);
}

bool TemporalTrajectory::TimeDistanceMapping::has_interpolator() const
{
  return static_cast<bool>(interpolator_);
}

interpolator::InterpolationResult TemporalTrajectory::TimeDistanceMapping::build(
  const std::vector<double> & time_bases, const std::vector<double> & distance_bases)
{
  if (!interpolator_) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"time_to_distance interpolator is nullptr"});
  }

  if (time_bases.empty()) {
    return tl::unexpected(interpolator::InterpolationFailure{"cannot interpolate 0 size points"});
  }
  if (time_bases.size() != distance_bases.size()) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"time and distance bases must have the same size"});
  }
  if (!std::is_sorted(time_bases.begin(), time_bases.end())) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"time_from_start must be sorted in ascending order"});
  }

  time_bases_ = time_bases;
  distance_bases_ = distance_bases;

  std::vector<double> sanitized_time_bases;
  std::vector<double> sanitized_distance_bases;
  sanitized_time_bases.reserve(time_bases_.size());
  sanitized_distance_bases.reserve(distance_bases_.size());

  for (size_t i = 0; i < time_bases_.size(); ++i) {
    if (
      !sanitized_time_bases.empty() &&
      std::abs(time_bases_.at(i) - sanitized_time_bases.back()) <= k_same_time_threshold) {
      if (
        std::abs(distance_bases_.at(i) - sanitized_distance_bases.back()) > k_same_time_threshold) {
        return tl::unexpected(
          interpolator::InterpolationFailure{
            "time_from_start contains duplicate timestamps with different distances"});
      }
      sanitized_time_bases.back() = time_bases_.at(i);
      sanitized_distance_bases.back() = distance_bases_.at(i);
      continue;
    }
    sanitized_time_bases.push_back(time_bases_.at(i));
    sanitized_distance_bases.push_back(distance_bases_.at(i));
  }

  if (const auto result = interpolator_->build(sanitized_time_bases, sanitized_distance_bases);
      !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{
        "failed to interpolate TemporalTrajectory::time_to_distance"} +
      result.error());
  }

  return interpolator::InterpolationSuccess{};
}

double TemporalTrajectory::TimeDistanceMapping::compute_distance(const double time) const
{
  return interpolator_->compute(time);
}

bool TemporalTrajectory::TimeDistanceMapping::empty() const
{
  return time_bases_.empty();
}

const std::vector<double> & TemporalTrajectory::TimeDistanceMapping::time_bases() const
{
  return time_bases_;
}

const std::vector<double> & TemporalTrajectory::TimeDistanceMapping::distance_bases() const
{
  return distance_bases_;
}

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
    time_bases.push_back(to_seconds(point.time_from_start));
  }

  time_offset_ = 0.0;
  distance_offset_ = 0.0;
  return set_time_distance_bases(time_bases, spatial_trajectory_.get_underlying_bases());
}

interpolator::InterpolationResult TemporalTrajectory::set_time_distance_bases(
  const std::vector<double> & time_bases, const std::vector<double> & distance_bases)
{
  if (const auto result = time_distance_mapping_.build(time_bases, distance_bases); !result) {
    return result;
  }

  start_time_ = time_distance_mapping_.time_bases().front();
  end_time_ = time_distance_mapping_.time_bases().back();
  return interpolator::InterpolationSuccess{};
}

double TemporalTrajectory::length() const
{
  return spatial_trajectory_.length();
}

double TemporalTrajectory::duration() const
{
  return time_distance_mapping_.empty() ? 0.0 : end_time_ - start_time_;
}

double TemporalTrajectory::start_time() const
{
  return time_distance_mapping_.empty() ? 0.0 : start_time_ - time_offset_;
}

double TemporalTrajectory::end_time() const
{
  return time_distance_mapping_.empty() ? 0.0 : end_time_ - time_offset_;
}

double TemporalTrajectory::time_offset() const
{
  return time_offset_;
}

void TemporalTrajectory::set_time_offset(const double offset)
{
  time_offset_ = offset;
}

std::vector<double> TemporalTrajectory::get_underlying_time_bases() const
{
  auto time_bases = detail::crop_bases(time_distance_mapping_.time_bases(), start_time_, end_time_);
  std::transform(time_bases.begin(), time_bases.end(), time_bases.begin(), [this](const double t) {
    return t - time_offset_;
  });
  return time_bases;
}

std::vector<double> TemporalTrajectory::get_underlying_distance_bases() const
{
  if (time_distance_mapping_.empty()) {
    return {};
  }

  std::vector<double> distance_bases;
  const auto & time_bases = time_distance_mapping_.time_bases();
  const auto & mapped_distance_bases = time_distance_mapping_.distance_bases();

  if (time_bases.empty() || time_bases.size() != mapped_distance_bases.size()) {
    return {};
  }

  if (std::none_of(time_bases.begin(), time_bases.end(), [this](const double t) {
        return has_same_time(t, start_time_);
      })) {
    distance_bases.push_back(
      time_distance_mapping_.compute_distance(start_time_) - distance_offset_);
  }

  for (size_t i = 0; i < time_bases.size(); ++i) {
    if (time_bases.at(i) >= start_time_ && time_bases.at(i) <= end_time_) {
      distance_bases.push_back(mapped_distance_bases.at(i) - distance_offset_);
    }
  }

  if (std::none_of(time_bases.begin(), time_bases.end(), [this](const double t) {
        return has_same_time(t, end_time_);
      })) {
    distance_bases.push_back(time_distance_mapping_.compute_distance(end_time_) - distance_offset_);
  }

  return distance_bases;
}

TemporalTrajectory::PointType TemporalTrajectory::compute_from_time(const double t) const
{
  const auto t_clamped = clamp_time(t, true);
  auto point = spatial_trajectory_.compute(
    time_distance_mapping_.compute_distance(t_clamped) - distance_offset_);
  point.time_from_start = to_duration_msg(t_clamped - time_offset_);
  return point;
}

std::vector<TemporalTrajectory::PointType> TemporalTrajectory::compute_from_time(
  const std::vector<double> & ts) const
{
  std::vector<PointType> points;
  points.reserve(ts.size());
  for (const auto t : ts) {
    points.push_back(compute_from_time(t));
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
    points.push_back(compute_from_distance(s));
  }
  return points;
}

double TemporalTrajectory::time_to_distance(const double t) const
{
  return time_distance_mapping_.compute_distance(clamp_time(t, true)) - distance_offset_;
}

std::optional<double> TemporalTrajectory::distance_to_time(const double s) const
{
  const auto s_clamped = std::clamp(s, 0.0, length());
  const auto absolute_distance = s_clamped + distance_offset_;

  if (
    time_distance_mapping_.distance_bases().empty() ||
    time_distance_mapping_.time_bases().empty() ||
    time_distance_mapping_.distance_bases().size() != time_distance_mapping_.time_bases().size()) {
    return std::nullopt;
  }

  for (const auto & stop_point : find_stop_points(*this)) {
    if (std::abs(s_clamped - stop_point.distance) <= k_same_time_threshold) {
      return stop_point.start_time;
    }
  }

  const auto start_time = start_time_;
  const auto end_time = end_time_;
  const auto start_distance = time_distance_mapping_.compute_distance(start_time);
  const auto end_distance = time_distance_mapping_.compute_distance(end_time);

  if (
    absolute_distance < start_distance - k_same_time_threshold ||
    absolute_distance > end_distance + k_same_time_threshold) {
    return std::nullopt;
  }

  if (std::abs(absolute_distance - start_distance) <= k_same_time_threshold) {
    return start_time - time_offset_;
  }

  if (std::abs(absolute_distance - end_distance) <= k_same_time_threshold) {
    return end_time - time_offset_;
  }

  auto lower_time = start_time;
  auto upper_time = end_time;
  constexpr size_t k_max_iterations = 100;
  for (size_t i = 0; i < k_max_iterations; ++i) {
    const auto mid_time = 0.5 * (lower_time + upper_time);
    const auto mid_distance = time_distance_mapping_.compute_distance(mid_time);

    if (std::abs(mid_distance - absolute_distance) <= k_same_time_threshold) {
      return mid_time - time_offset_;
    }

    if (mid_distance < absolute_distance) {
      lower_time = mid_time;
    } else {
      upper_time = mid_time;
    }

    if (upper_time - lower_time <= k_same_time_threshold) {
      break;
    }
  }

  return 0.5 * (lower_time + upper_time) - time_offset_;
}

std::vector<TemporalTrajectory::PointType> TemporalTrajectory::restore() const
{
  if (time_distance_mapping_.empty()) {
    return {};
  }

  const auto time_bases = get_underlying_time_bases();
  std::vector<double> sanitized_time_bases;
  sanitized_time_bases.reserve(time_bases.size());
  for (const auto t : time_bases) {
    if (sanitized_time_bases.empty() || !has_same_time(t, sanitized_time_bases.back())) {
      sanitized_time_bases.push_back(t);
    }
  }

  return compute_from_time(sanitized_time_bases);
}

void TemporalTrajectory::crop_time(const double start_time, const double duration)
{
  if (time_distance_mapping_.empty() || duration <= k_same_time_threshold) {
    return;
  }

  const auto absolute_start_time = clamp_time(start_time, false);
  const auto absolute_end_time = clamp_time(start_time + duration, false);
  if (absolute_end_time - absolute_start_time <= k_same_time_threshold) {
    return;
  }

  const auto absolute_start_distance = time_distance_mapping_.compute_distance(absolute_start_time);
  const auto absolute_end_distance = time_distance_mapping_.compute_distance(absolute_end_time);
  spatial_trajectory_.crop(
    absolute_start_distance - distance_offset_, absolute_end_distance - absolute_start_distance);
  start_time_ = absolute_start_time;
  end_time_ = absolute_end_time;
  distance_offset_ = absolute_start_distance;
}

void TemporalTrajectory::set_stopline(const double arc_length)
{
  const auto stop_time = distance_to_time(arc_length);
  if (!stop_time.has_value()) {
    return;
  }

  auto time_bases = get_underlying_time_bases();
  auto distance_bases = get_underlying_distance_bases();
  if (time_bases.empty()) {
    return;
  }

  const auto stop_distance = std::clamp(arc_length, 0.0, length());
  spatial_trajectory_.crop(0.0, stop_distance);
  spatial_trajectory_.longitudinal_velocity_mps().at(spatial_trajectory_.length()).set(0.0);
  insert_or_replace_base_at_time(time_bases, distance_bases, *stop_time, stop_distance);

  const auto stop_index = find_time_index(time_bases, *stop_time);
  std::fill(
    distance_bases.begin() + static_cast<std::ptrdiff_t>(stop_index), distance_bases.end(),
    stop_distance);

  std::transform(time_bases.begin(), time_bases.end(), time_bases.begin(), [this](const double t) {
    return t + time_offset_;
  });
  std::transform(
    distance_bases.begin(), distance_bases.end(), distance_bases.begin(),
    [this](const double s) { return s + distance_offset_; });

  if (const auto result = set_time_distance_bases(time_bases, distance_bases); !result) {
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
  const auto target_time = *stop_time + stop_duration;
  auto time_bases = get_underlying_time_bases();
  auto distance_bases = get_underlying_distance_bases();
  if (time_bases.empty()) {
    return;
  }

  const auto stop_distance = std::clamp(arc_length, 0.0, length());
  spatial_trajectory_.longitudinal_velocity_mps().at(stop_distance).set(0.0);
  insert_or_replace_base_at_time(time_bases, distance_bases, *stop_time, stop_distance);
  const auto stop_index = find_time_index(time_bases, *stop_time);

  if (stop_duration > k_same_time_threshold) {
    time_bases.insert(
      time_bases.begin() + static_cast<std::ptrdiff_t>(stop_index + 1), target_time);
    distance_bases.insert(
      distance_bases.begin() + static_cast<std::ptrdiff_t>(stop_index + 1), stop_distance);

    for (size_t i = stop_index + 2; i < time_bases.size(); ++i) {
      time_bases.at(i) += stop_duration;
    }
  }

  std::transform(time_bases.begin(), time_bases.end(), time_bases.begin(), [this](const double t) {
    return t + time_offset_;
  });
  std::transform(
    distance_bases.begin(), distance_bases.end(), distance_bases.begin(),
    [this](const double s) { return s + distance_offset_; });

  if (const auto result = set_time_distance_bases(time_bases, distance_bases); !result) {
    RCLCPP_WARN(
      rclcpp::get_logger("TemporalTrajectory"),
      "Failed to update stopline time-distance bases with duration");
  }
}

const TemporalTrajectory::SpatialTrajectory & TemporalTrajectory::spatial_trajectory() const
{
  return spatial_trajectory_;
}

double TemporalTrajectory::clamp_time(const double t, const bool show_warning) const
{
  if (time_distance_mapping_.empty()) {
    return t;
  }

  constexpr double eps = 1e-5;
  const auto public_start = start_time_ - time_offset_;
  const auto public_end = end_time_ - time_offset_;
  if (show_warning && (t < public_start - eps || t > public_end + eps)) {
    RCLCPP_WARN(
      rclcpp::get_logger("TemporalTrajectory"),
      "The time %f is out of the trajectory duration range [%f, %f]", t, public_start, public_end);
  }
  return std::clamp(t, public_start, public_end) + time_offset_;
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
