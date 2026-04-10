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

bool has_same_time(const double lhs, const double rhs)
{
  return std::abs(lhs - rhs) <= k_same_time_threshold;
}

size_t find_time_index(const std::vector<double> & time_bases, const double time)
{
  return static_cast<size_t>(std::distance(
    time_bases.begin(), std::lower_bound(time_bases.begin(), time_bases.end(), time)));
}

size_t insert_or_replace_point_at_time(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const std::vector<double> & time_bases, const double time,
  const autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  const auto insert_index = find_time_index(time_bases, time);
  if (insert_index == time_bases.size() || !has_same_time(time_bases[insert_index], time)) {
    points.insert(points.begin() + static_cast<std::ptrdiff_t>(insert_index), point);
    return insert_index;
  }
  points[insert_index] = point;
  return insert_index;
}

bool has_time_base(const std::vector<double> & time_bases, const double time)
{
  const auto index = find_time_index(time_bases, time);
  return index < time_bases.size() && has_same_time(time_bases[index], time);
}

TemporalTrajectory::PointType make_stop_point(
  const TemporalTrajectory::SpatialTrajectory & stopped_spatial_trajectory, const double arc_length,
  const double stop_time)
{
  auto stop_point = stopped_spatial_trajectory.compute(arc_length);
  stop_point.time_from_start = to_duration_msg(stop_time);
  return stop_point;
}
}  // namespace

TemporalTrajectory::TemporalTrajectory()
{
  Builder::defaults(this);
}

TemporalTrajectory::TemporalTrajectory(const TemporalTrajectory & rhs)
: spatial_trajectory_(rhs.spatial_trajectory_),
  time_to_distance_(rhs.time_to_distance_ ? rhs.time_to_distance_->clone() : nullptr),
  time_bases_(rhs.time_bases_),
  distance_bases_(rhs.distance_bases_)
{
}

TemporalTrajectory & TemporalTrajectory::operator=(const TemporalTrajectory & rhs)
{
  if (this != &rhs) {
    spatial_trajectory_ = rhs.spatial_trajectory_;
    time_to_distance_ = rhs.time_to_distance_ ? rhs.time_to_distance_->clone() : nullptr;
    time_bases_ = rhs.time_bases_;
    distance_bases_ = rhs.distance_bases_;
  }
  return *this;
}

interpolator::InterpolationResult TemporalTrajectory::build(const std::vector<PointType> & points)
{
  if (!time_to_distance_) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"time_to_distance interpolator is nullptr"});
  }

  if (const auto result = spatial_trajectory_.build(points); !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"failed to interpolate temporal spatial trajectory"} +
      result.error());
  }

  time_bases_.clear();
  time_bases_.reserve(points.size());
  for (const auto & point : points) {
    time_bases_.push_back(to_seconds(point.time_from_start));
  }

  if (time_bases_.empty()) {
    return tl::unexpected(interpolator::InterpolationFailure{"cannot interpolate 0 size points"});
  }
  if (!std::is_sorted(time_bases_.begin(), time_bases_.end())) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"time_from_start must be sorted in ascending order"});
  }

  distance_bases_ = spatial_trajectory_.get_underlying_bases();

  std::vector<double> sanitized_time_bases;
  std::vector<double> sanitized_distance_bases;
  sanitized_time_bases.reserve(time_bases_.size());
  sanitized_distance_bases.reserve(distance_bases_.size());

  for (size_t i = 0; i < time_bases_.size(); ++i) {
    if (
      !sanitized_time_bases.empty() &&
      std::abs(time_bases_[i] - sanitized_time_bases.back()) <= k_same_time_threshold) {
      if (std::abs(distance_bases_[i] - sanitized_distance_bases.back()) > k_same_time_threshold) {
        return tl::unexpected(
          interpolator::InterpolationFailure{
            "time_from_start contains duplicate timestamps with different distances"});
      }
      sanitized_time_bases.back() = time_bases_[i];
      sanitized_distance_bases.back() = distance_bases_[i];
      continue;
    }
    sanitized_time_bases.push_back(time_bases_[i]);
    sanitized_distance_bases.push_back(distance_bases_[i]);
  }

  if (const auto result = time_to_distance_->build(sanitized_time_bases, sanitized_distance_bases);
      !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{
        "failed to interpolate TemporalTrajectory::time_to_distance"} +
      result.error());
  }

  return interpolator::InterpolationSuccess{};
}

double TemporalTrajectory::length() const
{
  return spatial_trajectory_.length();
}

double TemporalTrajectory::duration() const
{
  return time_bases_.empty() ? 0.0 : time_bases_.back() - time_bases_.front();
}

double TemporalTrajectory::start_time() const
{
  return time_bases_.empty() ? 0.0 : time_bases_.front();
}

double TemporalTrajectory::end_time() const
{
  return time_bases_.empty() ? 0.0 : time_bases_.back();
}

std::vector<double> TemporalTrajectory::get_underlying_time_bases() const
{
  return time_bases_;
}

std::vector<double> TemporalTrajectory::get_underlying_distance_bases() const
{
  return distance_bases_;
}

TemporalTrajectory::PointType TemporalTrajectory::compute_from_time(const double t) const
{
  const auto t_clamped = clamp_time(t, true);
  auto point = spatial_trajectory_.compute(time_to_distance_->compute(t_clamped));
  point.time_from_start = to_duration_msg(t_clamped);
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
  return time_to_distance_->compute(clamp_time(t, true));
}

std::vector<double> TemporalTrajectory::time_to_distance(const std::vector<double> & ts) const
{
  std::vector<double> distances;
  distances.reserve(ts.size());
  for (const auto t : ts) {
    distances.push_back(time_to_distance(t));
  }
  return distances;
}

std::optional<double> TemporalTrajectory::distance_to_time(const double s) const
{
  const auto s_clamped = std::clamp(s, 0.0, length());

  if (
    distance_bases_.empty() || time_bases_.empty() ||
    distance_bases_.size() != time_bases_.size()) {
    return std::nullopt;
  }

  for (const auto & stop_point : find_stop_points(*this)) {
    if (std::abs(s_clamped - stop_point.distance) <= k_same_time_threshold) {
      return stop_point.start_time;
    }
  }

  const auto start_time = time_bases_.front();
  const auto end_time = time_bases_.back();
  const auto start_distance = time_to_distance_->compute(start_time);
  const auto end_distance = time_to_distance_->compute(end_time);

  if (
    s_clamped < start_distance - k_same_time_threshold ||
    s_clamped > end_distance + k_same_time_threshold) {
    return std::nullopt;
  }

  if (std::abs(s_clamped - start_distance) <= k_same_time_threshold) {
    return start_time;
  }

  if (std::abs(s_clamped - end_distance) <= k_same_time_threshold) {
    return end_time;
  }

  auto lower_time = start_time;
  auto upper_time = end_time;
  constexpr size_t k_max_iterations = 100;
  for (size_t i = 0; i < k_max_iterations; ++i) {
    const auto mid_time = 0.5 * (lower_time + upper_time);
    const auto mid_distance = time_to_distance_->compute(mid_time);

    if (std::abs(mid_distance - s_clamped) <= k_same_time_threshold) {
      return mid_time;
    }

    if (mid_distance < s_clamped) {
      lower_time = mid_time;
    } else {
      upper_time = mid_time;
    }

    if (upper_time - lower_time <= k_same_time_threshold) {
      break;
    }
  }

  return 0.5 * (lower_time + upper_time);
}

std::vector<TemporalTrajectory::PointType> TemporalTrajectory::restore() const
{
  if (time_bases_.empty()) {
    return {};
  }

  std::vector<double> sanitized_time_bases;
  sanitized_time_bases.reserve(time_bases_.size());
  for (const auto t : time_bases_) {
    if (sanitized_time_bases.empty() || !has_same_time(t, sanitized_time_bases.back())) {
      sanitized_time_bases.push_back(t);
    }
  }

  return compute_from_time(sanitized_time_bases);
}

void TemporalTrajectory::crop_time(const double start_time, const double duration)
{
  if (time_bases_.empty() || duration <= k_same_time_threshold) {
    return;
  }

  const auto absolute_start_time = clamp_time(start_time, false);
  const auto absolute_end_time = clamp_time(start_time + duration, false);
  if (absolute_end_time - absolute_start_time <= k_same_time_threshold) {
    return;
  }

  std::vector<double> time_samples;
  time_samples.reserve(time_bases_.size() + 2);
  time_samples.push_back(absolute_start_time);

  for (const auto t : time_bases_) {
    if (t <= absolute_start_time + k_same_time_threshold) {
      continue;
    }
    if (t >= absolute_end_time - k_same_time_threshold) {
      continue;
    }
    time_samples.push_back(t);
  }
  time_samples.push_back(absolute_end_time);

  std::vector<PointType> points;
  points.reserve(time_samples.size());
  for (const auto t : time_samples) {
    auto point = compute_from_time(t);
    point.time_from_start = to_duration_msg(t - absolute_start_time);
    points.push_back(point);
  }

  (void)build(points);
}

void TemporalTrajectory::set_stopline(const double arc_length)
{
  const auto stop_time = distance_to_time(arc_length);
  if (!stop_time.has_value()) {
    return;
  }

  auto stopped_spatial_trajectory = spatial_trajectory_;
  stopped_spatial_trajectory.set_stopline(arc_length);

  const auto original_points = compute_from_time(time_bases_);
  auto points = original_points;
  const auto stop_point = make_stop_point(stopped_spatial_trajectory, arc_length, *stop_time);
  const auto has_existing_stop_time = has_time_base(time_bases_, *stop_time);
  const auto insert_index =
    insert_or_replace_point_at_time(points, time_bases_, *stop_time, stop_point);

  for (size_t i = insert_index; i < points.size(); ++i) {
    points[i] = stop_point;
    if (i == insert_index) {
      points[i].time_from_start = to_duration_msg(*stop_time);
      continue;
    }

    const auto original_index = has_existing_stop_time ? i : i - 1;
    points[i].time_from_start = original_points[original_index].time_from_start;
  }

  (void)build(points);
}

void TemporalTrajectory::set_stopline(const double arc_length, const double duration)
{
  const auto stop_time = distance_to_time(arc_length);
  if (!stop_time.has_value()) {
    return;
  }

  const auto stop_duration = std::max(duration, 0.0);
  const auto target_time = *stop_time + stop_duration;
  auto stopped_spatial_trajectory = spatial_trajectory_;
  stopped_spatial_trajectory.set_stopline(arc_length);

  const auto original_points = compute_from_time(time_bases_);
  auto points = original_points;
  const auto stop_point = make_stop_point(stopped_spatial_trajectory, arc_length, *stop_time);
  const auto has_existing_stop_time = has_time_base(time_bases_, *stop_time);
  const auto insert_index =
    insert_or_replace_point_at_time(points, time_bases_, *stop_time, stop_point);

  if (stop_duration > k_same_time_threshold) {
    auto delayed_stop_point = stop_point;
    delayed_stop_point.time_from_start = to_duration_msg(target_time);
    points.insert(
      points.begin() + static_cast<std::ptrdiff_t>(insert_index + 1), delayed_stop_point);

    for (size_t i = insert_index + 2; i < points.size(); ++i) {
      const auto original_index = has_existing_stop_time ? i - 1 : i - 2;
      const auto shifted_time =
        to_seconds(original_points[original_index].time_from_start) + stop_duration;
      points[i].time_from_start = to_duration_msg(shifted_time);
    }
  }

  (void)build(points);
}

const TemporalTrajectory::SpatialTrajectory & TemporalTrajectory::spatial_trajectory() const
{
  return spatial_trajectory_;
}

double TemporalTrajectory::clamp_time(const double t, const bool show_warning) const
{
  if (time_bases_.empty()) {
    return t;
  }

  constexpr double eps = 1e-5;
  const auto start = time_bases_.front();
  const auto end = time_bases_.back();
  if (show_warning && (t < start - eps || t > end + eps)) {
    RCLCPP_WARN(
      rclcpp::get_logger("TemporalTrajectory"),
      "The time %f is out of the trajectory duration range [%f, %f]", t, start, end);
  }
  return std::clamp(t, start, end);
}

TemporalTrajectory::Builder::Builder() : trajectory_(std::make_unique<TemporalTrajectory>())
{
  defaults(trajectory_.get());
}

void TemporalTrajectory::Builder::defaults(TemporalTrajectory * trajectory)
{
  trajectory->time_to_distance_ = std::make_shared<interpolator::Linear>();
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
  return std::move(*trajectory_);
}

}  // namespace autoware::experimental::trajectory
