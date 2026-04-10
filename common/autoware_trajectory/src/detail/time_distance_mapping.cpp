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

#include "autoware/trajectory/detail/time_distance_mapping.hpp"

#include "autoware/trajectory/detail/helpers.hpp"
#include "autoware/trajectory/interpolator/nearest_neighbor.hpp"
#include "autoware/trajectory/threshold.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory::detail
{
namespace
{
constexpr double k_minimum_time_base_extension = k_same_time_threshold;

size_t find_time_index(const std::vector<double> & time_bases, const double time)
{
  return static_cast<size_t>(std::distance(
    time_bases.begin(), std::lower_bound(time_bases.begin(), time_bases.end(), time)));
}

size_t insert_or_replace_base_at_time(
  std::vector<double> & time_bases, std::vector<double> & values, const double time,
  const double value)
{
  const auto insert_index = find_time_index(time_bases, time);
  if (
    insert_index == time_bases.size() ||
    std::abs(time_bases.at(insert_index) - time) > k_same_time_threshold) {
    time_bases.insert(time_bases.begin() + static_cast<std::ptrdiff_t>(insert_index), time);
    values.insert(values.begin() + static_cast<std::ptrdiff_t>(insert_index), value);
    return insert_index;
  }

  time_bases.at(insert_index) = time;
  values.at(insert_index) = value;
  return insert_index;
}
}  // namespace

interpolator::InterpolationResult TimeDistanceMapping::extend_time_at(
  const double time, const double delta_time)
{
  if (delta_time <= k_same_time_threshold) {
    return interpolator::InterpolationSuccess{};
  }

  auto time_bases = cropped_time_bases();
  auto distance_bases = cropped_absolute_distance_bases();
  const auto clamped_time = clamp_time_without_warning(time);
  const auto pivot_distance = distance_at(clamped_time);
  const auto pivot_index =
    insert_or_replace_base_at_time(time_bases, distance_bases, clamped_time, pivot_distance);
  for (size_t i = pivot_index + 1; i < time_bases.size(); ++i) {
    time_bases.at(i) += delta_time;
  }

  const auto end_index = insert_or_replace_base_at_time(
    time_bases, distance_bases, clamped_time + delta_time, pivot_distance);
  std::fill(
    distance_bases.begin() + static_cast<std::ptrdiff_t>(pivot_index),
    distance_bases.begin() + static_cast<std::ptrdiff_t>(end_index + 1), pivot_distance);

  std::transform(time_bases.begin(), time_bases.end(), time_bases.begin(), [this](const double t) {
    return to_internal_time(t);
  });
  return build(time_bases, distance_bases);
}

interpolator::InterpolationResult TimeDistanceMapping::set_distance_range(
  const double start_time, const double end_time, const double distance)
{
  if (empty()) {
    return interpolator::InterpolationSuccess{};
  }

  auto time_bases = cropped_time_bases();
  auto distance_bases = cropped_absolute_distance_bases();

  const auto clamped_start = clamp_time_without_warning(start_time);
  const auto range_end = std::max(end_time, clamped_start);
  const auto clamped_end =
    (range_end <= this->end_time()) ? clamp_time_without_warning(range_end) : range_end;

  const auto start_index =
    insert_or_replace_base_at_time(time_bases, distance_bases, clamped_start, distance);
  const auto end_index =
    insert_or_replace_base_at_time(time_bases, distance_bases, clamped_end, distance);
  std::fill(
    distance_bases.begin() + static_cast<std::ptrdiff_t>(start_index),
    distance_bases.begin() + static_cast<std::ptrdiff_t>(end_index + 1), distance);

  std::transform(time_bases.begin(), time_bases.end(), time_bases.begin(), [this](const double t) {
    return to_internal_time(t);
  });
  return build(time_bases, distance_bases);
}

TimeDistanceMapping::TimeDistanceMapping(const TimeDistanceMapping & rhs)
: interpolator_(rhs.interpolator_ ? rhs.interpolator_->clone() : nullptr),
  time_bases_(rhs.time_bases_),
  distance_bases_(rhs.distance_bases_),
  start_time_(rhs.start_time_),
  end_time_(rhs.end_time_),
  time_offset_(rhs.time_offset_)
{
}

TimeDistanceMapping & TimeDistanceMapping::operator=(const TimeDistanceMapping & rhs)
{
  if (this != &rhs) {
    interpolator_ = rhs.interpolator_ ? rhs.interpolator_->clone() : nullptr;
    time_bases_ = rhs.time_bases_;
    distance_bases_ = rhs.distance_bases_;
    start_time_ = rhs.start_time_;
    end_time_ = rhs.end_time_;
    time_offset_ = rhs.time_offset_;
  }
  return *this;
}

void TimeDistanceMapping::set_interpolator(std::shared_ptr<InterpolatorInterface> interpolator)
{
  interpolator_ = std::move(interpolator);
}

interpolator::InterpolationResult TimeDistanceMapping::build(
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

  time_bases_.clear();
  distance_bases_.clear();
  time_bases_.reserve(time_bases.size());
  distance_bases_.reserve(distance_bases.size());

  for (size_t i = 0; i < time_bases.size(); ++i) {
    auto sanitized_time = time_bases.at(i);
    if (!time_bases_.empty() && (sanitized_time - time_bases_.back()) <= k_same_time_threshold) {
      sanitized_time = time_bases_.back() + k_minimum_time_base_extension;
    }

    time_bases_.emplace_back(sanitized_time);
    distance_bases_.emplace_back(distance_bases.at(i));
  }

  if (const auto result = detail::build_with_fallback(
        interpolator_, time_bases_, distance_bases_,
        [] { return std::make_shared<interpolator::NearestNeighbor<double>>(); });
      !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{
        "failed to interpolate TemporalTrajectory::time_to_distance"} +
      result.error());
  }

  start_time_ = time_bases_.front();
  end_time_ = time_bases_.back();

  return interpolator::InterpolationSuccess{};
}

double TimeDistanceMapping::distance_at(const double time) const
{
  return compute_distance(to_internal_time(clamp_time(time)));
}

double TimeDistanceMapping::compute_distance(const double time) const
{
  return interpolator_->compute(time);
}

double TimeDistanceMapping::duration() const
{
  return end_time_ - start_time_;
}

double TimeDistanceMapping::start_time() const
{
  return start_time_ - time_offset_;
}

double TimeDistanceMapping::end_time() const
{
  return end_time_ - time_offset_;
}

double TimeDistanceMapping::start_distance() const
{
  return compute_distance(start_time_);
}

double TimeDistanceMapping::end_distance() const
{
  return compute_distance(end_time_);
}

double TimeDistanceMapping::time_offset() const
{
  return time_offset_;
}

void TimeDistanceMapping::set_time_offset(const double offset)
{
  time_offset_ = offset;
}

void TimeDistanceMapping::set_time_range(const double start_time, const double end_time)
{
  const auto clamped_start_time = clamp_time(start_time);
  const auto clamped_end_time = std::max(clamp_time(end_time), clamped_start_time);
  start_time_ = to_internal_time(clamped_start_time);
  end_time_ = to_internal_time(clamped_end_time);
}

double TimeDistanceMapping::clamp_time(const double time, const bool show_warning) const
{
  if (empty()) {
    return time;
  }

  constexpr double eps = 1e-5;
  const auto public_start = start_time_ - time_offset_;
  const auto public_end = end_time_ - time_offset_;
  if (show_warning && (time < public_start - eps || time > public_end + eps)) {
    RCLCPP_WARN(
      rclcpp::get_logger("TemporalTrajectory"),
      "The time %f is out of the trajectory duration range [%f, %f]", time, public_start,
      public_end);
  }
  return std::clamp(time, public_start, public_end);
}

std::vector<double> TimeDistanceMapping::cropped_time_bases() const
{
  auto time_bases = cropped_internal_time_bases();
  std::transform(time_bases.begin(), time_bases.end(), time_bases.begin(), [this](const double t) {
    return to_public_time(t);
  });
  return time_bases;
}

std::optional<double> TimeDistanceMapping::time_at_distance(const double distance) const
{
  const auto start_distance_value = start_distance();
  const auto end_distance_value = end_distance();

  // Clamp distance to valid range
  const auto clamped_distance = std::clamp(distance, start_distance_value, end_distance_value);

  if (std::abs(clamped_distance - start_distance_value) <= k_same_time_threshold) {
    return start_time();
  }

  if (std::abs(clamped_distance - end_distance_value) <= k_same_time_threshold) {
    return end_time();
  }

  // Use binary search to find the time where distance is reached
  constexpr size_t k_max_iterations = 100;
  const auto result_time = binary_search(
    start_time_, end_time_,
    [this, clamped_distance](double time) { return compute_distance(time) >= clamped_distance; },
    k_max_iterations, k_same_time_threshold);

  return to_public_time(result_time);
}

bool TimeDistanceMapping::empty() const
{
  return time_bases_.empty();
}

std::vector<double> TimeDistanceMapping::cropped_internal_time_bases() const
{
  return crop_bases(time_bases_, start_time_, end_time_);
}

std::vector<double> TimeDistanceMapping::cropped_absolute_distance_bases() const
{
  const auto time_bases = crop_bases(time_bases_, start_time_, end_time_);
  std::vector<double> distance_bases;
  distance_bases.reserve(time_bases.size());
  for (const auto t : time_bases) {
    distance_bases.emplace_back(compute_distance(t));
  }
  return distance_bases;
}

double TimeDistanceMapping::clamp_time_without_warning(const double time) const
{
  return std::clamp(time, start_time(), end_time());
}

double TimeDistanceMapping::to_internal_time(const double time) const
{
  return time + time_offset_;
}

double TimeDistanceMapping::to_public_time(const double internal_time) const
{
  return internal_time - time_offset_;
}

}  // namespace autoware::experimental::trajectory::detail
