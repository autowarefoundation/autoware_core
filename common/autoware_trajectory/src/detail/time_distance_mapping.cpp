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
#include "autoware/trajectory/detail/validate_range.hpp"
#include "autoware/trajectory/interpolator/nearest_neighbor.hpp"
#include "autoware/trajectory/threshold.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory::detail
{
namespace
{

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
    std::abs(time_bases.at(insert_index) - time) > k_epsilon_time) {
    time_bases.insert(time_bases.begin() + static_cast<std::ptrdiff_t>(insert_index), time);
    values.insert(values.begin() + static_cast<std::ptrdiff_t>(insert_index), value);
    return insert_index;
  }

  time_bases.at(insert_index) = time;
  values.at(insert_index) = value;
  return insert_index;
}
}  // namespace

void TimeDistanceMapping::extend_time_at(const double time, const double delta_time)
{
  const auto original_start_time = start_time();
  const auto original_end_time = end_time();
  throw_if_out_of_range(time, original_start_time, original_end_time, "time");
  const auto pivot_internal_time = to_internal_time(time);
  const auto pivot_distance = compute_distance(pivot_internal_time);
  auto time_bases = time_bases_;
  auto distance_bases = distance_bases_;
  const auto pivot_index =
    insert_or_replace_base_at_time(time_bases, distance_bases, pivot_internal_time, pivot_distance);
  for (size_t i = pivot_index + 1; i < time_bases.size(); ++i) {
    time_bases.at(i) += delta_time;
  }

  const auto end_index = insert_or_replace_base_at_time(
    time_bases, distance_bases, pivot_internal_time + delta_time, pivot_distance);
  std::fill(
    distance_bases.begin() + static_cast<std::ptrdiff_t>(pivot_index),
    distance_bases.begin() + static_cast<std::ptrdiff_t>(end_index + 1), pivot_distance);

  const auto result = build(time_bases, distance_bases);
  assert(result);
  set_time_range(original_start_time, original_end_time + delta_time);
}

void TimeDistanceMapping::set_distance_range(
  const double start_time, const double end_time, const double distance)
{
  const auto original_start_time = this->start_time();
  const auto original_end_time = this->end_time();
  throw_if_out_of_range(start_time, original_start_time, original_end_time, "start_time");
  throw_if_out_of_range(end_time, original_start_time, original_end_time, "end_time");
  if (end_time < start_time) {
    throw std::invalid_argument("end_time must be greater than or equal to start_time");
  }
  auto time_bases = time_bases_;
  auto distance_bases = distance_bases_;
  const auto start_internal = to_internal_time(start_time);
  const auto end_internal = to_internal_time(end_time);

  const auto start_index =
    insert_or_replace_base_at_time(time_bases, distance_bases, start_internal, distance);
  const auto end_index =
    insert_or_replace_base_at_time(time_bases, distance_bases, end_internal, distance);
  std::fill(
    distance_bases.begin() + static_cast<std::ptrdiff_t>(start_index),
    distance_bases.begin() + static_cast<std::ptrdiff_t>(end_index + 1), distance);

  const auto result = build(time_bases, distance_bases);
  assert(result);
  set_time_range(original_start_time, original_end_time);
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

  time_bases_.emplace_back(time_bases.front());
  distance_bases_.emplace_back(distance_bases.front());

  for (size_t i = 1; i < time_bases.size(); ++i) {
    /**
       NOTE: This sanitisation is essential for avoiding the same base. The process of avoiding zero
       division is handled by each interpolator.
    */
    const auto time_diff =
      std::max(time_bases.at(i) - time_bases_.back(), std::numeric_limits<double>::epsilon());
    time_bases_.emplace_back(time_bases_.back() + time_diff);
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
  throw_if_out_of_range(time, start_time(), end_time(), "time");
  return compute_distance(to_internal_time(time));
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
  throw_if_out_of_range(start_time, this->start_time(), this->end_time(), "start_time");
  throw_if_out_of_range(end_time, this->start_time(), this->end_time(), "end_time");
  if (end_time < start_time) {
    throw std::invalid_argument("end_time must be greater than or equal to start_time");
  }
  start_time_ = to_internal_time(start_time);
  end_time_ = to_internal_time(end_time);
}

std::vector<double> TimeDistanceMapping::cropped_time_bases() const
{
  auto time_bases = crop_bases(time_bases_, start_time_, end_time_);
  std::transform(time_bases.begin(), time_bases.end(), time_bases.begin(), [this](const double t) {
    return to_public_time(t);
  });
  return time_bases;
}

double TimeDistanceMapping::time_at_distance(
  const double distance, const bool return_end_time) const
{
  const auto start_distance_value = start_distance();
  const auto end_distance_value = end_distance();

  throw_if_out_of_range(distance, start_distance_value, end_distance_value, "absolute_distance");

  constexpr size_t k_max_iterations = 100;

  if (!return_end_time) {
    const auto result_time = lower_bound_by_predicate(
      start_time_, end_time_,
      [this, distance](const double time) { return compute_distance(time) >= distance; },
      k_max_iterations, std::numeric_limits<double>::epsilon());
    return to_public_time(result_time);
  }
  if (std::abs(distance - end_distance_value) <= std::numeric_limits<double>::epsilon()) {
    return to_public_time(end_time_);
  }

  const auto result_time = upper_bound_by_predicate(
    start_time_, end_time_,
    [this, distance](const double time) { return compute_distance(time) <= distance; },
    k_max_iterations, std::numeric_limits<double>::epsilon());
  return to_public_time(result_time);
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
