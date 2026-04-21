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

#ifndef AUTOWARE__TRAJECTORY__DETAIL__TIME_DISTANCE_MAPPING_HPP_
#define AUTOWARE__TRAJECTORY__DETAIL__TIME_DISTANCE_MAPPING_HPP_

#include "autoware/trajectory/interpolator/interpolator.hpp"

#include <memory>
#include <vector>

namespace autoware::experimental::trajectory::detail
{

/**
 * @brief Internal helper that maintains a time-to-distance mapping for
 * `TemporalTrajectory`.
 * @details
 * This class stores time and absolute-distance bases, rebuilds an internal
 * interpolator, and exposes the minimum mutation/query operations needed by
 * `TemporalTrajectory` for cropping and stopline editing.
 */
class TimeDistanceMapping
{
public:
  using InterpolatorInterface = interpolator::InterpolatorInterface<double>;

  TimeDistanceMapping() = default;
  TimeDistanceMapping(const TimeDistanceMapping & rhs);
  TimeDistanceMapping(TimeDistanceMapping && rhs) noexcept = default;
  ~TimeDistanceMapping() = default;
  TimeDistanceMapping & operator=(const TimeDistanceMapping & rhs);
  TimeDistanceMapping & operator=(TimeDistanceMapping && rhs) noexcept = default;

  /**
   * @brief Set the interpolator used for time-to-distance evaluation.
   * @param[in] interpolator Interpolator instance used to compute distance from time.
   */
  void set_interpolator(std::shared_ptr<InterpolatorInterface> interpolator);

  /**
   * @brief Build the mapping from time and absolute-distance bases.
   * @param[in] time_bases Input times in seconds.
   * @param[in] distance_bases Input absolute distances in meters.
   * @return Success, or an interpolation failure.
   */
  [[nodiscard]] interpolator::InterpolationResult build(
    const std::vector<double> & time_bases, const std::vector<double> & distance_bases);

  /**
   * @brief Evaluate distance at the given time.
   * @param[in] time Query time in seconds.
   * @return Absolute distance in meters.
   * @throw std::out_of_range If time is outside [start_time(), end_time()].
   */
  [[nodiscard]] double distance_at(double time) const;

  /** @brief Return the visible duration in seconds. */
  [[nodiscard]] double duration() const;

  /** @brief Return the visible start time in seconds. */
  [[nodiscard]] double start_time() const;

  /** @brief Return the visible end time in seconds. */
  [[nodiscard]] double end_time() const;

  /** @brief Return the configured time offset in seconds. */
  [[nodiscard]] double time_offset() const;

  /**
   * @brief Set the time offset.
   * @param[in] offset Time offset in seconds.
   */
  void set_time_offset(double offset);

  /**
   * @brief Extend the schedule at a given time by inserting a constant-distance plateau.
   * @param[in] time Pivot time in seconds.
   * @param[in] delta_time Additional duration in seconds.
   * @throw std::out_of_range If time is outside [start_time(), end_time()].
   */
  void extend_time_at(double time, double delta_time);

  /**
   * @brief Set a time range to a constant absolute distance.
   * @param[in] start_time Range start time in seconds.
   * @param[in] end_time Range end time in seconds.
   * @param[in] distance Absolute distance in meters.
   * @throw std::out_of_range If start_time or end_time is outside [start_time(), end_time()].
   * @throw std::invalid_argument If end_time < start_time.
   */
  void set_distance_range(double start_time, double end_time, double distance);

  /**
   * @brief Restrict the visible time range.
   * @param[in] start_time Visible start time in seconds.
   * @param[in] end_time Visible end time in seconds.
   * @throw std::out_of_range If start_time or end_time is outside [start_time(), end_time()].
   * @throw std::invalid_argument If end_time < start_time.
   */
  void set_time_range(double start_time, double end_time);

  /**
   * @brief Return cropped time bases within the visible range.
   * @return Visible time bases in seconds.
   */
  [[nodiscard]] std::vector<double> cropped_time_bases() const;

  /**
   * @brief Convert an absolute distance to time.
   *
   * The distance-to-time mapping is monotonic non-decreasing, but may contain
   * plateaus (intervals where the vehicle is stopped and the distance remains constant).
   *
   * For a queried distance:
   *
   * - If the distance lies on a strictly increasing segment (no plateau, unique solution t*):
   *   - return_end_time == false:
   *       Returns the smallest time t such that distance(t) >= query.
   *       Due to numerical tolerance, the returned value may be slightly larger than t*.
   *   - return_end_time == true:
   *       Returns the largest time t such that distance(t) <= query.
   *       Due to numerical tolerance, the returned value may be slightly smaller than t*.
   *
   * - If the distance lies on a plateau (multiple valid times):
   *   - return_end_time == false:
   *       Returns the earliest time in the plateau (start of the plateau).
   *       The returned value may be slightly larger than the exact start time.
   *   - return_end_time == true:
   *       Returns the latest time in the plateau (end of the plateau).
   *       The returned value may be slightly smaller than the exact end time.
   *
   * @param[in] absolute_distance Query absolute distance in meters.
   * @param[in] return_end_time When true, return the last time for a plateau.
   * @return Time in seconds.
   * @throw std::out_of_range If absolute_distance is outside [start_distance(), end_distance()].
   */
  [[nodiscard]] double time_at_distance(
    double absolute_distance, bool return_end_time = false) const;

private:
  [[nodiscard]] double compute_distance(double internal_time) const;
  [[nodiscard]] double start_distance() const;
  [[nodiscard]] double end_distance() const;
  [[nodiscard]] double to_internal_time(double time) const;
  [[nodiscard]] double to_public_time(double internal_time) const;

  std::shared_ptr<InterpolatorInterface> interpolator_{nullptr};
  std::vector<double> time_bases_;
  std::vector<double> distance_bases_;
  double start_time_{0.0};
  double end_time_{0.0};
  double time_offset_{0.0};
};

}  // namespace autoware::experimental::trajectory::detail

#endif  // AUTOWARE__TRAJECTORY__DETAIL__TIME_DISTANCE_MAPPING_HPP_
