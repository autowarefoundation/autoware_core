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

#ifndef AUTOWARE__TRAJECTORY__TEMPORAL_TRAJECTORY_HPP_
#define AUTOWARE__TRAJECTORY__TEMPORAL_TRAJECTORY_HPP_

#include "autoware/trajectory/interpolator/interpolator.hpp"
#include "autoware/trajectory/trajectory_point.hpp"

#include <tl_expected/expected.hpp>

#include <memory>
#include <optional>
#include <utility>
#include <vector>
namespace autoware::experimental::trajectory
{

/**
 * @brief Trajectory wrapper that parameterizes a spatial trajectory by time.
 * @details
 * `TemporalTrajectory` keeps a `Trajectory<TrajectoryPoint>` for spatial interpolation and an
 * additional time-to-distance interpolator so callers can evaluate the trajectory from either time
 * or arc length.
 */
class TemporalTrajectory
{
public:
  using PointType = autoware_planning_msgs::msg::TrajectoryPoint;
  using SpatialTrajectory = Trajectory<PointType>;
  using InterpolatorInterface = interpolator::InterpolatorInterface<double>;

private:
  class TimeDistanceMapping
  {
  public:
    TimeDistanceMapping() = default;
    TimeDistanceMapping(const TimeDistanceMapping & rhs);
    TimeDistanceMapping(TimeDistanceMapping && rhs) noexcept = default;
    TimeDistanceMapping & operator=(const TimeDistanceMapping & rhs);
    TimeDistanceMapping & operator=(TimeDistanceMapping && rhs) noexcept = default;

    void set_interpolator(std::shared_ptr<InterpolatorInterface> interpolator);
    [[nodiscard]] bool has_interpolator() const;
    [[nodiscard]] interpolator::InterpolationResult build(
      const std::vector<double> & time_bases, const std::vector<double> & distance_bases);
    [[nodiscard]] double compute_distance(const double time) const;
    [[nodiscard]] bool empty() const;
    [[nodiscard]] const std::vector<double> & time_bases() const;
    [[nodiscard]] const std::vector<double> & distance_bases() const;

  private:
    std::shared_ptr<InterpolatorInterface> interpolator_{nullptr};
    std::vector<double> time_bases_;
    std::vector<double> distance_bases_;
  };

public:
  TemporalTrajectory();
  ~TemporalTrajectory() = default;
  TemporalTrajectory(const TemporalTrajectory & rhs) = default;
  TemporalTrajectory(TemporalTrajectory && rhs) noexcept = default;
  TemporalTrajectory & operator=(const TemporalTrajectory & rhs) = default;
  TemporalTrajectory & operator=(TemporalTrajectory && rhs) noexcept = default;

  /**
   * @brief Build the temporal trajectory from ordered trajectory points.
   * @param[in] points Input points sorted by `time_from_start`.
   * @return Success, or an interpolation failure.
   */
  [[nodiscard]] interpolator::InterpolationResult build(const std::vector<PointType> & points);

  /** @brief Return the spatial trajectory length in meters. */
  [[nodiscard]] double length() const;
  /** @brief Return the covered duration in seconds. */
  [[nodiscard]] double duration() const;
  /** @brief Return the absolute start time in seconds. */
  [[nodiscard]] double start_time() const;
  /** @brief Return the absolute end time in seconds. */
  [[nodiscard]] double end_time() const;
  /** @brief Return the user-configured time offset in seconds. */
  [[nodiscard]] double time_offset() const;

  /**
   * @brief Set a time offset applied to exported absolute times.
   * @param[in] offset Time offset in seconds.
   */
  void set_time_offset(double offset);

  /** @brief Return the stored time bases. */
  [[nodiscard]] std::vector<double> get_underlying_time_bases() const;
  /** @brief Return the stored distance bases. */
  [[nodiscard]] std::vector<double> get_underlying_distance_bases() const;

  /**
   * @brief Compute a point at a given time.
   * @param[in] t Query time in seconds.
   * @return Interpolated trajectory point.
   */
  [[nodiscard]] PointType compute_from_time(const double t) const;
  /**
   * @brief Compute points at the given times.
   * @param[in] ts Query times in seconds.
   * @return Interpolated trajectory points.
   */
  [[nodiscard]] std::vector<PointType> compute_from_time(const std::vector<double> & ts) const;

  /**
   * @brief Compute a point at a given arc length.
   * @param[in] s Query arc length in meters.
   * @return Interpolated trajectory point.
   */
  [[nodiscard]] PointType compute_from_distance(const double s) const;
  /**
   * @brief Compute points at the given arc lengths.
   * @param[in] ss Query arc lengths in meters.
   * @return Interpolated trajectory points.
   */
  [[nodiscard]] std::vector<PointType> compute_from_distance(const std::vector<double> & ss) const;

  /**
   * @brief Convert time to arc length.
   * @param[in] t Query time in seconds.
   * @return Arc length in meters.
   */
  [[nodiscard]] double time_to_distance(const double t) const;

  /**
   * @brief Convert arc length to time.
   * @param[in] s Query arc length in meters.
   * @return Time in seconds, or `std::nullopt` when the mapping is not uniquely invertible.
   */
  [[nodiscard]] std::optional<double> distance_to_time(const double s) const;

  /**
   * @brief Restore the trajectory at its underlying time bases.
   * @return Restored trajectory points.
   */
  [[nodiscard]] std::vector<PointType> restore() const;

  /**
   * @brief Crop the trajectory to a time interval and rebase the result to zero.
   * @param[in] start_time Crop start time in seconds.
   * @param[in] duration Crop duration in seconds.
   */
  void crop_time(const double start_time, const double duration);

  /**
   * @brief Insert a stopline that collapses all later points to the stop pose.
   * @param[in] arc_length Stopline position in meters.
   */
  void set_stopline(const double arc_length);
  /**
   * @brief Insert a stopline with an additional wait duration.
   * @param[in] arc_length Stopline position in meters.
   * @param[in] duration Stop duration in seconds.
   */
  void set_stopline(const double arc_length, const double duration);

  /** @brief Return the underlying spatial trajectory. */
  [[nodiscard]] const SpatialTrajectory & spatial_trajectory() const;

  /**
   * @brief Builder for `TemporalTrajectory`.
   * @details
   * This wraps `Trajectory<PointType>::Builder` for spatial interpolation and additionally manages
   * the time-to-distance interpolator.
   */
  class Builder
  {
  private:
    std::unique_ptr<TemporalTrajectory> trajectory_;
    SpatialTrajectory::Builder spatial_trajectory_builder_;

  public:
    Builder();

    /**
     * @brief Apply the default interpolator configuration.
     * @param[in,out] trajectory Target trajectory.
     */
    static void defaults(TemporalTrajectory * trajectory);

    /**
     * @brief Set the time-to-distance interpolator type.
     * @return This builder.
     */
    template <class InterpolatorType, class... Args>
    Builder & set_time_to_distance_interpolator(Args &&... args)
    {
      trajectory_->time_distance_mapping_.set_interpolator(
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...));
      return *this;
    }

    /** @brief Set the XY interpolator used by the spatial trajectory builder. */
    template <class InterpolatorType, class... Args>
    Builder & set_xy_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_xy_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    /** @brief Set the Z interpolator used by the spatial trajectory builder. */
    template <class InterpolatorType, class... Args>
    Builder & set_z_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_z_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    /** @brief Set the orientation interpolator used by the spatial trajectory builder. */
    template <class InterpolatorType, class... Args>
    Builder & set_orientation_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_orientation_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    /** @brief Set the longitudinal velocity interpolator used by the spatial builder. */
    template <class InterpolatorType, class... Args>
    Builder & set_longitudinal_velocity_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_longitudinal_velocity_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    /** @brief Set the lateral velocity interpolator used by the spatial builder. */
    template <class InterpolatorType, class... Args>
    Builder & set_lateral_velocity_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_lateral_velocity_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    /** @brief Set the heading rate interpolator used by the spatial builder. */
    template <class InterpolatorType, class... Args>
    Builder & set_heading_rate_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_heading_rate_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    /** @brief Set the acceleration interpolator used by the spatial builder. */
    template <class InterpolatorType, class... Args>
    Builder & set_acceleration_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_acceleration_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    /** @brief Set the front wheel angle interpolator used by the spatial builder. */
    template <class InterpolatorType, class... Args>
    Builder & set_front_wheel_angle_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_front_wheel_angle_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    /** @brief Set the rear wheel angle interpolator used by the spatial builder. */
    template <class InterpolatorType, class... Args>
    Builder & set_rear_wheel_angle_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_rear_wheel_angle_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    /**
     * @brief Build a temporal trajectory from points.
     * @param[in] points Input trajectory points.
     * @return Built temporal trajectory, or an interpolation failure.
     */
    [[nodiscard]] tl::expected<TemporalTrajectory, interpolator::InterpolationFailure> build(
      const std::vector<PointType> & points);
  };

private:
  SpatialTrajectory spatial_trajectory_;
  TimeDistanceMapping time_distance_mapping_;
  double start_time_{0.0};
  double end_time_{0.0};
  double time_offset_{0.0};
  double distance_offset_{0.0};

  [[nodiscard]] interpolator::InterpolationResult set_time_distance_bases(
    const std::vector<double> & time_bases, const std::vector<double> & distance_bases);
  [[nodiscard]] double clamp_time(const double t, bool show_warning = false) const;
};

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__TEMPORAL_TRAJECTORY_HPP_
