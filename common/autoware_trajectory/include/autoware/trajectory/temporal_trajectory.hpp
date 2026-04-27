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

#include "autoware/trajectory/detail/time_distance_mapping.hpp"
#include "autoware/trajectory/trajectory_point.hpp"

#include <tl_expected/expected.hpp>

#include <memory>
#include <utility>
#include <vector>
namespace autoware::experimental::trajectory
{

struct TimeDistancePair
{
  double time;      ///< Time in seconds.
  double distance;  ///< Distance in meters.
};

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

  /** @brief Return the stored time bases which is cropped and offset-adjusted. */
  [[nodiscard]] std::vector<double> get_underlying_time_bases() const;

  /**
   * @brief Compute a point at a given time.
   * @param[in] t Query time in seconds.
   * @return Interpolated trajectory point.
   * @throw std::out_of_range If t is outside [start_time(), end_time()].
   */
  [[nodiscard]] PointType compute_from_time(const double t) const;

  /**
   * @brief Compute a point at a given arc length.
   * @param[in] s Query arc length in meters.
   * @return Interpolated trajectory point.
   * @throw std::out_of_range If s is outside [0, length()].
   */
  [[nodiscard]] PointType compute_from_distance(const double s) const;

  /**
   * @brief Compute the azimuth angle at a given time.
   * @param[in] t Query time in seconds.
   * @return Azimuth in radians.
   * @throw std::out_of_range If t is outside [start_time(), end_time()].
   */
  [[nodiscard]] double azimuth_from_time(const double t) const;

  /**
   * @brief Compute the azimuth angle at a given arc length.
   * @param[in] s Query arc length in meters.
   * @return Azimuth in radians.
   * @throw std::out_of_range If s is outside [0, length()].
   */
  [[nodiscard]] double azimuth_from_distance(const double s) const;

  /**
   * @brief Compute the elevation angle at a given time.
   * @param[in] t Query time in seconds.
   * @return Elevation in radians.
   * @throw std::out_of_range If t is outside [start_time(), end_time()].
   */
  [[nodiscard]] double elevation_from_time(const double t) const;

  /**
   * @brief Compute the elevation angle at a given arc length.
   * @param[in] s Query arc length in meters.
   * @return Elevation in radians.
   * @throw std::out_of_range If s is outside [0, length()].
   */
  [[nodiscard]] double elevation_from_distance(const double s) const;

  /**
   * @brief Compute the curvature at a given time.
   * @param[in] t Query time in seconds.
   * @return Curvature.
   * @throw std::out_of_range If t is outside [start_time(), end_time()].
   */
  [[nodiscard]] double curvature_from_time(const double t) const;

  /**
   * @brief Compute the curvature at a given arc length.
   * @param[in] s Query arc length in meters.
   * @return Curvature.
   * @throw std::out_of_range If s is outside [0, length()].
   */
  [[nodiscard]] double curvature_from_distance(const double s) const;

  /**
   * @brief Convert time to arc length.
   * @param[in] t Query time in seconds.
   * @return Arc length in meters.
   * @throw std::out_of_range If t is outside [start_time(), end_time()].
   */
  [[nodiscard]] double time_to_distance(const double t) const;

  /**
   * @brief Convert arc length to time.
   * @param[in] s Query arc length in meters.
   * @param[in] return_end_time If true and the vehicle is stopped at distance @p s, returns the
   *            time when the stop ends instead of the time when it starts.
   * @return Time in seconds.
   * @throw std::out_of_range If s is outside [0, length()].
   */
  [[nodiscard]] double distance_to_time(const double s, bool return_end_time = false) const;

  /**
   * @brief Restore the trajectory at its underlying time bases.
   * @return Restored trajectory points.
   */
  [[nodiscard]] std::vector<PointType> restore() const;

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
  friend TemporalTrajectory crop_time(
    TemporalTrajectory trajectory, double start_time, double duration);
  friend TemporalTrajectory crop_distance(
    TemporalTrajectory trajectory, double start_distance, double length);
  friend TemporalTrajectory set_stopline(TemporalTrajectory trajectory, double arc_length);
  friend TemporalTrajectory insert_stop_duration(
    TemporalTrajectory trajectory, double arc_length, double duration);
  friend TemporalTrajectory set_time_offset(TemporalTrajectory trajectory, double offset);
  friend TemporalTrajectory align_orientation_with_trajectory_direction(
    TemporalTrajectory trajectory);

  SpatialTrajectory spatial_trajectory_;
  detail::TimeDistanceMapping time_distance_mapping_;
  double distance_offset_{0.0};
};

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__TEMPORAL_TRAJECTORY_HPP_
