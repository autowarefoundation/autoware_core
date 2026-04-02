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

class TemporalTrajectory
{
public:
  using PointType = autoware_planning_msgs::msg::TrajectoryPoint;
  using SpatialTrajectory = Trajectory<PointType>;
  using InterpolatorInterface = interpolator::InterpolatorInterface<double>;

  TemporalTrajectory();
  ~TemporalTrajectory() = default;
  TemporalTrajectory(const TemporalTrajectory & rhs);
  TemporalTrajectory(TemporalTrajectory && rhs) noexcept = default;
  TemporalTrajectory & operator=(const TemporalTrajectory & rhs);
  TemporalTrajectory & operator=(TemporalTrajectory && rhs) noexcept = default;

  [[nodiscard]] interpolator::InterpolationResult build(const std::vector<PointType> & points);

  [[nodiscard]] double length() const;
  [[nodiscard]] double duration() const;
  [[nodiscard]] double start_time() const;
  [[nodiscard]] double end_time() const;

  [[nodiscard]] std::vector<double> get_underlying_time_bases() const;
  [[nodiscard]] std::vector<double> get_underlying_distance_bases() const;

  [[nodiscard]] PointType compute_from_time(const double t) const;
  [[nodiscard]] std::vector<PointType> compute_from_time(const std::vector<double> & ts) const;

  [[nodiscard]] PointType compute_from_distance(const double s) const;
  [[nodiscard]] std::vector<PointType> compute_from_distance(const std::vector<double> & ss) const;

  [[nodiscard]] double time_to_distance(const double t) const;
  [[nodiscard]] std::vector<double> time_to_distance(const std::vector<double> & ts) const;

  [[nodiscard]] std::optional<double> distance_to_time(const double s) const;

  [[nodiscard]] std::vector<PointType> restore() const;

  void crop_time(const double start_time, const double duration);

  void set_stopline(const double arc_length);
  void set_stopline(const double arc_length, const double duration);

  [[nodiscard]] const SpatialTrajectory & spatial_trajectory() const;

  class Builder
  {
  private:
    std::unique_ptr<TemporalTrajectory> trajectory_;
    SpatialTrajectory::Builder spatial_trajectory_builder_;

  public:
    Builder();

    static void defaults(TemporalTrajectory * trajectory);

    template <class InterpolatorType, class... Args>
    Builder & set_time_to_distance_interpolator(Args &&... args)
    {
      trajectory_->time_to_distance_ =
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_xy_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_xy_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_z_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_z_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_orientation_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_orientation_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_longitudinal_velocity_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_longitudinal_velocity_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_lateral_velocity_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_lateral_velocity_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_heading_rate_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_heading_rate_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_acceleration_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_acceleration_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_front_wheel_angle_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_front_wheel_angle_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_rear_wheel_angle_interpolator(Args &&... args)
    {
      spatial_trajectory_builder_.template set_rear_wheel_angle_interpolator<InterpolatorType>(
        std::forward<Args>(args)...);
      return *this;
    }

    [[nodiscard]] tl::expected<TemporalTrajectory, interpolator::InterpolationFailure> build(
      const std::vector<PointType> & points);
  };

private:
  SpatialTrajectory spatial_trajectory_;
  std::shared_ptr<InterpolatorInterface> time_to_distance_{nullptr};
  std::vector<double> time_bases_;
  std::vector<double> distance_bases_;

  [[nodiscard]] double clamp_time(const double t, bool show_warning = false) const;
};

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__TEMPORAL_TRAJECTORY_HPP_
