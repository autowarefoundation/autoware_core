// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY__TRAJECTORY_POINT_HPP_
#define AUTOWARE__TRAJECTORY__TRAJECTORY_POINT_HPP_

#include "autoware/trajectory/detail/interpolated_array.hpp"
#include "autoware/trajectory/pose.hpp"

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{
template <>
class Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>
: public Trajectory<geometry_msgs::msg::Pose>
{
  using BaseClass = Trajectory<geometry_msgs::msg::Pose>;
  using PointType = autoware_planning_msgs::msg::TrajectoryPoint;

protected:
  std::shared_ptr<detail::InterpolatedArray<double>> longitudinal_velocity_mps_{
    nullptr};  //!< Longitudinal velocity in m/s
  std::shared_ptr<detail::InterpolatedArray<double>> lateral_velocity_mps_{
    nullptr};  //!< Lateral velocity in m/s
  std::shared_ptr<detail::InterpolatedArray<double>> heading_rate_rps_{
    nullptr};  //!< Heading rate in rad/s};
  std::shared_ptr<detail::InterpolatedArray<double>> acceleration_mps2_{
    nullptr};  //!< Longitudinal acceleration in m/s^2} Warning, this is not used
  std::shared_ptr<detail::InterpolatedArray<double>> front_wheel_angle_rad_{
    nullptr};  //!< Front wheel angle in rad} Warning, this is not used
  std::shared_ptr<detail::InterpolatedArray<double>> rear_wheel_angle_rad_{
    nullptr};  //!< Rear wheel angle in rad} Warning, this is not used

  /**
   * @brief add the event function to
   * longitudinal_velocity_mps/lateral_velocity_mps/heading_rate_mps/acceleration_mps2/front_wheel_angle_rad/rear_wheel_angle_rad
   * interpolator
   * @note when a new base is added to longitudinal_velocity_mps for example, the addition is also
   * notified and update_base() is triggered.
   */
  virtual void add_base_addition_callback();

public:
  Trajectory();
  ~Trajectory() override = default;
  Trajectory(const Trajectory & rhs);
  Trajectory(Trajectory && rhs) noexcept;
  Trajectory & operator=(const Trajectory & rhs);
  Trajectory & operator=(Trajectory && rhs) noexcept;

  [[deprecated]] std::vector<double> get_internal_bases() const override;

  std::vector<double> get_underlying_bases() const override;

  detail::InterpolatedArray<double> & longitudinal_velocity_mps()
  {
    return *longitudinal_velocity_mps_;
  }

  detail::InterpolatedArray<double> & lateral_velocity_mps() { return *lateral_velocity_mps_; }

  detail::InterpolatedArray<double> & heading_rate_rps() { return *heading_rate_rps_; }

  detail::InterpolatedArray<double> & acceleration_mps2() { return *acceleration_mps2_; }

  detail::InterpolatedArray<double> & front_wheel_angle_rad() { return *front_wheel_angle_rad_; }

  detail::InterpolatedArray<double> & rear_wheel_angle_rad() { return *rear_wheel_angle_rad_; }

  const detail::InterpolatedArray<double> & longitudinal_velocity_mps() const
  {
    return *longitudinal_velocity_mps_;
  }

  const detail::InterpolatedArray<double> & lateral_velocity_mps() const
  {
    return *lateral_velocity_mps_;
  }

  const detail::InterpolatedArray<double> & heading_rate_rps() const { return *heading_rate_rps_; }

  const detail::InterpolatedArray<double> & acceleration_mps2() const
  {
    return *acceleration_mps2_;
  }

  const detail::InterpolatedArray<double> & front_wheel_angle_rad() const
  {
    return *front_wheel_angle_rad_;
  }

  const detail::InterpolatedArray<double> & rear_wheel_angle_rad() const
  {
    return *rear_wheel_angle_rad_;
  }

  /**
   * @brief Build the trajectory from the points
   * @param points Vector of points
   * @return True if the build is successful
   */
  interpolator::InterpolationResult build(const std::vector<PointType> & points);

  /**
   * @brief Compute the point on the trajectory at a given s value
   * @param s Arc length
   * @return Point on the trajectory
   */
  PointType compute(const double s) const;

  /**
   * @brief Compute the points on the trajectory at given s values
   * @param ss Arc lengths
   * @return Points on the trajectory
   */
  std::vector<PointType> compute(const std::vector<double> & ss) const;

  /**
   * @brief Restore the trajectory points
   * @param min_points Minimum number of points
   * @return Vector of points
   */
  std::vector<PointType> restore(const size_t min_points = 4) const;

  class Builder : public BaseClass::Builder
  {
  private:
    std::unique_ptr<Trajectory> trajectory_;

  public:
    Builder();

    /**
     * @brief create the default interpolator setting
     * @note In addition to the base class, Stairstep for
     * longitudinal_velocity_mps, lateral_velocity_mps, heading_rate_rps, acceleration_mps2,
     * front_wheel_angle_rad, rear_wheel_angle_rad
     */
    static void defaults(Trajectory * trajectory);
    template <class InterpolatorType, class... Args>
    Builder & set_xy_interpolator(Args &&... args)
    {
      trajectory_->x_interpolator_ =
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
      trajectory_->y_interpolator_ =
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_z_interpolator(Args &&... args)
    {
      trajectory_->z_interpolator_ =
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_orientation_interpolator(Args &&... args)
    {
      trajectory_->orientation_interpolator_ =
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_longitudinal_velocity_interpolator(Args &&... args)
    {
      trajectory_->longitudinal_velocity_mps_ = std::make_shared<detail::InterpolatedArray<double>>(
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...));
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_lateral_velocity_interpolator(Args &&... args)
    {
      trajectory_->lateral_velocity_mps_ = std::make_shared<detail::InterpolatedArray<double>>(
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...));
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_heading_rate_interpolator(Args &&... args)
    {
      trajectory_->heading_rate_rps_ = std::make_shared<detail::InterpolatedArray<double>>(
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...));
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_acceleration_interpolator(Args &&... args)
    {
      trajectory_->acceleration_mps2_ = std::make_shared<detail::InterpolatedArray<double>>(
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...));
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_front_wheel_angle_interpolator(Args &&... args)
    {
      trajectory_->front_wheel_angle_rad_ = std::make_shared<detail::InterpolatedArray<double>>(
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...));
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_rear_wheel_angle_interpolator(Args &&... args)
    {
      trajectory_->rear_wheel_angle_rad_ = std::make_shared<detail::InterpolatedArray<double>>(
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...));
      return *this;
    }

    tl::expected<Trajectory, interpolator::InterpolationFailure> build(
      const std::vector<PointType> & points);
  };
};

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__TRAJECTORY_POINT_HPP_
