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

#ifndef AUTOWARE__TRAJECTORY__POSE_HPP_
#define AUTOWARE__TRAJECTORY__POSE_HPP_

#include "autoware/trajectory/interpolator/interpolator.hpp"
#include "autoware/trajectory/point.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{

/**
 * @brief Trajectory class for geometry_msgs::msg::Pose
 */
template <>
class Trajectory<geometry_msgs::msg::Pose> : public Trajectory<geometry_msgs::msg::Point>
{
  using BaseClass = Trajectory<geometry_msgs::msg::Point>;
  using PointType = geometry_msgs::msg::Pose;

protected:
  std::shared_ptr<interpolator::InterpolatorInterface<geometry_msgs::msg::Quaternion>>
    orientation_interpolator_{nullptr};  //!< Interpolator for orientations

public:
  Trajectory();
  ~Trajectory() override = default;
  explicit Trajectory(const Trajectory<geometry_msgs::msg::Point> & point_trajectory);
  Trajectory(const Trajectory & rhs);
  Trajectory(Trajectory && rhs) = default;
  Trajectory & operator=(const Trajectory & rhs);
  Trajectory & operator=(Trajectory && rhs) = default;

  interpolator::InterpolationResult build(const std::vector<PointType> & points);

  /**
   * @brief Get the underlying arc lengths of the trajectory
   * @return Vector of bases(arc lengths)
   */
  [[deprecated]] std::vector<double> get_internal_bases() const override;

  /**
   * @brief Get the underlying arc lengths of the trajectory
   * @return Vector of bases(arc lengths)
   */
  std::vector<double> get_underlying_bases() const override;

  /**
   * @brief Compute the pose on the trajectory at a given s value
   * @param s Arc length
   * @return Pose on the trajectory
   */
  PointType compute(const double s) const;

  /**
   * @brief Compute the poses on the trajectory at given s values
   * @param ss Arc lengths
   * @return Poses on the trajectory
   */
  std::vector<PointType> compute(const std::vector<double> & ss) const;

  std::vector<PointType> restore(const size_t min_points = 4) const;

  /**
   * @brief Align the orientation with the direction
   */
  void align_orientation_with_trajectory_direction();

  class Builder : public BaseClass::Builder
  {
  private:
    std::unique_ptr<Trajectory> trajectory_;

  public:
    Builder();

    /**
     * @brief create the default interpolator setting
     * @note In addition to the base class, SphericalLinear for orientation
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

    tl::expected<Trajectory, interpolator::InterpolationFailure> build(
      const std::vector<PointType> & points);
  };
};

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__POSE_HPP_
