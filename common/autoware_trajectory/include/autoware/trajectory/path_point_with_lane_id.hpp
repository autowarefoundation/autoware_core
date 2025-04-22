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

#ifndef AUTOWARE__TRAJECTORY__PATH_POINT_WITH_LANE_ID_HPP_
#define AUTOWARE__TRAJECTORY__PATH_POINT_WITH_LANE_ID_HPP_

#include "autoware/trajectory/path_point.hpp"

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{
template <>
class Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
: public Trajectory<autoware_planning_msgs::msg::PathPoint>
{
  using BaseClass = Trajectory<autoware_planning_msgs::msg::PathPoint>;
  using PointType = autoware_internal_planning_msgs::msg::PathPointWithLaneId;
  using LaneIdType = std::vector<int64_t>;

protected:
  std::shared_ptr<detail::InterpolatedArray<LaneIdType>> lane_ids_{nullptr};  //!< Lane ID

  /**
   * @brief add the event function to lane_ids additionally
   * @note when a new base is added to lane_ids for example, the addition is also
   * notified and update_base() is triggered.
   */
  void add_base_addition_callback() override;

public:
  Trajectory();
  ~Trajectory() override = default;
  Trajectory(const Trajectory & rhs);
  Trajectory(Trajectory && rhs) noexcept;
  Trajectory & operator=(const Trajectory & rhs);
  Trajectory & operator=(Trajectory && rhs) noexcept;

  detail::InterpolatedArray<LaneIdType> & lane_ids() { return *lane_ids_; }

  const detail::InterpolatedArray<LaneIdType> & lane_ids() const { return *lane_ids_; }

  [[nodiscard]] std::vector<int64_t> get_contained_lane_ids() const;

  /**
   * @brief Build the trajectory from the points
   * @param points Vector of points
   * @return True if the build is successful
   */
  interpolator::InterpolationResult build(const std::vector<PointType> & points);

  [[deprecated]] std::vector<double> get_internal_bases() const override;

  std::vector<double> get_underlying_bases() const override;

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
     * @note In addition to the base class, Stairstep for lane_ids
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
    Builder & set_lane_ids_interpolator(Args &&... args)
    {
      trajectory_->lane_ids_ = std::make_shared<detail::InterpolatedArray<LaneIdType>>(
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...));
      return *this;
    }

    tl::expected<Trajectory, interpolator::InterpolationFailure> build(
      const std::vector<PointType> & points);
  };
};
}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__PATH_POINT_WITH_LANE_ID_HPP_
