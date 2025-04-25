// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY__LANELET_POINT_HPP_
#define AUTOWARE__TRAJECTORY__LANELET_POINT_HPP_

#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/interpolator/interpolator.hpp"

#include <tl_expected/expected.hpp>

#include <lanelet2_core/primitives/Point.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{
template <>
class Trajectory<lanelet::ConstPoint3d>
{
protected:
  std::shared_ptr<interpolator::InterpolatorInterface<double>> x_interpolator_{
    nullptr};  //!< Interpolator for x
  std::shared_ptr<interpolator::InterpolatorInterface<double>> y_interpolator_{
    nullptr};  //!< Interpolator for y
  std::shared_ptr<interpolator::InterpolatorInterface<double>> z_interpolator_{
    nullptr};  //!< Interpolator for z

  std::vector<double> bases_;  //!< Axis of the trajectory

  //!< Start and end of the arc length of the trajectory
  double start_{0.0};
  double end_{0.0};

  /**
   * @brief Validate the arc length is within the trajectory
   * @param s Arc length
   */
  double clamp(const double s, bool show_warning = false) const;

public:
  Trajectory();
  virtual ~Trajectory() = default;
  Trajectory(const Trajectory & rhs);
  Trajectory(Trajectory && rhs) = default;
  Trajectory & operator=(const Trajectory & rhs);
  Trajectory & operator=(Trajectory && rhs) = default;

  /**
   * @brief Get the underlying arc lengths of the trajectory
   * @return Vector of bases(arc lengths)
   */
  std::vector<double> get_underlying_bases() const;

  double start() const { return start_; }

  double end() const { return end_; }

  /**
   * @brief Get the length of the trajectory
   * @return Length of the trajectory
   */
  double length() const { return end_ - start_; }

  /**
   * @brief Compute the point on the trajectory at a given s value
   * @param s Arc length
   * @return Point on the trajectory
   */
  lanelet::ConstPoint3d compute(const double s) const;

  /**
   * @brief Build the trajectory from the points
   * @param points Vector of points
   * @return True if the build is successful
   */
  interpolator::InterpolationResult build(const std::vector<lanelet::ConstPoint3d> & points);

  /**
   * @brief Get the azimuth angle at a given s value
   * @param s Arc length
   * @return Azimuth in radians
   */
  double azimuth(const double s) const;

  /**
   * @brief Get the elevation angle at a given s value
   * @param s Arc length
   * @return Elevation in radians
   */
  double elevation(const double s) const;

  /**
   * @brief Get the curvature at a given s value
   * @param s Arc length
   * @return Curvature
   */
  double curvature(const double s) const;

  /**
   * @brief return the list of base values from start_ to end_ with the given interval
   * @param tick the tick of interval
   * @return array of double from start_ to end_ including the end_
   */
  std::vector<double> base_arange(const double tick) const
  {
    std::vector<double> ss;
    for (double s = start_; s <= end_; s += tick) {
      ss.push_back(s);
    }
    if (ss.back() != end_) {
      ss.push_back(end_);
    }
    return ss;
  }

  /**
   * @brief return the list of base values from start_ to end_ with the given interval
   * @param interval the interval indicating start and end
   * @param tick the tick of interval
   * @param end_inclusive flag to include the interval end even if it is not exactly on the last
   * tick step does not match
   * @return array of double within [start_ to end_ ] and given interval
   */
  std::vector<double> base_arange(
    const std::pair<double, double> interval, const double tick,
    const bool end_inclusive = true) const
  {
    const auto & [start_input, end_input] = interval;
    const auto start = std::max<double>(start_, start_input);
    const auto end = std::min<double>(end_, end_input);
    std::vector<double> ss;
    for (double s = start; s <= end; s += tick) {
      ss.push_back(s);
    }
    if (end_inclusive && ss.back() != end) {
      ss.push_back(end);
    }
    return ss;
  }

  class Builder
  {
  private:
    std::unique_ptr<Trajectory> trajectory_;

  public:
    Builder();

    /**
     * @brief create the default interpolator setting
     * @note CubicSpline for x, y and Linear for z
     */
    static void defaults(Trajectory<lanelet::ConstPoint3d> * trajectory);

    tl::expected<Trajectory, interpolator::InterpolationFailure> build(
      const std::vector<lanelet::ConstPoint3d> & points);
  };
};
}  // namespace autoware::experimental::trajectory
#endif  // AUTOWARE__TRAJECTORY__LANELET_POINT_HPP_
