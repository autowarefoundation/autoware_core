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

#ifndef AUTOWARE__TRAJECTORY__UTILS__PRETTY_BUILD_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__PRETTY_BUILD_HPP_

#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/interpolator/akima_spline.hpp"
#include "autoware/trajectory/interpolator/cubic_spline.hpp"
#include "autoware/trajectory/interpolator/linear.hpp"
#include "autoware/trajectory/interpolator/spherical_linear.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/trajectory_point.hpp"
#include "autoware_utils_geometry/geometry.hpp"

#include <range/v3/to_container.hpp>
#include <range/v3/view/drop.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip_with.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <optional>
#include <set>
#include <string>
#include <vector>

namespace autoware::experimental::trajectory
{

namespace detail
{
template <typename PointType>
tl::expected<std::vector<PointType>, std::string> populate4(const std::vector<PointType> & inputs);

template <typename PointType>
tl::expected<std::vector<PointType>, std::string> populate5(const std::vector<PointType> & inputs);
}  // namespace detail

/**
 * @brief if the input path point size is less than 4, linearly interpolate the given points to 4,
 * and then apply either cubic or akima spline. if use_akima = true, the underlying points are
 * increased to at least 5
 * @param[in] path path object
 * @param[in] use_akima if true, use akima spline instead
 * @return return nullopt if the input point size is 0 or 1, otherwise return Trajectory class
 */
template <typename PointType>
std::optional<Trajectory<PointType>> pretty_build(
  const std::vector<PointType> & points, const bool use_akima = false)
{
  if (use_akima) {
    const auto try_input5 = detail::populate5(points);
    if (!try_input5) {
      return std::nullopt;
    }
    const auto & points_get5 = try_input5.value();

    using Builder = typename Trajectory<PointType>::Builder;
    const auto try_trajectory =
      Builder{}.template set_xy_interpolator<interpolator::AkimaSpline>().build(points_get5);
    if (!try_trajectory) {
      return std::nullopt;
    }
    return try_trajectory.value();
  }
  const auto try_input4 = detail::populate4(points);
  if (!try_input4) {
    return std::nullopt;
  }
  const auto & points_get4 = try_input4.value();

  using Builder = typename Trajectory<PointType>::Builder;
  const auto try_trajectory = Builder{}.build(points_get4);
  if (!try_trajectory) {
    return std::nullopt;
  }
  return try_trajectory.value();
}

namespace detail
{

template <typename T>
const geometry_msgs::msg::Pose & get_geometry_msgs_pose([[maybe_unused]] const T & point)
{
  static_assert("No specialization given");
  throw;
}

template <>
inline const geometry_msgs::msg::Pose & get_geometry_msgs_pose(
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point)
{
  return point.point.pose;
}

template <>
inline const geometry_msgs::msg::Pose & get_geometry_msgs_pose(
  const autoware_planning_msgs::msg::PathPoint & point)
{
  return point.pose;
}

template <>
inline const geometry_msgs::msg::Pose & get_geometry_msgs_pose(
  const autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  return point.pose;
}

[[maybe_unused]] inline std::vector<double> insert_middle_into_largest_interval(
  const std::vector<double> & bases)
{
  const auto base_diffs =
    ranges::views::zip_with(
      [](const double a, const double b) { return b - a; }, bases, bases | ranges::views::drop(1)) |
    ranges::to<std::vector>();
  const auto max_interval_start =
    std::distance(base_diffs.begin(), std::max_element(base_diffs.begin(), base_diffs.end()));
  const auto new_base = (bases.at(max_interval_start) + bases.at(max_interval_start + 1)) / 2.0;
  std::set<double> sorted_new_bases{bases.begin(), bases.end()};
  sorted_new_bases.insert(new_base);
  return {sorted_new_bases.begin(), sorted_new_bases.end()};
}

/**
 * @brief if the input point size is less than 3, add 3rd point, otherwise return as is
 * @param[in] inputs vector of point whose size is at least 2
 * @return the vector of points whose size is at least 3, or error reason
 * @note {x, y, z} are interpolated by Linear. other properties as
 * interpolated by default
 */
tl::expected<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>, std::string>
populate3(const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & inputs);

tl::expected<std::vector<autoware_planning_msgs::msg::PathPoint>, std::string> populate3(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & inputs);

tl::expected<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>, std::string> populate3(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & inputs);

/**
 * @brief if the input point size is less than 4, add 4th point, otherwise return as is
 * @param[in] inputs inputs
 * @return optional the vector of points whose size is at least 4, so that it can be interpolated by
 * CubicSpline, or error reason
 * @note {x, y, z} are interpolated by Linear. other properties as
 * interpolated by default
 */
template <typename PointType>
tl::expected<std::vector<PointType>, std::string> populate4(const std::vector<PointType> & inputs)
{
  if (inputs.size() >= 4) {
    return inputs;
  }
  if (inputs.size() < 2) {
    return tl::unexpected(std::string("cannot populate4() from less than 1 points!"));
  }

  const auto try_inputs3 = populate3(inputs);
  if (!try_inputs3) {
    return tl::unexpected(try_inputs3.error());
  }
  const auto & inputs3 = inputs.size() == 2 ? try_inputs3.value() : inputs;

  using Builder = typename Trajectory<PointType>::Builder;

  const auto input3_interpolation_result = Builder{}
                                             .template set_xy_interpolator<interpolator::Linear>()
                                             .template set_z_interpolator<interpolator::Linear>()
                                             .build(inputs3);

  // LCOV_EXCL_START
  if (!input3_interpolation_result) {
    // actually this block is impossible because 3 points are given, which is sufficient for Linear
    // interpolation
    return tl::unexpected(std::string("failed Linear interpolation in populate4()!"));
  }
  // LCOV_EXCL_END

  const auto & interpolation = input3_interpolation_result.value();

  const auto new_bases = insert_middle_into_largest_interval(interpolation.get_underlying_bases());
  return new_bases |
         ranges::views::transform([&](const double s) { return interpolation.compute(s); }) |
         ranges::to<std::vector>();
}

extern template tl::expected<
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>, std::string>
populate4(const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & inputs);

extern template tl::expected<std::vector<autoware_planning_msgs::msg::PathPoint>, std::string>
populate4(const std::vector<autoware_planning_msgs::msg::PathPoint> & inputs);

extern template tl::expected<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>, std::string>
populate4(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & inputs);

/**
 * @brief if the input point size is less than 5, add 5th point, otherwise return as is
 * @param[in] inputs inputs
 * @return the vector of points whose size is at least 5, so that it can be interpolated by
 * AkimaSpline, or error reason
 * @note all fields are internally interpolated by default
 */
template <typename PointType>
tl::expected<std::vector<PointType>, std::string> populate5(const std::vector<PointType> & inputs)
{
  if (inputs.size() >= 5) {
    return inputs;
  }
  if (inputs.size() < 2) {
    return tl::unexpected(std::string("cannot populate5() from less than 1 points!"));
  }

  const auto try_inputs4 = populate4(inputs);
  if (!try_inputs4) {
    return tl::unexpected(try_inputs4.error());
  }
  const auto & inputs4 = inputs.size() == 4 ? inputs : try_inputs4.value();

  using Builder = typename Trajectory<PointType>::Builder;
  const auto input4_interpolation_result = Builder{}.build(inputs4);

  // LCOV_EXCL_START
  if (!input4_interpolation_result) {
    // actually this block is impossible because 3 points are given, which is sufficient for Linear
    // interpolation
    return tl::unexpected(std::string("failed Linear interpolation in populate4()!"));
  }
  // LCOV_EXCL_END

  const auto & interpolation = input4_interpolation_result.value();

  const auto new_bases = insert_middle_into_largest_interval(interpolation.get_underlying_bases());
  // assert(bases.size() == 4);
  return new_bases |
         ranges::views::transform([&](const double s) { return interpolation.compute(s); }) |
         ranges::to<std::vector>();
}

extern template tl::expected<
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>, std::string>
populate5(const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & inputs);

extern template tl::expected<std::vector<autoware_planning_msgs::msg::PathPoint>, std::string>
populate5(const std::vector<autoware_planning_msgs::msg::PathPoint> & inputs);

extern template tl::expected<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>, std::string>
populate5(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & inputs);

}  // namespace detail

extern template std::optional<Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>
pretty_build(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const bool use_akima = false);

extern template std::optional<Trajectory<autoware_planning_msgs::msg::PathPoint>> pretty_build(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points, const bool use_akima = false);

extern template std::optional<Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>>
pretty_build(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const bool use_akima = false);

}  // namespace autoware::experimental::trajectory
#endif  // AUTOWARE__TRAJECTORY__UTILS__PRETTY_BUILD_HPP_
