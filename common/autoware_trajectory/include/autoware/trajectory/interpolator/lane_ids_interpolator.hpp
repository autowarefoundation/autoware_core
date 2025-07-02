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

#ifndef AUTOWARE__TRAJECTORY__INTERPOLATOR__LANE_IDS_INTERPOLATOR_HPP_
#define AUTOWARE__TRAJECTORY__INTERPOLATOR__LANE_IDS_INTERPOLATOR_HPP_

#include "autoware/trajectory/interpolator/detail/interpolator_mixin.hpp"

#include <Eigen/Dense>

#include <vector>

namespace autoware::experimental::trajectory::interpolator
{

// lane ids : std::vector<int64_t>

class LaneIdsInterpolator
: public detail::InterpolatorMixin<LaneIdsInterpolator, std::vector<int64_t>>
{
protected:
  std::vector<std::vector<int64_t>> values_;  ///< Interpolation values.

  /**
   * @brief Compute the interpolated lane IDs at the given point.
   *
   * Uses domain knowledge: the boundary of lane id should contain more than two elements,
   * so we prioritize returning single-element lane IDs over multi-element ones.
   *
   * @param s The point at which to compute the interpolated value.
   * @return The interpolated lane IDs.
   */
  std::vector<int64_t> compute_impl(const double s) const override;

  /**
   * @brief Build the interpolator with the given values.
   *
   * @param bases The bases values.
   * @param values The values to interpolate.
   * @return True if the interpolator was built successfully, false otherwise.
   */
  [[nodiscard]] bool build_impl(
    const std::vector<double> & bases, const std::vector<std::vector<int64_t>> & values) override;

  /**
   * @brief Build the interpolator with the given values.
   *
   * @param bases The bases values.
   * @param values The values to interpolate.
   * @return True if the interpolator was built successfully, false otherwise.
   */
  [[nodiscard]] bool build_impl(
    const std::vector<double> & bases, std::vector<std::vector<int64_t>> && values) override;

public:
  /**
   * @brief Get the minimum number of required points for the interpolator.
   *
   * @return The minimum number of required points.
   */
  size_t minimum_required_points() const override;
};

}  // namespace autoware::experimental::trajectory::interpolator

#endif  // AUTOWARE__TRAJECTORY__INTERPOLATOR__LANE_IDS_INTERPOLATOR_HPP_
