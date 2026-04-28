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

#ifndef AUTOWARE__TRAJECTORY__INTERPOLATOR__PCHIP_HPP_
#define AUTOWARE__TRAJECTORY__INTERPOLATOR__PCHIP_HPP_

#include "autoware/trajectory/interpolator/detail/interpolator_mixin.hpp"

#include <Eigen/Dense>

#include <limits>
#include <vector>

namespace autoware::experimental::trajectory::interpolator
{

/**
 * @brief Class for PCHIP interpolation.
 *
 * PCHIP (Piecewise Cubic Hermite Interpolating Polynomial) preserves the shape
 * of the input data better than ordinary cubic splines and avoids overshoot on
 * monotone data.
 */
class Pchip : public detail::InterpolatorMixin<Pchip, double>
{
private:
  double epsilon_{};               ///< Duplicate-base tolerance.
  Eigen::VectorXd a_, b_, c_, d_;  ///< Polynomial coefficients for each interval.
  Eigen::VectorXd h_;              ///< Interval sizes between consecutive bases.
  Eigen::VectorXd m_;              ///< First derivatives at knot points.

  /**
   * @brief Compute PCHIP polynomial parameters.
   *
   * Each interval is represented as:
   *   f(x) = a_i + b_i * dx + c_i * dx^2 + d_i * dx^3
   * where dx = x - bases[i].
   *
   * @param bases The strictly increasing bases.
   * @param values The values to interpolate.
   */
  void compute_parameters(
    const Eigen::Ref<const Eigen::VectorXd> & bases,
    const Eigen::Ref<const Eigen::VectorXd> & values);

  /**
   * @brief Compute endpoint derivative for PCHIP.
   *
   * This is the standard one-sided estimate with monotonicity limiting.
   *
   * @param h0 Interval size of the first adjacent segment.
   * @param h1 Interval size of the second adjacent segment.
   * @param delta0 Secant slope of the first adjacent segment.
   * @param delta1 Secant slope of the second adjacent segment.
   * @return Limited endpoint derivative.
   */
  [[nodiscard]] static double compute_endpoint_derivative(
    double h0, double h1, double delta0, double delta1);

  /**
   * @brief Build the interpolator with the given values.
   *
   * @param bases The bases values.
   * @param values The values to interpolate.
   * @return True if the interpolator was built successfully, false otherwise.
   */
  [[nodiscard]] bool build_impl(
    const std::vector<double> & bases, const std::vector<double> & values) override;

  /**
   * @brief Build the interpolator with the given values.
   *
   * @param bases The bases values.
   * @param values The values to interpolate.
   * @return True if the interpolator was built successfully, false otherwise.
   */
  [[nodiscard]] bool build_impl(
    const std::vector<double> & bases, std::vector<double> && values) override;

  /**
   * @brief Compute the interpolated value at the given point.
   *
   * @param s The point at which to compute the interpolated value.
   * @return The interpolated value.
   */
  double compute_impl(const double s) const override;

  /**
   * @brief Compute the first derivative at the given point.
   *
   * @param s The point at which to compute the first derivative.
   * @return The first derivative.
   */
  double compute_first_derivative_impl(const double s) const override;

  /**
   * @brief Compute the second derivative at the given point.
   *
   * @param s The point at which to compute the second derivative.
   * @return The second derivative.
   */
  double compute_second_derivative_impl(const double s) const override;

public:
  explicit Pchip(const double epsilon = std::numeric_limits<double>::epsilon()) : epsilon_(epsilon)
  {
  }

  /**
   * @brief Get the minimum number of required points for the interpolator.
   *
   * PCHIP can be constructed from at least 2 points.
   *
   * @return The minimum number of required points.
   */
  size_t minimum_required_points() const override { return 2; }
};

}  // namespace autoware::experimental::trajectory::interpolator

#endif  // AUTOWARE__TRAJECTORY__INTERPOLATOR__PCHIP_HPP_
