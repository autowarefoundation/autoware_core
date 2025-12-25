// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__INTERPOLATION__SPLINE_INTERPOLATION_HPP_
#define AUTOWARE__INTERPOLATION__SPLINE_INTERPOLATION_HPP_

#include "autoware/interpolation/interpolation_utils.hpp"

#include <Eigen/Core>
#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

namespace autoware::interpolation
{
// static spline interpolation functions
std::vector<double> spline(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const std::vector<double> & query_keys);
std::vector<double> splineByAkima(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const std::vector<double> & query_keys);

// non-static 1-dimensional spline interpolation
//
// Usage:
// ```
// SplineInterpolation spline;
// // memorize pre-interpolation result internally
// spline.calcSplineCoefficients(base_keys, base_values);
// const auto interpolation_result1 = spline.getSplineInterpolatedValues(
//   base_keys, query_keys1);
// const auto interpolation_result2 = spline.getSplineInterpolatedValues(
//   base_keys, query_keys2);
// ```
class SplineInterpolation
{
public:
  SplineInterpolation() = default;
  SplineInterpolation(
    const std::vector<double> & base_keys, const std::vector<double> & base_values)
  {
    calcSplineCoefficients(base_keys, base_values);
  }

  //!< @brief get values of spline interpolation on designated sampling points.
  //!< @details Assuming that query_keys are t vector for sampling, and interpolation is for x,
  //            meaning that spline interpolation was applied to x(t),
  //            return value will be x(t) vector
  std::vector<double> getSplineInterpolatedValues(const std::vector<double> & query_keys) const;

  //!< @brief get 1st differential values of spline interpolation on designated sampling points.
  //!< @details Assuming that query_keys are t vector for sampling, and interpolation is for x,
  //            meaning that spline interpolation was applied to x(t),
  //            return value will be dx/dt(t) vector
  std::vector<double> getSplineInterpolatedDiffValues(const std::vector<double> & query_keys) const;

  //!< @brief get 2nd differential values of spline interpolation on designated sampling points.
  //!< @details Assuming that query_keys are t vector for sampling, and interpolation is for x,
  //            meaning that spline interpolation was applied to x(t),
  //            return value will be d^2/dt^2(t) vector
  std::vector<double> getSplineInterpolatedQuadDiffValues(
    const std::vector<double> & query_keys) const;

  size_t getSize() const { return base_keys_.size(); }

  // Debug methods to expose spline coefficients
  const Eigen::VectorXd getCoefficients() const
  {
    const auto m = static_cast<Eigen::Index>(a_.size());
    Eigen::VectorXd coefficients(4 * m);
    coefficients << a_, b_, c_, d_;
    return coefficients;
  }
  std::vector<double> getKnots() const { return base_keys_; }

  void resize(const size_t size)
  {
    if (size > base_keys_.size()) {
      // Extending - just resize (new elements will be uninitialized, caller should fill them)
      a_.resize(size - 1);
      b_.resize(size - 1);
      c_.resize(size - 1);
      d_.resize(size - 1);
      base_keys_.resize(size);
    } else if (size < base_keys_.size()) {
      // Clipping - explicitly copy first N elements to preserve data
      const size_t n_segments = size - 1;
      Eigen::VectorXd a_new = a_.head(n_segments);
      Eigen::VectorXd b_new = b_.head(n_segments);
      Eigen::VectorXd c_new = c_.head(n_segments);
      Eigen::VectorXd d_new = d_.head(n_segments);
      a_ = std::move(a_new);
      b_ = std::move(b_new);
      c_ = std::move(c_new);
      d_ = std::move(d_new);
      base_keys_.resize(size);
    }
    // If size == base_keys_.size(), no-op
  }

private:
  Eigen::VectorXd a_;
  Eigen::VectorXd b_;
  Eigen::VectorXd c_;
  Eigen::VectorXd d_;

  std::vector<double> base_keys_;

  void calcSplineCoefficients(
    const std::vector<double> & base_keys, const std::vector<double> & base_values);

  Eigen::Index get_index(const double & key) const;
};
}  // namespace autoware::interpolation

#endif  // AUTOWARE__INTERPOLATION__SPLINE_INTERPOLATION_HPP_
