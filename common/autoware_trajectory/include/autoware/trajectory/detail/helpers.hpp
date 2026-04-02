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

#ifndef AUTOWARE__TRAJECTORY__DETAIL__HELPERS_HPP_
#define AUTOWARE__TRAJECTORY__DETAIL__HELPERS_HPP_

#include "autoware/trajectory/interpolator/result.hpp"

#include <functional>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory::detail
{
/**
 * @brief Return a failure when no fallback interpolator candidate remains.
 */
template <typename TargetPtr, typename ValueType>
interpolator::InterpolationResult build_with_fallback_candidates(
  TargetPtr &, const std::vector<double> &, const std::vector<ValueType> &)
{
  return tl::unexpected(interpolator::InterpolationFailure{"no available fallback interpolator"});
}

/**
 * @brief Try fallback interpolator factories until one successfully builds.
 * @param[out] target Interpolator pointer replaced with the successful candidate.
 * @param[in] bases Interpolation bases.
 * @param[in] values Interpolation values.
 * @param[in] factory First fallback factory to try.
 * @param[in] factories Remaining fallback factories.
 * @return Successful interpolation result, or the last failure if all candidates fail.
 */
template <typename TargetPtr, typename ValueType, typename Factory, typename... Factories>
interpolator::InterpolationResult build_with_fallback_candidates(
  TargetPtr & target, const std::vector<double> & bases, const std::vector<ValueType> & values,
  Factory && factory, Factories &&... factories)
{
  auto candidate = std::invoke(std::forward<Factory>(factory));
  auto result = candidate->build(bases, values);
  if (result) {
    target = std::move(candidate);
    return result;
  }

  if constexpr (sizeof...(factories) == 0) {
    return tl::unexpected(interpolator::InterpolationFailure{result.error().what});
  } else {
    return build_with_fallback_candidates(
      target, bases, values, std::forward<Factories>(factories)...);
  }
}

/**
 * @brief Build with the current interpolator and fall back to alternative factories on failure.
 * @param[out] target Interpolator pointer to build, replaced if a fallback succeeds.
 * @param[in] bases Interpolation bases.
 * @param[in] values Interpolation values.
 * @param[in] factories Fallback interpolator factories.
 * @return Successful interpolation result, or a failure if all attempts fail.
 */
template <typename TargetPtr, typename ValueType, typename... Factories>
interpolator::InterpolationResult build_with_fallback(
  TargetPtr & target, const std::vector<double> & bases, const std::vector<ValueType> & values,
  Factories &&... factories)
{
  if (auto result = target->build(bases, values); result) {
    return result;
  }

  return build_with_fallback_candidates(
    target, bases, values, std::forward<Factories>(factories)...);
}

/**
 * @brief Check whether bases are strictly increasing with epsilon margin.
 * @param[in] bases Interpolation bases.
 * @param[in] epsilon Minimum required positive difference between adjacent bases.
 * @return True when all adjacent base differences are greater than epsilon.
 */
inline bool has_strictly_increasing_bases(
  const std::vector<double> & bases, const double epsilon = std::numeric_limits<double>::epsilon())
{
  for (size_t i = 1; i < bases.size(); ++i) {
    if ((bases.at(i) - bases.at(i - 1)) <= epsilon) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Crop bases to the closed interval `[start, end]`, inserting boundaries if needed.
 * @param[in] x Input bases.
 * @param[in] start Crop start.
 * @param[in] end Crop end.
 * @return Cropped bases including start and end boundaries.
 */
std::vector<double> crop_bases(const std::vector<double> & x, const double start, const double end);

/**
 * @brief Find the first position where predicate becomes true using binary search.
 *
 * This function performs binary search where the predicate transitions from false to true.
 * It searches for the point where:
 * - predicate(x) is false for x < result
 * - predicate(x) is true for x >= result
 *
 * @tparam Predicate Callable that takes a double and returns bool
 * @param low Lower bound (predicate should be false here)
 * @param high Upper bound (predicate should be true here)
 * @param predicate Function to evaluate at each midpoint
 * @param max_iter Maximum number of iterations
 * @param tolerance Convergence tolerance (search stops when high - low <= tolerance)
 * @return Value where predicate transitions from false to true (returns high)
 *
 * @note If predicate is already true at low, returns low
 * @note If predicate is false at high, returns high (no valid transition found)
 */
template <typename Predicate>
inline double lower_bound_by_predicate(
  double low, double high, Predicate predicate, size_t max_iter,
  double tolerance = std::numeric_limits<double>::epsilon())
{
  for (size_t i = 0; i < max_iter; ++i) {
    if (high - low <= tolerance || i == max_iter - 1) {
      break;
    }

    const double mid = 0.5 * (low + high);
    if (predicate(mid)) {
      high = mid;
    } else {
      low = mid;
    }
  }

  return high;
}

/**
 * @brief Find the last position where predicate is true using binary search.
 *
 * This function performs binary search where the predicate transitions from true to false.
 * It searches for the point where:
 * - predicate(x) is true for x <= result
 * - predicate(x) is false for x > result
 *
 * @tparam Predicate Callable that takes a double and returns bool
 * @param low Lower bound (predicate should be true here)
 * @param high Upper bound (predicate should be false here)
 * @param predicate Function to evaluate at each midpoint
 * @param max_iter Maximum number of iterations
 * @param tolerance Convergence tolerance
 * @return Value where predicate transitions from true to false (returns low)
 */
template <typename Predicate>
inline double upper_bound_by_predicate(
  double low, double high, Predicate predicate, size_t max_iter,
  double tolerance = std::numeric_limits<double>::epsilon())
{
  for (size_t i = 0; i < max_iter; ++i) {
    if (high - low <= tolerance || i == max_iter - 1) {
      break;
    }

    const double mid = 0.5 * (low + high);
    if (predicate(mid)) {
      low = mid;
    } else {
      high = mid;
    }
  }

  return low;
}

}  // namespace autoware::experimental::trajectory::detail

#endif  // AUTOWARE__TRAJECTORY__DETAIL__HELPERS_HPP_
