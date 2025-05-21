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

#include <cstddef>
#include <vector>

namespace autoware::experimental::trajectory::detail
{
inline namespace helpers
{
/**
 * @brief Ensures the output vector has at least a specified number of points by linearly
 * interpolating values between each input intervals
 *
 * @param x Input vector of double values.
 * @param output_size_at_least Minimum number of points required.
 * @return A vector of size max(current_size, `output_size_at_least`)
 *
 * @note If `x.size() >= min_points`, the input vector is returned as-is.
 *
 * @code
 * std::vector<double> input = {1.0, 4.0, 6.0};
 * auto result = fill_bases(input, 6);
 * // result: {1.0, 2.0, 3.0, 4.0, 5.0, 6.0}
 * @endcode
 */
std::vector<double> fill_bases(const std::vector<double> & x, const size_t output_size_at_least);

std::vector<double> crop_bases(const std::vector<double> & x, const double start, const double end);
}  // namespace helpers
}  // namespace autoware::experimental::trajectory::detail

#endif  // AUTOWARE__TRAJECTORY__DETAIL__HELPERS_HPP_
