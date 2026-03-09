// Copyright 2022 Tier IV, Inc.
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

#ifndef AUTOWARE__INTERPOLATION__ZERO_ORDER_HOLD_HPP_
#define AUTOWARE__INTERPOLATION__ZERO_ORDER_HOLD_HPP_

#include "autoware/interpolation/interpolation_utils.hpp"

#include <vector>

namespace autoware::interpolation
{
/// @brief for each query key, return the index of the nearest segment in the base_keys
inline std::vector<size_t> calc_closest_segment_indices(
  const std::vector<double> & base_keys, const std::vector<double> & query_keys,
  const double overlap_threshold = 1e-3)
{
  // throw exception for invalid arguments
  const auto validated_query_keys = validateKeys(base_keys, query_keys);

  std::vector<size_t> closest_segment_indices;
  closest_segment_indices.reserve(validated_query_keys.size());
  size_t base_idx = 0;
  for (auto query_val : validated_query_keys) {
    // Search for the base segment such that segment.first <= query < segment.second
    while (base_idx + 1 < base_keys.size() &&
           base_keys[base_idx + 1] - overlap_threshold < query_val) {
      ++base_idx;
    }
    closest_segment_indices.push_back(base_idx);
  }
  return closest_segment_indices;
}

template <class T>
std::vector<T> zero_order_hold(
  const std::vector<double> & base_keys, const std::vector<T> & base_values,
  const std::vector<size_t> & closest_segment_indices)
{
  // throw exception for invalid arguments
  validateKeysAndValues(base_keys, base_values);

  std::vector<T> query_values(closest_segment_indices.size());
  for (size_t i = 0; i < closest_segment_indices.size(); ++i) {
    query_values.at(i) = base_values.at(closest_segment_indices.at(i));
  }

  return query_values;
}

template <class T>
std::vector<T> zero_order_hold(
  const std::vector<double> & base_keys, const std::vector<T> & base_values,
  const std::vector<double> & query_keys, const double overlap_threshold = 1e-3)
{
  return zero_order_hold(
    base_keys, base_values, calc_closest_segment_indices(base_keys, query_keys, overlap_threshold));
}
}  // namespace autoware::interpolation

#endif  // AUTOWARE__INTERPOLATION__ZERO_ORDER_HOLD_HPP_
