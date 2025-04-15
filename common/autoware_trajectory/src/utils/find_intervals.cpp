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

#include "autoware/trajectory/utils/find_intervals.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace autoware::experimental::trajectory::detail::impl
{

// Binary search where `low` is false, `high` is true
double binary_search_start(
  double low, double high, const std::function<bool(const double &)> & constraint, int max_iter)
{
  for (int i = 0; i < max_iter; ++i) {
    double mid = 0.5 * (low + high);
    if (constraint(mid)) {
      high = mid;  // Mid is valid → move end closer
    } else {
      low = mid;  // Mid is invalid → move start forward
    }
  }
  return high;
}

// Binary search where `low` is true, `high` is false
double binary_search_end(
  double low, double high, const std::function<bool(const double &)> & constraint, int max_iter)
{
  for (int i = 0; i < max_iter; ++i) {
    double mid = 0.5 * (low + high);
    if (constraint(mid)) {
      low = mid;  // Mid is still valid → move start forward
    } else {
      high = mid;  // Mid is invalid → move end backward
    }
  }
  return low;
}

std::vector<Interval> find_intervals_impl(
  const std::vector<double> & bases, const std::function<bool(const double &)> & constraint,
  int max_iter)
{
  std::vector<Interval> intervals;

  std::optional<double> start = std::nullopt;
  for (size_t i = 0; i < bases.size(); ++i) {
    if (!start && constraint(bases.at(i))) {
      if (i > 0) {
        start = binary_search_start(bases.at(i - 1), bases.at(i), constraint, max_iter);
      } else {
        start = bases.at(i);  // Start a new interval}
      }
    } else if (start && !constraint(bases.at(i))) {
      // End the current interval if the constraint fails or it's the last element
      double end = binary_search_end(bases.at(i - 1), bases.at(i), constraint, max_iter);
      intervals.emplace_back(Interval{start.value(), end});
      start = std::nullopt;  // Reset the start
    } else if (start && i == bases.size() - 1) {
      // If the last element is valid, end the interval
      double end = bases.at(i);
      intervals.emplace_back(Interval{start.value(), end});
      start = std::nullopt;  // Reset the start
    }
  }
  return intervals;
}

}  // namespace autoware::experimental::trajectory::detail::impl
