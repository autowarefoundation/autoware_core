// Copyright 2026 TIER IV, Inc.
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

#include "autoware/trajectory/utils/find_if.hpp"

#include <vector>

namespace autoware::experimental::trajectory
{

namespace detail::impl
{

namespace
{
// Binary search where `low` is false, `high` is true
double binary_search(
  double low, double high, const std::function<bool(const double &)> & constraint,
  const size_t max_iter)
{
  for (size_t i = 0; i < max_iter; ++i) {
    const auto mid = (low + high) / 2;
    if (constraint(mid)) {
      high = mid;  // If mid is valid, move end backward
    } else {
      low = mid;  // If mid is invalid, move start forward
    }
  }
  return high;
}
}  // namespace

std::optional<double> find_first_index_if_impl(
  const std::vector<double> & bases, const std::function<bool(const double &)> & constraint,
  const size_t max_iter)
{
  for (size_t i = 0; i < bases.size(); ++i) {
    if (!constraint(bases.at(i))) {
      continue;
    }
    if (i == 0) {
      return bases.at(i);
    }
    return binary_search(bases.at(i - 1), bases.at(i), constraint, max_iter);
  }
  return std::nullopt;
}

}  // namespace detail::impl

}  // namespace autoware::experimental::trajectory
