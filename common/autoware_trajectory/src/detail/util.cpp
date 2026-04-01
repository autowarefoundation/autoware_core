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

#include "autoware/trajectory/detail/helpers.hpp"

#include <algorithm>
#include <vector>

namespace autoware::experimental::trajectory::detail
{
std::vector<double> crop_bases(const std::vector<double> & x, const double start, const double end)
{
  std::vector<double> result;

  // Add start point if it's not already in x
  if (std::find(x.begin(), x.end(), start) == x.end()) {
    result.push_back(start);
  }

  // Copy all points within the range [start, end]
  std::copy_if(x.begin(), x.end(), std::back_inserter(result), [start, end](double i) {
    return i >= start && i <= end;
  });

  // Add end point if it's not already in x
  if (std::find(x.begin(), x.end(), end) == x.end()) {
    result.push_back(end);
  }

  return result;
}
}  // namespace autoware::experimental::trajectory::detail
