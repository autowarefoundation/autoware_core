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

#include "autoware/trajectory/interpolator/lane_ids_interpolator.hpp"

#include <cmath>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory::interpolator
{

std::vector<int64_t> LaneIdsInterpolator::compute_impl(const double s) const
{
  const int32_t idx = this->get_index(s);

  // Check for exact matches at base points
  if (s == this->bases_[idx]) {
    return values_.at(idx);
  }
  if (idx + 1 < static_cast<int32_t>(this->bases_.size()) && s == this->bases_[idx + 1]) {
    return values_.at(idx + 1);
  }

  // Get the two adjacent values for interpolation
  const std::vector<int64_t> & left_value = values_.at(idx);
  const std::vector<int64_t> & right_value = values_.at(idx + 1);

  // Domain knowledge: prefer boundaries with single lane IDs over multiple lane IDs
  // This handles the case where lane boundaries should contain more than two elements
  if (left_value.size() == 1 && right_value.size() > 1) {
    // Create a copy to avoid potential compiler optimization issues
    std::vector<int64_t> result;
    result.reserve(left_value.size());
    result.assign(left_value.begin(), left_value.end());
    return result;
  }
  if (left_value.size() > 1 && right_value.size() == 1) {
    // Create a copy to avoid potential compiler optimization issues
    std::vector<int64_t> result;
    result.reserve(right_value.size());
    result.assign(right_value.begin(), right_value.end());
    return result;
  }  // If both are single or both are multiple, choose the closest one
  const double left_distance = s - this->bases_[idx];
  const double right_distance = this->bases_[idx + 1] - s;
  
  // Create a copy to avoid potential compiler optimization issues
  const std::vector<int64_t> & chosen_value = (left_distance <= right_distance) ? left_value : right_value;
  std::vector<int64_t> result;
  result.reserve(chosen_value.size());
  result.assign(chosen_value.begin(), chosen_value.end());
  return result;
}

bool LaneIdsInterpolator::build_impl(
  const std::vector<double> & bases, const std::vector<std::vector<int64_t>> & values)
{
  this->bases_ = bases;
  this->values_ = values;
  return true;
}

bool LaneIdsInterpolator::build_impl(
  const std::vector<double> & bases, std::vector<std::vector<int64_t>> && values)
{
  this->bases_ = bases;
  this->values_ = std::move(values);
  return true;
}

size_t LaneIdsInterpolator::minimum_required_points() const
{
  return 2;
}

}  // namespace autoware::experimental::trajectory::interpolator
