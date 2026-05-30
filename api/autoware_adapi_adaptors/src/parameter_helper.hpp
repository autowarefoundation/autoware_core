// Copyright 2025 The Autoware Contributors
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

#ifndef PARAMETER_HELPER_HPP_
#define PARAMETER_HELPER_HPP_

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::adapi_adaptors
{

/// Validate that the vector has exactly N elements and copy it into a fixed-size array.
///
/// This is the pure, node-independent part of the covariance parameter parsing so that
/// the size-validation throw branch can be unit-tested without a live rclcpp::Node.
template <typename T, std::size_t N>
std::array<T, N> vector_to_array(const std::vector<T> & vector)
{
  if (vector.size() != N) {
    throw std::invalid_argument("The covariance parameter size is not " + std::to_string(N) + ".");
  }
  std::array<T, N> array;
  std::copy_n(vector.begin(), N, array.begin());
  return array;
}

}  // namespace autoware::adapi_adaptors

#endif  // PARAMETER_HELPER_HPP_
