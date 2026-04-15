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

#ifndef AUTOWARE__TRAJECTORY__DETAIL__VALIDATE_RANGE_HPP_
#define AUTOWARE__TRAJECTORY__DETAIL__VALIDATE_RANGE_HPP_

#include <stdexcept>
#include <string>

namespace autoware::experimental::trajectory::detail
{

/**
 * @brief Throw std::out_of_range if value is outside [min, max] with tolerance.
 * @param[in] value Value to validate.
 * @param[in] min Inclusive lower bound.
 * @param[in] max Inclusive upper bound.
 * @param[in] name Human-readable name of the value (used in the exception message).
 * @param[in] tolerance Epsilon margin around the bounds. Default is 1e-5.
 */
inline void throw_if_out_of_range(
  const double value, const double min, const double max, const char * name,
  const double tolerance = 1e-5)
{
  if (value < min - tolerance || value > max + tolerance) {
    throw std::out_of_range(
      std::string(name) + " is out of range [" + std::to_string(min) + ", " + std::to_string(max) +
      "]");
  }
}

}  // namespace autoware::experimental::trajectory::detail

#endif  // AUTOWARE__TRAJECTORY__DETAIL__VALIDATE_RANGE_HPP_
