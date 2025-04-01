// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__GEOGRAPHY_UTILS__HEIGHT_HPP_
#define AUTOWARE__GEOGRAPHY_UTILS__HEIGHT_HPP_

#include <functional>
#include <string>
#include <string_view>

namespace autoware::geography_utils
{
using HeightConversionFunction = std::function<double(double, double, double)>;

double convert_wgs84_to_egm2008(const double height, const double latitude, const double longitude);
double convert_egm2008_to_wgs84(const double height, const double latitude, const double longitude);
double convert_height(
  const double height, const double latitude, const double longitude,
  std::string_view source_vertical_datum, std::string_view target_vertical_datum);

}  // namespace autoware::geography_utils

#endif  // AUTOWARE__GEOGRAPHY_UTILS__HEIGHT_HPP_
