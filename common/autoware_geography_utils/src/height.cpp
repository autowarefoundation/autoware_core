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

#include "autoware/geography_utils/height.hpp"

#include <GeographicLib/Geoid.hpp>

#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace autoware::geography_utils
{

double convert_wgs84_to_egm2008(const double height, const double latitude, const double longitude)
{
  try {
    GeographicLib::Geoid egm2008("egm2008-1");
    // cSpell: ignore ELLIPSOIDTOGEOID
    return egm2008.ConvertHeight(
      latitude, longitude, height, GeographicLib::Geoid::ELLIPSOIDTOGEOID);
  } catch (const std::exception & e) {
    throw std::runtime_error(
      std::string{"Failed to convert WGS84 to EGM2008. Make sure to install geoid data with `sudo "
                  "geographiclib-get-geoids egm2008-1` "}
        .append(e.what()));
  }
}

double convert_egm2008_to_wgs84(const double height, const double latitude, const double longitude)
{
  try {
    GeographicLib::Geoid egm2008("egm2008-1");
    // cSpell: ignore GEOIDTOELLIPSOID
    return egm2008.ConvertHeight(
      latitude, longitude, height, GeographicLib::Geoid::GEOIDTOELLIPSOID);
  } catch (const std::exception & e) {
    throw std::runtime_error(
      std::string{"Failed to convert EGM2008 to WGS84. Make sure to install geoid data with `sudo "
                  "geographiclib-get-geoids egm2008-1` "}
        .append(e.what()));
  }
}

double convert_height(
  const double height, const double latitude, const double longitude,
  std::string_view source_vertical_datum, std::string_view target_vertical_datum)
{
  if (source_vertical_datum == target_vertical_datum) {
    return height;
  }
  static const std::map<std::pair<std::string_view, std::string_view>, HeightConversionFunction>
    conversion_map{
      {{"WGS84", "EGM2008"}, convert_wgs84_to_egm2008},
      {{"EGM2008", "WGS84"}, convert_egm2008_to_wgs84},
    };

  const auto key = std::make_pair(source_vertical_datum, target_vertical_datum);
  if (const auto it = conversion_map.find(key); it != conversion_map.end()) {
    return it->second(height, latitude, longitude);
  }

  throw std::invalid_argument(std::string{"Invalid conversion types: "}
                                .append(source_vertical_datum)
                                .append(" to ")
                                .append(target_vertical_datum));
}

}  // namespace autoware::geography_utils
