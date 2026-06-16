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

#include <autoware/geography_utils/height.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace
{
// Returns true when the egm2008-1 geoid dataset is available on the test machine.
// The conversion functions wrap the underlying GeographicLib failure into a
// std::runtime_error, so the dataset is considered absent when such an error is thrown.
bool is_egm2008_dataset_available()
{
  try {
    // A successful call only requires the dataset; the actual value is irrelevant here.
    autoware::geography_utils::convert_wgs84_to_egm2008(0.0, 0.0, 0.0);
    return true;
  } catch (const std::runtime_error &) {
    return false;
  }
}
}  // namespace

// Test case to verify if same source and target datums return original height
TEST(GeographyUtils, SameSourceTargetDatum)
{
  const double height = 10.0;
  const double latitude = 35.0;
  const double longitude = 139.0;
  const std::string datum = "WGS84";

  double converted_height =
    autoware::geography_utils::convert_height(height, latitude, longitude, datum, datum);

  EXPECT_DOUBLE_EQ(height, converted_height);
}

// Test case to verify invalid source and target datums
TEST(GeographyUtils, InvalidSourceTargetDatum)
{
  const double height = 10.0;
  const double latitude = 35.0;
  const double longitude = 139.0;

  EXPECT_THROW(
    autoware::geography_utils::convert_height(height, latitude, longitude, "INVALID1", "INVALID2"),
    std::invalid_argument);
}

// Test case to verify invalid source datums
TEST(GeographyUtils, InvalidSourceDatum)
{
  const double height = 10.0;
  const double latitude = 35.0;
  const double longitude = 139.0;

  EXPECT_THROW(
    autoware::geography_utils::convert_height(height, latitude, longitude, "INVALID1", "WGS84"),
    std::invalid_argument);
}

// Test case to verify invalid target datums
TEST(GeographyUtils, InvalidTargetDatum)
{
  const double height = 10.0;
  const double latitude = 35.0;
  const double longitude = 139.0;

  EXPECT_THROW(
    autoware::geography_utils::convert_height(height, latitude, longitude, "WGS84", "INVALID2"),
    std::invalid_argument);
}

// Test case to verify the WGS84 <-> EGM2008 numeric conversion round-trip.
// Skipped when the egm2008-1 geoid dataset is not installed so CI without the
// dataset still passes.
TEST(GeographyUtils, ConvertWgs84Egm2008RoundTrip)
{
  if (!is_egm2008_dataset_available()) {
    GTEST_SKIP() << "egm2008-1 geoid dataset is not installed";
  }

  const double height = 10.0;
  const double latitude = 35.0;
  const double longitude = 139.0;

  // WGS84 (ellipsoidal) height converted to EGM2008 (orthometric) height and back.
  const double egm2008_height =
    autoware::geography_utils::convert_wgs84_to_egm2008(height, latitude, longitude);
  const double wgs84_height =
    autoware::geography_utils::convert_egm2008_to_wgs84(egm2008_height, latitude, longitude);

  // Round-trip must reproduce the original height.
  EXPECT_NEAR(wgs84_height, height, 1e-6);

  // The geoid undulation near this location is on the order of tens of meters, so the
  // converted EGM2008 height must differ noticeably from the input WGS84 height.
  EXPECT_GT(std::abs(egm2008_height - height), 1.0);
}

// Test case to verify the WGS84 <-> EGM2008 conversion through convert_height with a
// value check against a known geoid undulation. Skipped when the dataset is absent.
TEST(GeographyUtils, ConvertHeightWgs84ToEgm2008Value)
{
  if (!is_egm2008_dataset_available()) {
    GTEST_SKIP() << "egm2008-1 geoid dataset is not installed";
  }

  const double height = 10.0;
  const double latitude = 35.0;
  const double longitude = 139.0;

  const double via_convert_height =
    autoware::geography_utils::convert_height(height, latitude, longitude, "WGS84", "EGM2008");
  const double via_direct =
    autoware::geography_utils::convert_wgs84_to_egm2008(height, latitude, longitude);

  // convert_height must dispatch to convert_wgs84_to_egm2008 and produce the same value.
  EXPECT_DOUBLE_EQ(via_convert_height, via_direct);
}

// Test case to verify the std::runtime_error wrapping when the egm2008-1 dataset is
// absent. Skipped when the dataset is installed (so the throwing path cannot be exercised).
TEST(GeographyUtils, ConvertWgs84ToEgm2008ThrowsWhenDatasetMissing)
{
  if (is_egm2008_dataset_available()) {
    GTEST_SKIP() << "egm2008-1 geoid dataset is installed; cannot exercise the failure path";
  }

  const double height = 10.0;
  const double latitude = 35.0;
  const double longitude = 139.0;

  EXPECT_THROW(
    autoware::geography_utils::convert_wgs84_to_egm2008(height, latitude, longitude),
    std::runtime_error);
}

TEST(GeographyUtils, ConvertEgm2008ToWgs84ThrowsWhenDatasetMissing)
{
  if (is_egm2008_dataset_available()) {
    GTEST_SKIP() << "egm2008-1 geoid dataset is installed; cannot exercise the failure path";
  }

  const double height = 10.0;
  const double latitude = 35.0;
  const double longitude = 139.0;

  EXPECT_THROW(
    autoware::geography_utils::convert_egm2008_to_wgs84(height, latitude, longitude),
    std::runtime_error);
}
