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

// cspell:ignore SBAS GBAS

// Unit tests of the rclcpp-free logic in gnss_poser.hpp. They run without a ROS context; the
// node-level behavior is covered by test_gnss_poser_node.cpp.
//
// The GnssPoser cases come from the characterization suite, under the same names: what the node
// published for a fix is what GnssPoser::input_fix() returns for it. Each case is written as
// Arrange / Act / Assert: the inputs that set the scene, the one input whose effect is checked, and
// the checks.

#include "gnss_poser.hpp"

#include <autoware/geography_utils/height.hpp>
#include <autoware/geography_utils/projection.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using autoware::gnss_poser::GnssPosePubMethod;
using autoware::gnss_poser::GnssPoser;
using autoware::gnss_poser::GnssPoserParams;
using autoware_map_msgs::msg::MapProjectorInfo;
using autoware_sensing_msgs::msg::GnssInsOrientation;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::Transform;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;

// ---------------------------------------------------------------------------------------
// Reference input used throughout (same point as autoware_geography_utils' own tests).
// ---------------------------------------------------------------------------------------
constexpr double reference_latitude = 35.62426;
constexpr double reference_longitude = 139.74252;
constexpr double reference_altitude = 10.0;
constexpr const char * reference_mgrs_grid = "54SUE";
// Cases that need several fixes offset them by multiples of 0.001 degrees: about 111 m in latitude
// and 90 m in longitude at this latitude, so the fixes project to clearly distinct positions and
// every output can be matched to the fix it came from.

// Golden values: MGRS(54SUE) projection of (reference_latitude, reference_longitude,
// reference_altitude) as observed from the node output (identity antenna->base TF, WGS84 vertical
// datum). Recorded on 2026-09-03 with the GeographicLib/lanelet2 versions bundled in the autoware
// core-devel jazzy image.
constexpr double golden_x = 86128.181788819958;
constexpr double golden_y = 43002.610125367064;
// Tolerances for computed values. A behavior change worth catching moves a position by centimeters
// or more and a heading by about 0.001 rad or more; the floating-point noise of the pipeline is
// many orders of magnitude below both. Values the node copies are compared exactly instead.
constexpr double position_tolerance = 1e-4;  // [m]
constexpr double angle_tolerance = 1e-6;     // [rad]
// EGM2008 geoid height at the reference point, from GeographicLib's GeoidEval on the egm2008-1
// dataset: independent of the node and of autoware_geography_utils.
constexpr double reference_geoid_height = 36.12;  // [m]
// The library and GeoidEval interpolate the same grid; 1 cm covers their difference while any
// missing (0 m) or wrongly signed (+36 m) conversion is far outside.
constexpr double geoid_height_tolerance = 0.01;  // [m]

// GnssInsOrientation::rmse_rotation_* are float32, so rmse^2 carries single-precision error.
constexpr double rmse_squared_tolerance = 1e-6;

// Covariance index helpers (6x6 row-major).
constexpr std::size_t cov_xx = 0;
constexpr std::size_t cov_yy = 7;
constexpr std::size_t cov_zz = 14;
constexpr std::size_t cov_roll_roll = 21;
constexpr std::size_t cov_pitch_pitch = 28;
constexpr std::size_t cov_yaw_yaw = 35;

NavSatFix make_fix(
  double latitude, double longitude, double altitude,
  NavSatStatus::_status_type status = NavSatStatus::STATUS_FIX)
{
  NavSatFix msg;
  msg.header.stamp.sec = 1700000000;
  msg.header.stamp.nanosec = 123456789U;
  msg.header.frame_id = "gnss";
  msg.status.status = status;
  msg.status.service = NavSatStatus::SERVICE_GPS;
  msg.latitude = latitude;
  msg.longitude = longitude;
  msg.altitude = altitude;
  msg.position_covariance = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  msg.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  return msg;
}

NavSatFix make_reference_fix(NavSatStatus::_status_type status = NavSatStatus::STATUS_FIX)
{
  return make_fix(reference_latitude, reference_longitude, reference_altitude, status);
}

// Fixes of the average cases: about 100 m apart (see the note at the reference constants).
std::vector<NavSatFix> make_average_fixes()
{
  return {
    make_fix(reference_latitude, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.001, reference_longitude + 0.002, reference_altitude + 20.0),
    make_fix(reference_latitude + 0.002, reference_longitude + 0.001, reference_altitude + 10.0),
    make_fix(reference_latitude + 0.003, reference_longitude + 0.003, reference_altitude + 30.0)};
}

// Fixes of the odd median cases.
//
// Chosen so that the median of x, of y and of z each come from a different sample, which is what
// makes the case discriminating: a "median sample" implementation would return one of the inputs.
std::vector<NavSatFix> make_median_fixes()
{
  return {
    make_fix(reference_latitude, reference_longitude, reference_altitude + 20.0),
    make_fix(reference_latitude + 0.001, reference_longitude + 0.002, reference_altitude),
    make_fix(reference_latitude + 0.002, reference_longitude + 0.001, reference_altitude + 10.0),
    make_fix(reference_latitude + 0.003, reference_longitude + 0.003, reference_altitude + 30.0)};
}

// Fixes of the even median cases.
std::vector<NavSatFix> make_even_median_fixes()
{
  return {
    make_fix(reference_latitude, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.003, reference_longitude + 0.001, reference_altitude + 30.0),
    make_fix(reference_latitude + 0.001, reference_longitude + 0.003, reference_altitude + 10.0),
    make_fix(reference_latitude + 0.002, reference_longitude + 0.002, reference_altitude + 20.0)};
}

MapProjectorInfo make_mgrs_projector_info(
  const std::string & grid = reference_mgrs_grid,
  const std::string & vertical_datum = MapProjectorInfo::WGS84)
{
  MapProjectorInfo msg;
  msg.projector_type = MapProjectorInfo::MGRS;
  msg.vertical_datum = vertical_datum;
  msg.mgrs_grid = grid;
  return msg;
}

MapProjectorInfo make_local_projector_info()
{
  MapProjectorInfo msg;
  msg.projector_type = MapProjectorInfo::LOCAL;
  msg.vertical_datum = MapProjectorInfo::WGS84;
  return msg;
}

MapProjectorInfo make_local_cartesian_utm_projector_info(
  double origin_lat, double origin_lon, double origin_alt)
{
  MapProjectorInfo msg;
  msg.projector_type = MapProjectorInfo::LOCAL_CARTESIAN_UTM;
  msg.vertical_datum = MapProjectorInfo::WGS84;
  msg.map_origin.latitude = origin_lat;
  msg.map_origin.longitude = origin_lon;
  msg.map_origin.altitude = origin_alt;
  return msg;
}

Quaternion yaw_to_quaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

double yaw_of(const Quaternion & quaternion)
{
  tf2::Quaternion q;
  tf2::fromMsg(quaternion, q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

// Two orientations agree when the rotation angle between them, 2 * acos(|q1 . q2|), is negligible.
void expect_same_rotation(
  const Quaternion & actual, const Quaternion & expected, double tol = angle_tolerance)
{
  const double dot =
    actual.x * expected.x + actual.y * expected.y + actual.z * expected.z + actual.w * expected.w;
  const double angle = 2.0 * std::acos(std::min(1.0, std::abs(dot)));
  EXPECT_LE(angle, tol) << "actual=(" << actual.x << "," << actual.y << "," << actual.z << ","
                        << actual.w << ") expected=(" << expected.x << "," << expected.y << ","
                        << expected.z << "," << expected.w << ")";
}

GnssInsOrientation make_orientation(
  double yaw, double rmse_x = 0.1, double rmse_y = 0.2, double rmse_z = 0.3)
{
  GnssInsOrientation msg;
  msg.orientation = yaw_to_quaternion(yaw);
  msg.rmse_rotation_x = rmse_x;
  msg.rmse_rotation_y = rmse_y;
  msg.rmse_rotation_z = rmse_z;
  return msg;
}

// Antenna position the node is expected to derive from a NavSatFix, computed with the same
// library the node uses (autoware_geography_utils). Used to build exact expectations for the
// buffering / orientation / TF composition logic, which is what these tests characterize.
Point project_antenna(const NavSatFix & fix, const MapProjectorInfo & projector_info)
{
  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.latitude = fix.latitude;
  geo_point.longitude = fix.longitude;
  geo_point.altitude = fix.altitude;
  Point position = autoware::geography_utils::project_forward(geo_point, projector_info);
  position.z = autoware::geography_utils::convert_height(
    position.z, geo_point.latitude, geo_point.longitude, MapProjectorInfo::WGS84,
    projector_info.vertical_datum);
  return position;
}

Point make_point(double x, double y, double z)
{
  Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

Point mean_of(const std::vector<Point> & points)
{
  Point mean = make_point(0.0, 0.0, 0.0);
  for (const auto & p : points) {
    mean.x += p.x;
    mean.y += p.y;
    mean.z += p.z;
  }
  const auto n = static_cast<double>(points.size());
  return make_point(mean.x / n, mean.y / n, mean.z / n);
}

double median_of(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  const std::size_t mid = values.size() / 2;
  return (values.size() % 2 == 1) ? values[mid] : (values[mid] + values[mid - 1]) / 2.0;
}

// Component-wise median (this is what the node computes, NOT the median sample).
Point component_wise_median_of(const std::vector<Point> & points)
{
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  for (const auto & p : points) {
    xs.push_back(p.x);
    ys.push_back(p.y);
    zs.push_back(p.z);
  }
  return make_point(median_of(xs), median_of(ys), median_of(zs));
}

void expect_point_near(
  const Point & actual, const Point & expected, double tol = position_tolerance)
{
  EXPECT_NEAR(actual.x, expected.x, tol);
  EXPECT_NEAR(actual.y, expected.y, tol);
  EXPECT_NEAR(actual.z, expected.z, tol);
}

bool is_egm2008_dataset_available()
{
  try {
    autoware::geography_utils::convert_wgs84_to_egm2008(0.0, 0.0, 0.0);
    return true;
  } catch (const std::runtime_error &) {
    return false;
  }
}

// Params of the buffering cases: gnss_pose_pub_method and buff_epoch, the rest at their defaults.
GnssPoserParams make_buffer_params(int gnss_pose_pub_method, int buff_epoch)
{
  GnssPoserParams params;
  params.gnss_pose_pub_method = static_cast<GnssPosePubMethod>(gnss_pose_pub_method);
  params.buff_epoch = buff_epoch;
  return params;
}

// Antenna position of a fix under the MGRS projector info of make_mgrs_projector_info().
Point project_mgrs(const NavSatFix & fix)
{
  return project_antenna(fix, make_mgrs_projector_info());
}

// Fixes of the instant cases with a large buff_epoch.
std::vector<NavSatFix> make_instant_fixes()
{
  // Three fixes about 100 m apart (see the note at the reference constants), so that each output
  // can be checked against its own fix and not a neighbor.
  return {
    make_fix(reference_latitude, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.001, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.002, reference_longitude + 0.001, reference_altitude + 1.0)};
}

// Fixes of the non-fixed buffer cases: an accepted fix, a NO_FIX, and another accepted fix.
std::vector<NavSatFix> make_non_fixed_buffer_fixes()
{
  return {
    make_fix(reference_latitude, reference_longitude, reference_altitude),
    make_fix(
      reference_latitude + 0.01, reference_longitude + 0.01, reference_altitude + 100.0,
      NavSatStatus::STATUS_NO_FIX),
    make_fix(reference_latitude + 0.001, reference_longitude + 0.001, reference_altitude + 10.0)};
}

// Fixes of the gated buffer cases: dropped without projector info, dropped under LOCAL, and two
// accepted ones.
std::vector<NavSatFix> make_gated_buffer_fixes()
{
  return {
    make_fix(reference_latitude + 0.01, reference_longitude + 0.01, reference_altitude + 100.0),
    make_fix(reference_latitude - 0.01, reference_longitude - 0.01, reference_altitude - 100.0),
    make_fix(reference_latitude, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.001, reference_longitude + 0.001, reference_altitude + 10.0)};
}

// Fixes of the motion orientation cases.
std::vector<NavSatFix> make_motion_fixes()
{
  return {
    make_fix(reference_latitude, reference_longitude, reference_altitude),
    make_fix(
      reference_latitude + 0.001, reference_longitude + 0.001,
      reference_altitude),  // north-east of fix1
    make_fix(
      reference_latitude + 0.001, reference_longitude - 0.001,
      reference_altitude)};  // west of fix2, north of fix1
}

// Fixes of the motion orientation cases with a buffer.
std::vector<NavSatFix> make_buffered_motion_fixes()
{
  return {
    make_fix(reference_latitude, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.001, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.001, reference_longitude + 0.002, reference_altitude)};
}

// The reference fix with a full covariance of the given type.
NavSatFix make_fix_with_covariance(NavSatFix::_position_covariance_type_type type)
{
  auto fix = make_reference_fix();
  // Only the diagonal (2.0, 3.0, 4.0) is propagated; off-diagonal entries such as 0.5 never are,
  // in any branch.
  fix.position_covariance = {2.0, 0.5, 0.6, 0.5, 3.0, 0.7, 0.6, 0.7, 4.0};
  fix.position_covariance_type = type;
  return fix;
}

void expect_position_variances(
  const std::array<double, 36> & covariance, double xx, double yy, double zz)
{
  EXPECT_DOUBLE_EQ(covariance[cov_xx], xx);
  EXPECT_DOUBLE_EQ(covariance[cov_yy], yy);
  EXPECT_DOUBLE_EQ(covariance[cov_zz], zz);
}

void expect_rotation_variances_near(
  const std::array<double, 36> & covariance, double roll, double pitch, double yaw)
{
  EXPECT_NEAR(covariance[cov_roll_roll], roll, rmse_squared_tolerance);
  EXPECT_NEAR(covariance[cov_pitch_pitch], pitch, rmse_squared_tolerance);
  EXPECT_NEAR(covariance[cov_yaw_yaw], yaw, rmse_squared_tolerance);
}

// The fixes of these cases carry the frame `gnss`, for which the node had no transform and
// composed with the identity; the lookup that stands in for TF provides the identity directly.
const GnssPoser::TransformLookup identity_lookup =
  [](const std::string &, const builtin_interfaces::msg::Time &) {
    return std::optional<Transform>(Transform{});
  };

// A GnssPoser that has received the MGRS projector info.
GnssPoser make_poser_with_mgrs_projector(const GnssPoserParams & params)
{
  GnssPoser poser(params, identity_lookup);
  poser.set_projector_info(make_mgrs_projector_info());
  return poser;
}

// Feeds the fixes that only prepare the checked one.
void input_fixes(GnssPoser & poser, const std::vector<NavSatFix> & fixes)
{
  for (const auto & fix : fixes) {
    poser.input_fix(fix);
  }
}

// What the node publishes for a fix is what its result carries: every output, gnss_fixed only, or
// nothing at all.
void expect_every_output(const GnssPoser::Result & result)
{
  EXPECT_TRUE(result.gnss_fixed.has_value());
  EXPECT_TRUE(result.gnss_pose.has_value());
  EXPECT_TRUE(result.gnss_pose_cov.has_value());
  EXPECT_TRUE(result.transform.has_value());
}

void expect_only_gnss_fixed(const GnssPoser::Result & result)
{
  EXPECT_TRUE(result.gnss_fixed.has_value());
  EXPECT_FALSE(result.gnss_pose.has_value());
  EXPECT_FALSE(result.gnss_pose_cov.has_value());
  EXPECT_FALSE(result.transform.has_value());
}

void expect_no_output(const GnssPoser::Result & result)
{
  EXPECT_FALSE(result.gnss_fixed.has_value());
  EXPECT_FALSE(result.gnss_pose.has_value());
  EXPECT_FALSE(result.gnss_pose_cov.has_value());
  EXPECT_FALSE(result.transform.has_value());
}

// The published pose and its covariance; std::optional::value() throws, and so fails the case,
// when the result carries none.
const geometry_msgs::msg::Pose & pose_of(const GnssPoser::Result & result)
{
  return result.gnss_pose.value().pose;
}

const std::array<double, 36> & covariance_of(const GnssPoser::Result & result)
{
  return result.gnss_pose_cov.value().pose.covariance;
}

// A fix status other than STATUS_FIX that still counts as fixed.
struct FixedStatus
{
  const char * name;
  NavSatStatus::_status_type status;
};

class GnssPoserFixedStatus : public ::testing::TestWithParam<FixedStatus>
{
};

// A cardinal direction for the motion heading: the offset of the second fix from the reference.
struct Heading
{
  const char * name;
  double latitude_offset;
  double longitude_offset;
};

class GnssPoserMotionHeading : public ::testing::TestWithParam<Heading>
{
};

struct CovarianceType
{
  const char * name;
  NavSatFix::_position_covariance_type_type type;
};

class GnssPoserKnownCovarianceType : public ::testing::TestWithParam<CovarianceType>
{
};
}  // namespace

// =======================================================================================
// 1. Gating
// =======================================================================================

// A LOCAL projector switches processing off, and the latest projector info always wins: MGRS after
// LOCAL switches it back on, LOCAL after MGRS off again, and neither arrival publishes anything by
// itself.
//
// NOTE(characterization): unlike the not-fixed case below, a fix dropped under LOCAL does not
// publish `gnss_fixed` either, so a consumer of that topic sees no update at all.
TEST(GnssPoser, Gate_LocalProjector_PublishesNothingNotEvenGnssFixed)
{
  // Arrange
  GnssPoser poser = make_poser_with_mgrs_projector({});
  poser.input_fix(make_reference_fix());
  // The latest projector info wins: LOCAL disables processing. Its arrival publishes nothing,
  // and the next fix is dropped entirely.
  poser.set_projector_info(make_local_projector_info());

  // Act
  const auto result = poser.input_fix(make_reference_fix());

  // Assert
  expect_no_output(result);
}

TEST(GnssPoser, Gate_MgrsProjectorAfterLocal_ReenablesProcessing)
{
  // Arrange
  GnssPoser poser = make_poser_with_mgrs_projector({});
  poser.input_fix(make_reference_fix());
  // The latest projector info wins: LOCAL disables processing. Its arrival publishes nothing,
  // and the next fix is dropped entirely.
  poser.set_projector_info(make_local_projector_info());
  poser.input_fix(make_reference_fix());
  // Switching back re-enables processing, again without replaying the dropped fix.
  poser.set_projector_info(make_mgrs_projector_info());

  // Act
  const auto result = poser.input_fix(make_reference_fix());

  // Assert
  expect_every_output(result);
}

// `is_fixed` is `status >= STATUS_FIX`: FIX, SBAS_FIX and GBAS_FIX all pass the gate and each
// produces one of every output, while NO_FIX adds a `gnss_fixed = false` and nothing else.
TEST_P(GnssPoserFixedStatus, Gate_SbasOrGbasFixStatus_Passes)
{
  // Arrange
  GnssPoser poser = make_poser_with_mgrs_projector({});

  // Act
  // is_fixed(status) <=> status >= STATUS_FIX (0)
  const auto result = poser.input_fix(make_reference_fix(GetParam().status));

  // Assert
  expect_every_output(result);
  EXPECT_TRUE(result.gnss_fixed.value().data);
}

INSTANTIATE_TEST_SUITE_P(
  AugmentedFixes, GnssPoserFixedStatus,
  ::testing::Values(
    FixedStatus{"SbasFix", NavSatStatus::STATUS_SBAS_FIX},
    FixedStatus{"GbasFix", NavSatStatus::STATUS_GBAS_FIX}),
  [](const ::testing::TestParamInfo<FixedStatus> & info) { return std::string(info.param.name); });

TEST(GnssPoser, Gate_StatusNoFixAfterAcceptedFixes_PublishesOnlyGnssFixedFalse)
{
  // Arrange
  GnssPoser poser = make_poser_with_mgrs_projector({});
  // is_fixed(status) <=> status >= STATUS_FIX (0)
  input_fixes(
    poser, {make_reference_fix(NavSatStatus::STATUS_FIX),
            make_reference_fix(NavSatStatus::STATUS_SBAS_FIX),
            make_reference_fix(NavSatStatus::STATUS_GBAS_FIX)});

  // Act
  const auto result = poser.input_fix(make_reference_fix(NavSatStatus::STATUS_NO_FIX));

  // Assert
  expect_only_gnss_fixed(result);
  EXPECT_FALSE(result.gnss_fixed.value().data);
}

// =======================================================================================
// 2. Position pipeline with gnss_pose_pub_method = 0 (instantaneous)
// =======================================================================================

// With method 0 the buffer is never used: whatever `buff_epoch` says, every fix yields its own
// instantaneous position.
TEST(GnssPoser, MethodInstant_LargeBuffEpoch_FirstFixPublishedInstantly)
{
  // Arrange
  GnssPoserParams params;
  params.gnss_pose_pub_method = GnssPosePubMethod::Instant;
  params.buff_epoch = 5;  // ignored for method 0
  GnssPoser poser = make_poser_with_mgrs_projector(params);
  const auto fixes = make_instant_fixes();

  // Act
  const auto result = poser.input_fix(fixes[0]);

  // Assert
  expect_every_output(result);
  expect_point_near(pose_of(result).position, project_mgrs(fixes[0]));
}

// The projector's `vertical_datum` is honored: with EGM2008 the WGS84 ellipsoidal altitude is
// converted to an orthometric height, i.e. lowered by the geoid height (about 36 m at the reference
// point), while x and y are the MGRS coordinates as before. The expectation is independent ground
// truth (the golden coordinates and GeoidEval's geoid height). Skipped when the geoid dataset is
// not installed.
TEST(GnssPoser, MethodInstant_Egm2008VerticalDatum_ConvertsHeightFromWgs84)
{
  if (!is_egm2008_dataset_available()) {
    GTEST_SKIP() << "egm2008-1 geoid dataset is not installed";
  }

  // Arrange
  GnssPoserParams params;
  params.gnss_pose_pub_method = GnssPosePubMethod::Instant;
  GnssPoser poser(params, identity_lookup);
  const auto projector = make_mgrs_projector_info(reference_mgrs_grid, MapProjectorInfo::EGM2008);
  poser.set_projector_info(projector);

  const auto fix = make_reference_fix();

  // Act
  const auto result = poser.input_fix(fix);

  // Assert
  expect_point_near(
    pose_of(result).position,
    make_point(golden_x, golden_y, reference_altitude - reference_geoid_height),
    geoid_height_tolerance);
}

// Projector info is forwarded to the projection library as-is: with LOCAL_CARTESIAN_UTM the map
// origin is honored, so a fix at the origin lands at (0, 0, altitude - origin altitude).
TEST(GnssPoser, MethodInstant_LocalCartesianUtmProjector_UsesMapOrigin)
{
  // Arrange
  GnssPoserParams params;
  params.gnss_pose_pub_method = GnssPosePubMethod::Instant;
  GnssPoser poser(params, identity_lookup);
  // Origin == reference point => antenna at (0, 0, alt - origin_alt).
  const auto projector =
    make_local_cartesian_utm_projector_info(reference_latitude, reference_longitude, -10.0);
  poser.set_projector_info(projector);

  const auto fix = make_reference_fix();

  // Act
  const auto result = poser.input_fix(fix);

  // Assert
  // Literal expectation: the fix sits on the origin, so only the altitude offset remains.
  expect_point_near(pose_of(result).position, make_point(0.0, 0.0, reference_altitude - (-10.0)));
}

// =======================================================================================
// 3. Position buffering (gnss_pose_pub_method = 1 average / 2 median)
// =======================================================================================

TEST(GnssPoser, MethodAverage_BufferBecomesFull_PublishesMean)
{
  // Arrange
  GnssPoser poser = make_poser_with_mgrs_projector(make_buffer_params(1, 3));
  const auto fixes = make_average_fixes();
  // Fixes 1 and 2 only fill the buffer: each publishes gnss_fixed, neither publishes a pose.
  input_fixes(poser, {fixes[0], fixes[1]});

  // Act
  // Fix 3 completes the buffer: the third gnss_fixed comes with the first pose, the mean of 1..3.
  const auto result = poser.input_fix(fixes[2]);

  // Assert
  expect_every_output(result);
  expect_point_near(
    pose_of(result).position,
    mean_of({project_mgrs(fixes[0]), project_mgrs(fixes[1]), project_mgrs(fixes[2])}));
}

TEST(GnssPoser, MethodAverage_BufferFull_SlidingWindowMean)
{
  // Arrange
  GnssPoser poser = make_poser_with_mgrs_projector(make_buffer_params(1, 3));
  const auto fixes = make_average_fixes();
  // Fixes 1 and 2 only fill the buffer: each publishes gnss_fixed, neither publishes a pose.
  // Fix 3 completes the buffer: the third gnss_fixed comes with the first pose, the mean of 1..3.
  input_fixes(poser, {fixes[0], fixes[1], fixes[2]});

  // Act
  // Fix 4 slides the window: the mean of 2..4, the oldest sample dropped.
  const auto result = poser.input_fix(fixes[3]);

  // Assert
  expect_every_output(result);
  expect_point_near(
    pose_of(result).position,
    mean_of({project_mgrs(fixes[1]), project_mgrs(fixes[2]), project_mgrs(fixes[3])}));
}

// With method 2 the median is taken per coordinate, so the result is in general not any of the
// input samples. The full-buffer gate and the sliding window apply as for the mean.
TEST(GnssPoser, MethodMedian_BufferNotFull_PublishesOnlyGnssFixed)
{
  // Arrange
  GnssPoser poser = make_poser_with_mgrs_projector(make_buffer_params(2, 3));
  const auto fixes = make_median_fixes();
  // Fixes 1 and 2 only fill the buffer: each publishes gnss_fixed, neither publishes a pose.
  input_fixes(poser, {fixes[0]});

  // Act
  const auto result = poser.input_fix(fixes[1]);

  // Assert
  expect_only_gnss_fixed(result);
}

TEST(GnssPoser, MethodMedian_OddBuffer_ComponentWiseMedian)
{
  // Arrange
  GnssPoser poser = make_poser_with_mgrs_projector(make_buffer_params(2, 3));
  const auto fixes = make_median_fixes();
  std::vector<Point> antenna;
  for (const auto & fix : fixes) {
    antenna.push_back(project_mgrs(fix));
  }
  const auto expected_123 = component_wise_median_of({antenna[0], antenna[1], antenna[2]});
  for (const auto & sample : antenna) {
    ASSERT_FALSE(
      std::abs(sample.x - expected_123.x) < position_tolerance &&
      std::abs(sample.y - expected_123.y) < position_tolerance &&
      std::abs(sample.z - expected_123.z) < position_tolerance)
      << "fixture data: the component-wise median must not coincide with a sample";
  }
  // Fixes 1 and 2 only fill the buffer: each publishes gnss_fixed, neither publishes a pose.
  input_fixes(poser, {fixes[0], fixes[1]});

  // Act
  // Fix 3 completes the buffer: the component-wise median of 1..3.
  const auto result = poser.input_fix(fixes[2]);

  // Assert
  expect_every_output(result);
  expect_point_near(pose_of(result).position, expected_123);
}

// An even buffer size has no single central value: the median of each coordinate is the average of
// the two central values, which the median computation handles as a separate case.
TEST(GnssPoser, MethodMedian_EvenBuffer_AveragesTwoCentralValues)
{
  // Arrange
  GnssPoser poser = make_poser_with_mgrs_projector(make_buffer_params(2, 4));
  const auto fixes = make_even_median_fixes();
  input_fixes(poser, {fixes[0], fixes[1], fixes[2]});

  // Act
  const auto result = poser.input_fix(fixes[3]);

  // Assert
  expect_point_near(
    pose_of(result).position, component_wise_median_of(
                                {project_mgrs(fixes[0]), project_mgrs(fixes[1]),
                                 project_mgrs(fixes[2]), project_mgrs(fixes[3])}));
}

// Non-fixed messages neither fill nor clear the position buffer: two accepted fixes around one
// NO_FIX still produce the mean of exactly those two.
TEST(GnssPoser, Buffer_IsNotAffectedByNonFixedMessages)
{
  // Arrange
  GnssPoser poser = make_poser_with_mgrs_projector(make_buffer_params(1, 2));
  const auto fixes = make_non_fixed_buffer_fixes();
  input_fixes(poser, {fixes[0], fixes[1]});

  // Act
  const auto result = poser.input_fix(fixes[2]);

  // Assert
  expect_point_near(
    pose_of(result).position, mean_of({project_mgrs(fixes[0]), project_mgrs(fixes[2])}));
}

// Fixes dropped at the projector gates (no projector info yet, or LOCAL) never reach the position
// buffer: after two dropped fixes, the first accepted one still fills only one of the two slots.
// Pins the order "gates first, then buffer", which a refactoring could plausibly swap.
TEST(GnssPoser, Buffer_FirstFixAfterGatedFixes_FillsOnlyOneSlot)
{
  // Arrange
  const auto fixes = make_gated_buffer_fixes();
  GnssPoser poser(make_buffer_params(1, 2), identity_lookup);
  poser.input_fix(fixes[0]);  // no projector info yet
  poser.set_projector_info(make_local_projector_info());
  poser.input_fix(fixes[1]);  // LOCAL projector
  poser.set_projector_info(make_mgrs_projector_info());

  // Act
  const auto result = poser.input_fix(fixes[2]);

  // Assert
  expect_only_gnss_fixed(result);  // one slot filled, not full
}

// =======================================================================================
// 4. Orientation
// =======================================================================================

// With use_gnss_ins_orientation = true the pose carries the orientation of the latest
// `autoware_orientation` message, and the orientation covariance diagonal is its RMSE squared
// (float32 precision). A newer message replaces the previous one; its header is irrelevant.
TEST(GnssPoser, InsOrientation_NewerMessage_ReplacesPrevious)
{
  // Arrange
  GnssPoser poser = make_poser_with_mgrs_projector({});
  poser.set_ins_orientation(make_orientation(M_PI / 2.0, 0.1, 0.2, 0.3));
  input_fixes(poser, {make_reference_fix()});
  // A newer orientation message replaces the previous one; its header is irrelevant.
  poser.set_ins_orientation(make_orientation(-M_PI / 4.0, 0.5, 0.6, 0.7));

  // Act
  const auto result = poser.input_fix(make_reference_fix());

  // Assert
  EXPECT_NEAR(yaw_of(pose_of(result).orientation), -M_PI / 4.0, angle_tolerance);
  // 0.5^2 = 0.25, 0.6^2 = 0.36, 0.7^2 = 0.49
  expect_rotation_variances_near(covariance_of(result), 0.25, 0.36, 0.49);
}

// With use_gnss_ins_orientation = true the node does not wait for an orientation message: before
// the first one it publishes the message-default orientation (identity, i.e. heading east in `map`)
// with the constructor's placeholder RMSE of 1.0 squared, and the antenna position unchanged.
//
// NOTE(characterization): a consumer cannot tell this placeholder from a measured orientation whose
// RMSE happens to be 1.0, and the identity heading also enters the antenna -> base_link lever-arm
// correction. Whether to gate on the first orientation (as on projector info) is a decision for the
// refactoring phase, not for this test.
TEST(GnssPoser, InsOrientation_NoMessageYet_IdentityAndUnitCovariance)
{
  // Arrange
  GnssPoserParams params;
  params.gnss_pose_pub_method = GnssPosePubMethod::Instant;
  params.use_gnss_ins_orientation = true;
  GnssPoser poser(params, identity_lookup);
  const auto projector = make_mgrs_projector_info();
  poser.set_projector_info(projector);

  const auto fix = make_reference_fix();
  const auto expected_antenna = project_antenna(fix, projector);

  // Act
  const auto result = poser.input_fix(fix);

  // Assert
  EXPECT_TRUE(result.gnss_fixed.value().data);
  expect_point_near(pose_of(result).position, expected_antenna);
  expect_same_rotation(pose_of(result).orientation, yaw_to_quaternion(0.0));
  // 1.0 comes from the constructor's placeholder rmse and is squared: 1.0^2 = 1.0.
  EXPECT_DOUBLE_EQ(covariance_of(result)[cov_roll_roll], 1.0);
  EXPECT_DOUBLE_EQ(covariance_of(result)[cov_pitch_pitch], 1.0);
  EXPECT_DOUBLE_EQ(covariance_of(result)[cov_yaw_yaw], 1.0);
  // Position covariance is unaffected.
  EXPECT_DOUBLE_EQ(covariance_of(result)[cov_xx], 1.0);
}

// With use_gnss_ins_orientation = false the heading comes from motion, and only from the last two
// accepted positions: the first fix has no previous position and gets the identity; every later fix
// gets yaw = atan2 of its displacement from the previous fix (roll = pitch = 0), whatever the
// heading before was. The orientation covariance is the constant 0.1 / 0.1 / 1.0 regardless of the
// displacement, and INS messages are ignored in this mode.
//
// NOTE(characterization): standing still gives a zero displacement and atan2(0, 0) = 0, so the
// heading snaps back to east instead of keeping the last one.
TEST(GnssPoser, MotionOrientation_FirstFix_IdentityAndConstantCovariance)
{
  // Arrange
  GnssPoserParams params;
  params.use_gnss_ins_orientation = false;
  GnssPoser poser = make_poser_with_mgrs_projector(params);
  // INS orientation messages are ignored in this mode.
  poser.set_ins_orientation(make_orientation(M_PI / 2.0, 0.1, 0.2, 0.3));
  const auto fixes = make_motion_fixes();

  // Act
  // First fix: no previous position => yaw = atan2(0, 0) = 0 (identity).
  const auto result = poser.input_fix(fixes[0]);

  // Assert
  expect_same_rotation(pose_of(result).orientation, yaw_to_quaternion(0.0));
  // use_gnss_ins_orientation = false always writes the constants 0.1 / 0.1 / 1.0.
  EXPECT_DOUBLE_EQ(covariance_of(result)[cov_roll_roll], 0.1);
  EXPECT_DOUBLE_EQ(covariance_of(result)[cov_pitch_pitch], 0.1);
  EXPECT_DOUBLE_EQ(covariance_of(result)[cov_yaw_yaw], 1.0);
}

TEST(GnssPoser, MotionOrientation_ThirdFix_YawFromLatestStepOnly)
{
  // Arrange
  GnssPoserParams params;
  params.use_gnss_ins_orientation = false;
  GnssPoser poser = make_poser_with_mgrs_projector(params);
  // INS orientation messages are ignored in this mode.
  poser.set_ins_orientation(make_orientation(M_PI / 2.0, 0.1, 0.2, 0.3));
  const auto fixes = make_motion_fixes();
  const auto p1 = project_mgrs(fixes[0]);
  const auto p2 = project_mgrs(fixes[1]);
  const auto p3 = project_mgrs(fixes[2]);
  const double yaw_step = std::atan2(p3.y - p2.y, p3.x - p2.x);
  const double yaw_from_start = std::atan2(p3.y - p1.y, p3.x - p1.x);
  ASSERT_GT(std::abs(yaw_step - yaw_from_start), 0.5) << "fixture data must tell the two apart";
  // First fix: no previous position => yaw = atan2(0, 0) = 0 (identity).
  // Second fix: yaw of the displacement in the map frame (roll = pitch = 0).
  input_fixes(poser, {fixes[0], fixes[1]});

  // Act
  // Third fix: the heading follows the latest displacement only. It is atan2(p3 - p2), the heading
  // of the step just made, and not atan2(p3 - p1) or anything carried over from the previous
  // heading.
  const auto result = poser.input_fix(fixes[2]);

  // Assert
  EXPECT_NEAR(yaw_of(pose_of(result).orientation), yaw_step, angle_tolerance);
}

TEST(GnssPoser, MotionOrientation_StandingStill_YawSnapsToZero)
{
  // Arrange
  GnssPoserParams params;
  params.use_gnss_ins_orientation = false;
  GnssPoser poser = make_poser_with_mgrs_projector(params);
  // INS orientation messages are ignored in this mode.
  poser.set_ins_orientation(make_orientation(M_PI / 2.0, 0.1, 0.2, 0.3));
  const auto fixes = make_motion_fixes();
  // First fix: no previous position => yaw = atan2(0, 0) = 0 (identity).
  // Second fix: yaw of the displacement in the map frame (roll = pitch = 0).
  // Third fix: the heading follows the latest displacement only. It is atan2(p3 - p2), the heading
  // of the step just made, and not atan2(p3 - p1) or anything carried over from the previous
  // heading.
  input_fixes(poser, {fixes[0], fixes[1], fixes[2]});

  // Act
  // Standing still: displacement is zero => yaw snaps back to 0, not "keep last heading".
  const auto result = poser.input_fix(fixes[2]);

  // Assert
  expect_same_rotation(pose_of(result).orientation, yaw_to_quaternion(0.0));
}

// The previous position used for the heading is not reset by a NO_FIX message in between: the
// heading is still the displacement between the two accepted fixes.
TEST(GnssPoser, MotionOrientation_PreviousPositionSurvivesNonFixedMessages)
{
  // Arrange
  GnssPoserParams params;
  params.gnss_pose_pub_method = GnssPosePubMethod::Instant;
  params.use_gnss_ins_orientation = false;
  GnssPoser poser(params, identity_lookup);
  const auto projector = make_mgrs_projector_info();
  poser.set_projector_info(projector);

  const auto fix1 = make_fix(reference_latitude, reference_longitude, reference_altitude);
  const auto no_fix = make_fix(
    reference_latitude - 0.01, reference_longitude - 0.01, reference_altitude,
    NavSatStatus::STATUS_NO_FIX);
  const auto fix2 = make_fix(reference_latitude + 0.001, reference_longitude, reference_altitude);
  const auto p1 = project_antenna(fix1, projector);
  const auto p2 = project_antenna(fix2, projector);
  const double expected_yaw = std::atan2(p2.y - p1.y, p2.x - p1.x);
  poser.input_fix(fix1);
  poser.input_fix(no_fix);

  // Act
  const auto result = poser.input_fix(fix2);

  // Assert
  EXPECT_NEAR(yaw_of(pose_of(result).orientation), expected_yaw, angle_tolerance);
}

// When a buffer is in use, the heading is derived from consecutive *averaged* positions, not from
// the raw antenna positions.
TEST(GnssPoser, MotionOrientation_WithBuffer_UsesFilteredPositions)
{
  // Arrange
  GnssPoserParams params = make_buffer_params(1, 2);
  params.use_gnss_ins_orientation = false;
  GnssPoser poser = make_poser_with_mgrs_projector(params);
  const auto fixes = make_buffered_motion_fixes();
  const auto m12 = mean_of({project_mgrs(fixes[0]), project_mgrs(fixes[1])});
  const auto m23 = mean_of({project_mgrs(fixes[1]), project_mgrs(fixes[2])});
  input_fixes(poser, {fixes[0], fixes[1]});

  // Act
  const auto result = poser.input_fix(fixes[2]);

  // Assert
  // Heading is derived from consecutive *averaged* positions, not raw antenna positions.
  EXPECT_NEAR(
    yaw_of(pose_of(result).orientation), std::atan2(m23.y - m12.y, m23.x - m12.x), angle_tolerance);
}

// The motion heading in the four cardinal directions, including the wrap-around at +-pi when
// moving west: yaw = atan2 of the displacement between the two projected positions.
TEST_P(GnssPoserMotionHeading, MotionOrientation_CardinalDirection_YawFromDisplacement)
{
  // Arrange
  GnssPoserParams params;
  params.use_gnss_ins_orientation = false;
  GnssPoser poser = make_poser_with_mgrs_projector(params);
  const auto from = make_reference_fix();
  const auto to = make_fix(
    reference_latitude + GetParam().latitude_offset,
    reference_longitude + GetParam().longitude_offset, reference_altitude);
  const auto p1 = project_mgrs(from);
  const auto p2 = project_mgrs(to);
  const double expected_yaw = std::atan2(p2.y - p1.y, p2.x - p1.x);
  input_fixes(poser, {from});

  // Act
  const auto result = poser.input_fix(to);

  // Assert
  expect_same_rotation(pose_of(result).orientation, yaw_to_quaternion(expected_yaw));
}

INSTANTIATE_TEST_SUITE_P(
  CardinalDirections, GnssPoserMotionHeading,
  ::testing::Values(
    Heading{"East", 0.0, 0.001}, Heading{"North", 0.001, 0.0}, Heading{"West", 0.0, -0.001},
    Heading{"South", -0.001, 0.0}),
  [](const ::testing::TestParamInfo<Heading> & info) { return std::string(info.param.name); });

// =======================================================================================
// 5. Covariance
// =======================================================================================

// The position covariance diagonal is copied from the fix for APPROXIMATED, DIAGONAL_KNOWN and
// KNOWN, and replaced by 10.0 for UNKNOWN; off-diagonal input entries are never propagated and
// every off-diagonal output entry stays 0. The orientation covariance is the INS RMSE squared.
TEST_P(GnssPoserKnownCovarianceType, Covariance_KnownType_CopiesDiagonalFromFix)
{
  // Arrange
  GnssPoser poser = make_poser_with_mgrs_projector({});
  poser.set_ins_orientation(make_orientation(0.0, 0.1, 0.2, 0.3));

  // Act
  const auto result = poser.input_fix(make_fix_with_covariance(GetParam().type));

  // Assert
  expect_position_variances(covariance_of(result), 2.0, 3.0, 4.0);
}

INSTANTIATE_TEST_SUITE_P(
  EveryKnownType, GnssPoserKnownCovarianceType,
  ::testing::Values(
    CovarianceType{"Approximated", NavSatFix::COVARIANCE_TYPE_APPROXIMATED},
    CovarianceType{"DiagonalKnown", NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN},
    CovarianceType{"Known", NavSatFix::COVARIANCE_TYPE_KNOWN}),
  [](const ::testing::TestParamInfo<CovarianceType> & info) {
    return std::string(info.param.name);
  });

TEST(GnssPoser, Covariance_UnknownType_PositionVariancesAreTen)
{
  // Arrange
  GnssPoser poser = make_poser_with_mgrs_projector({});
  poser.set_ins_orientation(make_orientation(0.0, 0.1, 0.2, 0.3));

  // Act
  const auto result = poser.input_fix(make_fix_with_covariance(NavSatFix::COVARIANCE_TYPE_UNKNOWN));

  // Assert
  {
    const auto & cov = covariance_of(result);
    // 10.0 is hard-coded in the node:
    // https://github.com/autowarefoundation/autoware_core/blob/904b5901488f24e13ae5c3add950b2b9985e1407/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp#L192-L197
    expect_position_variances(cov, 10.0, 10.0, 10.0);
    expect_rotation_variances_near(cov, 0.01, 0.04, 0.09);
    // Everything off the diagonal is zero.
    for (std::size_t i = 0; i < cov.size(); ++i) {
      if (i % 7 != 0) {
        EXPECT_DOUBLE_EQ(cov[i], 0.0) << "index " << i;
      }
    }
  }
}
