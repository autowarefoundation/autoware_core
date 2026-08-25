// Copyright 2026 The Autoware Contributors
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

#include "utils/localization_conversion.hpp"
#include "utils/route_conversion.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::default_adapi
{

namespace route = conversion;
namespace loc = localization_conversion;

// ---------------------------------------------------------------------------
// convert_state: exhaustive enum mapping (internal RouteState -> external)
// ---------------------------------------------------------------------------

namespace
{
route::InternalState make_internal_state(uint8_t state)
{
  route::InternalState internal;
  internal.state = state;
  internal.stamp.sec = 123;
  internal.stamp.nanosec = 456;
  return internal;
}
}  // namespace

TEST(ConvertState, AllDefinedEnumValues)
{
  using Internal = route::InternalState;
  using External = route::ExternalState;

  // Each row pins the non-obvious internal -> external collapse documented in
  // route_conversion.cpp convert_state().
  const std::vector<std::pair<uint8_t, uint16_t>> table = {
    {Internal::INITIALIZING, External::UNSET}, {Internal::UNSET, External::UNSET},
    {Internal::ROUTING, External::UNSET},      {Internal::SET, External::SET},
    {Internal::REROUTING, External::CHANGING}, {Internal::ARRIVED, External::ARRIVED},
    {Internal::ABORTED, External::SET},        {Internal::INTERRUPTED, External::SET},
  };

  for (const auto & [in_state, expected] : table) {
    const auto external = route::convert_state(make_internal_state(in_state));
    EXPECT_EQ(external.state, expected) << "internal state = " << static_cast<int>(in_state);
  }
}

TEST(ConvertState, UnknownInternalStateMapsToUnknown)
{
  // The internal UNKNOWN (0) and any out-of-range value hit the default branch.
  EXPECT_EQ(route::convert_state(make_internal_state(0)).state, route::ExternalState::UNKNOWN);
  EXPECT_EQ(route::convert_state(make_internal_state(200)).state, route::ExternalState::UNKNOWN);
}

TEST(ConvertState, StampIsCopiedThrough)
{
  const auto external = route::convert_state(make_internal_state(route::InternalState::SET));
  EXPECT_EQ(external.stamp.sec, 123);
  EXPECT_EQ(external.stamp.nanosec, 456u);
}

// ---------------------------------------------------------------------------
// convert_route / RouteSegment round-trips
// ---------------------------------------------------------------------------

TEST(ConvertRoute, PreferredPrimitiveIsExtractedFromAlternatives)
{
  route::InternalRoute internal;
  internal.header.frame_id = "map";
  internal.start_pose.position.x = 1.0;
  internal.goal_pose.position.x = 2.0;

  autoware_planning_msgs::msg::LaneletSegment segment;
  segment.preferred_primitive.id = 20;
  segment.preferred_primitive.primitive_type = "lane";

  autoware_planning_msgs::msg::LaneletPrimitive p10;
  p10.id = 10;
  p10.primitive_type = "lane";
  autoware_planning_msgs::msg::LaneletPrimitive p20;
  p20.id = 20;
  p20.primitive_type = "lane";
  autoware_planning_msgs::msg::LaneletPrimitive p30;
  p30.id = 30;
  p30.primitive_type = "lane";
  segment.primitives = {p10, p20, p30};
  internal.segments = {segment};

  const auto external = route::convert_route(internal);

  EXPECT_EQ(external.header.frame_id, "map");
  ASSERT_EQ(external.data.size(), 1u);
  EXPECT_EQ(external.data[0].start.position.x, 1.0);
  EXPECT_EQ(external.data[0].goal.position.x, 2.0);
  ASSERT_EQ(external.data[0].segments.size(), 1u);

  const auto & out_segment = external.data[0].segments[0];
  // The preferred id is pulled out of alternatives and the remaining two stay.
  EXPECT_EQ(out_segment.preferred.id, 20);
  ASSERT_EQ(out_segment.alternatives.size(), 2u);
  EXPECT_EQ(out_segment.alternatives[0].id, 10);
  EXPECT_EQ(out_segment.alternatives[1].id, 30);
}

TEST(ConvertRoute, MissingPreferredLeavesDefaultPreferredAndKeepsAllAlternatives)
{
  // Pins the untested error branch: when no alternative matches the preferred
  // id, api.preferred is left default-constructed (id == 0) and nothing is
  // erased from alternatives.
  route::InternalRoute internal;

  autoware_planning_msgs::msg::LaneletSegment segment;
  segment.preferred_primitive.id = 99;  // not present among primitives

  autoware_planning_msgs::msg::LaneletPrimitive p1;
  p1.id = 1;
  autoware_planning_msgs::msg::LaneletPrimitive p2;
  p2.id = 2;
  segment.primitives = {p1, p2};
  internal.segments = {segment};

  const auto external = route::convert_route(internal);

  ASSERT_EQ(external.data.size(), 1u);
  ASSERT_EQ(external.data[0].segments.size(), 1u);
  const auto & out_segment = external.data[0].segments[0];
  EXPECT_EQ(out_segment.preferred.id, 0);  // default-constructed
  ASSERT_EQ(out_segment.alternatives.size(), 2u);
  EXPECT_EQ(out_segment.alternatives[0].id, 1);
  EXPECT_EQ(out_segment.alternatives[1].id, 2);
}

TEST(ConvertRoute, EmptySegmentsProduceEmptyData)
{
  route::InternalRoute internal;
  internal.header.frame_id = "base_link";

  const auto external = route::convert_route(internal);
  EXPECT_EQ(external.header.frame_id, "base_link");
  ASSERT_EQ(external.data.size(), 1u);
  EXPECT_TRUE(external.data[0].segments.empty());
}

TEST(ConvertRoute, PrimitiveTypeAndIdAreMappedAcrossNamingDifference)
{
  // RoutePrimitive.type <-> LaneletPrimitive.primitive_type field rename.
  route::InternalRoute internal;
  autoware_planning_msgs::msg::LaneletSegment segment;
  segment.preferred_primitive.id = 5;
  segment.preferred_primitive.primitive_type = "crosswalk";
  autoware_planning_msgs::msg::LaneletPrimitive preferred;
  preferred.id = 5;
  preferred.primitive_type = "crosswalk";
  segment.primitives = {preferred};
  internal.segments = {segment};

  const auto external = route::convert_route(internal);
  const auto & out_segment = external.data[0].segments[0];
  EXPECT_EQ(out_segment.preferred.id, 5);
  EXPECT_EQ(out_segment.preferred.type, "crosswalk");
}

TEST(CreateEmptyRoute, OnlyStampIsSet)
{
  rclcpp::Time stamp(7, 8);
  const auto external = route::create_empty_route(stamp);
  EXPECT_EQ(rclcpp::Time(external.header.stamp).nanoseconds(), stamp.nanoseconds());
  EXPECT_TRUE(external.data.empty());
}

// ---------------------------------------------------------------------------
// route convert_request overloads
// ---------------------------------------------------------------------------

TEST(RouteConvertRequest, LaneletRequestMapsHeaderGoalSegmentsAndModification)
{
  auto external = std::make_shared<route::ExternalLaneletRequest::element_type>();
  external->header.frame_id = "map";
  external->goal.position.x = 11.0;
  external->option.allow_goal_modification = true;

  autoware_adapi_v1_msgs::msg::RouteSegment segment;
  segment.preferred.id = 42;
  segment.preferred.type = "lane";
  autoware_adapi_v1_msgs::msg::RoutePrimitive alt;
  alt.id = 43;
  alt.type = "lane";
  segment.alternatives = {alt};
  external->segments = {segment};

  const auto internal = route::convert_request(external);

  EXPECT_EQ(internal->header.frame_id, "map");
  EXPECT_EQ(internal->goal_pose.position.x, 11.0);
  EXPECT_TRUE(internal->allow_modification);
  ASSERT_EQ(internal->segments.size(), 1u);
  // LaneletSegment convert(RouteSegment): preferred first, then alternatives.
  EXPECT_EQ(internal->segments[0].preferred_primitive.id, 42);
  ASSERT_EQ(internal->segments[0].primitives.size(), 2u);
  EXPECT_EQ(internal->segments[0].primitives[0].id, 42);
  EXPECT_EQ(internal->segments[0].primitives[1].id, 43);
}

TEST(RouteConvertRequest, WaypointRequestMapsHeaderGoalWaypointsAndModification)
{
  auto external = std::make_shared<route::ExternalWaypointRequest::element_type>();
  external->header.frame_id = "map";
  external->goal.position.y = 5.0;
  external->option.allow_goal_modification = false;
  geometry_msgs::msg::Pose waypoint;
  waypoint.position.x = 3.0;
  external->waypoints = {waypoint};

  const auto internal = route::convert_request(external);

  EXPECT_EQ(internal->header.frame_id, "map");
  EXPECT_EQ(internal->goal_pose.position.y, 5.0);
  EXPECT_FALSE(internal->allow_modification);
  ASSERT_EQ(internal->waypoints.size(), 1u);
  EXPECT_EQ(internal->waypoints[0].position.x, 3.0);
}

TEST(RouteConvertRequest, ClearRequestProducesNonNullInternal)
{
  auto external = std::make_shared<route::ExternalClearRequest::element_type>();
  const auto internal = route::convert_request(external);
  EXPECT_NE(internal, nullptr);
}

// ---------------------------------------------------------------------------
// route convert_response
// ---------------------------------------------------------------------------

TEST(RouteConvertResponse, CopiesSuccessCodeAndMessage)
{
  route::InternalResponse internal;
  internal.success = true;
  internal.code = route::InternalResponse::SERVICE_TIMEOUT;
  internal.message = "done";

  const auto external = route::convert_response(internal);
  EXPECT_TRUE(external.success);
  EXPECT_EQ(external.code, route::InternalResponse::SERVICE_TIMEOUT);
  EXPECT_EQ(external.message, "done");
}

// ---------------------------------------------------------------------------
// localization convert_request / convert_response
// ---------------------------------------------------------------------------

TEST(LocalizationConvertRequest, SetsMethodAutoAndCopiesPose)
{
  auto external = std::make_shared<loc::ExternalInitializeRequest::element_type>();
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.pose.pose.position.x = 9.0;
  external->pose = {pose};

  const auto internal = loc::convert_request(external);

  EXPECT_EQ(
    internal->method, autoware_localization_msgs::srv::InitializeLocalization::Request::AUTO);
  ASSERT_EQ(internal->pose_with_covariance.size(), 1u);
  EXPECT_EQ(internal->pose_with_covariance[0].header.frame_id, "map");
  EXPECT_EQ(internal->pose_with_covariance[0].pose.pose.position.x, 9.0);
}

TEST(LocalizationConvertRequest, EmptyPoseIsPreservedWithMethodAuto)
{
  auto external = std::make_shared<loc::ExternalInitializeRequest::element_type>();
  const auto internal = loc::convert_request(external);
  EXPECT_EQ(
    internal->method, autoware_localization_msgs::srv::InitializeLocalization::Request::AUTO);
  EXPECT_TRUE(internal->pose_with_covariance.empty());
}

TEST(LocalizationConvertResponse, CopiesSuccessCodeAndMessage)
{
  loc::InternalResponse internal;
  internal.success = false;
  internal.code = loc::InternalResponse::PARAMETER_ERROR;
  internal.message = "bad parameter";

  const auto external = loc::convert_response(internal);
  EXPECT_FALSE(external.success);
  EXPECT_EQ(external.code, loc::InternalResponse::PARAMETER_ERROR);
  EXPECT_EQ(external.message, "bad parameter");
}

}  // namespace autoware::default_adapi

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
