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

#include "autoware/component_interface_utils/rclcpp.hpp"
#include "autoware/component_interface_utils/testing/manifest_drift.hpp"
#include "gtest/gtest-spi.h"
#include "gtest/gtest.h"

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/response_status.hpp>

#include <memory>

namespace ciu = autoware::component_interface_utils;

namespace test_specs
{
// A local, unversioned spec so the fragment does not need major/minor/patch
// keys. The message type only needs to exist, so reuse a dependency this
// package already declares (autoware_adapi_v1_msgs).
struct DriftPub
{
  using Message = autoware_adapi_v1_msgs::msg::ResponseStatus;
  static constexpr char name[] = "/test/drift/pub";
  static constexpr std::size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};
}  // namespace test_specs

namespace
{
// A node with one registered publisher, kept alive across the assertion --
// see manifest_drift.hpp for why the adaptor cannot outlive its node.
struct DriftFixture
{
  DriftFixture() : node(std::make_shared<rclcpp::Node>("test_drift")), adaptor(node.get())
  {
    pub = adaptor.create_publisher<test_specs::DriftPub>();
  }

  std::shared_ptr<rclcpp::Node> node;
  ciu::NodeAdaptor<rclcpp::Node> adaptor;
  ciu::Publisher<test_specs::DriftPub>::SharedPtr pub;
};
}  // namespace

TEST(manifest_drift, matches_the_committed_fragment)
{
  rclcpp::init(0, nullptr);
  DriftFixture fixture;
  ciu::testing::expect_manifest_matches(
    fixture.adaptor, fixture.node->get_fully_qualified_name(), MATCHING_FRAGMENT);
  rclcpp::shutdown();
}

// EXPECT_NONFATAL_FAILURE (gtest/gtest-spi.h) is used rather than EXPECT_THROW
// or a raw EXPECT_FALSE because expect_manifest_matches does not throw on a
// mismatch -- it reports through EXPECT_EQ, exactly like the production
// helper is meant to when a package's committed fragment drifts from what its
// node actually registers -- so the only way to prove that behavior is to
// capture the gtest failure it produces. EXPECT_NONFATAL_FAILURE (unlike
// EXPECT_FATAL_FAILURE) explicitly allows the wrapped statement to reference
// local variables, which this call needs (fixture.adaptor, the node name, the
// fragment path); it also requires the statement to raise exactly one
// non-fatal failure in the current thread, which holds here since the
// fragment is well-formed and readable, so only the trailing EXPECT_EQ in
// expect_manifest_matches fails, never the ASSERT_TRUE before it. The
// required substring "fragment" comes from the message
// expect_manifest_matches streams onto that EXPECT_EQ.
TEST(manifest_drift, rejects_a_divergent_fragment)
{
  rclcpp::init(0, nullptr);
  DriftFixture fixture;
  EXPECT_NONFATAL_FAILURE(
    ciu::testing::expect_manifest_matches(
      fixture.adaptor, fixture.node->get_fully_qualified_name(), DIVERGENT_FRAGMENT),
    "fragment");
  rclcpp::shutdown();
}
