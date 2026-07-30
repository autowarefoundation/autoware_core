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
#include "autoware/component_interface_utils/rclcpp/registration.hpp"
#include "gtest/gtest.h"

#include <autoware/component_interface_specs/localization.hpp>
#include <autoware/component_interface_specs/map.hpp>
#include <autoware/component_interface_specs/system.hpp>
#include <autoware/component_interface_specs/version.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/response_status.hpp>

#include <memory>
#include <string>

namespace ciu = autoware::component_interface_utils;

namespace test_specs
{
// A local, unversioned spec: no resolve_domain_version overload anywhere.
// The message type only needs to exist; ResponseStatus comes from a dependency
// this package already declares (autoware_adapi_v1_msgs).
struct Unversioned
{
  using Message = autoware_adapi_v1_msgs::msg::ResponseStatus;
  static constexpr char name[] = "/test/unversioned";
  static constexpr std::size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

// A spec whose domain explicitly opted out via a deleted overload
// (the map::PointCloudMap pattern): the trait must also be false.
struct Excluded
{
  using Message = autoware_adapi_v1_msgs::msg::ResponseStatus;
  static constexpr char name[] = "/test/excluded";
  static constexpr std::size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};
constexpr autoware::component_interface_specs::Version resolve_domain_version(const Excluded &) =
  delete;

// Two distinct specs sharing one Message type, used to pin relay_message's
// per-side spec deduction: a bug that reused the publisher's spec for the
// subscription side too (or vice versa) would register the wrong
// interface_name on one of the two records.
struct RelayPub
{
  using Message = autoware_adapi_v1_msgs::msg::ResponseStatus;
  static constexpr char name[] = "/test/relay/pub";
  static constexpr std::size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};
struct RelaySub
{
  using Message = autoware_adapi_v1_msgs::msg::ResponseStatus;
  static constexpr char name[] = "/test/relay/sub";
  static constexpr std::size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

// Two distinct specs sharing one Service type, for the same reason applied
// to relay_service's client/server pair.
struct RelayCli
{
  using Service = autoware_system_msgs::srv::ChangeOperationMode;
  static constexpr char name[] = "/test/relay/cli";
};
struct RelaySrv
{
  using Service = autoware_system_msgs::srv::ChangeOperationMode;
  static constexpr char name[] = "/test/relay/srv";
};

// A local, versioned spec whose domain version has three distinct, non-zero
// components. Asserting against {2, 3, 4} can never coincide with
// InterfaceRecord's own {0, 0, 0} defaults, unlike a real domain (all of which
// are currently 0.x.y).
struct Versioned
{
  using Message = autoware_adapi_v1_msgs::msg::ResponseStatus;
  static constexpr char name[] = "/test/versioned";
  static constexpr std::size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};
constexpr autoware::component_interface_specs::Version resolve_domain_version(const Versioned &)
{
  return {2, 3, 4};
}
}  // namespace test_specs

static_assert(
  ciu::HasDomainVersion<autoware::component_interface_specs::localization::KinematicState>::value);
static_assert(!ciu::HasDomainVersion<test_specs::Unversioned>::value);
static_assert(!ciu::HasDomainVersion<test_specs::Excluded>::value);
// PointCloudMap's domain declares the DEFINE_DOMAIN template overload for every spec
// in its Specs tuple *and* a deleted non-template overload for this one spec; the
// trait must still resolve to false, exercising the non-template-beats-template
// overload-resolution tiebreak rather than the single-deleted-candidate case above.
static_assert(
  !ciu::HasDomainVersion<autoware::component_interface_specs::map::PointCloudMap>::value);

TEST(registration, registry_accumulates_and_returns_records)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_registry");
  ciu::NodeInterface interface(node.get());

  ciu::InterfaceRecord first_record;
  first_record.kind = ciu::InterfaceRecord::Kind::Topic;
  first_record.role = ciu::InterfaceRecord::Role::Provide;
  first_record.interface_name = "/test/topic";
  first_record.resolved_name = "/test/topic";
  interface.register_interface(first_record);

  ciu::InterfaceRecord second_record;
  second_record.kind = ciu::InterfaceRecord::Kind::Service;
  second_record.role = ciu::InterfaceRecord::Role::Require;
  second_record.interface_name = "/test/service";
  second_record.resolved_name = "/test/service";
  interface.register_interface(second_record);

  const auto manifest = interface.manifest();
  ASSERT_EQ(manifest.size(), 2u);
  EXPECT_EQ(manifest[0].interface_name, "/test/topic");
  EXPECT_EQ(manifest[0].role, ciu::InterfaceRecord::Role::Provide);
  EXPECT_EQ(manifest[1].interface_name, "/test/service");
  EXPECT_EQ(manifest[1].role, ciu::InterfaceRecord::Role::Require);
  rclcpp::shutdown();
}

TEST(registration, make_record_resolves_the_domain_version_only_where_declared)
{
  using KinematicState = autoware::component_interface_specs::localization::KinematicState;
  const auto versioned = ciu::make_record<KinematicState>(
    ciu::InterfaceRecord::Kind::Topic, ciu::InterfaceRecord::Role::Provide, KinematicState::name,
    "nav_msgs/msg/Odometry", rmw_qos_profile_default);
  EXPECT_TRUE(versioned.has_version);
  const auto v = resolve_domain_version(KinematicState{});
  EXPECT_EQ(versioned.minor, v.minor);

  // The fields make_record forwards rather than resolves from the domain
  // version: the caller-supplied name/type and the QoS actually applied
  // (rmw_qos_profile_default is depth 10 / RELIABLE / VOLATILE, which differs
  // from InterfaceRecord's own defaults of 0 / SYSTEM_DEFAULT / SYSTEM_DEFAULT).
  EXPECT_EQ(versioned.resolved_name, KinematicState::name);
  EXPECT_EQ(versioned.type_name, "nav_msgs/msg/Odometry");
  EXPECT_EQ(versioned.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(versioned.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
  EXPECT_EQ(versioned.depth, 10u);

  // A local spec whose domain version has three distinct, non-zero components
  // (so major/minor/patch cannot coincide with InterfaceRecord's defaults),
  // called with Kind::Service/Role::Require (both differing from
  // InterfaceRecord's Kind::Topic/Role::Provide defaults) and a resolved_name
  // that differs from the spec-declared name (as under a remap). This pins
  // interface_name and resolved_name apart: a bug that copied one field into
  // the other would fail one of the two assertions below.
  const auto remapped = ciu::make_record<test_specs::Versioned>(
    ciu::InterfaceRecord::Kind::Service, ciu::InterfaceRecord::Role::Require, "/remapped/odom",
    "nav_msgs/msg/Odometry", rmw_qos_profile_default);
  EXPECT_TRUE(remapped.has_version);
  EXPECT_EQ(remapped.major, 2);
  EXPECT_EQ(remapped.minor, 3);
  EXPECT_EQ(remapped.patch, 4);
  EXPECT_EQ(remapped.interface_name, test_specs::Versioned::name);
  EXPECT_EQ(remapped.resolved_name, "/remapped/odom");
  EXPECT_EQ(remapped.kind, ciu::InterfaceRecord::Kind::Service);
  EXPECT_EQ(remapped.role, ciu::InterfaceRecord::Role::Require);

  const auto plain = ciu::make_record<test_specs::Unversioned>(
    ciu::InterfaceRecord::Kind::Topic, ciu::InterfaceRecord::Role::Provide,
    test_specs::Unversioned::name, "std_msgs/msg/Empty", rmw_qos_profile_default);
  EXPECT_FALSE(plain.has_version);
}

TEST(registration, create_publisher_registers_with_remap_resolved_name)
{
  using OperationModeState = autoware::component_interface_specs::system::OperationModeState;
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "-r", std::string(OperationModeState::name) + ":=/renamed/mode"});
  auto node = std::make_shared<rclcpp::Node>("test_remap", opts);
  ciu::NodeAdaptor adaptor(node.get());

  auto pub = adaptor.create_publisher<OperationModeState>();
  const auto manifest = adaptor.manifest();
  ASSERT_EQ(manifest.size(), 1u);
  EXPECT_EQ(manifest[0].interface_name, OperationModeState::name);
  EXPECT_EQ(manifest[0].resolved_name, "/renamed/mode");
  EXPECT_EQ(manifest[0].role, ciu::InterfaceRecord::Role::Provide);
  EXPECT_EQ(manifest[0].durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  // OperationModeState declares depth 1, distinct from both
  // rmw_qos_profile_default's 10 and InterfaceRecord's own default of 0 --
  // pins actual QoS, not the spec-declared value, ending up in the record.
  EXPECT_EQ(manifest[0].depth, 1u);
  EXPECT_TRUE(manifest[0].has_version);
  rclcpp::shutdown();
  (void)pub;
}

TEST(registration, every_entry_path_registers)
{
  using OperationModeState = autoware::component_interface_specs::system::OperationModeState;
  using ChangeOperationMode = autoware::component_interface_specs::system::ChangeOperationMode;
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_paths");
  ciu::NodeAdaptor adaptor(node.get());

  // Polling subscription (nullptr callback) -- Require.
  auto sub = adaptor.create_subscription<OperationModeState>(nullptr);
  // Callback-form subscription -- the dominant form in universe, and the
  // `if constexpr` branch the polling test above never instantiates. The
  // callback needs a concrete signature: a generic (auto-parameter) lambda
  // fails to compile because rclcpp::function_traits cannot deduce it.
  auto cb_sub = adaptor.create_subscription<OperationModeState>(
    [](const OperationModeState::Message::ConstSharedPtr) {});
  // Service server -- Provide; service client -- Require.
  auto srv = adaptor.create_service<ChangeOperationMode>([](auto, auto) {});
  auto cli = adaptor.create_client<ChangeOperationMode>();
  // Legacy init_* path (deprecated, must still register).
  ciu::Publisher<OperationModeState>::SharedPtr legacy_pub;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  adaptor.init_pub(legacy_pub);
#pragma GCC diagnostic pop

  const auto manifest = adaptor.manifest();
  ASSERT_EQ(manifest.size(), 5u);
  EXPECT_EQ(manifest[0].role, ciu::InterfaceRecord::Role::Require);  // sub
  EXPECT_EQ(manifest[0].interface_name, OperationModeState::name);
  EXPECT_EQ(manifest[1].role, ciu::InterfaceRecord::Role::Require);  // cb_sub
  EXPECT_EQ(manifest[1].interface_name, OperationModeState::name);
  EXPECT_EQ(manifest[2].kind, ciu::InterfaceRecord::Kind::Service);  // srv
  EXPECT_EQ(manifest[2].role, ciu::InterfaceRecord::Role::Provide);
  EXPECT_EQ(manifest[2].interface_name, ChangeOperationMode::name);
  EXPECT_EQ(manifest[3].role, ciu::InterfaceRecord::Role::Require);  // cli
  EXPECT_EQ(manifest[3].depth, 10u);                                 // shared service profile
  EXPECT_EQ(manifest[3].interface_name, ChangeOperationMode::name);
  EXPECT_EQ(manifest[4].role, ciu::InterfaceRecord::Role::Provide);  // init_pub
  EXPECT_EQ(manifest[4].interface_name, OperationModeState::name);
  rclcpp::shutdown();
  (void)sub;
  (void)cb_sub;
  (void)srv;
  (void)cli;
  (void)legacy_pub;
}

TEST(registration, relay_message_registers_both_sides_with_their_own_spec)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_relay_message");
  ciu::NodeAdaptor adaptor(node.get());

  ciu::Publisher<test_specs::RelayPub>::SharedPtr pub;
  ciu::Subscription<test_specs::RelaySub>::SharedPtr sub;
  adaptor.relay_message(pub, sub);

  const auto manifest = adaptor.manifest();
  ASSERT_EQ(manifest.size(), 2u);
  EXPECT_EQ(manifest[0].role, ciu::InterfaceRecord::Role::Provide);  // relayed publisher
  EXPECT_EQ(manifest[0].interface_name, test_specs::RelayPub::name);
  EXPECT_EQ(manifest[1].role, ciu::InterfaceRecord::Role::Require);  // relayed subscription
  EXPECT_EQ(manifest[1].interface_name, test_specs::RelaySub::name);
  rclcpp::shutdown();
  (void)pub;
  (void)sub;
}

TEST(registration, relay_service_registers_both_sides_with_their_own_spec)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_relay_service");
  ciu::NodeAdaptor adaptor(node.get());

  ciu::Client<test_specs::RelayCli>::SharedPtr cli;
  ciu::Service<test_specs::RelaySrv>::SharedPtr srv;
  adaptor.relay_service(cli, srv, nullptr);

  const auto manifest = adaptor.manifest();
  ASSERT_EQ(manifest.size(), 2u);
  EXPECT_EQ(manifest[0].role, ciu::InterfaceRecord::Role::Require);  // relayed client
  EXPECT_EQ(manifest[0].interface_name, test_specs::RelayCli::name);
  EXPECT_EQ(manifest[1].role, ciu::InterfaceRecord::Role::Provide);  // relayed service
  EXPECT_EQ(manifest[1].interface_name, test_specs::RelaySrv::name);
  rclcpp::shutdown();
  (void)cli;
  (void)srv;
}
