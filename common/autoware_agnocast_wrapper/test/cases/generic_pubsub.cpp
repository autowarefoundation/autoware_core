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

// Exercises the generic (type-erased) publisher/subscription surface end to end: Method 1 (macro
// + free function, on a plain rclcpp::Node) round-trips a serialized message through the runtime
// backend actually selected by ENABLE_AGNOCAST. Every test below is skipped, not run, when
// ENABLE_AGNOCAST=1 at runtime without the agnocast heaphook loaded: constructing an Agnocast
// endpoint in that state exits the whole process instead of throwing, which would take the rest
// of the test binary down with it — the same hazard polling_subscriber.cpp and
// service_introspection.cpp guard against with the same agnocast_heaphook_loaded() check.

#include "autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp"
#include "autoware/agnocast_wrapper/node.hpp"
#include "heaphook_probe.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <std_msgs/msg/string.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

#ifdef USE_AGNOCAST_ENABLED
#include <agnocast/agnocast.hpp>
#endif

namespace
{

using autoware::agnocast_wrapper::test::agnocast_heaphook_loaded;
using std_msgs::msg::String;

// GenericSubscriptionCallback takes the message by shared_ptr<const SerializedMessage>, the
// non-deprecated form on both Humble and Jazzy (see the type's doc comment) — pin that here so a
// future edit that quietly reintroduces the deprecated non-const shared_ptr<SerializedMessage>
// form fails to compile this test rather than only showing up as a deprecation warning.
static_assert(
  std::is_same_v<
    autoware::agnocast_wrapper::GenericSubscriptionCallback,
    std::function<void(std::shared_ptr<const rclcpp::SerializedMessage>)>>,
  "GenericSubscriptionCallback should take shared_ptr<const SerializedMessage>");

constexpr auto discovery_timeout = std::chrono::seconds(10);
constexpr auto poll_interval = std::chrono::milliseconds(10);

/// Same guard as PollingSubscriberTest (polling_subscriber.cpp) and ServiceIntrospectionTest
/// (service_introspection.cpp): every generic pub/sub test suite below aliases this fixture.
class GenericPubSubTestBase : public testing::Test
{
protected:
  void SetUp() override
  {
    if (autoware::agnocast_wrapper::use_agnocast() && !agnocast_heaphook_loaded()) {
      GTEST_SKIP() << "ENABLE_AGNOCAST=1 without the agnocast heaphook: the agnocast backend "
                      "cannot be exercised in this environment.";
    }
  }
};

using GenericPubSubMethod1Test = GenericPubSubTestBase;
using GenericPubSubMethod2Test = GenericPubSubTestBase;
#ifdef USE_AGNOCAST_ENABLED
using GenericPublisherOptionsTest = GenericPubSubTestBase;
using GenericSubscriptionOptionsTest = GenericPubSubTestBase;
#endif

rclcpp::SerializedMessage serialize(const String & msg)
{
  rclcpp::Serialization<String> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&msg, &serialized);
  return serialized;
}

String deserialize(const rclcpp::SerializedMessage & serialized)
{
  rclcpp::Serialization<String> serializer;
  String msg;
  serializer.deserialize_message(&serialized, &msg);
  return msg;
}

/// A plain rclcpp::Node subclass (Method 1: base class stays rclcpp::Node), exercising the
/// AUTOWARE_CREATE_GENERIC_PUBLISHER*/AUTOWARE_CREATE_GENERIC_SUBSCRIPTION macros the way a
/// Method 1 node would, rather than calling
/// create_generic_publisher()/create_generic_subscription() directly.
class GenericPubSubMethod1Node : public rclcpp::Node
{
public:
  explicit GenericPubSubMethod1Node(const std::string & name) : rclcpp::Node(name) {}

  AUTOWARE_GENERIC_PUBLISHER_PTR create_string_publisher(const std::string & topic)
  {
    return AUTOWARE_CREATE_GENERIC_PUBLISHER3(topic, "std_msgs/msg/String", rclcpp::QoS(1));
  }

  AUTOWARE_GENERIC_SUBSCRIPTION_PTR create_string_subscription(
    const std::string & topic, autoware::agnocast_wrapper::GenericSubscriptionCallback callback)
  {
    return AUTOWARE_CREATE_GENERIC_SUBSCRIPTION(
      topic, "std_msgs/msg/String", rclcpp::QoS(1), std::move(callback),
      AUTOWARE_SUBSCRIPTION_OPTIONS{});
  }

  AUTOWARE_GENERIC_PUBLISHER_PTR create_string_publisher_with_options(
    const std::string & topic, const AUTOWARE_PUBLISHER_OPTIONS & options)
  {
    return AUTOWARE_CREATE_GENERIC_PUBLISHER4(
      topic, "std_msgs/msg/String", rclcpp::QoS(1), options);
  }

  AUTOWARE_GENERIC_SUBSCRIPTION_PTR create_string_subscription_with_options(
    const std::string & topic, autoware::agnocast_wrapper::GenericSubscriptionCallback callback,
    const AUTOWARE_SUBSCRIPTION_OPTIONS & options)
  {
    return AUTOWARE_CREATE_GENERIC_SUBSCRIPTION(
      topic, "std_msgs/msg/String", rclcpp::QoS(1), std::move(callback), options);
  }
};

TEST_F(GenericPubSubMethod1Test, MacroRoundTrip)
{
  auto pub_node = std::make_shared<GenericPubSubMethod1Node>("generic_pubsub_method1_pub");
  auto sub_node = std::make_shared<GenericPubSubMethod1Node>("generic_pubsub_method1_sub");

  const auto pub = pub_node->create_string_publisher("/test/generic_method1");

  std::atomic<bool> received{false};
  std::string received_data;
  const auto sub = sub_node->create_string_subscription(
    "/test/generic_method1", [&received, &received_data](auto serialized) {
      received_data = deserialize(*serialized).data;
      received = true;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sub_node);

  const auto discovery_deadline = std::chrono::steady_clock::now() + discovery_timeout;
  while (std::chrono::steady_clock::now() < discovery_deadline &&
         pub->get_subscription_count() + pub->get_intra_process_subscription_count() == 0) {
    std::this_thread::sleep_for(poll_interval);
  }
  ASSERT_GT(pub->get_subscription_count() + pub->get_intra_process_subscription_count(), 0U);

  String msg;
  msg.data = "method1-generic";
  pub->publish(serialize(msg));

  const auto spin_deadline = std::chrono::steady_clock::now() + discovery_timeout;
  while (!received.load() && std::chrono::steady_clock::now() < spin_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(poll_interval);
  }

  ASSERT_TRUE(received.load());
  EXPECT_EQ(received_data, msg.data);
}

// Unlike the typed AUTOWARE_CREATE_PUBLISHER2/3 macros, which just forward to `this`'s own native
// create_publisher() under ENABLE_AGNOCAST=0, AUTOWARE_CREATE_GENERIC_PUBLISHER4/
// AUTOWARE_CREATE_GENERIC_SUBSCRIPTION route through the wrapper free function in both builds
// specifically so this check applies here too, not just to the Node-member entry point covered by
// GenericPubSubMethod2Test.NodeMember*RejectsQosOverridingOptions below.
TEST_F(GenericPubSubMethod1Test, MacroPublisherRejectsQosOverridingOptions)
{
  auto node = std::make_shared<GenericPubSubMethod1Node>("generic_method1_publisher_qos_reject");

  AUTOWARE_PUBLISHER_OPTIONS options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions{{rclcpp::QosPolicyKind::Depth}};

  EXPECT_THROW(
    node->create_string_publisher_with_options("/test/generic_method1_qos_override", options),
    std::invalid_argument);
}

TEST_F(GenericPubSubMethod1Test, MacroSubscriptionRejectsQosOverridingOptions)
{
  auto node = std::make_shared<GenericPubSubMethod1Node>("generic_method1_subscription_qos_reject");

  AUTOWARE_SUBSCRIPTION_OPTIONS options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions{{rclcpp::QosPolicyKind::Depth}};

  EXPECT_THROW(
    node->create_string_subscription_with_options(
      "/test/generic_method1_sub_qos_override", [](auto) {}, options),
    std::invalid_argument);
}

// The _ON_NODE macros (for use outside the node class — helper classes, free functions, member
// objects holding only a node pointer) route through the same wrapper free function as the
// this-implicit macros above, so they need their own coverage rather than assuming the
// this-implicit tests exercise them too.
TEST_F(GenericPubSubMethod1Test, MacroOnNodeRoundTrip)
{
  auto pub_node = std::make_shared<rclcpp::Node>("generic_pubsub_method1_on_node_pub");
  auto sub_node = std::make_shared<rclcpp::Node>("generic_pubsub_method1_on_node_sub");

  const AUTOWARE_GENERIC_PUBLISHER_PTR pub = AUTOWARE_CREATE_GENERIC_PUBLISHER3_ON_NODE(
    pub_node.get(), "/test/generic_method1_on_node", "std_msgs/msg/String", rclcpp::QoS(1));

  std::atomic<bool> received{false};
  std::string received_data;
  const AUTOWARE_GENERIC_SUBSCRIPTION_PTR sub = AUTOWARE_CREATE_GENERIC_SUBSCRIPTION_ON_NODE(
    sub_node.get(), "/test/generic_method1_on_node", "std_msgs/msg/String", rclcpp::QoS(1),
    ([&received, &received_data](auto serialized) {
      received_data = deserialize(*serialized).data;
      received = true;
    }),
    AUTOWARE_SUBSCRIPTION_OPTIONS{});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sub_node);

  const auto discovery_deadline = std::chrono::steady_clock::now() + discovery_timeout;
  while (std::chrono::steady_clock::now() < discovery_deadline &&
         pub->get_subscription_count() + pub->get_intra_process_subscription_count() == 0) {
    std::this_thread::sleep_for(poll_interval);
  }
  ASSERT_GT(pub->get_subscription_count() + pub->get_intra_process_subscription_count(), 0U);

  String msg;
  msg.data = "method1-on-node-generic";
  pub->publish(serialize(msg));

  const auto spin_deadline = std::chrono::steady_clock::now() + discovery_timeout;
  while (!received.load() && std::chrono::steady_clock::now() < spin_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(poll_interval);
  }

  ASSERT_TRUE(received.load());
  EXPECT_EQ(received_data, msg.data);
}

TEST_F(GenericPubSubMethod1Test, MacroOnNodePublisherRejectsQosOverridingOptions)
{
  auto node = std::make_shared<rclcpp::Node>("generic_method1_on_node_publisher_qos_reject");

  AUTOWARE_PUBLISHER_OPTIONS options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions{{rclcpp::QosPolicyKind::Depth}};

  EXPECT_THROW(
    (AUTOWARE_CREATE_GENERIC_PUBLISHER4_ON_NODE(
      node.get(), "/test/generic_method1_on_node_qos_override", "std_msgs/msg/String",
      rclcpp::QoS(1), options)),
    std::invalid_argument);
}

TEST_F(GenericPubSubMethod1Test, MacroOnNodeSubscriptionRejectsQosOverridingOptions)
{
  auto node = std::make_shared<rclcpp::Node>("generic_method1_on_node_subscription_qos_reject");

  AUTOWARE_SUBSCRIPTION_OPTIONS options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions{{rclcpp::QosPolicyKind::Depth}};

  EXPECT_THROW(
    (AUTOWARE_CREATE_GENERIC_SUBSCRIPTION_ON_NODE(
      node.get(), "/test/generic_method1_on_node_sub_qos_override", "std_msgs/msg/String",
      rclcpp::QoS(1), [](auto) {}, options)),
    std::invalid_argument);
}

TEST_F(GenericPubSubMethod2Test, NodeMemberRoundTrip)
{
  using autoware::agnocast_wrapper::Node;

  auto pub_node = std::make_shared<Node>("generic_pubsub_method2_pub");
  auto sub_node = std::make_shared<Node>("generic_pubsub_method2_sub");

  const auto pub = pub_node->create_generic_publisher(
    "/test/generic_method2", "std_msgs/msg/String", rclcpp::QoS(1));

  std::atomic<bool> received{false};
  std::string received_data;
  const auto sub = sub_node->create_generic_subscription(
    "/test/generic_method2", "std_msgs/msg/String", rclcpp::QoS(1),
    [&received, &received_data](auto serialized) {
      received_data = deserialize(*serialized).data;
      received = true;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sub_node->get_rclcpp_node());

  const auto discovery_deadline = std::chrono::steady_clock::now() + discovery_timeout;
  while (std::chrono::steady_clock::now() < discovery_deadline &&
         pub->get_subscription_count() + pub->get_intra_process_subscription_count() == 0) {
    std::this_thread::sleep_for(poll_interval);
  }
  ASSERT_GT(pub->get_subscription_count() + pub->get_intra_process_subscription_count(), 0U);

  String msg;
  msg.data = "method2-generic";
  pub->publish(serialize(msg));

  const auto spin_deadline = std::chrono::steady_clock::now() + discovery_timeout;
  while (!received.load() && std::chrono::steady_clock::now() < spin_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(poll_interval);
  }

  ASSERT_TRUE(received.load());
  EXPECT_EQ(received_data, msg.data);
}

// The depth + options overload of create_generic_subscription() (as opposed to the depth-only
// overload, which forwards to it with default options) now exists in both builds, mirroring
// create_subscription<MessageT>()'s own depth + options overload.
TEST_F(GenericPubSubMethod2Test, NodeMemberDepthAndOptionsOverload)
{
  using autoware::agnocast_wrapper::Node;

  auto pub_node = std::make_shared<Node>("generic_pubsub_method2_depth_pub");
  auto sub_node = std::make_shared<Node>("generic_pubsub_method2_depth_sub");

  const auto pub = pub_node->create_generic_publisher(
    "/test/generic_method2_depth", "std_msgs/msg/String", rclcpp::QoS(1));

  std::atomic<bool> received{false};
  std::string received_data;
  const auto sub = sub_node->create_generic_subscription(
    "/test/generic_method2_depth", "std_msgs/msg/String", /*qos_history_depth=*/1,
    [&received, &received_data](auto serialized) {
      received_data = deserialize(*serialized).data;
      received = true;
    },
    AUTOWARE_SUBSCRIPTION_OPTIONS{});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sub_node->get_rclcpp_node());

  const auto discovery_deadline = std::chrono::steady_clock::now() + discovery_timeout;
  while (std::chrono::steady_clock::now() < discovery_deadline &&
         pub->get_subscription_count() + pub->get_intra_process_subscription_count() == 0) {
    std::this_thread::sleep_for(poll_interval);
  }
  ASSERT_GT(pub->get_subscription_count() + pub->get_intra_process_subscription_count(), 0U);

  String msg;
  msg.data = "method2-generic-depth-options";
  pub->publish(serialize(msg));

  const auto spin_deadline = std::chrono::steady_clock::now() + discovery_timeout;
  while (!received.load() && std::chrono::steady_clock::now() < spin_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(poll_interval);
  }

  ASSERT_TRUE(received.load());
  EXPECT_EQ(received_data, msg.data);
}

TEST_F(GenericPubSubMethod2Test, UnknownTopicTypeThrows)
{
  using autoware::agnocast_wrapper::Node;

  auto node = std::make_shared<Node>("generic_unknown_type_node");

  // Both backends load topic_type's typesupport library at construction (see the @throws docs on
  // GenericPublisher / Node::create_generic_publisher()); a nonexistent package name can't resolve
  // on either.
  EXPECT_THROW(
    node->create_generic_publisher(
      "/test/generic_unknown_type", "no_such_package/msg/NoSuchType", rclcpp::QoS(1)),
    std::runtime_error);
}

// Unlike the Method 1 free functions below (only declared under USE_AGNOCAST_ENABLED), the
// qos_overriding_options rejection on
// Node::create_generic_publisher()/create_generic_subscription() itself is unconditional:
// check_generic_publisher_qos_overriding_options()/
// check_generic_subscription_qos_overriding_options() are declared outside any #ifdef so this
// Method 2 entry point rejects it the same way in both builds — otherwise the same caller code
// would compile and silently ignore qos_overriding_options under ENABLE_AGNOCAST=0 while throwing
// under ENABLE_AGNOCAST=1. AUTOWARE_PUBLISHER_OPTIONS/AUTOWARE_SUBSCRIPTION_OPTIONS resolve to
// whichever options type each build's Node member actually takes.
TEST_F(GenericPubSubMethod2Test, NodeMemberPublisherRejectsQosOverridingOptions)
{
  using autoware::agnocast_wrapper::Node;

  auto node = std::make_shared<Node>("generic_method2_publisher_qos_reject");

  AUTOWARE_PUBLISHER_OPTIONS options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions{{rclcpp::QosPolicyKind::Depth}};

  EXPECT_THROW(
    node->create_generic_publisher(
      "/test/generic_method2_qos_override", "std_msgs/msg/String", rclcpp::QoS(1), options),
    std::invalid_argument);
}

TEST_F(GenericPubSubMethod2Test, NodeMemberSubscriptionRejectsQosOverridingOptions)
{
  using autoware::agnocast_wrapper::Node;

  auto node = std::make_shared<Node>("generic_method2_subscription_qos_reject");

  AUTOWARE_SUBSCRIPTION_OPTIONS options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions{{rclcpp::QosPolicyKind::Depth}};

  EXPECT_THROW(
    node->create_generic_subscription(
      "/test/generic_method2_sub_qos_override", "std_msgs/msg/String", rclcpp::QoS(1), [](auto) {},
      options),
    std::invalid_argument);
}

// create_generic_publisher()/AgnocastGenericPublisher/ROS2GenericPublisher only exist in the
// Agnocast-enabled build (generic_publisher.hpp is guarded by USE_AGNOCAST_ENABLED end to end),
// so the Method 1 free functions' qos_overriding_options rejection can only be exercised there —
// the Method 2 test above already covers ENABLE_AGNOCAST=0.
#ifdef USE_AGNOCAST_ENABLED

TEST_F(GenericPublisherOptionsTest, RejectsQosOverridingOptions)
{
  auto node = std::make_shared<rclcpp::Node>("generic_publisher_options_reject");

  agnocast::PublisherOptions options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions{{rclcpp::QosPolicyKind::Depth}};

  EXPECT_THROW(
    autoware::agnocast_wrapper::create_generic_publisher(
      node.get(), "/test/generic_qos_override", "std_msgs/msg/String", rclcpp::QoS(1), options),
    std::invalid_argument);
}

TEST_F(GenericPublisherOptionsTest, DefaultOptionsDoNotThrow)
{
  auto node = std::make_shared<rclcpp::Node>("generic_publisher_options_ok");

  EXPECT_NO_THROW(
    autoware::agnocast_wrapper::create_generic_publisher(
      node.get(), "/test/generic_qos_default", "std_msgs/msg/String", rclcpp::QoS(1)));
}

TEST_F(GenericSubscriptionOptionsTest, RejectsQosOverridingOptions)
{
  auto node = std::make_shared<rclcpp::Node>("generic_subscription_options_reject");

  agnocast::SubscriptionOptions options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions{{rclcpp::QosPolicyKind::Depth}};

  EXPECT_THROW(
    autoware::agnocast_wrapper::create_generic_subscription(
      node.get(), "/test/generic_sub_qos_override", "std_msgs/msg/String", rclcpp::QoS(1),
      [](auto) {}, options),
    std::invalid_argument);
}

TEST_F(GenericSubscriptionOptionsTest, DefaultOptionsDoNotThrow)
{
  auto node = std::make_shared<rclcpp::Node>("generic_subscription_options_ok");

  EXPECT_NO_THROW(
    autoware::agnocast_wrapper::create_generic_subscription(
      node.get(), "/test/generic_sub_qos_default", "std_msgs/msg/String", rclcpp::QoS(1),
      [](auto) {}));
}

#endif  // USE_AGNOCAST_ENABLED

}  // namespace
