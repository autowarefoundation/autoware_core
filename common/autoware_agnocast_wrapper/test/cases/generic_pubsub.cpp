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
// backend actually selected by ENABLE_AGNOCAST (the ROS 2 one in this test environment, since no
// agnocast heaphook/kernel module is available here — see agnocast_heaphook_loaded()).

#include "autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp"
#include "autoware/agnocast_wrapper/node.hpp"

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
};

TEST(GenericPubSubMethod1Test, MacroRoundTrip)
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

TEST(GenericPubSubMethod2Test, NodeMemberRoundTrip)
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
TEST(GenericPubSubMethod2Test, NodeMemberDepthAndOptionsOverload)
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

TEST(GenericPubSubMethod2Test, UnknownTopicTypeThrows)
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

// create_generic_publisher()/AgnocastGenericPublisher/ROS2GenericPublisher only exist in the
// Agnocast-enabled build (generic_publisher.hpp is guarded by USE_AGNOCAST_ENABLED end to end),
// so the qos_overriding_options rejection they share can only be exercised there.
#ifdef USE_AGNOCAST_ENABLED

TEST(GenericPublisherOptionsTest, RejectsQosOverridingOptions)
{
  auto node = std::make_shared<rclcpp::Node>("generic_publisher_options_reject");

  agnocast::PublisherOptions options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions{{rclcpp::QosPolicyKind::Depth}};

  EXPECT_THROW(
    autoware::agnocast_wrapper::create_generic_publisher(
      node.get(), "/test/generic_qos_override", "std_msgs/msg/String", rclcpp::QoS(1), options),
    std::invalid_argument);
}

TEST(GenericPublisherOptionsTest, DefaultOptionsDoNotThrow)
{
  auto node = std::make_shared<rclcpp::Node>("generic_publisher_options_ok");

  EXPECT_NO_THROW(
    autoware::agnocast_wrapper::create_generic_publisher(
      node.get(), "/test/generic_qos_default", "std_msgs/msg/String", rclcpp::QoS(1)));
}

#endif  // USE_AGNOCAST_ENABLED

}  // namespace
