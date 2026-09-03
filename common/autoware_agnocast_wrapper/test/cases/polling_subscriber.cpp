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

// The expectations below are the ones autoware_utils_rclcpp pins for its own polling subscriber
// (autoware_utils_rclcpp/test/cases/polling_subscriber.cpp). They are written against the
// backend-agnostic interface, so the same expectations cover the ROS 2 and the agnocast backend.
//
// The All-policy cases and the last_taken_data_timestamp() assertions are not ported: neither has
// a counterpart in the wrapper, for the reasons given in polling_subscriber.hpp.

#include "autoware/agnocast_wrapper/polling_subscriber.hpp"

#include "autoware/agnocast_wrapper/node.hpp"
#include "autoware/agnocast_wrapper/runtime.hpp"

#include <std_msgs/msg/string.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

namespace polling = autoware::agnocast_wrapper::polling;
using autoware::agnocast_wrapper::Node;
using std_msgs::msg::String;

constexpr auto discovery_timeout = std::chrono::seconds(10);
constexpr auto publish_interval = std::chrono::milliseconds(50);
constexpr auto settle_delay = std::chrono::milliseconds(200);

/// agnocast exits the process from inside the subscription constructor when LD_PRELOAD lacks the
/// heaphook (validate_ld_preload() in agnocast_utils.cpp), which would take the whole test binary
/// down instead of failing one case. Probe the same condition so the test can skip instead.
bool agnocast_heaphook_loaded()
{
  const char * ld_preload = std::getenv("LD_PRELOAD");
  return ld_preload != nullptr &&
         std::string(ld_preload).find("libagnocast_heaphook.so") != std::string::npos;
}

class PollingSubscriberTest : public testing::Test
{
protected:
  void SetUp() override
  {
    if (autoware::agnocast_wrapper::use_agnocast() && !agnocast_heaphook_loaded()) {
      GTEST_SKIP() << "ENABLE_AGNOCAST=1 without the agnocast heaphook: the agnocast backend "
                      "cannot be exercised in this environment.";
    }
  }

  /// Republish until the first message lands: a message sent before the subscriber is matched is
  /// dropped, and the publisher's subscription count is no help because the agnocast backend
  /// counts only subscribers in other processes.
  // TODO(Koichi98): wait on get_subscription_count() once agnocast counts same-process subscribers.
  template <typename PublisherT, typename SubscriberT>
  static std::shared_ptr<const String> publish_until_delivered(
    const PublisherT & publisher, const SubscriberT & subscriber, const String & message)
  {
    const auto deadline = std::chrono::steady_clock::now() + discovery_timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      publisher->publish(message);
      std::this_thread::sleep_for(publish_interval);
      if (const auto taken = subscriber->take_data()) {
        return taken;
      }
    }
    return nullptr;
  }

  /// Republishing can leave one more message in flight, so let it land and take it before
  /// asserting on what the next take returns.
  template <typename SubscriberT>
  static std::shared_ptr<const String> settle(const SubscriberT & subscriber)
  {
    std::this_thread::sleep_for(settle_delay);
    return subscriber->take_data();
  }
};

TEST_F(PollingSubscriberTest, CheckQosDepthGreaterThanOneThrows)
{
  const auto node = std::make_shared<Node>("test_check_qos_throw");

  EXPECT_THROW(
    polling::create_polling_subscriber<String>(node.get(), "/test/latest_deep", rclcpp::QoS{10}),
    std::invalid_argument);

  EXPECT_THROW(
    (polling::create_polling_subscriber<String, polling::polling_policy::Newest>(
      node.get(), "/test/newest_deep", rclcpp::QoS{10})),
    std::invalid_argument);
}

TEST_F(PollingSubscriberTest, CheckQosDepthOneDoesNotThrow)
{
  const auto node = std::make_shared<Node>("test_check_qos_no_throw");

  EXPECT_NO_THROW(
    polling::create_polling_subscriber<String>(node.get(), "/test/latest_shallow", rclcpp::QoS{1}));

  EXPECT_NO_THROW((polling::create_polling_subscriber<String, polling::polling_policy::Newest>(
    node.get(), "/test/newest_shallow", rclcpp::QoS{1})));
}

TEST_F(PollingSubscriberTest, InitialValues)
{
  const auto node = std::make_shared<Node>("test_initial_values");

  const auto latest_sub =
    polling::create_polling_subscriber<String>(node.get(), "/test/initial_latest", 1);
  EXPECT_EQ(latest_sub->take_data(), nullptr);

  const auto newest_sub =
    polling::create_polling_subscriber<String, polling::polling_policy::Newest>(
      node.get(), "/test/initial_newest", 1);
  EXPECT_EQ(newest_sub->take_data(), nullptr);
}

TEST_F(PollingSubscriberTest, PubSub)
{
  const auto pub_node = std::make_shared<Node>("pub_node");
  const auto sub_node = std::make_shared<Node>("sub_node");

  const auto pub = pub_node->create_publisher<String>("/test/text", rclcpp::QoS{1});
  const auto sub = polling::create_polling_subscriber<String>(sub_node.get(), "/test/text", 1);

  String pub_msg;
  pub_msg.data = "foo-bar";

  const auto sub_msg = publish_until_delivered(pub, sub, pub_msg);
  ASSERT_NE(sub_msg, nullptr);
  EXPECT_EQ(sub_msg->data, pub_msg.data);
}

TEST_F(PollingSubscriberTest, LatestRedeliversUntilANewerMessageArrives)
{
  const auto pub_node = std::make_shared<Node>("pub_node_latest");
  const auto sub_node = std::make_shared<Node>("sub_node_latest");

  const auto pub = pub_node->create_publisher<String>("/test/latest_retention", rclcpp::QoS{1});
  const auto sub =
    polling::create_polling_subscriber<String>(sub_node.get(), "/test/latest_retention", 1);

  String pub_msg;
  pub_msg.data = "test-message";
  ASSERT_NE(publish_until_delivered(pub, sub, pub_msg), nullptr);

  const auto msg1 = settle(sub);
  ASSERT_NE(msg1, nullptr);

  const auto msg2 = sub->take_data();
  EXPECT_EQ(msg2, msg1);
}

TEST_F(PollingSubscriberTest, NewestReturnsNullWithoutNewMessage)
{
  const auto pub_node = std::make_shared<Node>("pub_node_newest");
  const auto sub_node = std::make_shared<Node>("sub_node_newest");

  const auto pub = pub_node->create_publisher<String>("/test/newest_clear", rclcpp::QoS{1});
  const auto sub = polling::create_polling_subscriber<String, polling::polling_policy::Newest>(
    sub_node.get(), "/test/newest_clear", 1);

  String pub_msg;
  pub_msg.data = "test-message";
  ASSERT_NE(publish_until_delivered(pub, sub, pub_msg), nullptr);

  settle(sub);

  EXPECT_EQ(sub->take_data(), nullptr);
}

}  // namespace
