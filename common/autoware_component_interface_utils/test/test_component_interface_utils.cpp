// Copyright 2023 The Autoware Contributors
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
#include "autoware/component_interface_utils/rclcpp/exceptions.hpp"
#include "autoware/component_interface_utils/rclcpp/interface.hpp"
#include "autoware/component_interface_utils/rclcpp/service_client.hpp"
#include "autoware/component_interface_utils/rclcpp/service_server.hpp"
#include "autoware/component_interface_utils/specs.hpp"
#include "autoware/component_interface_utils/status.hpp"
#include "gtest/gtest.h"

#include <autoware/component_interface_specs/control.hpp>
#include <autoware/component_interface_specs/planning.hpp>
#include <autoware/component_interface_specs/system.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

namespace
{

/// Minimal stand-in for a node type other than rclcpp::Node, with its own endpoint types.
template <class MessageT>
struct FakePublisher
{
  void publish(const MessageT &) {}
};

template <class MessageT>
struct FakeSubscription
{
};

struct FakeNode
{
  template <class MessageT>
  std::shared_ptr<FakePublisher<MessageT>> create_publisher(
    const std::string &, const rclcpp::QoS &)
  {
    return std::make_shared<FakePublisher<MessageT>>();
  }

  template <class MessageT, class CallbackT>
  std::shared_ptr<FakeSubscription<MessageT>> create_subscription(
    const std::string &, const rclcpp::QoS &, CallbackT &&)
  {
    return std::make_shared<FakeSubscription<MessageT>>();
  }
};

struct DerivedNode : public rclcpp::Node
{
};

/// Graph discovery of even local endpoints is asynchronous, so poll until the
/// topic has at least one publisher (or the timeout elapses) rather than assume
/// it is immediately visible.
bool wait_for_publisher(const rclcpp::Node::SharedPtr & node, const std::string & topic)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    if (!node->get_publishers_info_by_topic(topic).empty()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

/// Poll until the given service name appears in the node graph (or timeout).
bool wait_for_service(const rclcpp::Node::SharedPtr & node, const std::string & name)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    if (node->get_service_names_and_types().count(name) != 0) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

}  // namespace

TEST(interface, utils)
{
  {
    using autoware::component_interface_utils::ServiceException;
    using ResponseStatus = autoware_adapi_v1_msgs::msg::ResponseStatus;
    using ResponseStatusCode = ResponseStatus::_code_type;

    ResponseStatusCode code = 10;
    const std::string message = "test_exception";
    ServiceException service(code, message);
    ResponseStatus code_back;
    code_back = service.status();
    EXPECT_EQ(code_back.code, code);
    EXPECT_EQ(code_back.message, message);
  }

  {
    using autoware::component_interface_utils::ServiceException;
    using ResponseStatus = autoware_adapi_v1_msgs::msg::ResponseStatus;
    using ResponseStatusCode = ResponseStatus::_code_type;

    ResponseStatusCode code = 10;
    const std::string message = "test_exception";
    ServiceException service(code, message);
    ResponseStatus code_set;
    service.set(code_set);
    EXPECT_EQ(code_set.code, code);
    EXPECT_EQ(code_set.message, message);
  }

  {
    using autoware::component_interface_utils::ServiceException;
    using ResponseStatus = autoware_adapi_v1_msgs::msg::ResponseStatus;
    using ResponseStatusCode = ResponseStatus::_code_type;
    using autoware::component_interface_utils::status::copy;

    class status_test
    {
    public:
      status_test(ResponseStatusCode code, const std::string & message, bool success = false)
      {
        status.code = code;
        status.message = message;
        status.success = success;
      }
      ResponseStatus status;
    };

    const status_test status_in(10, "test_exception", true);
    auto status_copy = std::make_shared<status_test>(100, "test_exception_copy", false);
    copy(&status_in, status_copy);

    EXPECT_EQ(status_in.status.code, status_copy->status.code);
    EXPECT_EQ(status_in.status.message, status_copy->status.message);
    EXPECT_EQ(status_in.status.success, status_copy->status.success);
  }
}

TEST(interface, node_interface_no_service_log)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_node_interface");
  autoware::component_interface_utils::NodeInterface interface(node.get());
  EXPECT_EQ(interface.node, node.get());
#if AUTOWARE_COMPONENT_INTERFACE_UTILS_RCLCPP_GE_IRON
  // Service tracing is provided by ROS 2 service introspection, which defaults to OFF.
  EXPECT_EQ(interface.introspection_state, RCL_SERVICE_INTROSPECTION_OFF);
#endif
  rclcpp::shutdown();
}

#if AUTOWARE_COMPONENT_INTERFACE_UTILS_RCLCPP_GE_IRON
// Service introspection is Iron+; these cases exercise the introspection state
// and the service-event topics, which do not exist on ROS 2 Humble.
TEST(interface, node_interface_reads_introspection_parameter)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("component_interface.service_introspection", "metadata");
  auto node = std::make_shared<rclcpp::Node>("test_node_interface_metadata", opts);

  // First construction declares the parameter and reads the override ("metadata").
  autoware::component_interface_utils::NodeInterface first(node.get());
  EXPECT_EQ(first.introspection_state, RCL_SERVICE_INTROSPECTION_METADATA);

  // Second construction on the same node finds the parameter already declared and
  // reads it (the has_parameter path) without re-declaring / throwing.
  autoware::component_interface_utils::NodeInterface second(node.get());
  EXPECT_EQ(second.introspection_state, RCL_SERVICE_INTROSPECTION_METADATA);
  rclcpp::shutdown();
}
#endif

TEST(interface, service_wrappers_without_service_log)
{
  using ChangeOperationMode = autoware::component_interface_specs::system::ChangeOperationMode;
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_service");
  auto interface =
    std::make_shared<autoware::component_interface_utils::NodeInterface<>>(node.get());

  auto srv = autoware::component_interface_utils::Service<ChangeOperationMode>::make_shared(
    interface, [](auto, auto res) { res->status.success = true; }, nullptr);
  auto cli = autoware::component_interface_utils::Client<ChangeOperationMode>::make_shared(
    interface, nullptr);

  // The wrapper creates a real service server on the spec'd name.
  EXPECT_TRUE(wait_for_service(node, ChangeOperationMode::name));

  // ServiceLog is removed: the wrappers create no "/service_log" publisher.
  EXPECT_TRUE(node->get_publishers_info_by_topic("/service_log").empty());

  // Introspection defaults to OFF, so neither wrapper creates a service-event topic.
  const std::string event_topic = std::string(ChangeOperationMode::name) + "/_service_event";
  EXPECT_TRUE(node->get_publishers_info_by_topic(event_topic).empty());

  rclcpp::shutdown();
  (void)cli;
}

TEST(interface, client_async_send_request_callback_overload)
{
  using ChangeOperationMode = autoware::component_interface_specs::system::ChangeOperationMode;
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_async_callback");
  autoware::component_interface_utils::NodeAdaptor adaptor(node.get());
  auto cli = adaptor.create_client<ChangeOperationMode>();

  // The two-argument async_send_request(request, callback) overload is part of the
  // wrapper's public contract: universe consumers (e.g. the rviz panels,
  // command_mode_switcher) pass a response callback. Pin that it exists and returns
  // a valid shared future so it cannot be dropped again.
  auto req = std::make_shared<ChangeOperationMode::Service::Request>();
  auto future = cli->async_send_request(req, [](auto) {});
  EXPECT_TRUE(future.valid());
  rclcpp::shutdown();
}

TEST(interface, wrappers_deduce_endpoint_types_from_node)
{
  using OperationModeState = autoware::component_interface_specs::system::OperationModeState;
  using ChangeOperationMode = autoware::component_interface_specs::system::ChangeOperationMode;
  using Message = OperationModeState::Message;
  using Srv = ChangeOperationMode::Service;
  namespace utils = autoware::component_interface_utils;

  // The default node type keeps the endpoint types the wrappers used before they were templated.
  static_assert(
    std::is_same_v<
      typename utils::Publisher<OperationModeState>::WrapType, rclcpp::Publisher<Message>>);
  static_assert(
    std::is_same_v<
      typename utils::Subscription<OperationModeState>::WrapType, rclcpp::Subscription<Message>>);
  static_assert(
    std::is_same_v<typename utils::Client<ChangeOperationMode>::WrapType, rclcpp::Client<Srv>>);
  static_assert(
    std::is_same_v<typename utils::Service<ChangeOperationMode>::WrapType, rclcpp::Service<Srv>>);

  // Another node type contributes its own endpoint types instead.
  static_assert(
    std::is_same_v<
      typename utils::Publisher<OperationModeState, FakeNode>::WrapType, FakePublisher<Message>>);
  static_assert(std::is_same_v<
                typename utils::Subscription<OperationModeState, FakeNode>::WrapType,
                FakeSubscription<Message>>);

  // A derived node must not be deduced as the adaptor's node type, or the wrappers it creates
  // would stop matching the consumers' member declarations.
  static_assert(std::is_same_v<
                decltype(utils::NodeAdaptor(std::declval<DerivedNode *>())),
                utils::NodeAdaptor<rclcpp::Node>>);
  static_assert(std::is_same_v<
                decltype(utils::NodeAdaptor(std::declval<rclcpp::Node *>())),
                utils::NodeAdaptor<rclcpp::Node>>);
  static_assert(std::is_same_v<
                decltype(utils::NodeInterface(std::declval<DerivedNode *>())),
                utils::NodeInterface<rclcpp::Node>>);
}

TEST(interface, node_adaptor_create_publisher_qos)
{
  using OperationModeState = autoware::component_interface_specs::system::OperationModeState;
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_adaptor");
  autoware::component_interface_utils::NodeAdaptor adaptor(node.get());

  autoware::component_interface_utils::Publisher<OperationModeState>::SharedPtr pub;
  pub = adaptor.create_publisher<OperationModeState>();

  // The returning create_publisher<Spec>() applies the spec's QoS (TRANSIENT_LOCAL).
  ASSERT_TRUE(wait_for_publisher(node, OperationModeState::name));
  const auto infos = node->get_publishers_info_by_topic(OperationModeState::name);
  ASSERT_EQ(infos.size(), 1u);
  EXPECT_EQ(infos[0].qos_profile().durability(), rclcpp::DurabilityPolicy::TransientLocal);
  rclcpp::shutdown();
  (void)pub;
}

namespace
{

using OperationModeState = autoware::component_interface_specs::system::OperationModeState;
using ChangeOperationMode = autoware::component_interface_specs::system::ChangeOperationMode;

/// Callback host for the member-function overloads. Each member has a distinct
/// signature so which overload the compiler picks is observable: only the pointer
/// form can bind on_ptr, only the reference form can bind on_ref.
class CallbackHost
{
public:
  void on_ptr(const OperationModeState::Message::ConstSharedPtr msg)
  {
    ptr_mode = msg->mode;
    ptr_called = true;
  }
  void on_ref(const OperationModeState::Message & msg)
  {
    ref_mode = msg.mode;
    ref_called = true;
  }
  void on_change(
    const ChangeOperationMode::Service::Request::SharedPtr,
    const ChangeOperationMode::Service::Response::SharedPtr res)
  {
    res->status.success = true;
    srv_called = true;
  }

  bool ptr_called = false;
  bool ref_called = false;
  bool srv_called = false;
  uint8_t ptr_mode = 0;
  uint8_t ref_mode = 0;
};

/// Spin until the predicate holds (or the timeout elapses) so delivery is awaited
/// rather than assumed.
bool spin_until(const rclcpp::Node::SharedPtr & node, const std::function<bool()> & done)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    if (done()) {
      return true;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return done();
}

}  // namespace

TEST(interface, node_adaptor_relay_message)
{
  // relay_message needs two specs carrying the same message type; these are the
  // only such pair among the core specs, and the relay direction here has no
  // meaning beyond exercising the helper.
  using Source = autoware::component_interface_specs::planning::Trajectory;
  using Target = autoware::component_interface_specs::control::PredictedTrajectory;

  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_adaptor_relay");
  autoware::component_interface_utils::NodeAdaptor adaptor(node.get());

  autoware::component_interface_utils::Publisher<Target>::SharedPtr relay_pub;
  autoware::component_interface_utils::Subscription<Source>::SharedPtr relay_sub;
  adaptor.relay_message(relay_pub, relay_sub);

  size_t relayed_points = 0;
  auto observer = adaptor.create_subscription<Target>(
    [&relayed_points](const Target::Message::ConstSharedPtr msg) {
      relayed_points = msg->points.size();
    });
  auto source = adaptor.create_publisher<Source>();

  Source::Message msg;
  msg.points.resize(3);
  source->publish(msg);

  // The relay forwards what it receives on the source spec onto the target spec,
  // so a message published on the source has to come back out on the target.
  EXPECT_TRUE(spin_until(node, [&relayed_points] { return relayed_points != 0; }));
  EXPECT_EQ(relayed_points, 3u);

  rclcpp::shutdown();
  (void)observer;
}

TEST(interface, node_adaptor_create_subscription_member_callbacks)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_adaptor_member_sub");
  autoware::component_interface_utils::NodeAdaptor adaptor(node.get());
  CallbackHost host;

  // Both member-function shapes the out-parameter init_sub accepts are also
  // reachable through the returning form, so a caller does not have to spell out
  // a std::bind or a forwarding lambda to keep using a member callback.
  auto sub_ptr = adaptor.create_subscription<OperationModeState>(&host, &CallbackHost::on_ptr);
  auto sub_ref = adaptor.create_subscription<OperationModeState>(&host, &CallbackHost::on_ref);
  auto pub = adaptor.create_publisher<OperationModeState>();

  OperationModeState::Message msg;
  msg.mode = OperationModeState::Message::AUTONOMOUS;
  pub->publish(msg);

  EXPECT_TRUE(spin_until(node, [&host] { return host.ptr_called && host.ref_called; }));
  EXPECT_EQ(host.ptr_mode, OperationModeState::Message::AUTONOMOUS);
  EXPECT_EQ(host.ref_mode, OperationModeState::Message::AUTONOMOUS);

  rclcpp::shutdown();
  (void)sub_ptr;
  (void)sub_ref;
}

TEST(interface, node_adaptor_create_service_member_callback)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_adaptor_member_srv");
  autoware::component_interface_utils::NodeAdaptor adaptor(node.get());
  CallbackHost host;

  // The member-function service overload resolves against the returning form and
  // creates a real server on the spec'd name; the two-argument callback shape is
  // what distinguishes it from create_service<Spec>(callback, group).
  auto srv = adaptor.create_service<ChangeOperationMode>(&host, &CallbackHost::on_change);
  ASSERT_TRUE(wait_for_service(node, ChangeOperationMode::name));

  // Drive a real request through it, so the binding itself is asserted rather than
  // the mere existence of a server: only the bound member can set success.
  auto cli = node->create_client<ChangeOperationMode::Service>(ChangeOperationMode::name);
  auto future = cli->async_send_request(std::make_shared<ChangeOperationMode::Service::Request>())
                  .future.share();
  ASSERT_TRUE(spin_until(node, [&future] {
    return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }));
  EXPECT_TRUE(host.srv_called);
  EXPECT_TRUE(future.get()->status.success);

  rclcpp::shutdown();
  (void)srv;
}

#if AUTOWARE_COMPONENT_INTERFACE_UTILS_RCLCPP_GE_IRON
// Service-event topics only exist where ROS 2 service introspection is available
// (Iron onward), so these cases do not run on ROS 2 Humble.
TEST(interface, introspection_event_topic)
{
  using ChangeOperationMode = autoware::component_interface_specs::system::ChangeOperationMode;
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("component_interface.service_introspection", "contents");
  auto node = std::make_shared<rclcpp::Node>("test_introspection", opts);
  autoware::component_interface_utils::NodeAdaptor adaptor(node.get());

  auto srv =
    adaptor.create_service<ChangeOperationMode>([](auto, auto res) { res->status.success = true; });

  // With introspection enabled, the service exposes its "/_service_event" topic.
  const std::string event_topic = std::string(ChangeOperationMode::name) + "/_service_event";
  EXPECT_TRUE(wait_for_publisher(node, event_topic));
  rclcpp::shutdown();
  (void)srv;
}

TEST(interface, client_introspection_event_topic)
{
  using ChangeOperationMode = autoware::component_interface_specs::system::ChangeOperationMode;
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("component_interface.service_introspection", "contents");
  auto node = std::make_shared<rclcpp::Node>("test_client_introspection", opts);
  autoware::component_interface_utils::NodeAdaptor adaptor(node.get());

  auto cli = adaptor.create_client<ChangeOperationMode>();

  // The client wrapper configures introspection symmetrically with the service,
  // so it too exposes the "/_service_event" topic when enabled.
  const std::string event_topic = std::string(ChangeOperationMode::name) + "/_service_event";
  EXPECT_TRUE(wait_for_publisher(node, event_topic));
  rclcpp::shutdown();
  (void)cli;
}
#endif
