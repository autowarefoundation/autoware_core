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

// Exercises the generic (type-erased) service/client surface end to end: Method 1 (macro + free
// function, on a plain rclcpp::Node) and Method 2 (Node member) round-trip a std_srvs/SetBool call
// through the runtime backend actually selected by ENABLE_AGNOCAST, in both the basic and deferred
// callback styles. It also exercises cancel_request()/cancel_response() and their destructors'
// defensive cleanup -- the mechanism that stops an abandoned, borrowed (never sent) Agnocast
// request/response from calling std::terminate() when it is dropped (see generic_service.hpp's and
// generic_client.hpp's doc comments on agnocast::ipc_shared_ptr<void>::reset()). Every test below
// is skipped, not run, when ENABLE_AGNOCAST=1 at runtime without the agnocast heaphook loaded: same
// hazard generic_pubsub.cpp/polling_subscriber.cpp/service_introspection.cpp guard against with the
// same agnocast_heaphook_loaded() check.
//
// NOTE on real Agnocast dispatch: every test here uses rclcpp::executors::SingleThreadedExecutor,
// which cannot actually drive Agnocast callback dispatch (that needs
// agnocast::MultiThreadedAgnocastExecutor) -- so with ENABLE_AGNOCAST=1 and the heaphook loaded,
// tests that depend only on construction-time behavior (the error-path tests below) still verify
// the real Agnocast backend, but tests that need a request to actually be delivered (every
// round-trip test, and any error-path test built around one, such as
// CreateResponseCalledTwiceBackendSpecificBehavior and
// SendResponseWithWrongResponseObjectBackendSpecificBehavior) cannot be exercised by this binary
// under real Agnocast: Method 1 round trips simply time out, and Method 2 round trips fail
// earlier still, at get_rclcpp_node() itself (which throws under real Agnocast by design). Real
// end-to-end Agnocast dispatch coverage for this file's logic has to come from a separate
// process-level test built on agnocast::MultiThreadedAgnocastExecutor, not from this gtest binary.

#include "autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp"
#include "autoware/agnocast_wrapper/node.hpp"
#include "heaphook_probe.hpp"

#include <rclcpp/rclcpp.hpp>

#ifdef USE_AGNOCAST_ENABLED
#include <agnocast/agnocast_multi_threaded_executor.hpp>
#endif

#include <std_srvs/srv/set_bool.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

using autoware::agnocast_wrapper::GenericClient;
using autoware::agnocast_wrapper::GenericService;
using autoware::agnocast_wrapper::test::agnocast_heaphook_loaded;
using SetBool = std_srvs::srv::SetBool;

constexpr auto wait_timeout = std::chrono::seconds(10);
constexpr auto poll_interval = std::chrono::milliseconds(10);

/// Same guard as GenericPubSubTestBase (generic_pubsub.cpp), PollingSubscriberTest
/// (polling_subscriber.cpp) and ServiceIntrospectionTest (service_introspection.cpp): every
/// generic service/client test suite below aliases this fixture.
class GenericServiceClientTestBase : public testing::Test
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

using GenericServiceClientMethod1Test = GenericServiceClientTestBase;
using GenericServiceClientMethod2Test = GenericServiceClientTestBase;
using GenericServiceClientCancelTest = GenericServiceClientTestBase;
using GenericServiceClientErrorTest = GenericServiceClientTestBase;

/// A plain rclcpp::Node subclass (Method 1: base class stays rclcpp::Node), exercising the
/// AUTOWARE_CREATE_GENERIC_SERVICE*/AUTOWARE_CREATE_GENERIC_CLIENT* macros the way a Method 1
/// node would, rather than calling create_generic_service()/create_generic_client() directly.
class GenericServiceClientMethod1Node : public rclcpp::Node
{
public:
  explicit GenericServiceClientMethod1Node(const std::string & name) : rclcpp::Node(name) {}

  template <typename Func>
  AUTOWARE_GENERIC_SERVICE_PTR create_bool_service(const std::string & service_name, Func && cb)
  {
    return AUTOWARE_CREATE_GENERIC_SERVICE2(
      service_name, "std_srvs/srv/SetBool", std::forward<Func>(cb));
  }

  template <typename Func>
  AUTOWARE_GENERIC_SERVICE_PTR create_bool_service_with_qos(
    const std::string & service_name, Func && cb, const rclcpp::QoS & qos)
  {
    return AUTOWARE_CREATE_GENERIC_SERVICE3(
      service_name, "std_srvs/srv/SetBool", std::forward<Func>(cb), qos);
  }

  template <typename Func>
  AUTOWARE_GENERIC_SERVICE_PTR create_bool_service_with_qos_group(
    const std::string & service_name, Func && cb, const rclcpp::QoS & qos,
    const rclcpp::CallbackGroup::SharedPtr & group)
  {
    return AUTOWARE_CREATE_GENERIC_SERVICE4(
      service_name, "std_srvs/srv/SetBool", std::forward<Func>(cb), qos, group);
  }

  AUTOWARE_GENERIC_CLIENT_PTR create_bool_client(const std::string & service_name)
  {
    return AUTOWARE_CREATE_GENERIC_CLIENT1(service_name, "std_srvs/srv/SetBool");
  }

  AUTOWARE_GENERIC_CLIENT_PTR create_bool_client_with_qos(
    const std::string & service_name, const rclcpp::QoS & qos)
  {
    return AUTOWARE_CREATE_GENERIC_CLIENT2(service_name, "std_srvs/srv/SetBool", qos);
  }

  AUTOWARE_GENERIC_CLIENT_PTR create_bool_client_with_qos_group(
    const std::string & service_name, const rclcpp::QoS & qos,
    const rclcpp::CallbackGroup::SharedPtr & group)
  {
    return AUTOWARE_CREATE_GENERIC_CLIENT3(service_name, "std_srvs/srv/SetBool", qos, group);
  }
};

bool wait_until(const std::function<bool()> & predicate, std::chrono::nanoseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(poll_interval);
  }
  return predicate();
}

/// Spins @p executor until @p predicate is true or wait_timeout elapses; returns whether it
/// became true.
bool spin_until(rclcpp::Executor & executor, const std::function<bool()> & predicate)
{
  return wait_until(
    [&] {
      executor.spin_some();
      return predicate();
    },
    wait_timeout);
}

SetBool::Request * as_request(const std::shared_ptr<void> & ptr)
{
  return static_cast<SetBool::Request *>(ptr.get());
}

SetBool::Response * as_response(const std::shared_ptr<void> & ptr)
{
  return static_cast<SetBool::Response *>(ptr.get());
}

// ===== Method 1 (macro + free function, plain rclcpp::Node): basic callback =====

TEST_F(GenericServiceClientMethod1Test, BasicCallbackRoundTrip)
{
  auto server_node = std::make_shared<GenericServiceClientMethod1Node>("gsc_m1_basic_server");
  auto client_node = std::make_shared<GenericServiceClientMethod1Node>("gsc_m1_basic_client");

  const auto service = server_node->create_bool_service(
    "/test/gsc_m1_basic", [](std::shared_ptr<void> request, std::shared_ptr<void> response) {
      as_response(response)->success = as_request(request)->data;
      as_response(response)->message = as_request(request)->data ? "ok" : "no";
    });
  const auto client = client_node->create_bool_client("/test/gsc_m1_basic");

  // get_service_name() is part of both GenericService's and GenericClient's public interface but
  // otherwise never exercised by any test below; both must report the same, fully-qualified name.
  EXPECT_STREQ(service->get_service_name(), "/test/gsc_m1_basic");
  EXPECT_STREQ(client->get_service_name(), "/test/gsc_m1_basic");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  ASSERT_TRUE(spin_until(executor, [&] { return client->service_is_ready(); }));

  auto request = client->create_request();
  as_request(request)->data = true;

  std::atomic<bool> received{false};
  bool success = false;
  std::string message;
  client->async_send_request(request, [&](GenericClient::SharedFuture fut) {
    auto response = fut.get();
    success = as_response(response)->success;
    message = as_response(response)->message;
    received = true;
  });

  ASSERT_TRUE(spin_until(executor, [&] { return received.load(); }));
  EXPECT_TRUE(success);
  EXPECT_EQ(message, "ok");
}

// wait_for_service() (the blocking, timeout-based counterpart to polling service_is_ready() the
// way every other test below does) is otherwise never exercised.
TEST_F(GenericServiceClientMethod1Test, WaitForServiceTimesOutThenSucceeds)
{
  auto client_node = std::make_shared<GenericServiceClientMethod1Node>("gsc_wait_client");
  auto client = client_node->create_bool_client("/test/gsc_wait_for_service");

  // No server yet: a short timeout must elapse and report false, not block forever or throw.
  EXPECT_FALSE(client->wait_for_service(std::chrono::milliseconds(50)));

  auto server_node = std::make_shared<GenericServiceClientMethod1Node>("gsc_wait_server");
  const auto service = server_node->create_bool_service(
    "/test/gsc_wait_for_service", [](std::shared_ptr<void>, std::shared_ptr<void>) {});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  bool ready = false;
  ASSERT_TRUE(wait_until(
    [&] {
      executor.spin_some();
      ready = client->wait_for_service(std::chrono::milliseconds(0));
      return ready;
    },
    wait_timeout));
  EXPECT_TRUE(ready);
}

// ===== Method 1: deferred callback (create_response()/send_response() called later) =====

TEST_F(GenericServiceClientMethod1Test, DeferredCallbackRoundTrip)
{
  auto server_node = std::make_shared<GenericServiceClientMethod1Node>("gsc_m1_deferred_server");
  auto client_node = std::make_shared<GenericServiceClientMethod1Node>("gsc_m1_deferred_client");

  const auto service = server_node->create_bool_service(
    "/test/gsc_m1_deferred", [](GenericService::SharedPtr service, std::shared_ptr<void> request) {
      auto response = service->create_response(request);
      as_response(response)->success = as_request(request)->data;
      as_response(response)->message = "deferred";
      service->send_response(request, response);
    });
  const auto client = client_node->create_bool_client("/test/gsc_m1_deferred");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  ASSERT_TRUE(spin_until(executor, [&] { return client->service_is_ready(); }));

  auto request = client->create_request();
  as_request(request)->data = true;

  auto future = client->async_send_request(request);
  ASSERT_TRUE(spin_until(executor, [&] {
    return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }));
  auto response = future.get();
  EXPECT_TRUE(as_response(response)->success);
  EXPECT_EQ(as_response(response)->message, "deferred");
}

// ===== Method 1: the _ON_NODE macros (helper classes / free functions holding only a node ptr) ==

TEST_F(GenericServiceClientMethod1Test, OnNodeRoundTrip)
{
  auto server_node = std::make_shared<rclcpp::Node>("gsc_m1_on_node_server");
  auto client_node = std::make_shared<rclcpp::Node>("gsc_m1_on_node_client");

  const AUTOWARE_GENERIC_SERVICE_PTR service = AUTOWARE_CREATE_GENERIC_SERVICE2_ON_NODE(
    server_node.get(), "/test/gsc_m1_on_node", "std_srvs/srv/SetBool",
    ([](std::shared_ptr<void> request, std::shared_ptr<void> response) {
      as_response(response)->success = as_request(request)->data;
    }));
  const AUTOWARE_GENERIC_CLIENT_PTR client = AUTOWARE_CREATE_GENERIC_CLIENT1_ON_NODE(
    client_node.get(), "/test/gsc_m1_on_node", "std_srvs/srv/SetBool");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  ASSERT_TRUE(spin_until(executor, [&] { return client->service_is_ready(); }));

  auto request = client->create_request();
  as_request(request)->data = true;

  std::atomic<bool> received{false};
  bool success = false;
  client->async_send_request(request, [&](GenericClient::SharedFuture fut) {
    success = as_response(fut.get())->success;
    received = true;
  });

  ASSERT_TRUE(spin_until(executor, [&] { return received.load(); }));
  EXPECT_TRUE(success);
}

// The 3- and 4-argument (qos, qos+group) macro variants are only reachable through explicit
// macro invocation, not through any other codepath this file already exercises -- a typo in one
// of their expansions (AUTOWARE_CREATE_GENERIC_SERVICE3/4, AUTOWARE_CREATE_GENERIC_CLIENT2/3)
// would otherwise go undetected until some future caller happens to need that exact arity.
TEST_F(GenericServiceClientMethod1Test, MacroQosAndGroupArityVariantsRoundTrip)
{
  auto server_node = std::make_shared<GenericServiceClientMethod1Node>("gsc_m1_arity_server");
  auto client_node = std::make_shared<GenericServiceClientMethod1Node>("gsc_m1_arity_client");
  const rclcpp::QoS qos = rclcpp::ServicesQoS();
  // A callback group belongs to the node it was created on, so the service and client each need
  // their own -- passing server_group to the client (or vice versa) throws "callback group not in
  // node".
  const rclcpp::CallbackGroup::SharedPtr server_group =
    server_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const rclcpp::CallbackGroup::SharedPtr client_group =
    client_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  const auto service3 = server_node->create_bool_service_with_qos(
    "/test/gsc_m1_arity3",
    [](std::shared_ptr<void> request, std::shared_ptr<void> response) {
      as_response(response)->success = as_request(request)->data;
    },
    qos);
  const auto client2 = client_node->create_bool_client_with_qos("/test/gsc_m1_arity3", qos);

  const auto service4 = server_node->create_bool_service_with_qos_group(
    "/test/gsc_m1_arity4",
    [](std::shared_ptr<void> request, std::shared_ptr<void> response) {
      as_response(response)->success = as_request(request)->data;
    },
    qos, server_group);
  const auto client3 =
    client_node->create_bool_client_with_qos_group("/test/gsc_m1_arity4", qos, client_group);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  ASSERT_TRUE(spin_until(
    executor, [&] { return client2->service_is_ready() && client3->service_is_ready(); }));

  auto request3 = client2->create_request();
  as_request(request3)->data = true;
  std::atomic<bool> received3{false};
  bool success3 = false;
  client2->async_send_request(request3, [&](GenericClient::SharedFuture fut) {
    success3 = as_response(fut.get())->success;
    received3 = true;
  });

  auto request4 = client3->create_request();
  as_request(request4)->data = true;
  std::atomic<bool> received4{false};
  bool success4 = false;
  client3->async_send_request(request4, [&](GenericClient::SharedFuture fut) {
    success4 = as_response(fut.get())->success;
    received4 = true;
  });

  ASSERT_TRUE(spin_until(executor, [&] { return received3.load() && received4.load(); }));
  EXPECT_TRUE(success3);
  EXPECT_TRUE(success4);
}

// Same coverage as MacroQosAndGroupArityVariantsRoundTrip, for the _ON_NODE forms of the same
// macros (AUTOWARE_CREATE_GENERIC_SERVICE3/4_ON_NODE, AUTOWARE_CREATE_GENERIC_CLIENT2/3_ON_NODE).
TEST_F(GenericServiceClientMethod1Test, MacroOnNodeQosAndGroupArityVariantsRoundTrip)
{
  auto server_node = std::make_shared<rclcpp::Node>("gsc_m1_on_node_arity_server");
  auto client_node = std::make_shared<rclcpp::Node>("gsc_m1_on_node_arity_client");
  const rclcpp::QoS qos = rclcpp::ServicesQoS();
  // A callback group belongs to the node it was created on, so the service and client each need
  // their own -- passing server_group to the client (or vice versa) throws "callback group not in
  // node".
  const rclcpp::CallbackGroup::SharedPtr server_group =
    server_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const rclcpp::CallbackGroup::SharedPtr client_group =
    client_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  const AUTOWARE_GENERIC_SERVICE_PTR service3 = AUTOWARE_CREATE_GENERIC_SERVICE3_ON_NODE(
    server_node.get(), "/test/gsc_m1_on_node_arity3", "std_srvs/srv/SetBool",
    ([](std::shared_ptr<void> request, std::shared_ptr<void> response) {
      as_response(response)->success = as_request(request)->data;
    }),
    qos);
  const AUTOWARE_GENERIC_CLIENT_PTR client2 = AUTOWARE_CREATE_GENERIC_CLIENT2_ON_NODE(
    client_node.get(), "/test/gsc_m1_on_node_arity3", "std_srvs/srv/SetBool", qos);

  const AUTOWARE_GENERIC_SERVICE_PTR service4 = AUTOWARE_CREATE_GENERIC_SERVICE4_ON_NODE(
    server_node.get(), "/test/gsc_m1_on_node_arity4", "std_srvs/srv/SetBool",
    ([](std::shared_ptr<void> request, std::shared_ptr<void> response) {
      as_response(response)->success = as_request(request)->data;
    }),
    qos, server_group);
  const AUTOWARE_GENERIC_CLIENT_PTR client3 = AUTOWARE_CREATE_GENERIC_CLIENT3_ON_NODE(
    client_node.get(), "/test/gsc_m1_on_node_arity4", "std_srvs/srv/SetBool", qos, client_group);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  ASSERT_TRUE(spin_until(
    executor, [&] { return client2->service_is_ready() && client3->service_is_ready(); }));

  auto request3 = client2->create_request();
  as_request(request3)->data = true;
  std::atomic<bool> received3{false};
  bool success3 = false;
  client2->async_send_request(request3, [&](GenericClient::SharedFuture fut) {
    success3 = as_response(fut.get())->success;
    received3 = true;
  });

  auto request4 = client3->create_request();
  as_request(request4)->data = true;
  std::atomic<bool> received4{false};
  bool success4 = false;
  client3->async_send_request(request4, [&](GenericClient::SharedFuture fut) {
    success4 = as_response(fut.get())->success;
    received4 = true;
  });

  ASSERT_TRUE(spin_until(executor, [&] { return received3.load() && received4.load(); }));
  EXPECT_TRUE(success3);
  EXPECT_TRUE(success4);
}

// ===== Method 2 (autoware::agnocast_wrapper::Node member): basic callback =====

TEST_F(GenericServiceClientMethod2Test, BasicCallbackRoundTrip)
{
  using autoware::agnocast_wrapper::Node;

  auto server_node = std::make_shared<Node>("gsc_m2_basic_server");
  auto client_node = std::make_shared<Node>("gsc_m2_basic_client");

  const auto service = server_node->create_generic_service(
    "/test/gsc_m2_basic", "std_srvs/srv/SetBool",
    [](std::shared_ptr<void> request, std::shared_ptr<void> response) {
      as_response(response)->success = as_request(request)->data;
    });
  const auto client =
    client_node->create_generic_client("/test/gsc_m2_basic", "std_srvs/srv/SetBool");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node->get_rclcpp_node());
  executor.add_node(client_node->get_rclcpp_node());

  ASSERT_TRUE(spin_until(executor, [&] { return client->service_is_ready(); }));

  auto request = client->create_request();
  as_request(request)->data = true;

  std::atomic<bool> received{false};
  bool success = false;
  client->async_send_request(request, [&](GenericClient::SharedFuture fut) {
    success = as_response(fut.get())->success;
    received = true;
  });

  ASSERT_TRUE(spin_until(executor, [&] { return received.load(); }));
  EXPECT_TRUE(success);
}

// ===== Method 2: deferred callback =====

TEST_F(GenericServiceClientMethod2Test, DeferredCallbackRoundTrip)
{
  using autoware::agnocast_wrapper::Node;

  auto server_node = std::make_shared<Node>("gsc_m2_deferred_server");
  auto client_node = std::make_shared<Node>("gsc_m2_deferred_client");

  const auto service = server_node->create_generic_service(
    "/test/gsc_m2_deferred", "std_srvs/srv/SetBool",
    [](GenericService::SharedPtr service, std::shared_ptr<void> request) {
      auto response = service->create_response(request);
      as_response(response)->success = as_request(request)->data;
      service->send_response(request, response);
    });
  const auto client =
    client_node->create_generic_client("/test/gsc_m2_deferred", "std_srvs/srv/SetBool");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node->get_rclcpp_node());
  executor.add_node(client_node->get_rclcpp_node());

  ASSERT_TRUE(spin_until(executor, [&] { return client->service_is_ready(); }));

  auto request = client->create_request();
  as_request(request)->data = true;
  auto future = client->async_send_request(request);

  ASSERT_TRUE(spin_until(executor, [&] {
    return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }));
  EXPECT_TRUE(as_response(future.get())->success);
}

// ===== Concurrency: real multi-threaded dispatch, not the manual spin_some() polling every other
// test above uses (which serializes everything on one thread and can never exercise a genuine
// send-vs-response-delivery race). Regression coverage for a real bug found in review: an earlier
// version of ROS2GenericClient::async_send_request() called rcl_send_request() before registering
// the pending-response bookkeeping under pending_mutex_, so handle_response() could observe a
// sent-but-not-yet-registered sequence number (on a different executor thread, dispatched from the
// output's own dedicated callback group) and silently, permanently drop the response. Deterministic
// reproduction (with an artificial delay temporarily inserted between the send and the register)
// confirmed the drop before the fix and its absence after; this test provides ongoing, non-invasive
// regression coverage by making the natural race window matter across many iterations instead.

TEST_F(GenericServiceClientMethod1Test, ManyConcurrentRoundTripsUnderRealMultiThreadedDispatch)
{
  auto server_node = std::make_shared<GenericServiceClientMethod1Node>("gsc_stress_server");
  auto client_node = std::make_shared<GenericServiceClientMethod1Node>("gsc_stress_client");

  // A larger-than-default QoS depth: the default rclcpp::ServicesQoS() history depth (10) is
  // easily exceeded by firing kIterations requests back-to-back with no pacing below, which would
  // drop requests/responses at the DDS layer for reasons entirely unrelated to the race this test
  // exists to catch. Using kIterations as the depth keeps that a non-factor.
  constexpr int kIterations = 300;
  const rclcpp::QoS qos = rclcpp::ServicesQoS().keep_last(kIterations);

  // Dedicated (non-default) callback groups: every rclcpp::Node implicitly starts its own
  // parameter services (get/set/list/describe_parameters) in its default callback group unless
  // told not to, and under real Agnocast dispatch, MultiThreadedAgnocastExecutor's
  // validate_callback_group() hard-`exit()`s the whole process if an Agnocast-backed endpoint
  // shares a MutuallyExclusive group with any plain ROS2 callback -- including those built-in
  // parameter services. This is exactly the reason service_divider_plugin_base.cpp gives each
  // generic service/client its own dedicated group instead of relying on the node's default one.
  const rclcpp::CallbackGroup::SharedPtr server_group =
    server_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const rclcpp::CallbackGroup::SharedPtr client_group =
    client_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  const auto service = server_node->create_bool_service_with_qos_group(
    "/test/gsc_stress",
    [](std::shared_ptr<void> request, std::shared_ptr<void> response) {
      as_response(response)->success = as_request(request)->data;
    },
    qos, server_group);
  const auto client =
    client_node->create_bool_client_with_qos_group("/test/gsc_stress", qos, client_group);

  // A real multi-threaded executor, not the SingleThreadedExecutor + spin_some() polling loop used
  // elsewhere in this file: the response callback must be able to run on a genuinely different
  // thread than the one calling async_send_request(), concurrently with it, for this test to mean
  // anything. Unlike this file's other tests, that also means picking
  // agnocast::MultiThreadedAgnocastExecutor when running under real Agnocast (see the file-level
  // "NOTE on real Agnocast dispatch" above) -- ROS2GenericClient is what the bug this test guards
  // against was in, but keeping this test meaningful under both backends is little extra cost.
  std::shared_ptr<rclcpp::Executor> executor;
#ifdef USE_AGNOCAST_ENABLED
  if (autoware::agnocast_wrapper::use_agnocast()) {
    executor = std::make_shared<agnocast::MultiThreadedAgnocastExecutor>();
  } else {
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  }
#else
  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
#endif
  executor->add_node(server_node);
  executor->add_node(client_node);
  std::thread spin_thread([&executor] { executor->spin(); });

  ASSERT_TRUE(wait_until([&] { return client->service_is_ready(); }, wait_timeout));

  std::atomic<int> received_count{0};
  std::atomic<int> success_count{0};

  for (int i = 0; i < kIterations; ++i) {
    auto request = client->create_request();
    as_request(request)->data = true;
    client->async_send_request(request, [&](GenericClient::SharedFuture fut) {
      if (as_response(fut.get())->success) {
        ++success_count;
      }
      ++received_count;
    });
  }

  const bool all_received =
    wait_until([&] { return received_count.load() == kIterations; }, std::chrono::seconds(30));

  executor->cancel();
  spin_thread.join();

  ASSERT_TRUE(all_received) << "only " << received_count.load() << "/" << kIterations
                            << " responses arrived -- at least one was dropped";
  EXPECT_EQ(success_count.load(), kIterations);
}

// ===== cancel_request()/cancel_response() + destructor cleanup =====
//
// The deferred path borrows a shared-memory buffer (Agnocast) or heap buffer (ROS2) that must
// eventually be sent or explicitly cancelled; on the Agnocast side, dropping an unsent borrowed
// buffer without cancelling it calls std::terminate() (see generic_client.hpp/generic_service.hpp
// doc comments on agnocast::ipc_shared_ptr<void>::reset()). Simply reaching the end of each test
// below without crashing the test binary is itself the primary assertion.

TEST_F(GenericServiceClientCancelTest, ClientCancelUnsentRequestThenNormalCallSucceeds)
{
  using autoware::agnocast_wrapper::Node;

  auto server_node = std::make_shared<Node>("gsc_cancel_client_server");
  auto client_node = std::make_shared<Node>("gsc_cancel_client_client");

  const auto service = server_node->create_generic_service(
    "/test/gsc_cancel_client", "std_srvs/srv/SetBool",
    [](std::shared_ptr<void> request, std::shared_ptr<void> response) {
      as_response(response)->success = as_request(request)->data;
    });
  const auto client =
    client_node->create_generic_client("/test/gsc_cancel_client", "std_srvs/srv/SetBool");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node->get_rclcpp_node());
  executor.add_node(client_node->get_rclcpp_node());
  ASSERT_TRUE(spin_until(executor, [&] { return client->service_is_ready(); }));

  // Create a request and abandon it via cancel_request() without ever sending it.
  auto abandoned_request = client->create_request();
  as_request(abandoned_request)->data = true;
  EXPECT_NO_THROW(client->cancel_request(abandoned_request));

  // The client, and the service it targets, must still work normally afterward.
  auto request = client->create_request();
  as_request(request)->data = true;
  std::atomic<bool> received{false};
  bool success = false;
  client->async_send_request(request, [&](GenericClient::SharedFuture fut) {
    success = as_response(fut.get())->success;
    received = true;
  });
  ASSERT_TRUE(spin_until(executor, [&] { return received.load(); }));
  EXPECT_TRUE(success);
}

TEST_F(GenericServiceClientCancelTest, ServiceCancelResponseThenNormalCallSucceeds)
{
  using autoware::agnocast_wrapper::Node;

  auto server_node = std::make_shared<Node>("gsc_cancel_service_server");
  auto client_node = std::make_shared<Node>("gsc_cancel_service_client");

  // data=false: create_response() then cancel_response() instead of sending. data=true: sent
  // normally. This exercises the exact deferred-abandon path cancel_response() exists for.
  const auto service = server_node->create_generic_service(
    "/test/gsc_cancel_service", "std_srvs/srv/SetBool",
    [](GenericService::SharedPtr service, std::shared_ptr<void> request) {
      auto response = service->create_response(request);
      if (as_request(request)->data) {
        as_response(response)->success = true;
        service->send_response(request, response);
      } else {
        service->cancel_response(request, response);
      }
    });
  const auto client =
    client_node->create_generic_client("/test/gsc_cancel_service", "std_srvs/srv/SetBool");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node->get_rclcpp_node());
  executor.add_node(client_node->get_rclcpp_node());
  ASSERT_TRUE(spin_until(executor, [&] { return client->service_is_ready(); }));

  // Fire-and-forget: the server cancels this response, so no reply is ever expected.
  {
    auto request = client->create_request();
    as_request(request)->data = false;
    client->async_send_request(request, [](GenericClient::SharedFuture fut) {
      FAIL() << "server cancelled this response; no reply should ever arrive";
      (void)fut;
    });
  }
  // Give the server a chance to process and cancel the request above before checking it survived.
  wait_until(
    [&] {
      executor.spin_some();
      return false;
    },
    std::chrono::milliseconds(200));

  // The service must still answer normally afterward.
  auto request = client->create_request();
  as_request(request)->data = true;
  std::atomic<bool> received{false};
  bool success = false;
  client->async_send_request(request, [&](GenericClient::SharedFuture fut) {
    success = as_response(fut.get())->success;
    received = true;
  });
  ASSERT_TRUE(spin_until(executor, [&] { return received.load(); }));
  EXPECT_TRUE(success);
}

TEST_F(GenericServiceClientCancelTest, ClientDestructorCleansUpAbandonedRequest)
{
  using autoware::agnocast_wrapper::Node;

  auto client_node = std::make_shared<Node>("gsc_cancel_client_dtor");

  // No server needed: the request is never sent, only borrowed and then abandoned by destroying
  // the client outright, without calling cancel_request() first.
  auto client =
    client_node->create_generic_client("/test/gsc_cancel_client_dtor", "std_srvs/srv/SetBool");
  auto abandoned_request = client->create_request();
  as_request(abandoned_request)->data = true;

  EXPECT_NO_THROW(client.reset());
}

TEST_F(GenericServiceClientCancelTest, ServiceDestructorCleansUpAbandonedResponse)
{
  using autoware::agnocast_wrapper::Node;

  auto server_node = std::make_shared<Node>("gsc_cancel_service_dtor_server");
  auto client_node = std::make_shared<Node>("gsc_cancel_service_dtor_client");

  GenericService::SharedPtr held_response_owner;
  auto service = server_node->create_generic_service(
    "/test/gsc_cancel_service_dtor", "std_srvs/srv/SetBool",
    [&](GenericService::SharedPtr service, std::shared_ptr<void> request) {
      // Deliberately create a response and never send or cancel it here: the deferred call stays
      // pending inside `service`, so destroying `service` below must clean it up itself.
      // held_response_owner also doubles as the signal the test polls on below, to know the
      // callback (and thus create_response()) actually ran before the test proceeds.
      auto response = service->create_response(request);
      (void)response;
      held_response_owner = service;
    });
  const auto client =
    client_node->create_generic_client("/test/gsc_cancel_service_dtor", "std_srvs/srv/SetBool");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node->get_rclcpp_node());
  executor.add_node(client_node->get_rclcpp_node());
  ASSERT_TRUE(spin_until(executor, [&] { return client->service_is_ready(); }));

  auto request = client->create_request();
  as_request(request)->data = true;
  client->async_send_request(request, [](GenericClient::SharedFuture fut) {
    FAIL() << "the service never sends or cancels this response; no reply should ever arrive";
    (void)fut;
  });
  ASSERT_TRUE(spin_until(executor, [&] { return held_response_owner != nullptr; }));

  // Destroy every remaining reference to the service, including the one this test itself was
  // holding, so the service's own destructor runs and must cancel the still-pending response.
  held_response_owner.reset();
  EXPECT_NO_THROW(service.reset());
}

// ===== Error paths =====

TEST_F(GenericServiceClientErrorTest, UnknownServiceTypeThrows)
{
  auto node = std::make_shared<GenericServiceClientMethod1Node>("gsc_unknown_type_node");

  // Both backends load service_type's typesupport library at construction (see the doc comments
  // on ROS2GenericService/AgnocastGenericService's underlying typesupport loader); a nonexistent
  // package name can't resolve on either.
  EXPECT_THROW(
    autoware::agnocast_wrapper::create_generic_service(
      static_cast<rclcpp::Node *>(node.get()), "/test/gsc_unknown_type",
      "no_such_package/srv/NoSuchType", [](std::shared_ptr<void>, std::shared_ptr<void>) {}),
    std::runtime_error);
  EXPECT_THROW(
    autoware::agnocast_wrapper::create_generic_client(
      static_cast<rclcpp::Node *>(node.get()), "/test/gsc_unknown_type_client",
      "no_such_package/srv/NoSuchType"),
    std::runtime_error);
}

TEST_F(GenericServiceClientErrorTest, SendResponseWithUnknownRequestThrows)
{
  auto server_node = std::make_shared<GenericServiceClientMethod1Node>("gsc_unknown_req_server");

  GenericService::SharedPtr service;
  service = server_node->create_bool_service(
    "/test/gsc_unknown_req", [](std::shared_ptr<void>, std::shared_ptr<void>) {});

  // A request never obtained from this service's own deferred callback must be rejected, not
  // silently accepted -- see send_response()'s doc comment on GenericService.
  auto foreign_request = std::make_shared<SetBool::Request>();
  auto foreign_response = std::make_shared<SetBool::Response>();
  EXPECT_THROW(service->send_response(foreign_request, foreign_response), std::runtime_error);
}

TEST_F(GenericServiceClientErrorTest, CreateResponseWithUnknownRequestThrows)
{
  auto server_node = std::make_shared<GenericServiceClientMethod1Node>("gsc_unknown_req_create");

  GenericService::SharedPtr service;
  service = server_node->create_bool_service(
    "/test/gsc_unknown_req_create", [](GenericService::SharedPtr, std::shared_ptr<void>) {});

  // Symmetric with SendResponseWithUnknownRequestThrows above: create_response() must reject a
  // request that was never handed to this service's own deferred callback, not just
  // send_response().
  auto foreign_request = std::make_shared<SetBool::Request>();
  EXPECT_THROW(service->create_response(foreign_request), std::runtime_error);
}

TEST_F(GenericServiceClientErrorTest, AsyncSendRequestWithUnknownRequestBackendSpecificBehavior)
{
  auto client_node = std::make_shared<GenericServiceClientMethod1Node>("gsc_unknown_req_client");

  // On Agnocast, a request not obtained from this client's own create_request() has no borrowed
  // shared-memory buffer to look up and must be rejected. On ROS2, the request is an ordinary
  // heap buffer sent by pointer, so any correctly-typed request is accepted -- matching plain
  // rclcpp's own usability rather than restricting it to mirror Agnocast (see GenericClient's
  // class-level "Limitation" note). No server is needed for the Agnocast case: the rejection
  // happens before anything is actually sent.
  auto client = client_node->create_bool_client("/test/gsc_unknown_req_async");
  auto foreign_request = std::make_shared<SetBool::Request>();
  if (autoware::agnocast_wrapper::use_agnocast()) {
    EXPECT_THROW(client->async_send_request(foreign_request), std::runtime_error);
  } else {
    EXPECT_NO_THROW(client->async_send_request(foreign_request));
  }
}

TEST_F(GenericServiceClientErrorTest, CancelRequestOnUnknownRequestDoesNotThrow)
{
  auto client_node = std::make_shared<GenericServiceClientMethod1Node>("gsc_cancel_unknown_client");

  // cancel_request()'s doc comment promises tolerance of a request this client never issued (so
  // error-handling code can call it unconditionally, without its own try/catch); a request that
  // was never even obtained from create_request() -- as opposed to one already sent or cancelled
  // -- is the strongest form of that same guarantee.
  auto client = client_node->create_bool_client("/test/gsc_cancel_unknown");
  auto foreign_request = std::make_shared<SetBool::Request>();
  EXPECT_NO_THROW(client->cancel_request(foreign_request));
}

TEST_F(GenericServiceClientErrorTest, CancelResponseOnUnknownRequestDoesNotThrow)
{
  auto server_node = std::make_shared<GenericServiceClientMethod1Node>("gsc_cancel_unknown_server");

  // Same guarantee as CancelRequestOnUnknownRequestDoesNotThrow above, for the service side.
  GenericService::SharedPtr service;
  service = server_node->create_bool_service(
    "/test/gsc_cancel_unknown_service", [](GenericService::SharedPtr, std::shared_ptr<void>) {});
  auto foreign_request = std::make_shared<SetBool::Request>();
  auto foreign_response = std::make_shared<SetBool::Response>();
  EXPECT_NO_THROW(service->cancel_response(foreign_request, foreign_response));
}

TEST_F(GenericServiceClientErrorTest, CreateResponseCalledTwiceBackendSpecificBehavior)
{
  using autoware::agnocast_wrapper::Node;

  auto server_node = std::make_shared<Node>("gsc_double_create_response_server");
  auto client_node = std::make_shared<Node>("gsc_double_create_response_client");

  // On Agnocast, the response is a single borrowed shared-memory buffer, so a second
  // create_response() call for the same still-pending request must be rejected rather than
  // silently overwriting (and terminating the process on) the first borrow. On ROS2 the response
  // is an ordinary heap buffer, so a second call is harmless and simply yields another
  // independent buffer -- matching plain rclcpp's own permissiveness. See GenericService's
  // class-level "Limitations" note. Either way, the deferred call must still be answerable
  // normally afterward -- the rejected/extra second call must not corrupt the first response.
  const auto service = server_node->create_generic_service(
    "/test/gsc_double_create_response", "std_srvs/srv/SetBool",
    [](GenericService::SharedPtr service, std::shared_ptr<void> request) {
      auto response = service->create_response(request);
      if (autoware::agnocast_wrapper::use_agnocast()) {
        EXPECT_THROW(service->create_response(request), std::runtime_error);
      } else {
        EXPECT_NO_THROW(service->create_response(request));
      }
      as_response(response)->success = as_request(request)->data;
      service->send_response(request, response);
    });
  const auto client =
    client_node->create_generic_client("/test/gsc_double_create_response", "std_srvs/srv/SetBool");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node->get_rclcpp_node());
  executor.add_node(client_node->get_rclcpp_node());
  ASSERT_TRUE(spin_until(executor, [&] { return client->service_is_ready(); }));

  auto request = client->create_request();
  as_request(request)->data = true;
  std::atomic<bool> received{false};
  bool success = false;
  client->async_send_request(request, [&](GenericClient::SharedFuture fut) {
    success = as_response(fut.get())->success;
    received = true;
  });
  ASSERT_TRUE(spin_until(executor, [&] { return received.load(); }));
  EXPECT_TRUE(success);
}

TEST_F(GenericServiceClientErrorTest, SendResponseWithWrongResponseObjectBackendSpecificBehavior)
{
  using autoware::agnocast_wrapper::Node;

  auto server_node = std::make_shared<Node>("gsc_wrong_response_server");
  auto client_node = std::make_shared<Node>("gsc_wrong_response_client");

  // On Agnocast, send_response()'s @p response must be the exact object this service's own
  // create_response(request) returned: the shared-memory channel can only publish the buffer it
  // lent out, so passing back a different (if correctly-typed) response must be rejected rather
  // than silently substituted. On ROS2, the response is an ordinary heap buffer, so any
  // correctly-typed object is accepted -- matching plain rclcpp's own usability. See
  // GenericService's class-level "Limitations" note. The Agnocast rejection must itself be clean
  // (an exception, not a crash from dropping the still-unpublished correct response) and must
  // leave the service able to answer a later call normally.
  const auto service = server_node->create_generic_service(
    "/test/gsc_wrong_response", "std_srvs/srv/SetBool",
    [](GenericService::SharedPtr service, std::shared_ptr<void> request) {
      auto correct_response = service->create_response(request);
      auto wrong_response = std::make_shared<SetBool::Response>();
      as_response(wrong_response)->success = as_request(request)->data;
      if (autoware::agnocast_wrapper::use_agnocast()) {
        EXPECT_THROW(service->send_response(request, wrong_response), std::runtime_error);
      } else {
        EXPECT_NO_THROW(service->send_response(request, wrong_response));
      }
    });
  const auto client =
    client_node->create_generic_client("/test/gsc_wrong_response", "std_srvs/srv/SetBool");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node->get_rclcpp_node());
  executor.add_node(client_node->get_rclcpp_node());
  ASSERT_TRUE(spin_until(executor, [&] { return client->service_is_ready(); }));

  // Fire-and-forget on Agnocast (the rejected send_response() means no reply ever arrives there);
  // on ROS2 the wrong_response object is what actually gets sent.
  auto request = client->create_request();
  as_request(request)->data = true;
  std::atomic<bool> received{false};
  bool success = false;
  client->async_send_request(request, [&](GenericClient::SharedFuture fut) {
    success = as_response(fut.get())->success;
    received = true;
  });
  if (autoware::agnocast_wrapper::use_agnocast()) {
    wait_until(
      [&] {
        executor.spin_some();
        return false;
      },
      std::chrono::milliseconds(200));
    EXPECT_FALSE(received.load());
  } else {
    ASSERT_TRUE(spin_until(executor, [&] { return received.load(); }));
    EXPECT_TRUE(success);
  }

  // The service must still be able to answer a fresh, correctly-formed request afterward -- the
  // rejected call above must not have corrupted its internal state.
  auto second_service = server_node->create_generic_service(
    "/test/gsc_wrong_response_followup", "std_srvs/srv/SetBool",
    [](std::shared_ptr<void> req, std::shared_ptr<void> resp) {
      as_response(resp)->success = as_request(req)->data;
    });
  const auto second_client =
    client_node->create_generic_client("/test/gsc_wrong_response_followup", "std_srvs/srv/SetBool");
  ASSERT_TRUE(spin_until(executor, [&] { return second_client->service_is_ready(); }));
  auto second_request = second_client->create_request();
  as_request(second_request)->data = true;
  std::atomic<bool> second_received{false};
  bool second_success = false;
  second_client->async_send_request(second_request, [&](GenericClient::SharedFuture fut) {
    second_success = as_response(fut.get())->success;
    second_received = true;
  });
  ASSERT_TRUE(spin_until(executor, [&] { return second_received.load(); }));
  EXPECT_TRUE(second_success);
}

}  // namespace
