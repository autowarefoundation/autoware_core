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

// Exercises the generic (type-erased) service/client surface of agnocast_wrapper::Node end to
// end: a request goes out through create_generic_client(), is answered through
// create_generic_service() — immediately or after the callback has returned — and comes back as
// the typed response. The nodes are spun by an rclcpp executor, so every test below is skipped
// when the Agnocast backend is selected at runtime; the Agnocast backend needs an Agnocast
// executor and is exercised by the nodes that use it.

#include "autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp"
#include "autoware/agnocast_wrapper/node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

using autoware::agnocast_wrapper::GenericClient;
using autoware::agnocast_wrapper::GenericService;
using autoware::agnocast_wrapper::Node;
using SetBool = std_srvs::srv::SetBool;

constexpr auto discovery_timeout = std::chrono::seconds(10);
constexpr auto poll_interval = std::chrono::milliseconds(10);
constexpr auto set_bool_type = "std_srvs/srv/SetBool";

class GenericServiceTest : public testing::Test
{
protected:
  void SetUp() override
  {
    if (autoware::agnocast_wrapper::use_agnocast()) {
      GTEST_SKIP() << "These tests spin the nodes with an rclcpp executor.";
    }
    server_node_ = std::make_shared<Node>("generic_service_test_server");
    client_node_ = std::make_shared<Node>("generic_service_test_client");
    executor_.add_node(server_node_->get_node_base_interface());
    executor_.add_node(client_node_->get_node_base_interface());
  }

  /// Spin both nodes until @p done holds or the timeout passes.
  bool spin_until(const std::function<bool()> & done)
  {
    const auto deadline = std::chrono::steady_clock::now() + discovery_timeout;
    while (!done() && std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(poll_interval);
    }
    return done();
  }

  bool wait_for_server(const GenericClient::SharedPtr & client)
  {
    return spin_until([&client]() { return client->service_is_ready(); });
  }

  std::shared_ptr<Node> server_node_;
  std::shared_ptr<Node> client_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

/// Answers every request from inside the callback, echoing request.data.
GenericService::Callback echo_callback(std::vector<bool> & received)
{
  return [&received](
           const GenericService::SharedPtr & service, const GenericService::RequestPtr & request) {
    const auto & typed = *static_cast<const SetBool::Request *>(request->get());
    received.push_back(typed.data);
    SetBool::Response response;
    response.success = typed.data;
    response.message = typed.data ? "on" : "off";
    service->send_response(request, &response);
  };
}

TEST_F(GenericServiceTest, RoundTrip)
{
  std::vector<bool> received;
  const auto service = server_node_->create_generic_service(
    "/test/generic_service/round_trip", set_bool_type, echo_callback(received));
  const auto client =
    client_node_->create_generic_client("/test/generic_service/round_trip", set_bool_type);
  ASSERT_TRUE(wait_for_server(client));

  SetBool::Request request;
  request.data = true;
  const auto future = client->async_send_request(&request);
  ASSERT_TRUE(spin_until(
    [&future]() { return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; }));

  const auto & response = *static_cast<const SetBool::Response *>(future.get().get());
  EXPECT_EQ(received, std::vector<bool>{true});
  EXPECT_TRUE(response.success);
  EXPECT_EQ(response.message, "on");
}

TEST_F(GenericServiceTest, ClientCallbackReceivesTheFuture)
{
  std::vector<bool> received;
  const auto service = server_node_->create_generic_service(
    "/test/generic_service/callback", set_bool_type, echo_callback(received));
  const auto client =
    client_node_->create_generic_client("/test/generic_service/callback", set_bool_type);
  ASSERT_TRUE(wait_for_server(client));

  std::string message;
  SetBool::Request request;
  request.data = false;
  client->async_send_request(&request, [&message](GenericClient::SharedFuture future) {
    message = static_cast<const SetBool::Response *>(future.get().get())->message;
  });
  ASSERT_TRUE(spin_until([&message]() { return !message.empty(); }));
  EXPECT_EQ(message, "off");
}

// The divider-style use: hold the request, answer it after the callback has returned.
TEST_F(GenericServiceTest, DeferredResponse)
{
  std::mutex mutex;
  GenericService::SharedPtr held_service;
  GenericService::RequestPtr held_request;
  const auto service = server_node_->create_generic_service(
    "/test/generic_service/deferred", set_bool_type,
    [&](const GenericService::SharedPtr & svc, const GenericService::RequestPtr & request) {
      std::lock_guard<std::mutex> lock(mutex);
      held_service = svc;
      held_request = request;
    });
  const auto client =
    client_node_->create_generic_client("/test/generic_service/deferred", set_bool_type);
  ASSERT_TRUE(wait_for_server(client));

  SetBool::Request request;
  request.data = true;
  const auto future = client->async_send_request(&request);
  ASSERT_TRUE(spin_until([&]() {
    std::lock_guard<std::mutex> lock(mutex);
    return held_request != nullptr;
  }));
  EXPECT_NE(future.wait_for(std::chrono::seconds(0)), std::future_status::ready);

  SetBool::Response response;
  response.success = true;
  response.message = "deferred";
  held_service->send_response(held_request, &response);
  ASSERT_TRUE(spin_until(
    [&future]() { return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; }));
  EXPECT_EQ(static_cast<const SetBool::Response *>(future.get().get())->message, "deferred");
}

TEST_F(GenericServiceTest, SecondResponseToTheSameRequestThrows)
{
  bool threw = false;
  const auto service = server_node_->create_generic_service(
    "/test/generic_service/twice", set_bool_type,
    [&threw](const GenericService::SharedPtr & svc, const GenericService::RequestPtr & request) {
      SetBool::Response response;
      svc->send_response(request, &response);
      try {
        svc->send_response(request, &response);
      } catch (const std::runtime_error &) {
        threw = true;
      }
    });
  const auto client =
    client_node_->create_generic_client("/test/generic_service/twice", set_bool_type);
  ASSERT_TRUE(wait_for_server(client));

  SetBool::Request request;
  const auto future = client->async_send_request(&request);
  ASSERT_TRUE(spin_until(
    [&future]() { return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; }));
  EXPECT_TRUE(threw);
}

TEST_F(GenericServiceTest, ServiceAndClientReportTheResolvedName)
{
  std::vector<bool> received;
  const auto service =
    server_node_->create_generic_service("~/name", set_bool_type, echo_callback(received));
  const auto client = client_node_->create_generic_client("relative/name", set_bool_type);
  EXPECT_STREQ(service->get_service_name(), "/generic_service_test_server/name");
  EXPECT_STREQ(client->get_service_name(), "/relative/name");
}

TEST_F(GenericServiceTest, UnknownServiceTypeThrows)
{
  EXPECT_THROW(
    server_node_->create_generic_service(
      "/test/generic_service/unknown", "std_srvs/srv/DoesNotExist",
      [](const GenericService::SharedPtr &, const GenericService::RequestPtr &) {}),
    std::runtime_error);
  EXPECT_THROW(
    client_node_->create_generic_client(
      "/test/generic_service/unknown", "std_srvs/srv/DoesNotExist"),
    std::runtime_error);
}

TEST_F(GenericServiceTest, MalformedServiceTypeThrows)
{
  EXPECT_THROW(
    server_node_->create_generic_service(
      "/test/generic_service/malformed", "SetBool",
      [](const GenericService::SharedPtr &, const GenericService::RequestPtr &) {}),
    std::runtime_error);
  EXPECT_THROW(
    client_node_->create_generic_client("/test/generic_service/malformed", "SetBool"),
    std::runtime_error);
}

}  // namespace
