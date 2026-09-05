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

#include "autoware/agnocast_wrapper/parameter_client.hpp"

#include "autoware/agnocast_wrapper/node.hpp"
#include "autoware/agnocast_wrapper/runtime.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

namespace
{

using autoware::agnocast_wrapper::AsyncParametersClient;
using autoware::agnocast_wrapper::Node;

// Compile-time contract, so there is nothing for a TEST body to check at run time.
static_assert(!std::is_copy_constructible_v<AsyncParametersClient>);
static_assert(!std::is_copy_assignable_v<AsyncParametersClient>);
static_assert(!std::is_move_constructible_v<AsyncParametersClient>);
static_assert(!std::is_move_assignable_v<AsyncParametersClient>);

/// Same probe as test/cases/polling_subscriber.cpp: agnocast exits the process from inside the
/// subscription constructor when LD_PRELOAD lacks the heaphook, which would take the whole test
/// binary down instead of failing one case.
bool agnocast_heaphook_loaded()
{
  const char * ld_preload = std::getenv("LD_PRELOAD");
  return ld_preload != nullptr &&
         std::string(ld_preload).find("libagnocast_heaphook.so") != std::string::npos;
}

/// Stops the executor from the destructor, so an ASSERT_* that returns early out of a test does
/// not leave a joinable std::thread behind — that would call std::terminate and take the whole
/// test binary with it instead of failing the one case.
class SpinThread
{
public:
  explicit SpinThread(rclcpp::Executor & executor)
  : executor_(executor), thread_([&executor] { executor.spin(); })
  {
  }

  ~SpinThread()
  {
    executor_.cancel();
    thread_.join();
  }

  SpinThread(const SpinThread &) = delete;
  SpinThread & operator=(const SpinThread &) = delete;
  SpinThread(SpinThread &&) = delete;
  SpinThread & operator=(SpinThread &&) = delete;

private:
  rclcpp::Executor & executor_;
  std::thread thread_;
};

class AsyncParametersClientTest : public testing::Test
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

/// The constructor's name check, exercised directly. Going through the constructor would not pin
/// it: the rclcpp backend rejects the same names on its own, and with the same exception type, so
/// only the agnocast backend can tell a missing check from a working one -- and that backend needs
/// the kernel module to run at all. These cases hold in every build.
TEST(AsyncParametersClientNameCheck, RejectsAMalformedRemoteName)
{
  EXPECT_THROW(
    autoware::agnocast_wrapper::detail::checked_remote_node_name("bad name!", "n", "/"),
    rclcpp::exceptions::InvalidServiceNameError);
}

TEST(AsyncParametersClientNameCheck, RejectsARemoteNameTooLongForEveryParameterService)
{
  // rmw_validate_full_topic_name() rejects a name longer than RMW_TOPIC_MAX_NAME_LENGTH (247).
  // This one measures 246 with "/get_parameters" -- the shortest of the six parameter services --
  // and 257 with the longest, "/set_parameters_atomically". Checking anything but the longest
  // leaves the rejection to the backend, which does not throw the same type on both.
  const std::string remote_node_name = "/" + std::string(230, 'a');

  EXPECT_THROW(
    autoware::agnocast_wrapper::detail::checked_remote_node_name(remote_node_name, "n", "/"),
    rclcpp::exceptions::InvalidServiceNameError);
}

TEST(AsyncParametersClientNameCheck, AcceptsANameThatFitsEveryParameterService)
{
  const std::string remote_node_name = "/" + std::string(200, 'a');

  EXPECT_NO_THROW(
    autoware::agnocast_wrapper::detail::checked_remote_node_name(remote_node_name, "n", "/"));
}

TEST_F(AsyncParametersClientTest, WaitForServiceTimesOutForAnAbsentRemoteNode)
{
  const auto node = std::make_shared<Node>("parameter_client_timeout");
  AsyncParametersClient client(node.get(), "no_such_node");

  constexpr auto timeout = std::chrono::milliseconds(200);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(client.wait_for_service(timeout));

  // Both backends have to spend the timeout. Only the agnocast one can fail this: without the
  // wrapper's polling wait it returns after a single probe, because test/main.cpp brings up an
  // rclcpp context and not an AgnocastOnly one, so `agnocast::ok()` is false. Half the timeout
  // separates "waited" from "did not wait" without pinning how a backend accounts for the budget.
  EXPECT_GE(std::chrono::steady_clock::now() - start, timeout / 2);
}

TEST_F(AsyncParametersClientTest, WaitForServiceWithAZeroTimeoutDoesNotBlock)
{
  const auto node = std::make_shared<Node>("parameter_client_zero_timeout");
  AsyncParametersClient client(node.get(), "no_such_node");

  constexpr auto budget = std::chrono::milliseconds(100);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(client.wait_for_service(std::chrono::nanoseconds::zero()));

  EXPECT_LT(std::chrono::steady_clock::now() - start, budget);
}

TEST_F(AsyncParametersClientTest, ServiceIsNotReadyForAnAbsentRemoteNode)
{
  const auto node = std::make_shared<Node>("parameter_client_not_ready");
  AsyncParametersClient client(node.get(), "no_such_node");

  EXPECT_FALSE(client.service_is_ready());
}

TEST_F(AsyncParametersClientTest, RejectsARemoteNameThatCannotFormAServiceName)
{
  const auto node = std::make_shared<Node>("parameter_client_bad_name");

  EXPECT_THROW(
    AsyncParametersClient(node.get(), "bad name!"), rclcpp::exceptions::InvalidServiceNameError);
}

TEST_F(AsyncParametersClientTest, AcceptsAnEmptyRemoteNameAsThisNode)
{
  const auto node = std::make_shared<Node>("parameter_client_self");

  // The empty default is the "this node" case: the backends fill it in with the node's own fully
  // qualified name. (The constructor's early-out for it is only a shortcut -- the bare suffix is a
  // valid service name on its own -- so this pins the contract, not that branch.)
  EXPECT_NO_THROW(AsyncParametersClient(node.get()));
}

TEST_F(AsyncParametersClientTest, GetParametersReadsARemoteNode)
{
  if (autoware::agnocast_wrapper::use_agnocast()) {
    GTEST_SKIP() << "the agnocast backend is served by an agnocast executor, which this test "
                    "does not spin.";
  }

  const auto server = std::make_shared<Node>("parameter_client_server");
  server->declare_parameter<bool>("enable_partial_load", true);

  const auto node = std::make_shared<Node>("parameter_client_reader");
  AsyncParametersClient client(node.get(), "parameter_client_server");

  // Both backends satisfy the promise before they invoke the callback, so a resolved future says
  // nothing about whether the callback has run yet. Declared before the spin thread: the callback
  // holds a reference to it, and members are destroyed in reverse declaration order, so the
  // executor has to stop first.
  std::promise<void> callback_done;
  const auto callback_ran = callback_done.get_future();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server->get_node_base_interface());
  executor.add_node(node->get_node_base_interface());
  SpinThread spin(executor);

  ASSERT_TRUE(client.wait_for_service(std::chrono::seconds(10)));
  EXPECT_TRUE(client.service_is_ready());

  const auto future = client.get_parameters(
    {"enable_partial_load"}, [&callback_done](std::shared_future<std::vector<rclcpp::Parameter>>) {
      callback_done.set_value();
    });
  ASSERT_EQ(future.wait_for(std::chrono::seconds(10)), std::future_status::ready);

  const auto parameters = future.get();
  ASSERT_EQ(parameters.size(), 1u);
  EXPECT_TRUE(parameters.front().as_bool());
  EXPECT_EQ(callback_ran.wait_for(std::chrono::seconds(10)), std::future_status::ready);
}

}  // namespace
