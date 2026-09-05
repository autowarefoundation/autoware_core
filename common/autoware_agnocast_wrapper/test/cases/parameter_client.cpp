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

TEST_F(AsyncParametersClientTest, WaitForServiceTimesOutForAnAbsentRemoteNode)
{
  const auto node = std::make_shared<Node>("parameter_client_timeout");
  AsyncParametersClient client(node.get(), "no_such_node");

  constexpr auto timeout = std::chrono::milliseconds(200);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(client.wait_for_service(timeout));
  const auto elapsed = std::chrono::steady_clock::now() - start;

  // Half the timeout only has to separate "waited" from "did not wait", which is the whole of the
  // documented divergence; it does not pin how the backends account for the budget.
  if (autoware::agnocast_wrapper::use_agnocast()) {
    // test/main.cpp brings up an rclcpp context, not an AgnocastOnly one, so agnocast::ok() is
    // false and the agnocast backend returns after one non-blocking readiness probe.
    EXPECT_LT(elapsed, timeout / 2);
  } else {
    EXPECT_GE(elapsed, timeout / 2);
  }
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

  // The empty default skips the constructor's name check; the backends fill it in with the node's
  // own fully qualified name.
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
