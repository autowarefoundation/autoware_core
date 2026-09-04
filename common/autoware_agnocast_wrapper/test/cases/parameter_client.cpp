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
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

namespace
{

using autoware::agnocast_wrapper::AsyncParametersClient;
using autoware::agnocast_wrapper::Node;

/// Same probe as test/cases/polling_subscriber.cpp: agnocast exits the process from inside the
/// subscription constructor when LD_PRELOAD lacks the heaphook, which would take the whole test
/// binary down instead of failing one case.
bool agnocast_heaphook_loaded()
{
  const char * ld_preload = std::getenv("LD_PRELOAD");
  return ld_preload != nullptr &&
         std::string(ld_preload).find("libagnocast_heaphook.so") != std::string::npos;
}

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

TEST_F(AsyncParametersClientTest, NeitherCopyableNorMovable)
{
  static_assert(!std::is_copy_constructible_v<AsyncParametersClient>);
  static_assert(!std::is_copy_assignable_v<AsyncParametersClient>);
  static_assert(!std::is_move_constructible_v<AsyncParametersClient>);
  static_assert(!std::is_move_assignable_v<AsyncParametersClient>);
}

TEST_F(AsyncParametersClientTest, WaitForServiceTimesOutForAnAbsentRemoteNode)
{
  const auto node = std::make_shared<Node>("parameter_client_timeout");
  AsyncParametersClient client(node.get(), "no_such_node");

  EXPECT_FALSE(client.wait_for_service(std::chrono::milliseconds(200)));
}

TEST_F(AsyncParametersClientTest, GetParametersReadsARemoteNode)
{
  if (autoware::agnocast_wrapper::use_agnocast()) {
    GTEST_SKIP() << "the agnocast backend is served by an agnocast executor, and "
                    "get_rclcpp_node() throws in agnocast mode.";
  }

  const auto server = std::make_shared<Node>("parameter_client_server");
  server->declare_parameter<bool>("enable_partial_load", true);

  const auto node = std::make_shared<Node>("parameter_client_reader");
  AsyncParametersClient client(node.get(), "parameter_client_server");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server->get_node_base_interface());
  executor.add_node(node->get_node_base_interface());
  std::thread spin_thread([&executor] { executor.spin(); });

  ASSERT_TRUE(client.wait_for_service(std::chrono::seconds(10)));

  bool callback_ran = false;
  const auto future = client.get_parameters(
    {"enable_partial_load"},
    [&callback_ran](std::shared_future<std::vector<rclcpp::Parameter>>) { callback_ran = true; });
  ASSERT_EQ(future.wait_for(std::chrono::seconds(10)), std::future_status::ready);

  const auto parameters = future.get();
  ASSERT_EQ(parameters.size(), 1u);
  EXPECT_TRUE(parameters.front().as_bool());
  EXPECT_TRUE(callback_ran);

  executor.cancel();
  spin_thread.join();
}

}  // namespace
