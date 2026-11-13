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

// What each backend does with a request is covered by that backend's own tests, so what is left
// here is the wrapper's own: the surface it presents in both builds, the arguments it rejects
// itself, and that each member reaches its backend at all.

#include "autoware/agnocast_wrapper/parameter_client.hpp"

#include "autoware/agnocast_wrapper/node.hpp"
#include "heaphook_probe.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <type_traits>

namespace
{

using autoware::agnocast_wrapper::AsyncParametersClient;
using autoware::agnocast_wrapper::Node;

static_assert(!std::is_copy_constructible_v<AsyncParametersClient>);
static_assert(!std::is_copy_assignable_v<AsyncParametersClient>);
static_assert(!std::is_move_constructible_v<AsyncParametersClient>);
static_assert(!std::is_move_assignable_v<AsyncParametersClient>);

class AsyncParametersClientTest : public testing::Test
{
protected:
  void SetUp() override { AUTOWARE_SKIP_WITHOUT_AGNOCAST_HEAPHOOK(); }
};

TEST_F(AsyncParametersClientTest, RejectsATransientLocalQos)
{
  // Arrange
  const auto node = std::make_shared<Node>("parameter_client_transient_local");

  // Act & Assert
  EXPECT_THROW(
    AsyncParametersClient(node.get(), "no_such_node", rclcpp::ParametersQoS().transient_local()),
    std::invalid_argument);
}

TEST_F(AsyncParametersClientTest, RejectsABestEffortQos)
{
  // Arrange
  const auto node = std::make_shared<Node>("parameter_client_best_effort");

  // Act & Assert
  EXPECT_THROW(
    AsyncParametersClient(node.get(), "no_such_node", rclcpp::ParametersQoS().best_effort()),
    std::invalid_argument);
}

TEST_F(AsyncParametersClientTest, AcceptsAVolatileReliableQos)
{
  // Arrange
  const auto node = std::make_shared<Node>("parameter_client_volatile");

  // Act & Assert
  EXPECT_NO_THROW(AsyncParametersClient(
    node.get(), "no_such_node", rclcpp::ParametersQoS().durability_volatile().reliable()));
}

// wait_for_service() is a member template, so without a call nothing instantiates it and a typo in
// either build's forwarding would still compile. An absent remote node answers both readiness
// queries the same way on either backend.
TEST_F(AsyncParametersClientTest, ReachesTheBackendForAnAbsentRemoteNode)
{
  // Arrange
  const auto node = std::make_shared<Node>("parameter_client_absent_remote");
  AsyncParametersClient client(node.get(), "no_such_node");

  // Act & Assert
  EXPECT_FALSE(client.service_is_ready());
  EXPECT_FALSE(client.wait_for_service(std::chrono::milliseconds(0)));
}

}  // namespace
