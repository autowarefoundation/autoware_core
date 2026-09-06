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
// here is the wrapper's own: the surface it presents in both builds, and the argument it rejects
// itself.

#include "autoware/agnocast_wrapper/parameter_client.hpp"

#include "autoware/agnocast_wrapper/node.hpp"
#include "autoware/agnocast_wrapper/runtime.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace
{

using autoware::agnocast_wrapper::AsyncParametersClient;
using autoware::agnocast_wrapper::Node;

static_assert(!std::is_copy_constructible_v<AsyncParametersClient>);
static_assert(!std::is_copy_assignable_v<AsyncParametersClient>);
static_assert(!std::is_move_constructible_v<AsyncParametersClient>);
static_assert(!std::is_move_assignable_v<AsyncParametersClient>);

/// Same probe as test/cases/polling_subscriber.cpp: agnocast exits the process from inside the
/// constructor when LD_PRELOAD lacks the heaphook, which would take the whole test binary down
/// instead of failing one case.
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

TEST_F(AsyncParametersClientTest, RejectsATransientLocalQos)
{
  // Arrange
  const auto node = std::make_shared<Node>("parameter_client_transient_local");

  // Act & Assert
  EXPECT_THROW(
    AsyncParametersClient(node.get(), "no_such_node", rclcpp::ParametersQoS().transient_local()),
    std::invalid_argument);
}

TEST_F(AsyncParametersClientTest, AcceptsAVolatileQos)
{
  // Arrange
  const auto node = std::make_shared<Node>("parameter_client_volatile");

  // Act & Assert
  EXPECT_NO_THROW(AsyncParametersClient(
    node.get(), "no_such_node", rclcpp::ParametersQoS().durability_volatile()));
}

}  // namespace
