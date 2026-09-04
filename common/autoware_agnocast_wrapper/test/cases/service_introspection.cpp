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

// configure_introspection() only exists on rclcpp 21 (Iron) and newer, so on Humble every case
// here reports a skip rather than disappearing: the test list should not change with the distro.

#include "autoware/agnocast_wrapper/client.hpp"
#include "autoware/agnocast_wrapper/macros.hpp"
#include "autoware/agnocast_wrapper/node.hpp"
#include "autoware/agnocast_wrapper/runtime.hpp"
#include "autoware/agnocast_wrapper/service.hpp"

#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <rclcpp/version.h>

#include <cstdlib>
#include <memory>
#include <string>

namespace
{

using autoware::agnocast_wrapper::Node;
using ListParameters = rcl_interfaces::srv::ListParameters;

/// Same probe as test/cases/polling_subscriber.cpp: agnocast exits the process from inside the
/// endpoint constructor when LD_PRELOAD lacks the heaphook, which would take the whole test
/// binary down instead of failing one case.
bool agnocast_heaphook_loaded()
{
  const char * ld_preload = std::getenv("LD_PRELOAD");
  return ld_preload != nullptr &&
         std::string(ld_preload).find("libagnocast_heaphook.so") != std::string::npos;
}

class ServiceIntrospectionTest : public testing::Test
{
protected:
  void SetUp() override
  {
    if (autoware::agnocast_wrapper::use_agnocast() && !agnocast_heaphook_loaded()) {
      GTEST_SKIP() << "ENABLE_AGNOCAST=1 without the agnocast heaphook: the agnocast backend "
                      "cannot be exercised in this environment.";
    }
#if RCLCPP_VERSION_MAJOR < 21
    GTEST_SKIP() << "rclcpp " << RCLCPP_VERSION_MAJOR
                 << " has no service introspection, so configure_introspection() is not declared "
                    "on the wrapper handles either.";
#endif
  }
};

// The arguments below are the ones autoware_component_interface_utils passes: the node's clock,
// rclcpp::QoS(1), and the state it resolved from the
// "component_interface.service_introspection" parameter. The handles are held through the
// abstract base, which is the point -- that is the type callers deduce from create_client() and
// create_service().
TEST_F(ServiceIntrospectionTest, ClientAcceptsEveryState)
{
#if RCLCPP_VERSION_MAJOR >= 21
  const auto node = std::make_shared<Node>("introspected_client");
  AUTOWARE_CLIENT_PTR(ListParameters) client = node->create_client<ListParameters>("~/introspect");

  EXPECT_NO_THROW(client->configure_introspection(
    node->get_clock(), rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_METADATA));
  EXPECT_NO_THROW(client->configure_introspection(
    node->get_clock(), rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_CONTENTS));
  EXPECT_NO_THROW(client->configure_introspection(
    node->get_clock(), rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_OFF));
#endif
}

TEST_F(ServiceIntrospectionTest, ServiceAcceptsEveryState)
{
#if RCLCPP_VERSION_MAJOR >= 21
  const auto node = std::make_shared<Node>("introspected_service");
  AUTOWARE_SERVICE_PTR(ListParameters)
  service = node->create_service<ListParameters>(
    "~/introspect", [](
                      AUTOWARE_SERVER_REQUEST_PTR(ListParameters) &&,
                      AUTOWARE_SERVER_RESPONSE_PTR(ListParameters) &&) {});

  EXPECT_NO_THROW(service->configure_introspection(
    node->get_clock(), rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_METADATA));
  EXPECT_NO_THROW(service->configure_introspection(
    node->get_clock(), rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_CONTENTS));
  EXPECT_NO_THROW(service->configure_introspection(
    node->get_clock(), rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_OFF));
#endif
}

}  // namespace
