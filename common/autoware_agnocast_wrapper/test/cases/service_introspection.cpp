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
#include "heaphook_probe.hpp"

#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <rclcpp/version.h>

#include <memory>
#include <stdexcept>

namespace
{

using autoware::agnocast_wrapper::Node;
using autoware::agnocast_wrapper::test::agnocast_heaphook_loaded;
using ListParameters = rcl_interfaces::srv::ListParameters;

/// The handles are held through the abstract base because that is the type callers deduce from
/// create_client() and create_service(). They take different names so that neither depends on the
/// node answering its own service.
class ServiceIntrospectionTest : public testing::Test
{
protected:
  void SetUp() override
  {
#if !RCLCPP_VERSION_GTE(21, 0, 0)
    GTEST_SKIP() << "rclcpp " << RCLCPP_VERSION_MAJOR
                 << " has no service introspection, so configure_introspection() is not declared "
                    "on the wrapper handles either.";
#else
    if (autoware::agnocast_wrapper::use_agnocast() && !agnocast_heaphook_loaded()) {
      GTEST_SKIP() << "ENABLE_AGNOCAST=1 without the agnocast heaphook: the agnocast backend "
                      "cannot be exercised in this environment.";
    }
    node_ = std::make_shared<Node>("service_introspection");
    client_ = node_->create_client<ListParameters>("~/introspected_client");
    service_ = node_->create_service<ListParameters>(
      "~/introspected_service", [](
                                  AUTOWARE_SERVER_REQUEST_PTR(ListParameters) &&,
                                  AUTOWARE_SERVER_RESPONSE_PTR(ListParameters) &&) {});
#endif
  }

  std::shared_ptr<Node> node_;
  AUTOWARE_CLIENT_PTR(ListParameters) client_;
  AUTOWARE_SERVICE_PTR(ListParameters) service_;
};

TEST_F(ServiceIntrospectionTest, AcceptsEveryState)
{
#if RCLCPP_VERSION_GTE(21, 0, 0)
  // Act: the states in this order walk every transition once -- OFF to METADATA creates the event
  // publisher, METADATA to CONTENTS keeps it, CONTENTS to OFF destroys it.
  const auto walk_every_state = [&](const auto & handle) {
    for (const auto state :
         {RCL_SERVICE_INTROSPECTION_METADATA, RCL_SERVICE_INTROSPECTION_CONTENTS,
          RCL_SERVICE_INTROSPECTION_OFF}) {
      handle->configure_introspection(node_->get_clock(), rclcpp::QoS(1), state);
    }
  };

  // Assert
  EXPECT_NO_THROW(walk_every_state(client_));
  EXPECT_NO_THROW(walk_every_state(service_));
#endif
}

// The handle rejects a null clock before it dispatches, so this pins that check and not the
// forwarding: no case in this file would fail if a backend override did nothing.
TEST_F(ServiceIntrospectionTest, RejectsNullClock)
{
#if RCLCPP_VERSION_GTE(21, 0, 0)
  // Act
  const auto configure_with_null_clock =
    [&](const auto & handle, rcl_service_introspection_state_t state) {
      handle->configure_introspection(nullptr, rclcpp::QoS(1), state);
    };

  // Assert: rejected in every state, OFF included.
  EXPECT_THROW(
    configure_with_null_clock(client_, RCL_SERVICE_INTROSPECTION_CONTENTS), std::invalid_argument);
  EXPECT_THROW(
    configure_with_null_clock(client_, RCL_SERVICE_INTROSPECTION_OFF), std::invalid_argument);
  EXPECT_THROW(
    configure_with_null_clock(service_, RCL_SERVICE_INTROSPECTION_CONTENTS), std::invalid_argument);
  EXPECT_THROW(
    configure_with_null_clock(service_, RCL_SERVICE_INTROSPECTION_OFF), std::invalid_argument);
#endif
}

// Agnocast rejects KeepAll where it creates the event publisher and rclcpp accepts it, so the
// handle rejects it in every state -- stricter than either backend, but the same in both.
TEST_F(ServiceIntrospectionTest, RejectsKeepAll)
{
#if RCLCPP_VERSION_GTE(21, 0, 0)
  // Act
  const auto configure_with_keep_all =
    [&](const auto & handle, rcl_service_introspection_state_t state) {
      handle->configure_introspection(node_->get_clock(), rclcpp::QoS(rclcpp::KeepAll()), state);
    };

  // Assert
  EXPECT_THROW(
    configure_with_keep_all(client_, RCL_SERVICE_INTROSPECTION_CONTENTS), std::invalid_argument);
  EXPECT_THROW(
    configure_with_keep_all(client_, RCL_SERVICE_INTROSPECTION_OFF), std::invalid_argument);
  EXPECT_THROW(
    configure_with_keep_all(service_, RCL_SERVICE_INTROSPECTION_CONTENTS), std::invalid_argument);
  EXPECT_THROW(
    configure_with_keep_all(service_, RCL_SERVICE_INTROSPECTION_OFF), std::invalid_argument);
#endif
}

}  // namespace
