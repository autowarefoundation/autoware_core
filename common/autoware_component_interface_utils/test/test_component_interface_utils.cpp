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

#include <autoware/component_interface_specs/system.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <memory>
#include <string>

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
  // Service tracing is provided by ROS 2 service introspection, which defaults to OFF.
  EXPECT_EQ(interface.introspection_state, RCL_SERVICE_INTROSPECTION_OFF);
  rclcpp::shutdown();
}

TEST(interface, service_wrappers_without_service_log)
{
  using ChangeOperationMode = autoware::component_interface_specs::system::ChangeOperationMode;
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_service");
  auto interface = std::make_shared<autoware::component_interface_utils::NodeInterface>(node.get());

  auto srv = autoware::component_interface_utils::Service<ChangeOperationMode>::make_shared(
    interface, [](auto, auto res) { res->status.success = true; }, nullptr);
  auto cli = autoware::component_interface_utils::Client<ChangeOperationMode>::make_shared(
    interface, nullptr);

  // The wrapper creates a real service server on the spec'd name.
  EXPECT_NE(node->get_service_names_and_types().count(ChangeOperationMode::name), 0u);

  // ServiceLog is removed: the wrappers create no "/service_log" publisher.
  EXPECT_TRUE(node->get_publishers_info_by_topic("/service_log").empty());

  // Introspection defaults to OFF, so neither wrapper creates a service-event topic.
  const std::string event_topic = std::string(ChangeOperationMode::name) + "/_service_event";
  EXPECT_TRUE(node->get_publishers_info_by_topic(event_topic).empty());

  rclcpp::shutdown();
  (void)cli;
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
  const auto infos = node->get_publishers_info_by_topic(OperationModeState::name);
  ASSERT_EQ(infos.size(), 1u);
  EXPECT_EQ(infos[0].qos_profile().durability(), rclcpp::DurabilityPolicy::TransientLocal);
  rclcpp::shutdown();
  (void)pub;
}

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
  EXPECT_FALSE(node->get_publishers_info_by_topic(event_topic).empty());
  rclcpp::shutdown();
  (void)srv;
}
