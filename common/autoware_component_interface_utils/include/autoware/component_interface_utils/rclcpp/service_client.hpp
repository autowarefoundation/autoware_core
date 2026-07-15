// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_CLIENT_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_CLIENT_HPP_

#include <autoware/component_interface_utils/rclcpp/exceptions.hpp>
#include <autoware/component_interface_utils/rclcpp/interface.hpp>
#include <rclcpp/node.hpp>

#include <chrono>
#include <future>
#include <optional>

namespace autoware::component_interface_utils
{

/// The wrapper class of rclcpp::Client. Service-call tracing is provided by ROS 2
/// service introspection (enabled via NodeInterface::introspection_state), not a
/// custom log topic.
template <class SpecT>
class Client
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Client)
  using SpecType = SpecT;
  using WrapType = rclcpp::Client<typename SpecT::Service>;

  /// Constructor.
  Client(NodeInterface::SharedPtr interface, rclcpp::CallbackGroup::SharedPtr group)
  : interface_(interface)
  {
#if AUTOWARE_COMPONENT_INTERFACE_UTILS_RCLCPP_GE_IRON
    client_ = interface_->node->create_client<typename SpecT::Service>(
      SpecT::name, rclcpp::ServicesQoS(), group);
    if (interface_->introspection_state != RCL_SERVICE_INTROSPECTION_OFF) {
      client_->configure_introspection(
        interface_->node->get_clock(), rclcpp::QoS(1), interface_->introspection_state);
    }
#else
    // ROS 2 Humble exposes only the (non-deprecated) rmw_qos_profile_t overload.
    client_ = interface_->node->create_client<typename SpecT::Service>(
      SpecT::name, rmw_qos_profile_services_default, group);
#endif
  }

  /// Send request.
  typename WrapType::SharedResponse call(
    const typename WrapType::SharedRequest request, std::optional<double> timeout = std::nullopt)
  {
    if (!client_->service_is_ready()) {
      throw ServiceUnready(SpecT::name);
    }

    const auto future = this->async_send_request(request);
    if (timeout) {
      const auto duration = std::chrono::duration<double, std::ratio<1>>(timeout.value());
      if (future.wait_for(duration) != std::future_status::ready) {
        throw ServiceTimeout(SpecT::name);
      }
    }
    return future.get();
  }

  /// Send request.
  typename WrapType::SharedFuture async_send_request(typename WrapType::SharedRequest request)
  {
    return client_->async_send_request(request).future;
  }

  /// Send request with a response callback. The callback is wrapped in a
  /// concrete-signature lambda so callers may pass a generic (auto-parameter)
  /// callback, which rclcpp::Client::async_send_request would otherwise reject.
  template <class CallbackT>
  typename WrapType::SharedFuture async_send_request(
    typename WrapType::SharedRequest request, CallbackT && callback)
  {
    return client_
      ->async_send_request(
        request, [callback](typename WrapType::SharedFuture future) { callback(future); })
      .future;
  }

  /// Check if the service is ready.
  bool service_is_ready() const { return client_->service_is_ready(); }

private:
  RCLCPP_DISABLE_COPY(Client)
  typename WrapType::SharedPtr client_;
  NodeInterface::SharedPtr interface_;
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_CLIENT_HPP_
