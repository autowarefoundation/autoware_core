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
#include <autoware/component_interface_utils/rclcpp/registration.hpp>
#include <rclcpp/node.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

#include <chrono>
#include <future>
#include <memory>
#include <optional>
#include <utility>

namespace autoware::component_interface_utils
{

/// Create the underlying client handle. Gated in one place because the handle type is deduced
/// from this call, and Humble (rclcpp 16) has no rclcpp::QoS overload of create_client().
template <class SpecT, class NodeT>
auto create_client_handle(NodeT * node, rclcpp::CallbackGroup::SharedPtr group)
{
#if AUTOWARE_COMPONENT_INTERFACE_UTILS_RCLCPP_GE_IRON
  return node->template create_client<typename SpecT::Service>(
    SpecT::name, rclcpp::ServicesQoS(), group);
#else
  return node->template create_client<typename SpecT::Service>(
    SpecT::name, rmw_qos_profile_services_default, group);
#endif
}

/// The wrapper class of a service client. Service-call tracing is provided by ROS 2
/// service introspection (enabled via NodeInterface::introspection_state), not a
/// custom log topic.
template <class SpecT, class NodeT = rclcpp::Node>
class Client
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Client)
  using SpecType = SpecT;
  using NodeType = NodeT;
  using WrapSharedPtr = decltype(create_client_handle<SpecT>(
    std::declval<NodeT *>(), std::declval<rclcpp::CallbackGroup::SharedPtr>()));
  using WrapType = typename WrapSharedPtr::element_type;

  using SharedRequest = std::shared_ptr<typename SpecT::Service::Request>;
  // Taken off the handle rather than spelled from the spec: a node type may hand back a response
  // pointer that is not std::shared_ptr (Agnocast returns a shared-memory pointer), and both
  // rclcpp::Client and the Agnocast wrapper expose these two names. SharedRequest stays a plain
  // std::shared_ptr because every handle accepts one, so callers keep using std::make_shared.
  using SharedResponse = typename WrapType::SharedResponse;
  using SharedFuture = typename WrapType::SharedFuture;

  /// Constructor.
  Client(typename NodeInterface<NodeT>::SharedPtr interface, rclcpp::CallbackGroup::SharedPtr group)
  : interface_(interface)
  {
    client_ = create_client_handle<SpecT>(interface_->node, group);
#if AUTOWARE_COMPONENT_INTERFACE_UTILS_RCLCPP_GE_IRON
    if constexpr (has_configure_introspection<WrapType>::value) {
      if (interface_->introspection_state != RCL_SERVICE_INTROSPECTION_OFF) {
        client_->configure_introspection(
          interface_->node->get_clock(), rclcpp::QoS(1), interface_->introspection_state);
      }
    }
#endif
    interface_->register_interface(
      make_record<SpecT>(
        InterfaceRecord::Kind::Service, InterfaceRecord::Role::Require, client_->get_service_name(),
        rosidl_generator_traits::name<typename SpecT::Service>(),
        rmw_qos_profile_services_default));
  }

  /// Send request.
  SharedResponse call(const SharedRequest request, std::optional<double> timeout = std::nullopt)
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
  SharedFuture async_send_request(SharedRequest request)
  {
    return client_->async_send_request(request).future;
  }

  /// Send request with a response callback. The callback is wrapped in a
  /// concrete-signature lambda so callers may pass a generic (auto-parameter)
  /// callback, which rclcpp::Client::async_send_request would otherwise reject.
  template <class CallbackT>
  SharedFuture async_send_request(SharedRequest request, CallbackT && callback)
  {
    return client_
      ->async_send_request(request, [callback](SharedFuture future) { callback(future); })
      .future;
  }

  /// Check if the service is ready.
  bool service_is_ready() const { return client_->service_is_ready(); }

private:
  RCLCPP_DISABLE_COPY(Client)
  WrapSharedPtr client_;
  typename NodeInterface<NodeT>::SharedPtr interface_;
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_CLIENT_HPP_
