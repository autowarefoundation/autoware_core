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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_SERVER_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_SERVER_HPP_

#include <autoware/component_interface_utils/rclcpp/exceptions.hpp>
#include <autoware/component_interface_utils/rclcpp/interface.hpp>
#include <rclcpp/node.hpp>

#include <type_traits>
#include <utility>

namespace autoware::component_interface_utils
{

/// The wrapper class of rclcpp::Service. Service-call tracing is provided by ROS 2
/// service introspection (enabled via NodeInterface::introspection_state), not a
/// custom log topic.
template <class SpecT>
class Service
{
private:
  // Detect if the service response has status.
  template <class, template <class> class, class = std::void_t<>>
  struct detect : std::false_type
  {
  };
  template <class T, template <class> class Check>
  struct detect<T, Check, std::void_t<Check<T>>> : std::true_type
  {
  };
  template <class T>
  using has_status_impl = decltype(std::declval<T>().status);
  template <class T>
  using has_status_type = detect<T, has_status_impl>;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(Service)
  using SpecType = SpecT;
  using WrapType = rclcpp::Service<typename SpecT::Service>;

  /// Constructor.
  template <class CallbackT>
  Service(
    NodeInterface::SharedPtr interface, CallbackT && callback,
    rclcpp::CallbackGroup::SharedPtr group)
  : interface_(interface)
  {
    service_ = interface_->node->create_service<typename SpecT::Service>(
      SpecT::name, wrap(callback), rclcpp::ServicesQoS(), group);
#if AUTOWARE_COMPONENT_INTERFACE_UTILS_HAS_SERVICE_INTROSPECTION
    if (interface_->introspection_state != RCL_SERVICE_INTROSPECTION_OFF) {
      service_->configure_introspection(
        interface_->node->get_clock(), rclcpp::QoS(1), interface_->introspection_state);
    }
#endif
  }

  /// Create a service callback that converts exceptions into the response status.
  template <class CallbackT>
  typename WrapType::CallbackType wrap(CallbackT && callback)
  {
    return [callback](
             typename SpecT::Service::Request::SharedPtr request,
             typename SpecT::Service::Response::SharedPtr response) {
      // If the response has status, convert it from the exception.
      if constexpr (!has_status_type<typename SpecT::Service::Response>::value) {
        callback(request, response);
      } else {
        try {
          callback(request, response);
        } catch (const ServiceException & error) {
          error.set(response->status);
        }
      }
    };
  }

private:
  RCLCPP_DISABLE_COPY(Service)
  typename WrapType::SharedPtr service_;
  NodeInterface::SharedPtr interface_;
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_SERVER_HPP_
