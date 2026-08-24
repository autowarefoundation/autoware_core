// Copyright 2025 TIER IV, Inc.
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

#pragma once

// Service<ServiceT> abstraction and the callback-shape traits.

#include "autoware/agnocast_wrapper/macros.hpp"

#include <rclcpp/rclcpp.hpp>

#include <rclcpp/version.h>

#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#ifdef USE_AGNOCAST_ENABLED

#include "autoware/agnocast_wrapper/message_ptr.hpp"
#include "autoware/agnocast_wrapper/runtime.hpp"

#include <agnocast/agnocast.hpp>

namespace autoware::agnocast_wrapper
{

template <typename ServiceT>
class Service
{
public:
  using SharedPtr = std::shared_ptr<Service<ServiceT>>;

  virtual ~Service() = default;

  /// Remap-resolved service name, mirroring rclcpp::ServiceBase::get_service_name().
  virtual const char * get_service_name() const = 0;
};

// True when Callback takes the preferred AUTOWARE_SERVER_REQUEST_PTR/RESPONSE_PTR (message_ptr)
// pair, i.e. it is written against the wrapper's zero-copy service API.
template <typename Func, typename ServiceT>
inline constexpr bool is_message_ptr_service_callback_v = std::is_invocable_v<
  std::decay_t<Func>, AUTOWARE_SERVER_REQUEST_PTR(ServiceT) &&,
  AUTOWARE_SERVER_RESPONSE_PTR(ServiceT) &&>;

// True when Callback is an rclcpp-style handler taking std::shared_ptr request/response. This lets
// utilities written for rclcpp::Node (e.g. autoware_utils_logging's LoggerLevelConfigure) be used
// unchanged on the wrapper Node, at the cost noted on the convenience paths below.
template <typename Func, typename ServiceT>
inline constexpr bool is_shared_ptr_service_callback_v = std::is_invocable_v<
  std::decay_t<Func>, std::shared_ptr<typename ServiceT::Request> &,
  std::shared_ptr<typename ServiceT::Response> &>;

template <typename ServiceT>
class AgnocastService : public Service<ServiceT>
{
  typename agnocast::Service<ServiceT>::SharedPtr srv_;

public:
  template <typename NodeT, typename Func>
  explicit AgnocastService(
    NodeT * node, const std::string & service_name, Func && callback, const rclcpp::QoS & qos,
    rclcpp::CallbackGroup::SharedPtr group)
  {
    static_assert(
      is_message_ptr_service_callback_v<Func, ServiceT>,
      "Callback should be invocable with AUTOWARE_SERVER_REQUEST_PTR and "
      "AUTOWARE_SERVER_RESPONSE_PTR (const&, &&, or by-value)");

    srv_ = agnocast::create_service<ServiceT>(
      node, service_name,
      [callback = std::forward<Func>(callback)](
        agnocast::ipc_shared_ptr<const typename ServiceT::Request> && agnocast_request,
        agnocast::ipc_shared_ptr<typename ServiceT::Response> && agnocast_response) {
        callback(
          AUTOWARE_SERVER_REQUEST_PTR(ServiceT){std::move(agnocast_request)},
          AUTOWARE_SERVER_RESPONSE_PTR(ServiceT){std::move(agnocast_response)});
      },
      qos, group);
  }

  const char * get_service_name() const override { return srv_->get_service_name(); }
};

template <typename ServiceT>
class ROS2Service : public Service<ServiceT>
{
  typename rclcpp::Service<ServiceT>::SharedPtr srv_;

public:
  template <typename Func>
  explicit ROS2Service(
    rclcpp::Node * node, const std::string & service_name, Func && callback,
    const rclcpp::QoS & qos, rclcpp::CallbackGroup::SharedPtr group)
  {
    static_assert(
      is_message_ptr_service_callback_v<Func, ServiceT>,
      "Callback should be invocable with AUTOWARE_SERVER_REQUEST_PTR and "
      "AUTOWARE_SERVER_RESPONSE_PTR (const&, &&, or by-value)");

    srv_ = node->create_service<ServiceT>(
      service_name,
      [callback = std::forward<Func>(callback)](
        std::shared_ptr<const typename ServiceT::Request> && ros2_request,
        std::shared_ptr<typename ServiceT::Response> && ros2_response) {
        callback(
          AUTOWARE_SERVER_REQUEST_PTR(ServiceT){std::move(ros2_request)},
          AUTOWARE_SERVER_RESPONSE_PTR(ServiceT){std::move(ros2_response)});
      },
#if RCLCPP_VERSION_MAJOR >= 28
      qos, group);
#else
      qos.get_rmw_qos_profile(), group);
#endif
  }

  const char * get_service_name() const override { return srv_->get_service_name(); }
};

template <typename ServiceT, typename Func>
AUTOWARE_SERVICE_PTR(ServiceT)
create_service(
  rclcpp::Node * node, const std::string & service_name, Func && callback,
  const rclcpp::QoS & qos = rclcpp::ServicesQoS(), rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastService<ServiceT>>(
      node, service_name, std::forward<Func>(callback), qos, group);
  } else {
    return std::make_shared<ROS2Service<ServiceT>>(
      node, service_name, std::forward<Func>(callback), qos, group);
  }
}

}  // namespace autoware::agnocast_wrapper

#else

namespace autoware::agnocast_wrapper
{

// ===== Service, non-Agnocast build =====
//
// Mirrors the Agnocast-build Service<ServiceT> abstraction so code written against
// AUTOWARE_SERVICE_PTR compiles unchanged in both builds.

template <typename ServiceT>
class Service
{
public:
  using SharedPtr = std::shared_ptr<Service<ServiceT>>;

  virtual ~Service() = default;

  /// Remap-resolved service name, mirroring rclcpp::ServiceBase::get_service_name().
  virtual const char * get_service_name() const = 0;
};

// True when Callback takes the preferred AUTOWARE_SERVER_REQUEST_PTR/RESPONSE_PTR pair, i.e. it
// is written against the wrapper's service API.
template <typename Func, typename ServiceT>
inline constexpr bool is_message_ptr_service_callback_v = std::is_invocable_v<
  std::decay_t<Func>, AUTOWARE_SERVER_REQUEST_PTR(ServiceT) &&,
  AUTOWARE_SERVER_RESPONSE_PTR(ServiceT) &&>;

// True when Callback is an rclcpp-style handler taking std::shared_ptr request/response. This lets
// utilities written for rclcpp::Node be used unchanged on the wrapper Node.
template <typename Func, typename ServiceT>
inline constexpr bool is_shared_ptr_service_callback_v = std::is_invocable_v<
  std::decay_t<Func>, std::shared_ptr<typename ServiceT::Request> &,
  std::shared_ptr<typename ServiceT::Response> &>;

template <typename ServiceT>
class ROS2Service : public Service<ServiceT>
{
  typename rclcpp::Service<ServiceT>::SharedPtr srv_;

public:
  template <typename Func>
  explicit ROS2Service(
    rclcpp::Node * node, const std::string & service_name, Func && callback,
    const rclcpp::QoS & qos, rclcpp::CallbackGroup::SharedPtr group)
  {
    static_assert(
      is_message_ptr_service_callback_v<Func, ServiceT>,
      "Callback should be invocable with AUTOWARE_SERVER_REQUEST_PTR and "
      "AUTOWARE_SERVER_RESPONSE_PTR (const&, &&, or by-value)");

    srv_ = node->create_service<ServiceT>(
      service_name,
      [callback = std::forward<Func>(callback)](
        std::shared_ptr<const typename ServiceT::Request> && ros2_request,
        std::shared_ptr<typename ServiceT::Response> && ros2_response) {
        callback(
          AUTOWARE_SERVER_REQUEST_PTR(ServiceT){std::move(ros2_request)},
          AUTOWARE_SERVER_RESPONSE_PTR(ServiceT){std::move(ros2_response)});
      },
#if RCLCPP_VERSION_MAJOR >= 28
      qos, group);
#else
      qos.get_rmw_qos_profile(), group);
#endif
  }

  const char * get_service_name() const override { return srv_->get_service_name(); }
};

template <typename ServiceT, typename Func>
AUTOWARE_SERVICE_PTR(ServiceT)
create_service(
  rclcpp::Node * node, const std::string & service_name, Func && callback,
  const rclcpp::QoS & qos = rclcpp::ServicesQoS(), rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<ROS2Service<ServiceT>>(
    node, service_name, std::forward<Func>(callback), qos, group);
}

}  // namespace autoware::agnocast_wrapper

#endif
