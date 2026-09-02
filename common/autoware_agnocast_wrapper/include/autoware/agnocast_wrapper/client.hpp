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

// Client<ServiceT> abstraction.

#include "autoware/agnocast_wrapper/macros.hpp"

#include <rclcpp/rclcpp.hpp>

#include <rclcpp/version.h>

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <utility>

#ifdef USE_AGNOCAST_ENABLED

#include "autoware/agnocast_wrapper/message_ptr.hpp"
#include "autoware/agnocast_wrapper/runtime.hpp"

#include <agnocast/agnocast.hpp>

namespace autoware::agnocast_wrapper
{

template <typename ServiceT>
class Client
{
protected:
  virtual bool wait_for_service_impl(std::chrono::nanoseconds timeout) const = 0;

public:
  using SharedPtr = std::shared_ptr<Client<ServiceT>>;

  // Named to match rclcpp::Client, so generic code can spell the request/response pointer types
  // off the client rather than assuming std::shared_ptr.
  using SharedRequest = AUTOWARE_CLIENT_REQUEST_PTR(ServiceT);
  using SharedResponse = AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT);

  using Future = std::future<AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT)>;
  using SharedFuture = std::shared_future<AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT)>;

  struct FutureAndRequestId : rclcpp::detail::FutureAndRequestId<Future>
  {
    using rclcpp::detail::FutureAndRequestId<Future>::FutureAndRequestId;
    SharedFuture share() noexcept { return this->future.share(); }
  };
  struct SharedFutureAndRequestId : rclcpp::detail::FutureAndRequestId<SharedFuture>
  {
    using rclcpp::detail::FutureAndRequestId<SharedFuture>::FutureAndRequestId;
  };

  virtual ~Client() = default;

  virtual AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) allocate_output_service_request() = 0;

  virtual const char * get_service_name() const = 0;

  virtual bool service_is_ready() const = 0;

  template <typename RepT, typename RatioT>
  bool wait_for_service(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::nanoseconds(-1)) const
  {
    return wait_for_service_impl(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  virtual FutureAndRequestId async_send_request(
    AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) && request) = 0;
  virtual SharedFutureAndRequestId async_send_request(
    AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) && request,
    std::function<void(SharedFuture)> callback) = 0;

  /// rclcpp-shaped overloads taking an ordinary std::shared_ptr request by const reference, for
  /// callers that build the request themselves and hold it as an lvalue -- notably
  /// autoware_component_interface_utils, whose Client::call() does exactly that. The request is
  /// copied into an owned (on the Agnocast path, shared-memory) request, so prefer
  /// allocate_output_service_request() plus the rvalue overloads above on hot paths.
  ///
  /// Taking by const reference rather than by value keeps `async_send_request(std::move(req))`
  /// unambiguous: an rvalue still binds better to the rvalue-reference overloads.
  FutureAndRequestId async_send_request(const std::shared_ptr<typename ServiceT::Request> & request)
  {
    return async_send_request(copy_into_owned_request(request));
  }

  SharedFutureAndRequestId async_send_request(
    const std::shared_ptr<typename ServiceT::Request> & request,
    std::function<void(SharedFuture)> callback)
  {
    return async_send_request(copy_into_owned_request(request), std::move(callback));
  }

private:
  AUTOWARE_CLIENT_REQUEST_PTR(ServiceT)
  copy_into_owned_request(const std::shared_ptr<typename ServiceT::Request> & request)
  {
    AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) owned = allocate_output_service_request();
    *owned = *request;
    return owned;
  }
};

template <typename ServiceT>
class AgnocastClient : public Client<ServiceT>
{
  typename agnocast::Client<ServiceT>::SharedPtr client_;

protected:
  bool wait_for_service_impl(std::chrono::nanoseconds timeout) const override
  {
    return client_->wait_for_service(timeout);
  }

public:
  template <typename NodeT>
  explicit AgnocastClient(
    NodeT * node, const std::string & service_name, const rclcpp::QoS & qos,
    rclcpp::CallbackGroup::SharedPtr group)
  : client_(agnocast::create_client<ServiceT>(node, service_name, qos, group))
  {
  }

  AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) allocate_output_service_request() override
  {
    return AUTOWARE_CLIENT_REQUEST_PTR(ServiceT){client_->borrow_loaned_request()};
  }

  const char * get_service_name() const override { return client_->get_service_name(); }

  bool service_is_ready() const override { return client_->service_is_ready(); }

  AUTOWARE_CLIENT_FUTURE_AND_REQUEST_ID(ServiceT)
  async_send_request(AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) && request) override
  {
    // Wrap the promise in a shared_ptr so that the callback lambda can call set_value()
    // through the pointer without needing 'mutable'. A unique_ptr wouldn't work here because
    // the lambda is stored in a std::function, which requires its callable to be copyable.
    auto promise_ptr = std::make_shared<std::promise<AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT)>>();
    AUTOWARE_CLIENT_FUTURE(ServiceT) future = promise_ptr->get_future();

    auto agnocast_request = std::move(request).move_agnocast_ptr();
    auto request_id =
      client_
        ->async_send_request(
          std::move(agnocast_request),
          [promise_ptr = std::move(promise_ptr)](
            typename agnocast::Client<ServiceT>::SharedFuture agnocast_shared_future) {
            try {
              typename agnocast::ipc_shared_ptr<const typename ServiceT::Response>
                agnocast_response = agnocast_shared_future.get();
              promise_ptr->set_value(
                AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT){std::move(agnocast_response)});
            } catch (...) {
              promise_ptr->set_exception(std::current_exception());
            }
          })
        .request_id;

    return AUTOWARE_CLIENT_FUTURE_AND_REQUEST_ID(ServiceT)(std::move(future), request_id);
  }

  AUTOWARE_CLIENT_SHARED_FUTURE_AND_REQUEST_ID(ServiceT)
  async_send_request(
    AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) && request,
    std::function<void(AUTOWARE_CLIENT_SHARED_FUTURE(ServiceT))> callback) override
  {
    auto promise_ptr = std::make_shared<std::promise<AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT)>>();
    AUTOWARE_CLIENT_SHARED_FUTURE(ServiceT) shared_future = promise_ptr->get_future().share();

    auto agnocast_request = std::move(request).move_agnocast_ptr();
    auto request_id =
      client_
        ->async_send_request(
          std::move(agnocast_request),
          [callback = std::move(callback), promise_ptr = std::move(promise_ptr), shared_future](
            typename agnocast::Client<ServiceT>::SharedFuture agnocast_shared_future) {
            // If an exception is set in the underlying future, propagate it to our promise.
            try {
              typename agnocast::ipc_shared_ptr<const typename ServiceT::Response>
                agnocast_response = agnocast_shared_future.get();
              promise_ptr->set_value(
                AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT){std::move(agnocast_response)});
            } catch (...) {
              promise_ptr->set_exception(std::current_exception());
              return;
            }
            callback(std::move(shared_future));
          })
        .request_id;

    return AUTOWARE_CLIENT_SHARED_FUTURE_AND_REQUEST_ID(ServiceT)(
      std::move(shared_future), request_id);
  }
};

template <typename ServiceT>
class ROS2Client : public Client<ServiceT>
{
  typename rclcpp::Client<ServiceT>::SharedPtr client_;

protected:
  bool wait_for_service_impl(std::chrono::nanoseconds timeout) const override
  {
    return client_->wait_for_service(timeout);
  }

public:
  explicit ROS2Client(
    rclcpp::Node * node, const std::string & service_name, const rclcpp::QoS & qos,
    rclcpp::CallbackGroup::SharedPtr group)
#if RCLCPP_VERSION_MAJOR >= 28
  : client_(node->create_client<ServiceT>(service_name, qos, group))
#else
  : client_(node->create_client<ServiceT>(service_name, qos.get_rmw_qos_profile(), group))
#endif
  {
  }

  AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) allocate_output_service_request() override
  {
    return AUTOWARE_CLIENT_REQUEST_PTR(ServiceT){std::make_shared<typename ServiceT::Request>()};
  }

  const char * get_service_name() const override { return client_->get_service_name(); }

  bool service_is_ready() const override { return client_->service_is_ready(); }

  AUTOWARE_CLIENT_FUTURE_AND_REQUEST_ID(ServiceT)
  async_send_request(AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) && request) override
  {
    auto promise_ptr = std::make_shared<std::promise<AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT)>>();
    AUTOWARE_CLIENT_FUTURE(ServiceT) future = promise_ptr->get_future();

    auto ros2_request = std::move(request).move_ros2_ptr();
    auto request_id = client_
                        ->async_send_request(
                          ros2_request,
                          [promise_ptr = std::move(promise_ptr)](
                            typename rclcpp::Client<ServiceT>::SharedFuture ros2_shared_future) {
                            try {
                              std::shared_ptr<const typename ServiceT::Response> ros2_response =
                                ros2_shared_future.get();
                              promise_ptr->set_value(
                                AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT){std::move(ros2_response)});
                            } catch (...) {
                              promise_ptr->set_exception(std::current_exception());
                            }
                          })
                        .request_id;

    return AUTOWARE_CLIENT_FUTURE_AND_REQUEST_ID(ServiceT)(std::move(future), request_id);
  }

  AUTOWARE_CLIENT_SHARED_FUTURE_AND_REQUEST_ID(ServiceT)
  async_send_request(
    AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) && request,
    std::function<void(AUTOWARE_CLIENT_SHARED_FUTURE(ServiceT))> callback) override
  {
    auto promise_ptr = std::make_shared<std::promise<AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT)>>();
    AUTOWARE_CLIENT_SHARED_FUTURE(ServiceT) shared_future = promise_ptr->get_future().share();

    auto ros2_request = std::move(request).move_ros2_ptr();
    auto request_id =
      client_
        ->async_send_request(
          ros2_request,
          [callback = std::move(callback), promise_ptr = std::move(promise_ptr),
           shared_future](typename rclcpp::Client<ServiceT>::SharedFuture ros2_shared_future) {
            // If an exception is set in the underlying future, propagate it to our promise.
            try {
              std::shared_ptr<const typename ServiceT::Response> ros2_response =
                ros2_shared_future.get();
              promise_ptr->set_value(
                AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT){std::move(ros2_response)});
            } catch (...) {
              promise_ptr->set_exception(std::current_exception());
              return;
            }
            callback(std::move(shared_future));
          })
        .request_id;

    return AUTOWARE_CLIENT_SHARED_FUTURE_AND_REQUEST_ID(ServiceT)(
      std::move(shared_future), request_id);
  }
};

template <typename ServiceT>
AUTOWARE_CLIENT_PTR(ServiceT)
create_client(
  rclcpp::Node * node, const std::string & service_name,
  const rclcpp::QoS & qos = rclcpp::ServicesQoS(), rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastClient<ServiceT>>(node, service_name, qos, group);
  } else {
    return std::make_shared<ROS2Client<ServiceT>>(node, service_name, qos, group);
  }
}

}  // namespace autoware::agnocast_wrapper

#else

namespace autoware::agnocast_wrapper
{

// ===== Client, non-Agnocast build =====
//
// Mirrors the Agnocast-build Client<ServiceT> abstraction so code written against
// AUTOWARE_CLIENT_PTR compiles unchanged in both builds.
// async_send_request() still bridges through a promise: AUTOWARE_CLIENT_FUTURE(ServiceT) and
// rclcpp::Client<ServiceT>::Future are different std::future instantiations, and std::future has
// no covariant conversion between them.

template <typename ServiceT>
class Client
{
protected:
  virtual bool wait_for_service_impl(std::chrono::nanoseconds timeout) const = 0;

public:
  using SharedPtr = std::shared_ptr<Client<ServiceT>>;

  // Named to match rclcpp::Client, so generic code can spell the request/response pointer types
  // off the client rather than assuming std::shared_ptr.
  using SharedRequest = AUTOWARE_CLIENT_REQUEST_PTR(ServiceT);
  using SharedResponse = AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT);

  using Future = std::future<AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT)>;
  using SharedFuture = std::shared_future<AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT)>;

  struct FutureAndRequestId : rclcpp::detail::FutureAndRequestId<Future>
  {
    using rclcpp::detail::FutureAndRequestId<Future>::FutureAndRequestId;
    SharedFuture share() noexcept { return this->future.share(); }
  };
  struct SharedFutureAndRequestId : rclcpp::detail::FutureAndRequestId<SharedFuture>
  {
    using rclcpp::detail::FutureAndRequestId<SharedFuture>::FutureAndRequestId;
  };

  virtual ~Client() = default;

  virtual AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) allocate_output_service_request() = 0;

  virtual const char * get_service_name() const = 0;

  virtual bool service_is_ready() const = 0;

  template <typename RepT, typename RatioT>
  bool wait_for_service(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::nanoseconds(-1)) const
  {
    return wait_for_service_impl(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  virtual FutureAndRequestId async_send_request(
    AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) && request) = 0;
  virtual SharedFutureAndRequestId async_send_request(
    AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) && request,
    std::function<void(SharedFuture)> callback) = 0;

  /// rclcpp-shaped overloads taking an ordinary std::shared_ptr request by const reference, for
  /// callers that build the request themselves and hold it as an lvalue -- notably
  /// autoware_component_interface_utils, whose Client::call() does exactly that. The request is
  /// copied into an owned (on the Agnocast path, shared-memory) request, so prefer
  /// allocate_output_service_request() plus the rvalue overloads above on hot paths.
  ///
  /// Taking by const reference rather than by value keeps `async_send_request(std::move(req))`
  /// unambiguous: an rvalue still binds better to the rvalue-reference overloads.
  FutureAndRequestId async_send_request(const std::shared_ptr<typename ServiceT::Request> & request)
  {
    return async_send_request(copy_into_owned_request(request));
  }

  SharedFutureAndRequestId async_send_request(
    const std::shared_ptr<typename ServiceT::Request> & request,
    std::function<void(SharedFuture)> callback)
  {
    return async_send_request(copy_into_owned_request(request), std::move(callback));
  }

private:
  AUTOWARE_CLIENT_REQUEST_PTR(ServiceT)
  copy_into_owned_request(const std::shared_ptr<typename ServiceT::Request> & request)
  {
    AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) owned = allocate_output_service_request();
    *owned = *request;
    return owned;
  }
};

template <typename ServiceT>
class ROS2Client : public Client<ServiceT>
{
  typename rclcpp::Client<ServiceT>::SharedPtr client_;

protected:
  bool wait_for_service_impl(std::chrono::nanoseconds timeout) const override
  {
    return client_->wait_for_service(timeout);
  }

public:
  explicit ROS2Client(
    rclcpp::Node * node, const std::string & service_name, const rclcpp::QoS & qos,
    rclcpp::CallbackGroup::SharedPtr group)
#if RCLCPP_VERSION_MAJOR >= 28
  : client_(node->create_client<ServiceT>(service_name, qos, group))
#else
  : client_(node->create_client<ServiceT>(service_name, qos.get_rmw_qos_profile(), group))
#endif
  {
  }

  AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) allocate_output_service_request() override
  {
    return std::make_shared<typename ServiceT::Request>();
  }

  const char * get_service_name() const override { return client_->get_service_name(); }

  bool service_is_ready() const override { return client_->service_is_ready(); }

  // rclcpp::Client<ServiceT>::Future (std::future<std::shared_ptr<Response>>) and
  // AUTOWARE_CLIENT_FUTURE(ServiceT) (std::future<std::shared_ptr<const Response>>) are different
  // std::future instantiations with no covariant conversion between them, so the result can't be
  // returned directly -- bridge it through a promise instead.
  AUTOWARE_CLIENT_FUTURE_AND_REQUEST_ID(ServiceT)
  async_send_request(AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) && request) override
  {
    auto promise_ptr = std::make_shared<std::promise<AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT)>>();
    AUTOWARE_CLIENT_FUTURE(ServiceT) future = promise_ptr->get_future();

    auto request_id = client_
                        ->async_send_request(
                          std::move(request),
                          [promise_ptr = std::move(promise_ptr)](
                            typename rclcpp::Client<ServiceT>::SharedFuture ros2_shared_future) {
                            try {
                              promise_ptr->set_value(
                                AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT){ros2_shared_future.get()});
                            } catch (...) {
                              promise_ptr->set_exception(std::current_exception());
                            }
                          })
                        .request_id;

    return AUTOWARE_CLIENT_FUTURE_AND_REQUEST_ID(ServiceT)(std::move(future), request_id);
  }

  AUTOWARE_CLIENT_SHARED_FUTURE_AND_REQUEST_ID(ServiceT)
  async_send_request(
    AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) && request,
    std::function<void(AUTOWARE_CLIENT_SHARED_FUTURE(ServiceT))> callback) override
  {
    auto promise_ptr = std::make_shared<std::promise<AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT)>>();
    AUTOWARE_CLIENT_SHARED_FUTURE(ServiceT) shared_future = promise_ptr->get_future().share();

    auto request_id =
      client_
        ->async_send_request(
          std::move(request),
          [callback = std::move(callback), promise_ptr = std::move(promise_ptr),
           shared_future](typename rclcpp::Client<ServiceT>::SharedFuture ros2_shared_future) {
            // If an exception is set in the underlying future, propagate it to our promise.
            try {
              promise_ptr->set_value(
                AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT){ros2_shared_future.get()});
            } catch (...) {
              promise_ptr->set_exception(std::current_exception());
              return;
            }
            callback(std::move(shared_future));
          })
        .request_id;

    return AUTOWARE_CLIENT_SHARED_FUTURE_AND_REQUEST_ID(ServiceT)(
      std::move(shared_future), request_id);
  }
};

template <typename ServiceT>
AUTOWARE_CLIENT_PTR(ServiceT)
create_client(
  rclcpp::Node * node, const std::string & service_name,
  const rclcpp::QoS & qos = rclcpp::ServicesQoS(), rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<ROS2Client<ServiceT>>(node, service_name, qos, group);
}

}  // namespace autoware::agnocast_wrapper

#endif
