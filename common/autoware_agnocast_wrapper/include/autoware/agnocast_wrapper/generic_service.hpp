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

#pragma once

// Type-erased ("generic") service server and client. Unlike the generic publisher/subscription,
// both builds use the wrapper's own types: Humble's rclcpp has neither a generic service nor a
// generic client, so the rclcpp backend is implemented here as well.

#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <future>
#include <memory>
#include <string>

#ifdef USE_AGNOCAST_ENABLED
namespace agnocast
{
class Node;
}  // namespace agnocast
#endif

namespace autoware::agnocast_wrapper
{

/// Mirrors a service server whose type is supplied as a runtime string (e.g.
/// "std_srvs/srv/SetBool") rather than a compile-time template argument, for nodes that forward
/// arbitrary services without linking against their interface packages.
///
/// Requests are always answered through send_response(), possibly after the callback has
/// returned, so a node can forward a request and reply once the forwarded call completes.
///
/// @throws std::runtime_error at construction if service_type is unknown or its typesupport
///         libraries cannot be loaded.
class GenericService : public std::enable_shared_from_this<GenericService>
{
public:
  using SharedPtr = std::shared_ptr<GenericService>;

  /// A received request. It keeps the request payload (on the Agnocast path, its shared-memory
  /// entry) alive until the response is sent, so hold it until then.
  class Request
  {
  public:
    virtual ~Request() = default;
    /// The request payload: a ServiceT::Request of the service's type.
    virtual const void * get() const = 0;
  };
  using RequestPtr = std::shared_ptr<Request>;

  using Callback = std::function<void(const SharedPtr & service, const RequestPtr & request)>;

  virtual ~GenericService() = default;

  virtual const char * get_service_name() const = 0;

  /// Reply to @p request with a copy of @p response, a ServiceT::Response of the service's type.
  /// @throws std::runtime_error if @p request has already been answered, or if the response
  ///         cannot be sent.
  virtual void send_response(const RequestPtr & request, const void * response) = 0;
};

/// The client counterpart of GenericService.
///
/// The response is handed over as a std::shared_ptr<const void> pointing at a ServiceT::Response.
/// On the Agnocast path it aliases the received shared-memory entry, so it must not outlive the
/// client, the same rule as for the typed Client.
///
/// @throws std::runtime_error at construction if service_type is unknown or its typesupport
///         libraries cannot be loaded.
class GenericClient
{
public:
  using SharedPtr = std::shared_ptr<GenericClient>;
  using SharedResponse = std::shared_ptr<const void>;
  using SharedFuture = std::shared_future<SharedResponse>;
  using Callback = std::function<void(SharedFuture)>;

  virtual ~GenericClient() = default;

  virtual const char * get_service_name() const = 0;

  virtual bool service_is_ready() const = 0;

  /// Send a copy of @p request, a ServiceT::Request of the service's type. @p callback, if given,
  /// is called with the returned future once the response arrives.
  /// @throws std::runtime_error if the request cannot be sent.
  virtual SharedFuture async_send_request(const void * request, Callback callback = nullptr) = 0;
};

namespace detail
{

GenericService::SharedPtr create_ros2_generic_service(
  rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
  GenericService::Callback callback, const rclcpp::QoS & qos,
  const rclcpp::CallbackGroup::SharedPtr & group);

GenericClient::SharedPtr create_ros2_generic_client(
  rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
  const rclcpp::QoS & qos, const rclcpp::CallbackGroup::SharedPtr & group);

#ifdef USE_AGNOCAST_ENABLED
GenericService::SharedPtr create_agnocast_generic_service(
  agnocast::Node * node, const std::string & service_name, const std::string & service_type,
  GenericService::Callback callback, const rclcpp::QoS & qos,
  const rclcpp::CallbackGroup::SharedPtr & group);

GenericClient::SharedPtr create_agnocast_generic_client(
  agnocast::Node * node, const std::string & service_name, const std::string & service_type,
  const rclcpp::QoS & qos, const rclcpp::CallbackGroup::SharedPtr & group);
#endif

}  // namespace detail

}  // namespace autoware::agnocast_wrapper
