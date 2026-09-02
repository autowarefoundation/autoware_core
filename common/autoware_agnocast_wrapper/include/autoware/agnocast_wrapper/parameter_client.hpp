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

#include "autoware/agnocast_wrapper/node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <rclcpp/version.h>

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifdef USE_AGNOCAST_ENABLED
#include <agnocast/agnocast.hpp>
#include <agnocast/node/agnocast_parameter_client.hpp>
#endif

namespace autoware::agnocast_wrapper
{

/// @brief Backend-agnostic async parameters client.
///
/// rclcpp::AsyncParametersClient cannot be built on an Agnocast node: it goes through
/// NodeServicesInterface::add_client(), which Agnocast does not support (Agnocast services do not
/// pass through rcl). Agnocast ships its own agnocast::AsyncParametersClient instead, so callers
/// that must work in both builds need this switch.
///
/// Only the read path is exposed, because that is what the wrapper's callers use. Extend it when
/// a caller needs the setter or descriptor calls -- both backends already provide them.
class AsyncParametersClient
{
public:
  using SharedPtr = std::shared_ptr<AsyncParametersClient>;

  virtual ~AsyncParametersClient() = default;

  /// @brief Read parameters from the remote node.
  /// @param names Parameter names to read.
  /// @param callback Invoked with the resolved future when the response arrives.
  /// @return Shared future resolving to the parameters.
  virtual std::shared_future<std::vector<rclcpp::Parameter>> get_parameters(
    const std::vector<std::string> & names,
    std::function<void(std::shared_future<std::vector<rclcpp::Parameter>>)> callback = nullptr) = 0;

  /// @brief Block until the parameter service is available or the timeout expires.
  /// @param timeout Maximum duration to wait (-1 = wait forever).
  /// @return true if the service became available, false on timeout.
  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_service(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    return wait_for_service_impl(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

protected:
  virtual bool wait_for_service_impl(std::chrono::nanoseconds timeout) = 0;
};

class ROS2AsyncParametersClient : public AsyncParametersClient
{
  rclcpp::AsyncParametersClient::SharedPtr client_;

protected:
  bool wait_for_service_impl(std::chrono::nanoseconds timeout) override
  {
    return client_->wait_for_service(timeout);
  }

public:
  ROS2AsyncParametersClient(
    rclcpp::Node * node, const std::string & remote_node_name, const rclcpp::QoS & qos,
    rclcpp::CallbackGroup::SharedPtr group)
  : client_(
      std::make_shared<rclcpp::AsyncParametersClient>(
        node, remote_node_name,
#if RCLCPP_VERSION_MAJOR >= 28
        qos,
#else
        qos.get_rmw_qos_profile(),
#endif
        std::move(group)))
  {
  }

  std::shared_future<std::vector<rclcpp::Parameter>> get_parameters(
    const std::vector<std::string> & names,
    std::function<void(std::shared_future<std::vector<rclcpp::Parameter>>)> callback =
      nullptr) override
  {
    return client_->get_parameters(names, std::move(callback));
  }
};

#ifdef USE_AGNOCAST_ENABLED

class AgnocastAsyncParametersClient : public AsyncParametersClient
{
  std::shared_ptr<agnocast::AsyncParametersClient> client_;

protected:
  bool wait_for_service_impl(std::chrono::nanoseconds timeout) override
  {
    return client_->wait_for_service(timeout);
  }

public:
  AgnocastAsyncParametersClient(
    agnocast::Node * node, const std::string & remote_node_name, const rclcpp::QoS & qos,
    rclcpp::CallbackGroup::SharedPtr group)
  : client_(
      std::make_shared<agnocast::AsyncParametersClient>(
        node, remote_node_name, qos, std::move(group)))
  {
  }

  std::shared_future<std::vector<rclcpp::Parameter>> get_parameters(
    const std::vector<std::string> & names,
    std::function<void(std::shared_future<std::vector<rclcpp::Parameter>>)> callback =
      nullptr) override
  {
    return client_->get_parameters(names, std::move(callback));
  }
};

/// @note The returned client references the node's backend by raw pointer, so it must not outlive
/// @p node.
inline AsyncParametersClient::SharedPtr create_async_parameters_client(
  Node * node, const std::string & remote_node_name = "",
  const rclcpp::QoS & qos = rclcpp::ParametersQoS(),
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastAsyncParametersClient>(
      node->get_agnocast_node().get(), remote_node_name, qos, std::move(group));
  }
  return std::make_shared<ROS2AsyncParametersClient>(
    node->get_rclcpp_node().get(), remote_node_name, qos, std::move(group));
}

#else  // USE_AGNOCAST_ENABLED

/// @note The returned client references the node's rclcpp node by raw pointer, so it must not
/// outlive @p node.
inline AsyncParametersClient::SharedPtr create_async_parameters_client(
  Node * node, const std::string & remote_node_name = "",
  const rclcpp::QoS & qos = rclcpp::ParametersQoS(),
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<ROS2AsyncParametersClient>(
    node->get_rclcpp_node().get(), remote_node_name, qos, std::move(group));
}

#endif  // USE_AGNOCAST_ENABLED

/// Overload for a plain rclcpp::Node, so callers templated on the node type compile either way.
inline AsyncParametersClient::SharedPtr create_async_parameters_client(
  rclcpp::Node * node, const std::string & remote_node_name = "",
  const rclcpp::QoS & qos = rclcpp::ParametersQoS(),
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<ROS2AsyncParametersClient>(node, remote_node_name, qos, std::move(group));
}

}  // namespace autoware::agnocast_wrapper
