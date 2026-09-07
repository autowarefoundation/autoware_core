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
#include "autoware/agnocast_wrapper/runtime.hpp"

#include <rclcpp/callback_group.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/qos.hpp>

#include <rclcpp/version.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <ratio>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace autoware::agnocast_wrapper
{
namespace detail
{

/// @brief Return qos unchanged, rejecting the policies the two backends do not agree on.
///
/// @throws std::invalid_argument if the durability is transient-local or the reliability is
///         best-effort. rclcpp lets both through and then never matches the service, while
///         Agnocast coerces them away, so rejecting is what makes the two builds agree.
inline const rclcpp::QoS & checked_parameters_qos(const rclcpp::QoS & qos)
{
  if (qos.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
    throw std::invalid_argument(
      "AsyncParametersClient: transient-local durability is not supported, use volatile instead");
  }
  if (qos.reliability() == rclcpp::ReliabilityPolicy::BestEffort) {
    throw std::invalid_argument(
      "AsyncParametersClient: best-effort reliability is not supported, use reliable instead");
  }
  return qos;
}

/// @brief Spell a QoS the way this rclcpp's AsyncParametersClient takes it: rclcpp::QoS from Jazzy
///        (28+), rmw_qos_profile_t on Humble (16.x). Same gate as ROS2Client's constructor in
///        client.hpp.
inline auto to_rclcpp_parameters_qos(const rclcpp::QoS & qos)
{
#if RCLCPP_VERSION_MAJOR >= 28
  return qos;
#else
  return qos.get_rmw_qos_profile();
#endif
}

}  // namespace detail
}  // namespace autoware::agnocast_wrapper

#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/node/agnocast_parameter_client.hpp>

namespace autoware::agnocast_wrapper
{

/// @brief Wrapper AsyncParametersClient that dispatches between
///        ::rclcpp::AsyncParametersClient (rclcpp mode) and ::agnocast::AsyncParametersClient
///        (agnocast mode) at runtime, depending on whether the given
///        autoware::agnocast_wrapper::Node is running in agnocast mode.
///
/// @invariant The backend is selected from use_agnocast() at construction and never changes.
class AsyncParametersClient
{
public:
  /// @brief Construct a parameters client bound to a wrapper Node.
  ///
  /// @pre The given Node must outlive this client: both backends keep their service clients
  ///      bound to it.
  ///
  /// @throws std::invalid_argument if the QoS is transient-local or best-effort; see
  ///         detail::checked_parameters_qos(). Both backends also reject a remote_node_name that
  ///         cannot form a service name, with a backend-dependent type.
  ///
  /// @param node             Wrapper node providing access to either an agnocast::Node or an
  ///                         rclcpp::Node.
  /// @param remote_node_name Name of the node whose parameters are read. Empty means this node.
  /// @param qos              QoS of the underlying service clients.
  /// @param group            Callback group the underlying service clients are added to. Left
  ///                         null, they land in the node's default MutuallyExclusive group, which
  ///                         at ENABLE_AGNOCAST=1 aborts the process if that group also holds
  ///                         rclcpp entities. Pass a group of their own, or a Reentrant one.
  explicit AsyncParametersClient(
    autoware::agnocast_wrapper::Node * node, const std::string & remote_node_name = "",
    const rclcpp::QoS & qos = rclcpp::ParametersQoS(),
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  : impl_(
      // A conditional, not an if/else: only the selected branch is evaluated, and the other
      // one's accessor would throw.
      use_agnocast()
        ? decltype(impl_)(
            std::in_place_type<AgnocastImpl>, node->get_agnocast_node().get(), remote_node_name,
            detail::checked_parameters_qos(qos), std::move(group))
        : decltype(impl_)(
            std::in_place_type<RclcppImpl>, node->get_rclcpp_node().get(), remote_node_name,
            detail::to_rclcpp_parameters_qos(detail::checked_parameters_qos(qos)),
            std::move(group)))
  {
  }

  /// @brief Read parameters from the remote node.
  ///
  /// @note The Agnocast backend answers over an Agnocast subscription, so the future resolves only
  ///       while an Agnocast executor spins the node, whatever wait_for_service() said.
  /// @note Do not block on the future from inside a callback, on either backend: the executor
  ///       that would deliver the response is the one being blocked.
  ///
  /// @param names    Parameter names to read.
  /// @param callback Invoked with the resolved future when the response arrives.
  /// @return Shared future resolving to the parameters, in the order they were requested.
  std::shared_future<std::vector<rclcpp::Parameter>> get_parameters(
    const std::vector<std::string> & names,
    std::function<void(std::shared_future<std::vector<rclcpp::Parameter>>)> callback = nullptr)
  {
    return std::visit(
      [&](auto & impl) { return impl.get_parameters(names, std::move(callback)); }, impl_);
  }

  /// @brief Block until the remote node's parameter services are available, or the timeout
  ///        expires.
  ///
  /// The Agnocast backend honours the timeout only where agnocast::init() ran, which
  /// autoware_agnocast_wrapper_register_node() arranges for an AgnocastOnly executor. In a
  /// component container agnocast::ok() is false and it returns false after one probe instead,
  /// which a caller cannot tell from a timeout.
  ///
  /// @param timeout Maximum duration to wait; a negative duration waits forever, and a zero
  ///                duration is a non-blocking probe.
  /// @return true if the services became available, false on timeout.
  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_service(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    return std::visit([&](auto & impl) { return impl.wait_for_service(timeout); }, impl_);
  }

  /// @brief Report whether the remote node's parameter services are available right now.
  ///
  /// Non-blocking, unlike wait_for_service().
  ///
  /// @return true if the remote node's parameter services are available.
  bool service_is_ready() const
  {
    return std::visit([](const auto & impl) { return impl.service_is_ready(); }, impl_);
  }

  AsyncParametersClient(const AsyncParametersClient &) = delete;
  AsyncParametersClient & operator=(const AsyncParametersClient &) = delete;
  AsyncParametersClient(AsyncParametersClient &&) = delete;
  AsyncParametersClient & operator=(AsyncParametersClient &&) = delete;

private:
  using RclcppImpl = ::rclcpp::AsyncParametersClient;
  using AgnocastImpl = ::agnocast::AsyncParametersClient;

  std::variant<RclcppImpl, AgnocastImpl> impl_;
};

}  // namespace autoware::agnocast_wrapper

#else  // !USE_AGNOCAST_ENABLED

namespace autoware::agnocast_wrapper
{

// Agnocast-disabled build: a thin composition wrapper over ::rclcpp::AsyncParametersClient rather
// than a derived class, so the full upstream API cannot leak into the =0 build and let =0-only code
// compile that breaks under =1. Signatures and semantics match the agnocast-enabled build above,
// which carries the documentation.
class AsyncParametersClient
{
public:
  explicit AsyncParametersClient(
    autoware::agnocast_wrapper::Node * node, const std::string & remote_node_name = "",
    const rclcpp::QoS & qos = rclcpp::ParametersQoS(),
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  : impl_(
      node->get_rclcpp_node().get(), remote_node_name,
      detail::to_rclcpp_parameters_qos(detail::checked_parameters_qos(qos)), std::move(group))
  {
  }

  std::shared_future<std::vector<rclcpp::Parameter>> get_parameters(
    const std::vector<std::string> & names,
    std::function<void(std::shared_future<std::vector<rclcpp::Parameter>>)> callback = nullptr)
  {
    return impl_.get_parameters(names, std::move(callback));
  }

  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_service(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    return impl_.wait_for_service(timeout);
  }

  bool service_is_ready() const { return impl_.service_is_ready(); }

  AsyncParametersClient(const AsyncParametersClient &) = delete;
  AsyncParametersClient & operator=(const AsyncParametersClient &) = delete;
  AsyncParametersClient(AsyncParametersClient &&) = delete;
  AsyncParametersClient & operator=(AsyncParametersClient &&) = delete;

private:
  ::rclcpp::AsyncParametersClient impl_;
};

}  // namespace autoware::agnocast_wrapper

#endif
