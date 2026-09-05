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
#include <rclcpp/expand_topic_or_service_name.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/qos.hpp>

#include <rclcpp/version.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <ratio>
#include <string>
#include <thread>
#include <type_traits>
#include <variant>
#include <vector>

namespace autoware::agnocast_wrapper::detail
{

/// @brief Resolve remote_node_name as a service name, throwing if it cannot be one.
///
/// Both backends reject an unusable name, but through different rcl entry points and therefore
/// with different exception types (rcl_client_init vs. resolve_service_name). Rejecting here
/// first makes the wrapper throw rclcpp::exceptions::InvalidServiceNameError in every build and
/// runtime mode. An empty name means "this node"; the guard below only skips an rcl round trip,
/// because the bare suffix is a valid absolute service name on its own.
///
/// The check runs on the longest of the six parameter service names a backend goes on to build.
/// Length is the one validation rule whose answer the suffix can change (the limit is
/// RMW_TOPIC_MAX_NAME_LENGTH), so checking a shorter suffix would let a name through here only
/// for the backend to reject it with its own exception type.
///
/// @return remote_node_name unchanged, so this can wrap the argument at the call site.
inline const std::string & checked_remote_node_name(
  const std::string & remote_node_name, const char * node_name, const char * node_namespace)
{
  if (!remote_node_name.empty()) {
    (void)rclcpp::expand_topic_or_service_name(
      remote_node_name + "/set_parameters_atomically", node_name, node_namespace, true);
  }
  return remote_node_name;
}

}  // namespace autoware::agnocast_wrapper::detail

#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/node/agnocast_parameter_client.hpp>

namespace autoware::agnocast_wrapper
{

/// @brief Wrapper AsyncParametersClient that dispatches between
///        ::rclcpp::AsyncParametersClient (rclcpp mode) and ::agnocast::AsyncParametersClient
///        (agnocast mode) at runtime, depending on whether the given
///        autoware::agnocast_wrapper::Node is running in agnocast mode.
///
/// ::rclcpp::AsyncParametersClient cannot be built on an agnocast node at all: it reaches the
/// remote node's parameter services through NodeServicesInterface::add_client(), which agnocast
/// does not support because its services do not pass through rcl. That is why this is a wrapper
/// type rather than something a caller can spell directly.
///
/// Only the read path is exposed, because that is what the wrapper's callers use:
/// get_parameters(), wait_for_service() and service_is_ready(). Both backends also provide the
/// setter and descriptor calls; add them here when a caller needs one.
///
/// Shaped like tf2.hpp and diagnostic_updater.hpp (one concrete class over a std::variant) rather
/// than like client.hpp (an abstract base plus a create_ function), even though a parameters
/// client is the nearer relative of a service client. client.hpp needs the virtual boundary
/// because Client<ServiceT> is a template whose two backends spell the request and response types
/// differently; here both backends already agree on rclcpp::Parameter, so nothing has to be
/// erased and the variant keeps the backend choice a construction-time detail.
///
/// @invariant The backend variant is selected from use_agnocast() at construction and never
///            changes.
///
/// @code
/// #include <autoware/agnocast_wrapper/parameter_client.hpp>
///
/// class MyNode : public autoware::agnocast_wrapper::Node
/// {
/// public:
///   explicit MyNode(const rclcpp::NodeOptions & options)
///   : Node("my_node", options),
///     params_(std::make_unique<autoware::agnocast_wrapper::AsyncParametersClient>(
///       this, "pointcloud_map_loader"))
///   {
///     if (!params_->wait_for_service(std::chrono::seconds(5))) {
///       RCLCPP_WARN(get_logger(), "pointcloud_map_loader parameters are not up yet");
///       return;
///     }
///     params_->get_parameters(
///       {"enable_partial_load"},
///       [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
///         const auto parameters = future.get();
///         if (!parameters.empty()) {
///           RCLCPP_INFO(get_logger(), "partial load: %d", parameters.front().as_bool());
///         }
///       });
///   }
///
/// private:
///   std::unique_ptr<autoware::agnocast_wrapper::AsyncParametersClient> params_;
/// };
/// @endcode
class AsyncParametersClient
{
public:
  /// @brief Construct a parameters client bound to a wrapper Node.
  ///
  /// Selects the backend from use_agnocast() at construction; the choice is fixed for the
  /// wrapper's lifetime.
  ///
  /// @pre The given Node must outlive this client. Both backends build their service clients
  ///      from the node and keep them bound to it.
  ///
  /// @throws rclcpp::exceptions::InvalidServiceNameError if remote_node_name cannot form a
  ///         service name.
  ///
  /// @param node             Wrapper node providing access to either an agnocast::Node or an
  ///                         rclcpp::Node.
  /// @param remote_node_name Name of the node whose parameters are read. Empty means this node.
  /// @param qos              QoS of the underlying service clients.
  /// @param group            Callback group the underlying service clients are added to.
  explicit AsyncParametersClient(
    autoware::agnocast_wrapper::Node * node, const std::string & remote_node_name = "",
    const rclcpp::QoS & qos = rclcpp::ParametersQoS(),
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  : impl_(
      // Only the selected branch of the conditional is evaluated, which matters: in an
      // agnocast-enabled build get_rclcpp_node() throws when the node is in agnocast mode, and
      // get_agnocast_node() throws when it is not.
      use_agnocast() ? decltype(impl_)(
                         std::in_place_type<AgnocastImpl>, node->get_agnocast_node().get(),
                         detail::checked_remote_node_name(
                           remote_node_name, node->get_name(), node->get_namespace()),
                         qos, group)
                     : decltype(impl_)(
                         std::in_place_type<RclcppImpl>, node->get_rclcpp_node().get(),
                         detail::checked_remote_node_name(
                           remote_node_name, node->get_name(), node->get_namespace()),
// rclcpp 28+ (Jazzy) takes the QoS as rclcpp::QoS; Humble uses rclcpp 16.x, whose
// AsyncParametersClient still takes an rmw_qos_profile_t. Same normalization as ROS2Client's
// constructor in client.hpp.
#if RCLCPP_VERSION_MAJOR >= 28
                         qos,
#else
                         qos.get_rmw_qos_profile(),
#endif
                         group))
  {
  }

  /// @brief Read parameters from the remote node.
  ///
  /// @note The Agnocast backend delivers the response over an Agnocast subscription of its own, so
  ///       the future resolves and the callback runs only while an Agnocast executor spins the
  ///       node. A plain rclcpp executor never serves them, whatever wait_for_service() said.
  ///
  /// @param names    Parameter names to read.
  /// @param callback Invoked with the resolved future when the response arrives.
  /// @return Shared future resolving to the parameters, in the order they were requested.
  std::shared_future<std::vector<rclcpp::Parameter>> get_parameters(
    const std::vector<std::string> & names,
    std::function<void(std::shared_future<std::vector<rclcpp::Parameter>>)> callback = nullptr)
  {
    return std::visit([&](auto & impl) { return impl.get_parameters(names, callback); }, impl_);
  }

  /// @brief Block until the remote node's parameter services are available, or the timeout
  ///        expires.
  ///
  /// Templated on the duration because both upstream clients are, so callers keep passing the
  /// std::chrono literal they already use.
  ///
  /// The timeout is honoured the same way on both backends. The Agnocast client does not do that
  /// on its own -- it stops as soon as agnocast::ok() is false, which it is in anything but an
  /// AgnocastOnly executable -- so the wrapper waits by polling service_is_ready() there.
  ///
  /// @param timeout Maximum duration to wait; a negative duration waits forever, and a zero
  ///                duration is a non-blocking probe.
  /// @return true if the services became available, false on timeout.
  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_service(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    return std::visit(
      [&](auto & impl) {
        if constexpr (std::is_same_v<std::decay_t<decltype(impl)>, AgnocastImpl>) {
          return wait_by_polling(
            impl, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
        } else {
          return impl.wait_for_service(timeout);
        }
      },
      impl_);
  }

  /// @brief Report whether the remote node's parameter services are available right now.
  ///
  /// Non-blocking. What "available" covers differs slightly: the Agnocast backend checks all six
  /// parameter services, the rclcpp one checks five (it leaves out set_parameters_atomically), so
  /// the Agnocast answer is the stricter of the two. Every parameter service of a node comes up
  /// together, so the two only disagree inside that window.
  ///
  /// @return true if the remote node's parameter services are available.
  bool service_is_ready() const
  {
    return std::visit([](const auto & impl) { return impl.service_is_ready(); }, impl_);
  }

  // Non-copyable and non-movable: the backend is chosen at construction and the underlying
  // clients are bound to the node then, so a second handle onto the same impl has no meaning.
  AsyncParametersClient(const AsyncParametersClient &) = delete;
  AsyncParametersClient & operator=(const AsyncParametersClient &) = delete;
  AsyncParametersClient(AsyncParametersClient &&) = delete;
  AsyncParametersClient & operator=(AsyncParametersClient &&) = delete;

private:
  /// @brief rclcpp-backed implementation held inside impl_.
  using RclcppImpl = ::rclcpp::AsyncParametersClient;

  /// @brief Agnocast-backed implementation held inside impl_.
  using AgnocastImpl = ::agnocast::AsyncParametersClient;

  /// @brief Wait for the Agnocast backend by polling, so that it honours the timeout.
  ///
  /// agnocast::AsyncParametersClient::wait_for_service() stops as soon as agnocast::ok() is false,
  /// and that is the case in anything but an AgnocastOnly executable -- an ordinary mixed-mode
  /// node gets one non-blocking probe and an immediate false. That false is indistinguishable
  /// from a real timeout, so a caller cannot retry on it any more sensibly than it could give up;
  /// a caller that waits once during construction gets neither the wait nor a usable answer.
  /// service_is_ready() carries no such context check, so poll that instead and keep one timeout
  /// contract across every build and runtime mode.
  ///
  /// @param impl    Agnocast backend to poll.
  /// @param timeout Negative waits forever, zero probes once, positive bounds the wait.
  /// @return true if the services became available, false on timeout or on shutdown.
  static bool wait_by_polling(const AgnocastImpl & impl, std::chrono::nanoseconds timeout)
  {
    // The cadence agnocast's own wait uses once it gets past its context check.
    constexpr auto poll_interval =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(100));
    const auto start = std::chrono::steady_clock::now();

    while (true) {
      if (impl.service_is_ready()) {
        return true;
      }
      // Both upstream clients treat a zero timeout as "ask once, do not block".
      if (timeout == std::chrono::nanoseconds::zero() || !autoware::agnocast_wrapper::ok()) {
        return false;
      }

      auto nap = poll_interval;
      if (timeout > std::chrono::nanoseconds::zero()) {
        const auto left = timeout - (std::chrono::steady_clock::now() - start);
        if (left <= std::chrono::nanoseconds::zero()) {
          return false;
        }
        nap = std::min(poll_interval, std::chrono::duration_cast<std::chrono::nanoseconds>(left));
      }
      std::this_thread::sleep_for(nap);
    }
  }

  /// Held by value: neither client captures `this`, so nothing needs the address of the active
  /// alternative to stay put.
  std::variant<RclcppImpl, AgnocastImpl> impl_;
};

}  // namespace autoware::agnocast_wrapper

#else  // !USE_AGNOCAST_ENABLED

namespace autoware::agnocast_wrapper
{

/// @brief Curated AsyncParametersClient for the non-Agnocast build.
///
/// Holds a ::rclcpp::AsyncParametersClient by value and forwards only the member set shared with
/// the Agnocast build, instead of deriving from it. This keeps the public surface identical in
/// both builds, so code that compiles under ENABLE_AGNOCAST=0 also compiles under =1. Deriving
/// would leak the full upstream API — the setters, the parameter-event subscription, the
/// node-interface constructors — into the =0 build and allow =0-only code that breaks under =1.
class AsyncParametersClient
{
public:
  /// @brief Construct from a wrapper Node.
  ///
  /// In this build autoware::agnocast_wrapper::Node owns an internal rclcpp::Node, so the call
  /// forwards to ::rclcpp::AsyncParametersClient built on node->get_rclcpp_node().
  ///
  /// @pre The given Node must outlive this client.
  ///
  /// @throws rclcpp::exceptions::InvalidServiceNameError if remote_node_name cannot form a
  ///         service name.
  ///
  /// @param node             Wrapper node providing the underlying rclcpp::Node.
  /// @param remote_node_name Name of the node whose parameters are read. Empty means this node.
  /// @param qos              QoS of the underlying service clients.
  /// @param group            Callback group the underlying service clients are added to.
  explicit AsyncParametersClient(
    autoware::agnocast_wrapper::Node * node, const std::string & remote_node_name = "",
    const rclcpp::QoS & qos = rclcpp::ParametersQoS(),
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  : impl_(
      node->get_rclcpp_node().get(),
      detail::checked_remote_node_name(remote_node_name, node->get_name(), node->get_namespace()),
// See the Agnocast-build constructor above for why the QoS argument is version-gated.
#if RCLCPP_VERSION_MAJOR >= 28
      qos,
#else
      qos.get_rmw_qos_profile(),
#endif
      group)
  {
  }

  /// @brief Read parameters from the remote node.
  ///
  /// @param names    Parameter names to read.
  /// @param callback Invoked with the resolved future when the response arrives.
  /// @return Shared future resolving to the parameters, in the order they were requested.
  std::shared_future<std::vector<rclcpp::Parameter>> get_parameters(
    const std::vector<std::string> & names,
    std::function<void(std::shared_future<std::vector<rclcpp::Parameter>>)> callback = nullptr)
  {
    return impl_.get_parameters(names, callback);
  }

  /// @brief Block until the remote node's parameter services are available, or the timeout
  ///        expires.
  ///
  /// This build has only the rclcpp backend. The Agnocast-build class polls to reach the same
  /// contract, so the two agree: a negative duration waits forever, a zero duration is a
  /// non-blocking probe, and a positive one bounds the wait.
  ///
  /// @param timeout Maximum duration to wait; a negative duration waits forever, and a zero
  ///                duration is a non-blocking probe.
  /// @return true if the services became available, false on timeout.
  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_service(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    return impl_.wait_for_service(timeout);
  }

  /// @brief Report whether the remote node's parameter services are available right now.
  ///
  /// Non-blocking. rclcpp checks five of the six parameter services (it leaves out
  /// set_parameters_atomically), so this is marginally laxer than the Agnocast backend, which
  /// checks all six.
  ///
  /// @return true if the remote node's parameter services are available.
  bool service_is_ready() const { return impl_.service_is_ready(); }

  // Non-copyable and non-movable: matches the Agnocast-build client.
  AsyncParametersClient(const AsyncParametersClient &) = delete;
  AsyncParametersClient & operator=(const AsyncParametersClient &) = delete;
  AsyncParametersClient(AsyncParametersClient &&) = delete;
  AsyncParametersClient & operator=(AsyncParametersClient &&) = delete;

private:
  ::rclcpp::AsyncParametersClient impl_;
};

}  // namespace autoware::agnocast_wrapper

#endif
