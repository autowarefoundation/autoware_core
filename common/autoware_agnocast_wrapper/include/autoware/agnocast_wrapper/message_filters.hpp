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

#include "autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp"
#include "autoware/agnocast_wrapper/node.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/message_filters/subscriber.hpp>
#include <agnocast/message_filters/sync_policies/approximate_time.hpp>
#include <agnocast/message_filters/sync_policies/exact_time.hpp>
#include <agnocast/message_filters/synchronizer.hpp>

namespace autoware::agnocast_wrapper
{
namespace message_filters
{

/// @brief Wrapper message_filters Subscriber that switches between
///        rclcpp and agnocast message_filters at runtime.
///
/// @invariant The backend is selected from use_agnocast() at construction and never
///            changes, so the held subscriber object keeps a stable identity for the
///            Subscriber's lifetime. subscribe() may therefore be called repeatedly
///            without invalidating a Synchronizer already wired to this Subscriber.
template <class M>
class Subscriber
{
public:
  using RclcppSubscriber = ::message_filters::Subscriber<M, rclcpp::Node>;
  using AgnocastSubscriber = agnocast::message_filters::Subscriber<M, agnocast::Node>;

  Subscriber()
  : sub_(
      use_agnocast() ? decltype(sub_)(std::in_place_type<AgnocastSubscriber>)
                     : decltype(sub_)(std::in_place_type<RclcppSubscriber>))
  {
  }

  Subscriber(
    autoware::agnocast_wrapper::Node * node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default)
  : Subscriber()
  {
    subscribe(node, topic, qos);
  }

  void subscribe(
    autoware::agnocast_wrapper::Node * node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    std::visit(
      [&](auto & sub) {
        if constexpr (std::is_same_v<std::decay_t<decltype(sub)>, AgnocastSubscriber>) {
          sub.subscribe(node->get_agnocast_node().get(), topic, qos);
        } else {
          sub.subscribe(node->get_rclcpp_node().get(), topic, qos);
        }
      },
      sub_);
  }

  void unsubscribe()
  {
    std::visit([](auto & sub) { sub.unsubscribe(); }, sub_);
  }

  // Internal API: used by PolicySynchronizer.
  // Not intended for downstream use.
  RclcppSubscriber & rclcpp_subscriber() { return std::get<RclcppSubscriber>(sub_); }
  AgnocastSubscriber & agnocast_subscriber() { return std::get<AgnocastSubscriber>(sub_); }

private:
  std::variant<RclcppSubscriber, AgnocastSubscriber> sub_;
};

/// @brief Common synchronizer wrapper parameterized by the underlying policy types.
///        Use through the user-facing Synchronizer<sync_policies::ApproximateTime<M0, M1>> /
///        Synchronizer<sync_policies::ExactTime<M0, M1>>.
///
/// At construction, use_agnocast() selects which backend synchronizer to instantiate
/// inside the internal std::variant; the choice is fixed for the wrapper's lifetime
/// and all subsequent calls (e.g. registerCallback) are dispatched via std::visit to
/// the active backend. To add a third policy, instantiate this template with the
/// corresponding rclcpp and agnocast policy types.
///
/// @tparam RclcppPolicy   ::message_filters sync policy type used when running on rclcpp
///                        (e.g. ::message_filters::sync_policies::ApproximateTime<M0, M1>).
/// @tparam AgnocastPolicy agnocast::message_filters sync policy type used when running on
///                        agnocast (e.g. agnocast::message_filters::sync_policies::ExactTime<M0,
///                        M1>).
/// @tparam M0             First message type to synchronize.
/// @tparam M1             Second message type to synchronize.
template <typename RclcppPolicy, typename AgnocastPolicy, typename M0, typename M1>
class PolicySynchronizer
{
public:
  using Callback = std::function<void(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(M0) &, const AUTOWARE_MESSAGE_CONST_SHARED_PTR(M1) &)>;

  PolicySynchronizer(uint32_t queue_size, Subscriber<M0> & sub0, Subscriber<M1> & sub1)
  : sync_(
      use_agnocast() ? decltype(sync_)(
                         std::in_place_type<AgnocastSync>, AgnocastPolicy(queue_size),
                         sub0.agnocast_subscriber(), sub1.agnocast_subscriber())
                     : decltype(sync_)(
                         std::in_place_type<RclcppSync>, RclcppPolicy(queue_size),
                         sub0.rclcpp_subscriber(), sub1.rclcpp_subscriber()))
  {
  }

  // Non-copyable and non-movable: upstream holds raw pointers into this object
  // (see CallbackAdapter), so its address must stay stable for the registrations' lifetime.
  ~PolicySynchronizer() = default;
  PolicySynchronizer(const PolicySynchronizer &) = delete;
  PolicySynchronizer & operator=(const PolicySynchronizer &) = delete;
  PolicySynchronizer(PolicySynchronizer &&) = delete;
  PolicySynchronizer & operator=(PolicySynchronizer &&) = delete;

  /// @brief Register a callable for each matching tuple. Mirrors the four upstream
  ///        `::message_filters::Synchronizer::registerCallback` overloads (free callable
  ///        or member-fn-ptr + instance; const and non-const).
  ///
  /// Signature: `void(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(M0) &,
  ///                  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(M1) &)`.
  /// Returns `::message_filters::Connection` whose `.disconnect()` removes THIS callable
  /// only (not RAII — scope exit does NOT unregister).
  ///
  /// @note Multiple callables can be registered; all fire on each matching tuple. Dispatch
  ///       is delegated to the underlying rclcpp/agnocast synchronizer, so ordering and
  ///       concurrency semantics match upstream.
  /// @note The returned Connection captures `this`; do not invoke `.disconnect()` after
  ///       this Synchronizer has been destroyed. Mirrors upstream
  ///       `::message_filters::Synchronizer`, whose Connection has the same lifetime contract.
  template <class C>
  ::message_filters::Connection registerCallback(C & callback)
  {
    return registerCallbackInternal(Callback(callback));
  }

  template <class C>
  ::message_filters::Connection registerCallback(const C & callback)
  {
    return registerCallbackInternal(Callback(callback));
  }

  template <class C, typename T>
  ::message_filters::Connection registerCallback(C & callback, T * t)
  {
    return registerCallbackInternal(bindMemberCallback(callback, t));
  }

  template <class C, typename T>
  ::message_filters::Connection registerCallback(const C & callback, T * t)
  {
    return registerCallbackInternal(bindMemberCallback(callback, t));
  }

private:
  // Per-registration adapter: owns the user callable and bridges upstream's MessageEvent /
  // ConstSharedPtr arguments to the wrapper's message_ptr type. Upstream keeps only a raw
  // pointer (adapter.get()), so it is held in `adapters_` to keep it alive.
  struct CallbackAdapter
  {
    Callback fn;

    using M0Event = agnocast::message_filters::MessageEvent<const M0>;
    using M1Event = agnocast::message_filters::MessageEvent<const M1>;

    void agnocastInvoke(const M0Event & e0, const M1Event & e1)
    {
      // Wrap ipc_shared_ptr in message_ptr (copies ipc_shared_ptr refcount, not data)
      const auto p0 =
        AUTOWARE_MESSAGE_CONST_SHARED_PTR(M0)(agnocast::ipc_shared_ptr<const M0>(e0.getMessage()));
      const auto p1 =
        AUTOWARE_MESSAGE_CONST_SHARED_PTR(M1)(agnocast::ipc_shared_ptr<const M1>(e1.getMessage()));
      fn(p0, p1);
    }

    void rclcppInvoke(
      const typename M0::ConstSharedPtr & m0, const typename M1::ConstSharedPtr & m1)
    {
      const auto p0 = AUTOWARE_MESSAGE_CONST_SHARED_PTR(M0)(std::shared_ptr<const M0>(m0));
      const auto p1 = AUTOWARE_MESSAGE_CONST_SHARED_PTR(M1)(std::shared_ptr<const M1>(m1));
      fn(p0, p1);
    }
  };

  using AdapterPtr = std::unique_ptr<CallbackAdapter>;
  using RclcppSync = ::message_filters::Synchronizer<RclcppPolicy>;
  using AgnocastSync = agnocast::message_filters::Synchronizer<AgnocastPolicy>;

  template <class C, typename T>
  static Callback bindMemberCallback(C callback, T * t)
  {
    return Callback{
      [callback = std::move(callback), t](
        const AUTOWARE_MESSAGE_CONST_SHARED_PTR(M0) & m0,
        const AUTOWARE_MESSAGE_CONST_SHARED_PTR(M1) & m1) { (t->*callback)(m0, m1); }};
  }

  ::message_filters::Connection registerCallbackInternal(Callback callback)
  {
    auto adapter = std::make_unique<CallbackAdapter>();
    adapter->fn = std::move(callback);
    auto * const adapter_raw = adapter.get();

    auto upstream_conn = std::visit(
      [adapter_raw](auto & sync) -> ::message_filters::Connection {
        using SyncT = std::decay_t<decltype(sync)>;
        if constexpr (std::is_same_v<SyncT, AgnocastSync>) {
          return sync.registerCallback(&CallbackAdapter::agnocastInvoke, adapter_raw);
        } else {
          return sync.registerCallback(&CallbackAdapter::rclcppInvoke, adapter_raw);
        }
      },
      sync_);

    {
      std::lock_guard<std::mutex> lock(adapters_mutex_);
      adapters_.push_back(std::move(adapter));
    }

    // Wrap upstream's Connection so the user's disconnect also drops our adapter entry.
    return ::message_filters::Connection(
      ::message_filters::Connection::VoidDisconnectFunction(
        [this, adapter_raw, upstream_conn = std::move(upstream_conn)]() mutable {
          upstream_conn.disconnect();
          std::lock_guard<std::mutex> lock(adapters_mutex_);
          adapters_.erase(
            std::remove_if(
              adapters_.begin(), adapters_.end(),
              [adapter_raw](const AdapterPtr & p) { return p.get() == adapter_raw; }),
            adapters_.end());
        }));
  }

  std::mutex adapters_mutex_;
  std::vector<AdapterPtr> adapters_;
  std::variant<RclcppSync, AgnocastSync> sync_;
};

/// @brief Policy and Synchronizer types that mirror the rclcpp message_filters API.
///        Allows node code to use the same pattern as rclcpp:
///          using SyncPolicy = sync_policies::ApproximateTime<M0, M1>;
///          using Sync = Synchronizer<SyncPolicy>;
///          sync = std::make_shared<Sync>(SyncPolicy(10), sub0, sub1);
namespace sync_policies
{
/// @brief Wrapper-layer ApproximateTime policy tag.
///
/// Carries only the queue size; the underlying rclcpp/agnocast policy is selected
/// inside Synchronizer<ApproximateTime<M0, M1>>. Distinct from
/// ::message_filters::sync_policies::ApproximateTime and
/// agnocast::message_filters::sync_policies::ApproximateTime.
///
/// @tparam M0 First message type to synchronize.
/// @tparam M1 Second message type to synchronize.
template <typename M0, typename M1>
struct ApproximateTime
{
  uint32_t queue_size;  ///< Queue size forwarded to the underlying sync policy.
  explicit ApproximateTime(uint32_t qs) noexcept : queue_size(qs) {}
};

/// @brief Wrapper-layer ExactTime policy tag.
///
/// Carries only the queue size; the underlying rclcpp/agnocast policy is selected
/// inside Synchronizer<ExactTime<M0, M1>>. Distinct from
/// ::message_filters::sync_policies::ExactTime and
/// agnocast::message_filters::sync_policies::ExactTime.
///
/// @tparam M0 First message type to synchronize.
/// @tparam M1 Second message type to synchronize.
template <typename M0, typename M1>
struct ExactTime
{
  uint32_t queue_size;  ///< Queue size forwarded to the underlying sync policy.
  explicit ExactTime(uint32_t qs) noexcept : queue_size(qs) {}
};
}  // namespace sync_policies

/// @brief Primary Synchronizer template — supports ApproximateTime and ExactTime specializations.
template <typename Policy>
class Synchronizer
{
  static_assert(
    sizeof(Policy) == 0,
    "Only sync_policies::ApproximateTime<M0, M1> and sync_policies::ExactTime<M0, M1> "
    "are supported. Policies with more than 2 message types are not implemented.");
};

/// @brief Synchronizer specialization for the wrapper-layer ApproximateTime policy.
///        Switches between rclcpp and agnocast message_filters at runtime.
///
/// The callback receives `(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(M0)&,
///                         const AUTOWARE_MESSAGE_CONST_SHARED_PTR(M1)&)`.
/// In agnocast mode, message_ptrs are created from the ipc_shared_ptrs, preserving
/// zero-copy semantics during the callback lifetime.
///
/// @note Current limitations:
///   - Maximum 2 message types per Synchronizer.
///   - connectInput() is not supported; pass Subscriber references at construction time.
///
/// @code
/// using namespace autoware::agnocast_wrapper::message_filters;
///
/// Subscriber<sensor_msgs::msg::Image> image_sub;
/// Subscriber<sensor_msgs::msg::CameraInfo> info_sub;
/// image_sub.subscribe(node, "/camera/image", rmw_qos_profile_sensor_data);
/// info_sub.subscribe(node, "/camera/info", rmw_qos_profile_sensor_data);
///
/// using Policy = sync_policies::ApproximateTime<
///     sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
/// auto sync = std::make_shared<Synchronizer<Policy>>(Policy(10), image_sub, info_sub);
///
/// // Pass a member-function pointer and `this` (mirrors upstream message_filters):
/// sync->registerCallback(&MyNode::onSynchronized, this);
/// // Equivalent form (still supported):
/// // sync->registerCallback(std::bind(
/// //   &MyNode::onSynchronized, this, std::placeholders::_1, std::placeholders::_2));
/// @endcode
template <typename M0, typename M1>
class Synchronizer<sync_policies::ApproximateTime<M0, M1>>
: public PolicySynchronizer<
    ::message_filters::sync_policies::ApproximateTime<M0, M1>,
    agnocast::message_filters::sync_policies::ApproximateTime<M0, M1>, M0, M1>
{
  using Base = PolicySynchronizer<
    ::message_filters::sync_policies::ApproximateTime<M0, M1>,
    agnocast::message_filters::sync_policies::ApproximateTime<M0, M1>, M0, M1>;

public:
  Synchronizer(
    const sync_policies::ApproximateTime<M0, M1> & policy, Subscriber<M0> & sub0,
    Subscriber<M1> & sub1)
  : Base(policy.queue_size, sub0, sub1)
  {
  }
};

/// @brief Synchronizer specialization for the wrapper-layer ExactTime policy.
///
/// Same callback signature and zero-copy semantics as the ApproximateTime specialization;
/// only the sync policy differs (messages must share identical timestamps). Subject to the
/// same limitations (max 2 message types, connectInput() not supported).
/// @see Synchronizer<sync_policies::ApproximateTime<M0, M1>> for a usage example.
template <typename M0, typename M1>
class Synchronizer<sync_policies::ExactTime<M0, M1>>
: public PolicySynchronizer<
    ::message_filters::sync_policies::ExactTime<M0, M1>,
    agnocast::message_filters::sync_policies::ExactTime<M0, M1>, M0, M1>
{
  using Base = PolicySynchronizer<
    ::message_filters::sync_policies::ExactTime<M0, M1>,
    agnocast::message_filters::sync_policies::ExactTime<M0, M1>, M0, M1>;

public:
  Synchronizer(
    const sync_policies::ExactTime<M0, M1> & policy, Subscriber<M0> & sub0, Subscriber<M1> & sub1)
  : Base(policy.queue_size, sub0, sub1)
  {
  }
};

}  // namespace message_filters
}  // namespace autoware::agnocast_wrapper

#else

namespace autoware
{
namespace agnocast_wrapper
{
namespace message_filters
{

template <class M>
using Subscriber = ::message_filters::Subscriber<M>;

namespace sync_policies
{
template <typename M0, typename M1>
using ApproximateTime = ::message_filters::sync_policies::ApproximateTime<M0, M1>;
template <typename M0, typename M1>
using ExactTime = ::message_filters::sync_policies::ExactTime<M0, M1>;
}  // namespace sync_policies

template <typename Policy>
using Synchronizer = ::message_filters::Synchronizer<Policy>;

}  // namespace message_filters
}  // namespace agnocast_wrapper
}  // namespace autoware

#endif
