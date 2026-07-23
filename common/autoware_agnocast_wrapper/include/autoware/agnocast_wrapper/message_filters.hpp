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
///        Use through the user-facing Synchronizer<sync_policies::ApproximateTime<Ms...>> /
///        Synchronizer<sync_policies::ExactTime<Ms...>>.
///
/// At construction, use_agnocast() selects which backend synchronizer to instantiate
/// inside the internal std::variant; the choice is fixed for the wrapper's lifetime
/// and all subsequent calls (e.g. registerCallback) are dispatched via std::visit to
/// the active backend. To add a third policy, instantiate this template with the
/// corresponding rclcpp and agnocast policy types.
///
/// @tparam RclcppPolicy   ::message_filters sync policy type used when running on rclcpp
///                        (e.g. ::message_filters::sync_policies::ApproximateTime<Ms...>).
/// @tparam AgnocastPolicy agnocast::message_filters sync policy type used when running on
///                        agnocast (e.g.
///                        agnocast::message_filters::sync_policies::ExactTime<Ms...>).
/// @tparam Ms             Message types to synchronize (2..8).
template <typename RclcppPolicy, typename AgnocastPolicy, typename... Ms>
class PolicySynchronizer
{
  static_assert(
    sizeof...(Ms) >= 2 && sizeof...(Ms) <= 8,
    "PolicySynchronizer supports 2 to 8 message types (upstream Signal9 has no "
    "9-arg MFP overload, which the wrapper relies on for registration).");

public:
  using Callback = std::function<void(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(Ms) & ...)>;

  PolicySynchronizer(uint32_t queue_size, Subscriber<Ms> &... subs)
  : sync_(
      use_agnocast()
        ? decltype(sync_)(
            std::in_place_type<AgnocastSync>, AgnocastPolicy(queue_size),
            subs.agnocast_subscriber()...)
        : decltype(sync_)(
            std::in_place_type<RclcppSync>, RclcppPolicy(queue_size), subs.rclcpp_subscriber()...))
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
  /// Signature: `void(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(Ms) &...)`.
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

    void agnocastInvoke(const agnocast::message_filters::MessageEvent<const Ms> &... es)
    {
      // Wrap ipc_shared_ptr in message_ptr (copies ipc_shared_ptr refcount, not data)
      fn(AUTOWARE_MESSAGE_CONST_SHARED_PTR(Ms)(
        agnocast::ipc_shared_ptr<const Ms>(es.getMessage()))...);
    }

    void rclcppInvoke(const typename Ms::ConstSharedPtr &... ms)
    {
      fn(AUTOWARE_MESSAGE_CONST_SHARED_PTR(Ms)(std::shared_ptr<const Ms>(ms))...);
    }
  };

  using AdapterPtr = std::unique_ptr<CallbackAdapter>;
  using RclcppSync = ::message_filters::Synchronizer<RclcppPolicy>;
  using AgnocastSync = agnocast::message_filters::Synchronizer<AgnocastPolicy>;

  template <class C, typename T>
  static Callback bindMemberCallback(C && callback, T * t)
  {
    return Callback{
      [callback = std::forward<C>(callback),
       t](const AUTOWARE_MESSAGE_CONST_SHARED_PTR(Ms) & ... ms) { (t->*callback)(ms...); }};
  }

  ::message_filters::Connection registerCallbackInternal(Callback && callback)
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

    // Upstream now holds `adapter_raw`; any throw before return would leave it
    // dangling (Connection's dtor does not auto-disconnect). `upstream_conn` is
    // copy-captured (not moved) so the catch handler still has a live local if
    // the closure / Connection construction throws.
    try {
      {
        std::lock_guard<std::mutex> lock(adapters_mutex_);
        adapters_.push_back(std::move(adapter));
      }

      return ::message_filters::Connection(
        ::message_filters::Connection::VoidDisconnectFunction(
          [this, adapter_raw, upstream_conn]() mutable {
            upstream_conn.disconnect();
            std::lock_guard<std::mutex> lock(adapters_mutex_);
            adapters_.erase(
              std::remove_if(
                adapters_.begin(), adapters_.end(),
                [adapter_raw](const AdapterPtr & p) { return p.get() == adapter_raw; }),
              adapters_.end());
          }));
    } catch (...) {
      upstream_conn.disconnect();
      std::lock_guard<std::mutex> lock(adapters_mutex_);
      adapters_.erase(
        std::remove_if(
          adapters_.begin(), adapters_.end(),
          [adapter_raw](const AdapterPtr & p) { return p.get() == adapter_raw; }),
        adapters_.end());
      throw;
    }
  }

  std::mutex adapters_mutex_;
  std::vector<AdapterPtr> adapters_;
  std::variant<RclcppSync, AgnocastSync> sync_;
};

/// @brief Policy and Synchronizer types that mirror the rclcpp message_filters API.
///        Allows node code to use the same pattern as rclcpp:
///          using SyncPolicy = sync_policies::ApproximateTime<Ms...>;
///          using Sync = Synchronizer<SyncPolicy>;
///          sync = std::make_shared<Sync>(SyncPolicy(10), subs...);
namespace sync_policies
{
/// @brief Wrapper-layer ApproximateTime policy tag.
///
/// Carries only the queue size; the underlying rclcpp/agnocast policy is selected
/// inside Synchronizer<ApproximateTime<Ms...>>. Distinct from
/// ::message_filters::sync_policies::ApproximateTime and
/// agnocast::message_filters::sync_policies::ApproximateTime.
///
/// @tparam Ms Message types to synchronize (2..8).
template <typename... Ms>
struct ApproximateTime
{
  uint32_t queue_size;  ///< Queue size forwarded to the underlying sync policy.
  explicit ApproximateTime(uint32_t qs) noexcept : queue_size(qs) {}
};

/// @brief Wrapper-layer ExactTime policy tag.
///
/// Carries only the queue size; the underlying rclcpp/agnocast policy is selected
/// inside Synchronizer<ExactTime<Ms...>>. Distinct from
/// ::message_filters::sync_policies::ExactTime and
/// agnocast::message_filters::sync_policies::ExactTime.
///
/// @tparam Ms Message types to synchronize (2..8).
template <typename... Ms>
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
    "Only sync_policies::ApproximateTime<Ms...> and sync_policies::ExactTime<Ms...> "
    "are supported (2..8 message types).");
};

/// @brief Synchronizer specialization for the wrapper-layer ApproximateTime policy.
///        Switches between rclcpp and agnocast message_filters at runtime.
///
/// The callback receives `(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(Ms)&...)`.
/// In agnocast mode, message_ptrs are created from the ipc_shared_ptrs, preserving
/// zero-copy semantics during the callback lifetime.
///
/// @note Current limitations:
///   - 2..8 message types per Synchronizer (upstream Signal9 has no 9-arg MFP overload).
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
/// @endcode
template <typename... Ms>
class Synchronizer<sync_policies::ApproximateTime<Ms...>>
: public PolicySynchronizer<
    ::message_filters::sync_policies::ApproximateTime<Ms...>,
    agnocast::message_filters::sync_policies::ApproximateTime<Ms...>, Ms...>
{
  using Base = PolicySynchronizer<
    ::message_filters::sync_policies::ApproximateTime<Ms...>,
    agnocast::message_filters::sync_policies::ApproximateTime<Ms...>, Ms...>;

public:
  Synchronizer(const sync_policies::ApproximateTime<Ms...> & policy, Subscriber<Ms> &... subs)
  : Base(policy.queue_size, subs...)
  {
  }
};

/// @brief Synchronizer specialization for the wrapper-layer ExactTime policy.
///
/// Same callback signature and zero-copy semantics as the ApproximateTime specialization;
/// only the sync policy differs (messages must share identical timestamps). Subject to the
/// same limitations (2..8 message types, connectInput() not supported).
/// @see Synchronizer<sync_policies::ApproximateTime<Ms...>> for a usage example.
template <typename... Ms>
class Synchronizer<sync_policies::ExactTime<Ms...>>
: public PolicySynchronizer<
    ::message_filters::sync_policies::ExactTime<Ms...>,
    agnocast::message_filters::sync_policies::ExactTime<Ms...>, Ms...>
{
  using Base = PolicySynchronizer<
    ::message_filters::sync_policies::ExactTime<Ms...>,
    agnocast::message_filters::sync_policies::ExactTime<Ms...>, Ms...>;

public:
  Synchronizer(const sync_policies::ExactTime<Ms...> & policy, Subscriber<Ms> &... subs)
  : Base(policy.queue_size, subs...)
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

/// @brief Curated message_filters Subscriber for the non-Agnocast build.
///
/// Holds a ::message_filters::Subscriber<M, rclcpp::Node> by value and forwards only the curated
/// member set (subscribe / unsubscribe), instead of deriving from it. This keeps the public surface
/// identical to the Agnocast build (which takes the wrapper Node), so code that compiles under
/// ENABLE_AGNOCAST=0 also compiles under =1. Deriving from the upstream Subscriber would instead
/// leak its full filter API (e.g. registerCallback, the raw node-pointer subscribe overloads) into
/// the =0 build, allowing =0-only code that breaks under =1.
template <class M>
class Subscriber
{
public:
  using Base = ::message_filters::Subscriber<M, rclcpp::Node>;

  Subscriber() = default;

  Subscriber(
    autoware::agnocast_wrapper::Node * node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    subscribe(node, topic, qos);
  }

  void subscribe(
    autoware::agnocast_wrapper::Node * node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    sub_.subscribe(node->get_rclcpp_node().get(), topic, qos);
  }

  void unsubscribe() { sub_.unsubscribe(); }

  // Internal API: used by Synchronizer to connect the underlying filter.
  // Not intended for downstream use.
  Base & rclcpp_subscriber() { return sub_; }

private:
  Base sub_;
};

/// @brief Policy tags mirroring the Agnocast build. Carry only the queue size; the underlying
///        rclcpp policy is selected inside Synchronizer<...>. Distinct from
///        ::message_filters::sync_policies::ApproximateTime / ExactTime so the wrapper owns its
///        Synchronizer surface rather than aliasing upstream.
namespace sync_policies
{
template <typename... Ms>
struct ApproximateTime
{
  uint32_t queue_size;  ///< Queue size forwarded to the underlying sync policy.
  explicit ApproximateTime(uint32_t qs) noexcept : queue_size(qs) {}
};

template <typename... Ms>
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
    "Only sync_policies::ApproximateTime<Ms...> and sync_policies::ExactTime<Ms...> are "
    "supported.");
};

/// @brief Common synchronizer wrapper parameterized by the underlying upstream policy
///        (non-Agnocast build). Use through Synchronizer<sync_policies::ApproximateTime<Ms...>> /
///        Synchronizer<sync_policies::ExactTime<Ms...>>.
///
/// Exposes only the constructor (policy + Subscriber refs) and registerCallback, instead of
/// aliasing ::message_filters::Synchronizer directly. This keeps the surface identical to the
/// Agnocast build (e.g. connectInput stays unsupported in both), so code compiles under both
/// ENABLE_AGNOCAST=0 and =1. The callback receives `(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(Ms)&
/// ...)`, which in this build is `std::shared_ptr<const Ms>` — exactly what upstream delivers, so
/// the callable is forwarded to the underlying synchronizer unchanged.
template <typename UpstreamPolicy, typename... Ms>
class PolicySynchronizer
{
public:
  PolicySynchronizer(uint32_t queue_size, Subscriber<Ms> &... subs)
  : sync_(UpstreamPolicy(queue_size), subs.rclcpp_subscriber()...)
  {
  }

  // Non-copyable and non-movable: the underlying synchronizer holds connections into the
  // supplied subscribers, so its address must stay stable for the registrations' lifetime.
  PolicySynchronizer(const PolicySynchronizer &) = delete;
  PolicySynchronizer & operator=(const PolicySynchronizer &) = delete;
  PolicySynchronizer(PolicySynchronizer &&) = delete;
  PolicySynchronizer & operator=(PolicySynchronizer &&) = delete;

  template <class C>
  ::message_filters::Connection registerCallback(C & callback)
  {
    return sync_.registerCallback(callback);
  }

  template <class C>
  ::message_filters::Connection registerCallback(const C & callback)
  {
    return sync_.registerCallback(callback);
  }

  template <class C, typename T>
  ::message_filters::Connection registerCallback(C & callback, T * t)
  {
    return sync_.registerCallback(callback, t);
  }

  template <class C, typename T>
  ::message_filters::Connection registerCallback(const C & callback, T * t)
  {
    return sync_.registerCallback(callback, t);
  }

private:
  ::message_filters::Synchronizer<UpstreamPolicy> sync_;
};

template <typename... Ms>
class Synchronizer<sync_policies::ApproximateTime<Ms...>>
: public PolicySynchronizer<::message_filters::sync_policies::ApproximateTime<Ms...>, Ms...>
{
  using Base = PolicySynchronizer<::message_filters::sync_policies::ApproximateTime<Ms...>, Ms...>;

public:
  Synchronizer(const sync_policies::ApproximateTime<Ms...> & policy, Subscriber<Ms> &... subs)
  : Base(policy.queue_size, subs...)
  {
  }
};

/// @brief Synchronizer specialization for the wrapper-layer ExactTime policy (non-Agnocast build).
/// @see Synchronizer<sync_policies::ApproximateTime<Ms...>> for the shared implementation.
template <typename... Ms>
class Synchronizer<sync_policies::ExactTime<Ms...>>
: public PolicySynchronizer<::message_filters::sync_policies::ExactTime<Ms...>, Ms...>
{
  using Base = PolicySynchronizer<::message_filters::sync_policies::ExactTime<Ms...>, Ms...>;

public:
  Synchronizer(const sync_policies::ExactTime<Ms...> & policy, Subscriber<Ms> &... subs)
  : Base(policy.queue_size, subs...)
  {
  }
};

}  // namespace message_filters
}  // namespace agnocast_wrapper
}  // namespace autoware

#endif
