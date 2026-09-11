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
#include <tuple>
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
  /// Signature: `void(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(Ms) &...)` or
  /// `void(const typename Ms::ConstSharedPtr &...)`; the message_ptr form is probed first, so a
  /// callable accepting both keeps resolving to it.
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
    return registerCallbackInternal(makeAdapter(callback));
  }

  template <class C>
  ::message_filters::Connection registerCallback(const C & callback)
  {
    return registerCallbackInternal(makeAdapter(callback));
  }

  template <class C, typename T>
  ::message_filters::Connection registerCallback(C & callback, T * t)
  {
    return registerCallbackInternal(makeAdapter(callback, t));
  }

  template <class C, typename T>
  ::message_filters::Connection registerCallback(const C & callback, T * t)
  {
    return registerCallbackInternal(makeAdapter(callback, t));
  }

private:
  // Type-erased handle for `adapters_`. Each registration makes a distinct adapter type, so the
  // vector needs a common base; upstream keeps only a raw pointer into the adapter, which is why
  // the synchronizer owns them at all.
  struct AdapterBase
  {
    virtual ~AdapterBase() = default;
  };

  // std::shared_ptr<const void> names no message type, so it needs a pattern that does in order to
  // be repeated once per synchronized message.
  template <typename>
  struct as_void_ptr
  {
    using type = std::shared_ptr<const void>;
  };

  /// Per-registration adapter: owns the user callable and gives each backend the argument shape
  /// that callable declares.
  ///
  /// @tparam C     the user callable, or a pointer to member function when `Bound` is given
  /// @tparam Bound empty for a free callable; the instance pointer for a member function
  template <class C, class... Bound>
  struct CallbackAdapter : AdapterBase
  {
    C fn;
    std::tuple<Bound...> bound;

    CallbackAdapter(C fn_in, Bound... bound_in) : fn(std::move(fn_in)), bound(bound_in...) {}

    static constexpr bool takes_message_ptr =
      std::is_invocable_v<C &, Bound..., const AUTOWARE_MESSAGE_CONST_SHARED_PTR(Ms) & ...>;

    // The two exclusions are shapes std::shared_ptr<const M> converts to but upstream
    // message_filters has no ParameterAdapter for, so they would compile in the agnocast-enabled
    // build only. subscription.hpp excludes the same two.
    static constexpr bool takes_const_shared_ptr =
      std::is_invocable_v<C &, Bound..., const typename Ms::ConstSharedPtr &...> &&
      !std::is_invocable_v<C &, Bound..., std::weak_ptr<const Ms>...> &&
      !std::is_invocable_v<C &, Bound..., typename as_void_ptr<Ms>::type...>;

    static_assert(
      takes_message_ptr || takes_const_shared_ptr,
      "synchronizer callback should be invocable with either "
      "const AUTOWARE_MESSAGE_CONST_SHARED_PTR(M) & ... or const M::ConstSharedPtr & ..., and not "
      "with std::weak_ptr<const M> ... or std::shared_ptr<const void> ..., which upstream "
      "message_filters cannot deliver");

    template <typename... Args>
    void call(const Args &... args)
    {
      std::apply([&](Bound... b) { std::invoke(fn, b..., args...); }, bound);
    }

    void agnocastInvoke(const agnocast::message_filters::MessageEvent<const Ms> &... es)
    {
      if constexpr (takes_message_ptr) {
        // Wrap ipc_shared_ptr in message_ptr (copies ipc_shared_ptr refcount, not data)
        call(AUTOWARE_MESSAGE_CONST_SHARED_PTR(Ms)(
          agnocast::ipc_shared_ptr<const Ms>(es.getMessage()))...);
      } else {
        call(detail::to_std_shared_ptr(agnocast::ipc_shared_ptr<const Ms>(es.getMessage()))...);
      }
    }

    void rclcppInvoke(const typename Ms::ConstSharedPtr &... ms)
    {
      if constexpr (takes_message_ptr) {
        call(AUTOWARE_MESSAGE_CONST_SHARED_PTR(Ms)(std::shared_ptr<const Ms>(ms))...);
      } else {
        call(ms...);
      }
    }
  };

  using AdapterPtr = std::unique_ptr<AdapterBase>;
  using RclcppSync = ::message_filters::Synchronizer<RclcppPolicy>;
  using AgnocastSync = agnocast::message_filters::Synchronizer<AgnocastPolicy>;

  template <class C, class... Bound>
  static auto makeAdapter(C callback, Bound... bound)
  {
    // Upstream constrains the two-argument overload to void (T::*)(P0, P1). std::invoke would
    // otherwise also accept a callable whose first parameter merely takes the instance, which
    // upstream turns away in overload resolution.
    static_assert(
      sizeof...(Bound) == 0 || std::is_member_function_pointer_v<C>,
      "registerCallback(callback, instance) takes a pointer to a member function of the instance");
    return std::make_unique<CallbackAdapter<C, Bound...>>(std::move(callback), bound...);
  }

  template <class AdapterT>
  ::message_filters::Connection registerCallbackInternal(std::unique_ptr<AdapterT> adapter)
  {
    auto * const adapter_raw = adapter.get();

    auto upstream_conn = std::visit(
      [adapter_raw](auto & sync) -> ::message_filters::Connection {
        using SyncT = std::decay_t<decltype(sync)>;
        if constexpr (std::is_same_v<SyncT, AgnocastSync>) {
          return sync.registerCallback(&AdapterT::agnocastInvoke, adapter_raw);
        } else {
          return sync.registerCallback(&AdapterT::rclcppInvoke, adapter_raw);
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
/// The callback receives `(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(Ms)&...)` or
/// `(const typename Ms::ConstSharedPtr&...)`. In agnocast mode both forms alias the
/// ipc_shared_ptr rather than copying, and either may be retained past the callback, but not past
/// the subscription that delivered it: releasing one after that point ends the process.
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

namespace autoware::agnocast_wrapper
{
namespace message_filters
{

/// @brief message_filters Subscriber (non-Agnocast build); forwards the wrapper Node to the
///        underlying rclcpp::Node. Inherited upstream members (connectInput, registerCallback, ...)
///        are non-portable — they do not exist on the Agnocast-build Subscriber.
template <class M>
class Subscriber : public ::message_filters::Subscriber<M, rclcpp::Node>
{
public:
  using Base = ::message_filters::Subscriber<M, rclcpp::Node>;

  Subscriber() = default;

  Subscriber(
    autoware::agnocast_wrapper::Node * node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default)
  : Base(node->get_rclcpp_node().get(), topic, qos)
  {
  }

  void subscribe(
    autoware::agnocast_wrapper::Node * node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    Base::subscribe(node->get_rclcpp_node().get(), topic, qos);
  }
};

namespace sync_policies
{
template <typename... Ms>
using ApproximateTime = ::message_filters::sync_policies::ApproximateTime<Ms...>;
template <typename... Ms>
using ExactTime = ::message_filters::sync_policies::ExactTime<Ms...>;
}  // namespace sync_policies

template <typename Policy>
using Synchronizer = ::message_filters::Synchronizer<Policy>;

}  // namespace message_filters
}  // namespace autoware::agnocast_wrapper

#endif
