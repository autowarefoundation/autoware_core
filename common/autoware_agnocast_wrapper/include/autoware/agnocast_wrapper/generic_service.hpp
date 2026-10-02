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

// Type-erased ("generic") service abstraction, mirroring generic_publisher.hpp /
// generic_subscription.hpp: the service type is supplied as a runtime string (e.g.
// "std_srvs/srv/SetBool") rather than a compile-time template argument, for nodes -- such as
// autoware_generic_service_divider -- that forward arbitrary services without linking against
// their srv packages.
//
// Unlike generic pub/sub, neither Humble's nor Jazzy's rclcpp has a native GenericService (Jazzy
// has rclcpp::GenericClient but not the server side), so this header always defines its own
// ROS2GenericService -- there is no native type to fall back to in the non-Agnocast build the way
// generic_publisher.hpp falls back to rclcpp::GenericPublisher. The abstract GenericService
// interface and ROS2GenericService are therefore declared unconditionally, and only
// AgnocastGenericService plus the runtime use_agnocast() switch are gated behind
// USE_AGNOCAST_ENABLED.
//
// Both request and response are always std::shared_ptr<void> rather than a typed message_ptr --
// there is no compile-time type to allocate one for. On the Agnocast path, AgnocastGenericService
// hands out that std::shared_ptr<void> as an *alias* onto the real shared-memory buffer (its
// deleter just keeps the underlying agnocast::ipc_shared_ptr<void> alive), not a copy: reading or
// writing through it touches the same memory Agnocast will publish, so this stays genuinely
// zero-copy. A generic byte-for-byte copy between two buffers of a runtime-only-known type would
// not be safe in general (a std::string/std::vector member owns heap memory the copy wouldn't
// duplicate), which is why aliasing -- not copying -- is the mechanism here.

#include "autoware/agnocast_wrapper/detail/service_typesupport.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/service.hpp>

#include <rcl/service.h>
#include <rmw/types.h>

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>

namespace autoware::agnocast_wrapper
{

class GenericService;

/// "Basic" callback: invoked with (request, response); the response is populated in place and
/// sent automatically once the callback returns.
using GenericServiceBasicCallback =
  std::function<void(std::shared_ptr<void> /*request*/, std::shared_ptr<void> /*response*/)>;
/// "Deferred" callback: invoked with (service, request); the service must later be answered by
/// exactly one send_response() call (e.g. after fanning the request out to other services and
/// collecting their responses, as autoware_generic_service_divider does).
using GenericServiceDeferredCallback =
  std::function<void(std::shared_ptr<GenericService>, std::shared_ptr<void> /*request*/)>;

template <typename Func>
inline constexpr bool is_generic_service_basic_callback_v =
  std::is_invocable_v<std::decay_t<Func>, std::shared_ptr<void>, std::shared_ptr<void>>;

template <typename Func>
inline constexpr bool is_generic_service_deferred_callback_v =
  std::is_invocable_v<std::decay_t<Func>, std::shared_ptr<GenericService>, std::shared_ptr<void>>;

/// Common interface implemented by both ROS2GenericService and (when Agnocast is enabled)
/// AgnocastGenericService, so a node can hold AUTOWARE_GENERIC_SERVICE_PTR without caring which
/// backend actually answers it. The two backends are kept behaviorally identical wherever the
/// underlying transport allows it; where they cannot be, the ROS2 side is never restricted just
/// to match Agnocast -- rclcpp's own usability is preserved, and the gap is documented below
/// instead of papered over.
///
/// Limitations (unavoidable differences between the two backends):
/// - create_response(request) may be called more than once for the same still-pending request on
///   the ROS2 backend (each call yields an independent heap buffer); on the Agnocast backend it
///   may be called only once per pending request until that response is sent or cancelled,
///   because the response is a single borrowed shared-memory buffer, not a heap allocation --
///   handing out a second one without resolving the first would drop the still-borrowed first
///   buffer, which terminates the process (see cancel_response() below). A second call throws
///   instead.
/// - send_response(request, response)'s @p response must be the exact object this service's own
///   create_response(request) returned, but this is only load-bearing on the Agnocast backend
///   (the shared-memory channel can only publish the buffer it lent out). The ROS2 backend never
///   enforces this and keeps accepting any correctly-typed heap object, matching plain rclcpp.
/// - A borrowed-but-unsent Agnocast response left dangling (see cancel_response() below)
///   terminates the process; the ROS2 equivalent is an ordinary heap object that is simply freed.
///   This is inherent to Agnocast's zero-copy shared-memory model and has no ROS2-side
///   counterpart to unify against.
class GenericService : public std::enable_shared_from_this<GenericService>
{
public:
  using SharedPtr = std::shared_ptr<GenericService>;
  using SharedRequest = std::shared_ptr<void>;
  using SharedResponse = std::shared_ptr<void>;

  virtual ~GenericService() = default;

  /// Service name after remapping.
  virtual const char * get_service_name() const = 0;

  /// Allocate a zero-initialized response buffer sized for this service's response type, for
  /// @p request (the object a deferred callback received). Only needed by a deferred callback; a
  /// basic callback receives one already.
  /// May be called more than once for the same pending @p request on the ROS2 backend (each call
  /// simply heap-allocates another independent buffer; discarding an earlier one is harmless) --
  /// this matches plain rclcpp's own permissiveness and is intentionally left unrestricted here.
  /// On the Agnocast backend a second call for the same still-pending request instead throws: see
  /// the class-level "Limitations" note for why the shared-memory transport cannot safely hand out
  /// a second independent borrowed buffer without first publishing or cancelling the one already
  /// borrowed.
  /// @throws std::runtime_error if @p request is not a currently pending deferred call on this
  ///         service (already answered, or not obtained from this service's own callback) -- on
  ///         both backends -- or, on the Agnocast backend only, if create_response() was already
  ///         called for this same request and that response has not yet been sent or cancelled.
  virtual SharedResponse create_response(const SharedRequest & request) = 0;

  /// Send @p response for @p request. @p request must be the exact object the callback received
  /// (identity, not value, is used to find the matching call); every request accepted by a
  /// deferred callback must eventually reach exactly one send_response() or cancel_response() call.
  /// On the Agnocast path, @p response must additionally be the exact object this service's own
  /// create_response(request) returned: Agnocast has no way to publish an arbitrary heap object
  /// through a service's shared-memory channel, so answering with a response that came from
  /// somewhere else (a different GenericClient's own received response, or a freshly-built error
  /// response, for example) requires copying its content into a buffer obtained from this
  /// service's own create_response(request) first. The ROS2 path has no such requirement -- being
  /// an ordinary heap buffer, any correctly-typed object works, matching plain rclcpp's own
  /// usability -- but code relying on that is not portable to Agnocast; see the class-level
  /// "Limitations" note.
  /// @throws std::runtime_error if @p request is not a currently pending deferred call on this
  ///         service (already answered, or not obtained from this service's own callback), or (on
  ///         the Agnocast path only) if @p response was not obtained via this service's own
  ///         create_response(request) for this same request.
  virtual void send_response(const SharedRequest & request, SharedResponse response) = 0;

  /// Abandon @p response for @p request without sending it (e.g. the deferred handler errored out
  /// after calling create_response()). On the Agnocast path @p response is a borrowed,
  /// not-yet-published shared-memory buffer; dropping it without either sending or explicitly
  /// cancelling it terminates the process (agnocast::ipc_shared_ptr<void>::reset() has no other way
  /// to free a type-erased publisher-side buffer), so every create_response() must be paired with
  /// exactly one send_response() or cancel_response(). A no-op on the ROS2 path, whose response is
  /// an ordinary heap buffer.
  virtual void cancel_response(const SharedRequest & request, SharedResponse response) = 0;
};

/// Hand-rolled GenericService for plain rclcpp/rcl (there is no native one to wrap on either
/// Humble or Jazzy). Adapted from autoware_generic_service_divider's own GenericService, which has
/// run this exact rcl_service_init()/rcl_send_response() pattern in production; the two are kept
/// as separate types because they live in different packages, not because the logic differs.
class ROS2GenericService : public GenericService, public rclcpp::ServiceBase
{
  class CallbackDispatcher
  {
    std::variant<GenericServiceBasicCallback, GenericServiceDeferredCallback> callback_;

  public:
    template <typename Func>
    explicit CallbackDispatcher(Func && callback)
    {
      static_assert(
        is_generic_service_basic_callback_v<Func> || is_generic_service_deferred_callback_v<Func>,
        "Callback must be invocable with either (SharedRequest, SharedResponse) [basic] or "
        "(GenericService::SharedPtr, SharedRequest) [deferred]");
      if constexpr (is_generic_service_basic_callback_v<Func>) {
        callback_.emplace<GenericServiceBasicCallback>(std::forward<Func>(callback));
      } else {
        callback_.emplace<GenericServiceDeferredCallback>(std::forward<Func>(callback));
      }
    }

    bool is_deferred() const
    {
      return std::holds_alternative<GenericServiceDeferredCallback>(callback_);
    }

    void invoke_basic(
      const GenericService::SharedRequest & request,
      const GenericService::SharedResponse & response)
    {
      std::get<GenericServiceBasicCallback>(callback_)(request, response);
    }

    void invoke_deferred(
      const GenericService::SharedPtr & service, const GenericService::SharedRequest & request)
    {
      std::get<GenericServiceDeferredCallback>(callback_)(service, request);
    }
  };

public:
  template <typename Func>
  ROS2GenericService(
    rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
    Func && callback, const rclcpp::QoS & qos = rclcpp::ServicesQoS())
  : rclcpp::ServiceBase(node->get_node_base_interface()->get_shared_rcl_node_handle()),
    dispatcher_(std::forward<Func>(callback)),
    service_name_(node->get_node_services_interface()->resolve_service_name(service_name)),
    ts_bundle_(detail::load_service_typesupport(service_type))
  {
    init_service_handle(qos);
  }

  ~ROS2GenericService() override = default;

  // Stored explicitly rather than delegating to ServiceBase::get_service_name() the way
  // ROS2GenericClient delegates to ClientBase::get_service_name(): unlike ClientBase's version,
  // rclcpp::ServiceBase::get_service_name() is not const, so it cannot be called from this
  // override's const context.
  const char * get_service_name() const override { return service_name_.c_str(); }

  // --- rclcpp::ServiceBase overrides (driven by the executor) ---
  std::shared_ptr<void> create_request() override;
  std::shared_ptr<rmw_request_id_t> create_request_header() override;
  void handle_request(
    std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> request) override;

  // --- GenericService overrides ---
  SharedResponse create_response(const SharedRequest & request) override;
  void send_response(const SharedRequest & request, SharedResponse response) override;
  // A plain heap buffer, so cancelling is just erasing the pending request_header entry;
  // response's own deleter frees it normally when it goes out of scope.
  void cancel_response(const SharedRequest & request, SharedResponse response) override;

private:
  void init_service_handle(const rclcpp::QoS & qos);
  void send_response_impl(rmw_request_id_t & request_header, const SharedResponse & response);
  // Unvalidated allocation, shared by create_response() (which additionally checks `request`
  // against pending_) and handle_request()'s basic-callback branch (which has no pending_ entry
  // to check against, since only a deferred call registers one).
  SharedResponse allocate_response();

  CallbackDispatcher dispatcher_;
  std::string service_name_;
  detail::ServiceTsBundle ts_bundle_;

  std::mutex pending_mutex_;
  std::unordered_map<const void *, std::shared_ptr<rmw_request_id_t>> pending_;
};

}  // namespace autoware::agnocast_wrapper

#ifdef USE_AGNOCAST_ENABLED

#include "autoware/agnocast_wrapper/runtime.hpp"

#include <agnocast/agnocast.hpp>

namespace autoware::agnocast_wrapper
{

/// Wraps agnocast::GenericService, aliasing (never copying) its ipc_shared_ptr<void> request and
/// response buffers as std::shared_ptr<void> -- see the file-level comment.
class AgnocastGenericService : public GenericService
{
public:
  template <typename NodeT, typename Func>
  explicit AgnocastGenericService(
    NodeT * node, const std::string & service_name, const std::string & service_type,
    Func && callback, const rclcpp::QoS & qos, const rclcpp::CallbackGroup::SharedPtr & group)
  {
    static_assert(
      is_generic_service_basic_callback_v<Func> || is_generic_service_deferred_callback_v<Func>,
      "Callback must be invocable with either (SharedRequest, SharedResponse) [basic] or "
      "(GenericService::SharedPtr, SharedRequest) [deferred]");

    if constexpr (is_generic_service_basic_callback_v<Func>) {
      service_ = std::make_shared<agnocast::GenericService>(
        node, service_name, service_type,
        [callback = std::forward<Func>(callback)](
          agnocast::ipc_shared_ptr<void> && agnocast_request,
          agnocast::ipc_shared_ptr<void> && agnocast_response) {
          // response aliases the same memory agnocast::GenericService publishes right after this
          // callback returns (agnocast_response and its internal "response_double" share one
          // control block), so writes through it need no copy-back.
          auto request = alias(std::move(agnocast_request));
          auto response = alias(std::move(agnocast_response));
          callback(request, response);
        },
        qos, group);
    } else {
      service_ = std::make_shared<agnocast::GenericService>(
        node, service_name, service_type,
        [this, callback = std::forward<Func>(callback)](
          std::shared_ptr<agnocast::GenericService> agnocast_service,
          agnocast::ipc_shared_ptr<void> && agnocast_request) {
          // Keep a copy of the ipc_shared_ptr for create_response()/send_response() to use later;
          // ipc_shared_ptr is copyable (shared, refcounted), so this is a cheap refcount bump, not
          // a payload copy.
          agnocast::ipc_shared_ptr<void> request_copy = agnocast_request;
          auto request = alias(std::move(agnocast_request));
          {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending_.emplace(
              request.get(),
              PendingRequest{std::move(agnocast_service), std::move(request_copy), {}});
          }
          callback(shared_from_this(), request);
        },
        qos, group);
    }
  }

  const char * get_service_name() const override { return service_->get_service_name(); }

  SharedResponse create_response(const SharedRequest & request) override
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    auto it = pending_.find(request.get());
    if (it == pending_.end()) {
      throw std::runtime_error(
        "AgnocastGenericService::create_response(): request is not a pending deferred call on "
        "this service (already answered, or not obtained from this service's own callback)");
    }
    // Unlike the ROS2 backend (a heap allocation, so a second call is harmless), the response
    // here is a single borrowed shared-memory buffer: overwriting it below without resolving the
    // first would drop an unpublished agnocast::ipc_shared_ptr<void>, which terminates the
    // process. Reject the second call instead -- see the class-level "Limitations" note.
    if (it->second.response) {
      throw std::runtime_error(
        "AgnocastGenericService::create_response(): a response was already created for this "
        "request and has not yet been sent or cancelled (create_response() may only be called "
        "once per pending request on the Agnocast backend)");
    }
    agnocast::ipc_shared_ptr<void> response =
      it->second.service->borrow_loaned_response(it->second.request);
    it->second.response = response;  // copy: keep one, alias the other out to the caller
    return alias(std::move(response));
  }

  void send_response(const SharedRequest & request, SharedResponse response) override
  {
    PendingRequest pending;
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      auto it = pending_.find(request.get());
      if (it == pending_.end()) {
        throw std::runtime_error(
          "AgnocastGenericService::send_response(): request is not a pending deferred call on "
          "this service (already answered, or not obtained from this service's own callback)");
      }
      pending = std::move(it->second);
      pending_.erase(it);
    }
    // Identity check against the caller-supplied handle, not just "was create_response() ever
    // called": passing back a different (if same-typed) response object must be rejected rather
    // than silently substituting the one this service actually borrowed, since the two are not
    // interchangeable on the shared-memory transport the way they would be on plain rclcpp.
    if (!pending.response || pending.response.get() != response.get()) {
      // pending.response (if any) is the borrowed-but-still-unpublished buffer this service
      // actually lent out; the throw below would otherwise destroy it via stack unwinding while
      // still unresolved, terminating the process (agnocast::ipc_shared_ptr<void>::reset() has no
      // other way to free a type-erased publisher-side buffer). Release it the same way
      // cancel_response() does before reporting the caller's mistake.
      if (pending.response) {
        pending.service->cancel_response(std::move(pending.request), std::move(pending.response));
      }
      throw std::runtime_error(
        "AgnocastGenericService::send_response(): response was not obtained via this service's "
        "own create_response(request)");
    }
    pending.service->send_response(std::move(pending.request), std::move(pending.response));
  }

  // Tolerant of an unknown/already-resolved request (unlike send_response()): meant to be callable
  // from error-handling code without wrapping it in its own try/catch.
  void cancel_response(const SharedRequest & request, SharedResponse /*response*/) override
  {
    PendingRequest pending;
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      auto it = pending_.find(request.get());
      if (it == pending_.end()) {
        return;
      }
      pending = std::move(it->second);
      pending_.erase(it);
    }
    // pending.response is null (a plain, safe-to-drop subscriber-side request) when the deferred
    // handler never called create_response(); only a borrowed response needs the explicit cancel.
    if (pending.response) {
      pending.service->cancel_response(std::move(pending.request), std::move(pending.response));
    }
  }

  // Guards against a server shutting down with deferred calls still outstanding: a borrowed
  // response left in pending_ would otherwise terminate the process when this destructor's
  // implicit pending_ teardown drops it (see cancel_response()'s doc comment on GenericService).
  ~AgnocastGenericService() override
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    for (auto & [unused_key, pending] : pending_) {
      (void)unused_key;
      if (pending.response) {
        pending.service->cancel_response(std::move(pending.request), std::move(pending.response));
      }
    }
  }

private:
  static SharedRequest alias(agnocast::ipc_shared_ptr<void> && held)
  {
    void * raw = held.get();
    // The deleter's only job is to keep `held` (and, through it, agnocast's shared-memory slot)
    // alive as long as this std::shared_ptr<void> or one of its copies is; it never touches `raw`
    // itself, since that memory is agnocast's, not ours to free.
    return std::shared_ptr<void>(raw, [held = std::move(held)](void *) mutable {});
  }

  struct PendingRequest
  {
    std::shared_ptr<agnocast::GenericService> service;
    agnocast::ipc_shared_ptr<void> request;
    agnocast::ipc_shared_ptr<void> response;  // set once create_response() is called
  };

  std::shared_ptr<agnocast::GenericService> service_;
  std::mutex pending_mutex_;
  std::unordered_map<const void *, PendingRequest> pending_;
};

/// Free-function form for incremental adoption on a node that stays an ordinary rclcpp::Node (see
/// the Node member of the same name for the wrapper-Node form, which also supports
/// agnocast::Node). Reach it through AUTOWARE_CREATE_GENERIC_SERVICE2/3/4(_ON_NODE) rather than
/// calling it directly, so the same call site also compiles under ENABLE_AGNOCAST=0, where this
/// same name resolves to the ROS2GenericService-returning overload below instead.
template <typename Func>
GenericService::SharedPtr create_generic_service(
  rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
  Func && callback, const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
  const rclcpp::CallbackGroup::SharedPtr & group = nullptr)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastGenericService>(
      node, service_name, service_type, std::forward<Func>(callback), qos, group);
  }
  auto service = std::make_shared<ROS2GenericService>(
    node, service_name, service_type, std::forward<Func>(callback), qos);
  std::shared_ptr<rclcpp::ServiceBase> base = service;
  node->get_node_services_interface()->add_service(std::move(base), group);
  return service;
}

}  // namespace autoware::agnocast_wrapper

#else

namespace autoware::agnocast_wrapper
{

/// Free-function form for incremental adoption on a node that stays an ordinary rclcpp::Node. This
/// is the Method 1 (macro + free function) entry point; reach it through
/// AUTOWARE_CREATE_GENERIC_SERVICE2/3/4(_ON_NODE) rather than calling it directly, so the same call
/// site also compiles under ENABLE_AGNOCAST=1, where this overload does not exist and the macro
/// instead forwards to the AgnocastGenericService/ROS2GenericService-backed free function above.
template <typename Func>
GenericService::SharedPtr create_generic_service(
  rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
  Func && callback, const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
  const rclcpp::CallbackGroup::SharedPtr & group = nullptr)
{
  auto service = std::make_shared<ROS2GenericService>(
    node, service_name, service_type, std::forward<Func>(callback), qos);
  std::shared_ptr<rclcpp::ServiceBase> base = service;
  node->get_node_services_interface()->add_service(std::move(base), group);
  return service;
}

}  // namespace autoware::agnocast_wrapper

#endif
