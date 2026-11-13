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

// Type-erased ("generic") client abstraction; see generic_service.hpp for the design rationale
// this mirrors (runtime service_type string, std::shared_ptr<void> request/response, aliased --
// not copied -- onto Agnocast's shared-memory buffers).
//
// Jazzy's rclcpp does have a native GenericClient, but its async_send_request() has no
// callback overload (future-only) and its Request/Response are raw `void *`, not owning
// pointers, so the caller must manage typesupport-based allocation itself; Humble has no
// GenericClient at all. Rather than special-case Jazzy for half an API, ROS2GenericClient is
// hand-rolled the same way ROS2GenericService is, giving one convenience API (create_request(),
// callback or future) on every supported distro and both ENABLE_AGNOCAST values.

#include "autoware/agnocast_wrapper/detail/service_typesupport.hpp"

#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>

#include <rcl/client.h>
#include <rmw/types.h>

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

namespace autoware::agnocast_wrapper
{

/// Common interface implemented by both ROS2GenericClient and (when Agnocast is enabled)
/// AgnocastGenericClient. The two backends are kept behaviorally identical wherever the
/// underlying transport allows it; where they cannot be, the ROS2 side is never restricted just
/// to match Agnocast -- rclcpp's own usability is preserved, and the gap is documented below
/// instead of papered over.
///
/// Limitation (unavoidable difference between the two backends): async_send_request()/
/// cancel_request()'s requirement that @p request be the exact object this client's own
/// create_request() returned is load-bearing only on the Agnocast backend, which must look the
/// borrowed shared-memory buffer back up by that identity to send or release it -- a foreign or
/// already-resolved handle has nothing valid to send/release and is rejected. The ROS2 backend
/// has no such bookkeeping: @p request is an ordinary heap buffer that is sent by pointer and
/// freed by its own deleter regardless of how it was allocated, so it accepts any correctly-typed
/// request, matching plain rclcpp's own usability -- but code relying on that is not portable to
/// Agnocast.
class GenericClient
{
public:
  using SharedPtr = std::shared_ptr<GenericClient>;
  using SharedRequest = std::shared_ptr<void>;
  using SharedResponse = std::shared_ptr<void>;
  using SharedFuture = std::shared_future<SharedResponse>;
  using ResponseCallback = std::function<void(SharedFuture)>;

  virtual ~GenericClient() = default;

  /// Service name after remapping.
  virtual const char * get_service_name() const = 0;

  /// True if a matching service server is currently available.
  virtual bool service_is_ready() const = 0;

  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_service(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    return wait_for_service_impl(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  /// Allocate a zero-initialized request buffer sized for this client's request type. On the
  /// Agnocast path, the returned request must eventually reach exactly one async_send_request()
  /// or cancel_request() call -- see cancel_request()'s doc comment for why. On the ROS2 path
  /// there is no such obligation: dropping the returned request without sending or cancelling it
  /// simply frees it, matching plain rclcpp's own usability -- see the class-level "Limitation"
  /// note.
  virtual SharedRequest create_request() = 0;

  /// Send @p request asynchronously; @p callback is invoked with the response once it arrives.
  /// On the Agnocast path, @p request must be the exact object this client's own create_request()
  /// returned, and this is enforced. The ROS2 path accepts any correctly-typed request -- see the
  /// class-level "Limitation" note.
  /// @throws std::runtime_error on the Agnocast path if @p request was not obtained from this
  ///         client's own create_request(), or was already sent or cancelled.
  virtual SharedFuture async_send_request(SharedRequest request, ResponseCallback callback) = 0;
  /// Send @p request asynchronously; call .get() on the returned future to block for the
  /// response. See the callback overload above for the identity requirement on the Agnocast path.
  /// @throws std::runtime_error under the same condition as the callback overload above.
  virtual SharedFuture async_send_request(SharedRequest request) = 0;

  /// Abandon @p request without sending it (e.g. deciding not to call after all). On the Agnocast
  /// path @p request must be the exact object this client's own create_request() returned: it is
  /// a borrowed, not-yet-published shared-memory buffer, and dropping it without either sending or
  /// explicitly cancelling it terminates the process (agnocast::ipc_shared_ptr<void>::reset() has
  /// no other way to free a type-erased publisher-side buffer). A no-op on the ROS2 path, whose
  /// request is an ordinary heap buffer freed by its own deleter regardless of origin -- see the
  /// class-level "Limitation" note.
  virtual void cancel_request(SharedRequest request) = 0;

protected:
  virtual bool wait_for_service_impl(std::chrono::nanoseconds timeout) = 0;
};

/// Hand-rolled GenericClient for plain rclcpp/rcl -- see the file-level comment for why this isn't
/// just rclcpp::GenericClient. Adapted from autoware_generic_service_divider's own GenericClient,
/// which has run this exact rcl_client_init()/rcl_send_request() pattern in production.
class ROS2GenericClient : public GenericClient, public rclcpp::ClientBase
{
public:
  ROS2GenericClient(
    rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
    const rclcpp::QoS & qos = rclcpp::ServicesQoS());

  ~ROS2GenericClient() override = default;

  const char * get_service_name() const override { return ClientBase::get_service_name(); }
  bool service_is_ready() const override { return ClientBase::service_is_ready(); }

  // --- rclcpp::ClientBase overrides (driven by the executor) ---
  std::shared_ptr<void> create_response() override;
  std::shared_ptr<rmw_request_id_t> create_request_header() override;
  void handle_response(
    std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> response) override;

  // --- GenericClient overrides ---
  SharedRequest create_request() override;
  // Accepts any request pointing to a correctly-typed heap buffer, whether or not it came from
  // this (or any) client's own create_request() -- create_request() is a convenience allocator
  // here, not a precondition, unlike on the Agnocast backend. See GenericClient's class-level
  // "Limitation" note for why this is intentionally more permissive than Agnocast.
  SharedFuture async_send_request(SharedRequest request, ResponseCallback callback) override;
  SharedFuture async_send_request(SharedRequest request) override;
  // A true no-op: request's own deleter (fini_function + delete[]) frees the heap buffer normally
  // when it goes out of scope, so there is nothing to release here.
  void cancel_request(SharedRequest request) override;

protected:
  bool wait_for_service_impl(std::chrono::nanoseconds timeout) override
  {
    return ClientBase::wait_for_service_nanoseconds(timeout);
  }

private:
  struct PendingResponse
  {
    std::promise<SharedResponse> promise;
    SharedFuture future;        // promise.get_future().share(), retrieved exactly once up front
    ResponseCallback callback;  // empty for the future-only overload
  };

  detail::ServiceTsBundle ts_bundle_;
  std::mutex pending_mutex_;
  std::unordered_map<int64_t, PendingResponse> pending_;
};

}  // namespace autoware::agnocast_wrapper

#ifdef USE_AGNOCAST_ENABLED

#include "autoware/agnocast_wrapper/runtime.hpp"

#include <agnocast/agnocast.hpp>

#include <stdexcept>

namespace autoware::agnocast_wrapper
{

/// Wraps agnocast::GenericClient, aliasing (never copying) its ipc_shared_ptr<void> request and
/// response buffers as std::shared_ptr<void> -- see generic_service.hpp's file-level comment.
class AgnocastGenericClient : public GenericClient
{
public:
  template <typename NodeT>
  AgnocastGenericClient(
    NodeT * node, const std::string & service_name, const std::string & service_type,
    const rclcpp::QoS & qos, const rclcpp::CallbackGroup::SharedPtr & group)
  : client_(std::make_shared<agnocast::GenericClient>(node, service_name, service_type, qos, group))
  {
  }

  const char * get_service_name() const override { return client_->get_service_name(); }
  bool service_is_ready() const override { return client_->service_is_ready(); }

  SharedRequest create_request() override
  {
    agnocast::ipc_shared_ptr<void> request = client_->borrow_loaned_request();
    void * raw = request.get();
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_.emplace(raw, std::move(request));
    }
    // No custom deleter here (unlike the service side): ownership stays with pending_ until
    // async_send_request() or cancel_request() claims it, so dropping this handle without calling
    // either does nothing by itself -- see cancel_request()'s doc comment on GenericClient for why
    // that matters.
    return std::shared_ptr<void>(raw, [](void *) {});
  }

  SharedFuture async_send_request(SharedRequest request, ResponseCallback callback) override
  {
    auto agnocast_request = take_pending(request);
    auto promise = std::make_shared<std::promise<SharedResponse>>();
    SharedFuture future = promise->get_future().share();
    // agnocast's own callback fires from whichever thread its executor uses when the response
    // arrives; no extra thread is spun up here.
    client_->async_send_request(
      std::move(agnocast_request),
      [promise, callback, future](agnocast::GenericClient::SharedFuture agnocast_future) {
        agnocast::ipc_shared_ptr<void> response_copy = agnocast_future.get();
        promise->set_value(alias(std::move(response_copy)));
        if (callback) {
          callback(future);
        }
      });
    return future;
  }

  SharedFuture async_send_request(SharedRequest request) override
  {
    auto agnocast_request = take_pending(request);
    auto promise = std::make_shared<std::promise<SharedResponse>>();
    SharedFuture future = promise->get_future().share();
    client_->async_send_request(
      std::move(agnocast_request),
      [promise](agnocast::GenericClient::SharedFuture agnocast_future) {
        agnocast::ipc_shared_ptr<void> response_copy = agnocast_future.get();
        promise->set_value(alias(std::move(response_copy)));
      });
    return future;
  }

  // Tolerant of an unknown/already-sent request (mirrors
  // AgnocastGenericService::cancel_response()): meant to be callable from error-handling code
  // without its own try/catch.
  void cancel_request(SharedRequest request) override
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    auto it = pending_.find(request.get());
    if (it == pending_.end()) {
      return;
    }
    client_->cancel_request(std::move(it->second));
    pending_.erase(it);
  }

  // Guards against this client being destroyed with created-but-unsent requests still
  // outstanding: a borrowed request left in pending_ would otherwise terminate the process when
  // this destructor's implicit pending_ teardown drops it (see cancel_request()'s doc comment on
  // GenericClient).
  ~AgnocastGenericClient() override
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    for (auto & [unused_key, request] : pending_) {
      (void)unused_key;
      client_->cancel_request(std::move(request));
    }
  }

protected:
  bool wait_for_service_impl(std::chrono::nanoseconds timeout) override
  {
    return client_->wait_for_service(timeout);
  }

private:
  agnocast::ipc_shared_ptr<void> take_pending(const SharedRequest & request)
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    auto it = pending_.find(request.get());
    if (it == pending_.end()) {
      throw std::runtime_error(
        "AgnocastGenericClient::async_send_request(): request was not obtained from this "
        "client's own create_request(), or was already sent or cancelled");
    }
    auto held = std::move(it->second);
    pending_.erase(it);
    return held;
  }

  static SharedResponse alias(agnocast::ipc_shared_ptr<void> && held)
  {
    void * raw = held.get();
    return std::shared_ptr<void>(raw, [held = std::move(held)](void *) mutable {});
  }

  std::shared_ptr<agnocast::GenericClient> client_;
  std::mutex pending_mutex_;
  std::unordered_map<const void *, agnocast::ipc_shared_ptr<void>> pending_;
};

/// Free-function form for incremental adoption on a node that stays an ordinary rclcpp::Node (see
/// the Node member of the same name for the wrapper-Node form, which also supports
/// agnocast::Node). Reach it through AUTOWARE_CREATE_GENERIC_CLIENT1/2/3(_ON_NODE) rather than
/// calling it directly, so the same call site also compiles under ENABLE_AGNOCAST=0, where this
/// same name resolves to the ROS2GenericClient-returning overload below instead.
inline GenericClient::SharedPtr create_generic_client(
  rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
  const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
  const rclcpp::CallbackGroup::SharedPtr & group = nullptr)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastGenericClient>(node, service_name, service_type, qos, group);
  }
  auto client = std::make_shared<ROS2GenericClient>(node, service_name, service_type, qos);
  std::shared_ptr<rclcpp::ClientBase> base = client;
  node->get_node_services_interface()->add_client(std::move(base), group);
  return client;
}

}  // namespace autoware::agnocast_wrapper

#else

namespace autoware::agnocast_wrapper
{

/// Free-function form for incremental adoption on a node that stays an ordinary rclcpp::Node. This
/// is the Method 1 (macro + free function) entry point; reach it through
/// AUTOWARE_CREATE_GENERIC_CLIENT1/2/3(_ON_NODE) rather than calling it directly, so the same call
/// site also compiles under ENABLE_AGNOCAST=1, where this overload does not exist and the macro
/// instead forwards to the AgnocastGenericClient/ROS2GenericClient-backed free function above.
inline GenericClient::SharedPtr create_generic_client(
  rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
  const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
  const rclcpp::CallbackGroup::SharedPtr & group = nullptr)
{
  auto client = std::make_shared<ROS2GenericClient>(node, service_name, service_type, qos);
  std::shared_ptr<rclcpp::ClientBase> base = client;
  node->get_node_services_interface()->add_client(std::move(base), group);
  return client;
}

}  // namespace autoware::agnocast_wrapper

#endif
