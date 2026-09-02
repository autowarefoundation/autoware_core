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

// Subscription abstraction for the Agnocast build.

#ifdef USE_AGNOCAST_ENABLED

#include "autoware/agnocast_wrapper/macros.hpp"
#include "autoware/agnocast_wrapper/message_ptr.hpp"
#include "autoware/agnocast_wrapper/runtime.hpp"

#include <agnocast/agnocast.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace autoware::agnocast_wrapper
{

/// Translate rclcpp subscription options into the Agnocast ones. Only the three fields Agnocast
/// understands carry over; the rest (event callbacks, intra-process settings, allocators) have no
/// Agnocast counterpart because Agnocast does not go through rmw.
inline agnocast::SubscriptionOptions to_agnocast_subscription_options(
  const rclcpp::SubscriptionOptions & options)
{
  agnocast::SubscriptionOptions result;
  result.callback_group = options.callback_group;
  result.ignore_local_publications = options.ignore_local_publications;
  result.qos_overriding_options = options.qos_overriding_options;
  return result;
}

template <typename MessageT>
class Subscription
{
public:
  using SharedPtr = std::shared_ptr<Subscription<MessageT>>;

  virtual ~Subscription() = default;

  /// Polling retrieval, mirroring rclcpp::Subscription::take(). Copies the message out; prefer
  /// take_data(), which does not copy on the Agnocast path.
  /// @return true if a new message was written to @p out.
  /// @throw std::runtime_error on the Agnocast path when the subscription has a callback.
  virtual bool take(MessageT & out, rclcpp::MessageInfo & info) = 0;

  /// Latest new message as a shared pointer, or nullptr when nothing new has arrived. On the
  /// Agnocast path the returned pointer aliases the shared-memory message, so no payload is
  /// copied; while any copy is alive it pins one shared-memory entry.
  /// @throw std::runtime_error on the Agnocast path when the subscription has a callback.
  virtual std::shared_ptr<const MessageT> take_data() = 0;

  /// Effective QoS, mirroring rclcpp::SubscriptionBase::get_actual_qos().
  virtual rclcpp::QoS get_actual_qos() const = 0;

  /// Remap-resolved topic name, mirroring rclcpp::SubscriptionBase::get_topic_name().
  virtual const char * get_topic_name() const = 0;
};

template <typename MessageT>
class AgnocastSubscription : public Subscription<MessageT>
{
  // Exactly one of these holds the subscription. Callback delivery and polling are exclusive on
  // the Agnocast path, and that is a kernel constraint rather than a wrapper choice:
  // entry_node::referencing_subscribers carries one bit per (subscriber, entry), so a callback and
  // a take() on the same subscriber would each build an ipc_shared_ptr on that single bit, and
  // whichever was destroyed first would release the entry under the other. rclcpp::Subscription
  // can mix the two only because its take() copies into caller storage; Agnocast hands out a
  // reference into the publisher's shared memory.
  typename agnocast::Subscription<MessageT>::SharedPtr callback_subscription_;
  typename agnocast::TakeSubscription<MessageT>::SharedPtr take_subscription_;

  /// The subscription in whichever mode this object was constructed, for the queries that do not
  /// care which one it is.
  const agnocast::SubscriptionBase & handle() const
  {
    return callback_subscription_
             ? static_cast<const agnocast::SubscriptionBase &>(*callback_subscription_)
             : static_cast<const agnocast::SubscriptionBase &>(*take_subscription_);
  }

  agnocast::TakeSubscription<MessageT> & polling_handle()
  {
    if (!take_subscription_) {
      throw std::runtime_error(
        std::string("agnocast subscription on '") + handle().get_topic_name() +
        "' was created with a callback, so it cannot be polled: Agnocast fixes the delivery mode "
        "at construction. Create it without a callback to use take()/take_data().");
    }
    return *take_subscription_;
  }

public:
  template <typename NodeT, typename Func>
  explicit AgnocastSubscription(
    NodeT * node, const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
    const agnocast::SubscriptionOptions & options)
  {
    // TODO(Koichi98): AUTOWARE_MESSAGE_UNIQUE_PTR should be disallowed for Agnocast subscriptions.
    // Agnocast uses shared memory, so mutable exclusive ownership is semantically incorrect and
    // risks corrupting data read by other subscribers. Currently kept for compatibility with
    // CudaPointcloudPreprocessorNode which uses UNIQUE_PTR callbacks.
    static_assert(
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&> ||
        std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_CONST_SHARED_PTR(MessageT) &&> ||
        std::is_invocable_v<std::decay_t<Func>, std::shared_ptr<const MessageT>> ||
        std::is_invocable_v<std::decay_t<Func>, const MessageT &>,
      "callback should be invocable with an rvalue reference to either "
      "AUTOWARE_MESSAGE_UNIQUE_PTR or AUTOWARE_MESSAGE_CONST_SHARED_PTR, with "
      "MessageT::ConstSharedPtr, or with a const reference to the message type");

    constexpr bool is_message_ptr_callback =
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&> ||
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_CONST_SHARED_PTR(MessageT) &&>;
    // MessageT::ConstSharedPtr, for interfaces whose callback signature cannot be templated on
    // the pointer type -- notably autoware_component_interface_utils, which binds member
    // functions taking Message::ConstSharedPtr. Aliases the shared-memory message rather than
    // copying it, at the cost of one control-block allocation per message.
    constexpr bool is_std_shared_ptr_callback =
      !is_message_ptr_callback &&
      std::is_invocable_v<std::decay_t<Func>, std::shared_ptr<const MessageT>>;
    constexpr auto ownership =
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&>
        ? OwnershipType::Unique
        : OwnershipType::Shared;

    callback_subscription_ = agnocast::create_subscription<MessageT>(
      node, topic_name, qos,
      [callback = std::forward<Func>(callback)](agnocast::ipc_shared_ptr<MessageT> && msg) {
        if constexpr (is_std_shared_ptr_callback) {
          callback(to_std_shared_ptr(agnocast::ipc_shared_ptr<const MessageT>(std::move(msg))));
        } else if constexpr (!is_message_ptr_callback) {
          // msg keeps the shared-memory entry alive only while the callback runs: the
          // reference is valid for the duration of the callback and no copy is made, but
          // it must not be stored or used after the callback returns. Callbacks that need
          // to extend the message lifetime should take AUTOWARE_MESSAGE_CONST_SHARED_PTR.
          // as_const prevents generic callbacks from mutating the shared-memory entry,
          // which other processes may be reading concurrently.
          callback(std::as_const(*msg));
        } else if constexpr (ownership == OwnershipType::Unique) {
          callback(message_ptr<MessageT, ownership>(std::move(msg)));
        } else {
          callback(
            message_ptr<const MessageT, ownership>(
              agnocast::ipc_shared_ptr<const MessageT>(std::move(msg))));
        }
      },
      options);
  }

  /// Callback-less construction, for polling via take()/take_data().
  template <typename NodeT>
  explicit AgnocastSubscription(
    NodeT * node, const std::string & topic_name, const rclcpp::QoS & qos,
    const agnocast::SubscriptionOptions & options)
  : take_subscription_(
      std::make_shared<agnocast::TakeSubscription<MessageT>>(node, topic_name, qos, options))
  {
  }

  bool take(MessageT & out, rclcpp::MessageInfo & /*info*/) override
  {
    agnocast::ipc_shared_ptr<const MessageT> data = polling_handle().take(false);
    if (!data) {
      return false;
    }
    out = *data;
    return true;
  }

  std::shared_ptr<const MessageT> take_data() override
  {
    // Zero-copy: the returned pointer aliases the shared-memory message.
    return to_std_shared_ptr(polling_handle().take(false));
  }

  rclcpp::QoS get_actual_qos() const override { return handle().get_actual_qos(); }

  const char * get_topic_name() const override { return handle().get_topic_name(); }
};

template <typename MessageT>
class ROS2Subscription : public Subscription<MessageT>
{
  typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;

public:
  template <typename Func>
  explicit ROS2Subscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
    const agnocast::SubscriptionOptions & options)
  {
    static_assert(
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&> ||
        std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_CONST_SHARED_PTR(MessageT) &&> ||
        std::is_invocable_v<std::decay_t<Func>, std::shared_ptr<const MessageT>> ||
        std::is_invocable_v<std::decay_t<Func>, const MessageT &>,
      "callback should be invocable with an rvalue reference to either "
      "AUTOWARE_MESSAGE_UNIQUE_PTR or AUTOWARE_MESSAGE_CONST_SHARED_PTR, with "
      "MessageT::ConstSharedPtr, or with a const reference to the message type");

    constexpr bool is_message_ptr_callback =
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&> ||
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_CONST_SHARED_PTR(MessageT) &&>;
    // See the AgnocastSubscription counterpart. In the non-Agnocast build
    // AUTOWARE_MESSAGE_CONST_SHARED_PTR already is std::shared_ptr<const MessageT>, so the
    // message_ptr branch above claims it and this one stays false.
    constexpr bool is_std_shared_ptr_callback =
      !is_message_ptr_callback &&
      std::is_invocable_v<std::decay_t<Func>, std::shared_ptr<const MessageT>>;
    constexpr auto ownership =
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&>
        ? OwnershipType::Unique
        : OwnershipType::Shared;

    rclcpp::SubscriptionOptions ros2_options;
    ros2_options.callback_group = options.callback_group;
    subscription_ = node->create_subscription<MessageT>(
      topic_name, qos,
      [callback = std::forward<Func>(callback)](std::unique_ptr<MessageT> msg) {
        if constexpr (is_std_shared_ptr_callback) {
          callback(std::shared_ptr<const MessageT>(std::move(msg)));
        } else if constexpr (!is_message_ptr_callback) {
          // as_const keeps this fallback consistent with the Agnocast path: generic
          // callbacks must not observe a mutable reference on either path.
          callback(std::as_const(*msg));
        } else if constexpr (ownership == OwnershipType::Unique) {
          callback(message_ptr<MessageT, ownership>(std::move(msg)));
        } else {
          callback(
            message_ptr<const MessageT, ownership>(
              std::shared_ptr<const MessageT>(std::move(msg))));
        }
      },
      ros2_options);
  }

  /// Callback-less construction, for polling via take()/take_data(). Mirrors the rclcpp idiom:
  /// a no-op callback in a callback group that is never added to an executor.
  explicit ROS2Subscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    const agnocast::SubscriptionOptions & options)
  {
    rclcpp::SubscriptionOptions ros2_options;
    ros2_options.callback_group =
      options.callback_group
        ? options.callback_group
        : node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    subscription_ = node->create_subscription<MessageT>(
      topic_name, qos, [](std::unique_ptr<MessageT>) {}, ros2_options);
  }

  bool take(MessageT & out, rclcpp::MessageInfo & info) override
  {
    return subscription_->take(out, info);
  }

  std::shared_ptr<const MessageT> take_data() override
  {
    // Drain the queue and keep the newest, so the semantics match the Agnocast path's
    // take(allow_same_message=false): the latest new message, or nullptr if nothing arrived.
    auto data = std::make_shared<MessageT>();
    rclcpp::MessageInfo info;
    bool got_any = false;
    for (size_t i = 0; i < subscription_->get_actual_qos().depth(); ++i) {
      if (!subscription_->take(*data, info)) {
        break;
      }
      got_any = true;
    }
    return got_any ? data : nullptr;
  }

  rclcpp::QoS get_actual_qos() const override { return subscription_->get_actual_qos(); }

  const char * get_topic_name() const override { return subscription_->get_topic_name(); }
};

template <typename MessageT, typename Func>
typename Subscription<MessageT>::SharedPtr create_subscription(
  rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
  const agnocast::SubscriptionOptions & options)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastSubscription<MessageT>>(
      node, topic_name, qos, std::forward<Func>(callback), options);
  } else {
    return std::make_shared<ROS2Subscription<MessageT>>(
      node, topic_name, qos, std::forward<Func>(callback), options);
  }
}

template <typename MessageT, typename Func>
typename Subscription<MessageT>::SharedPtr create_subscription(
  rclcpp::Node * node, const std::string & topic_name, const size_t qos_history_depth,
  Func && callback, const agnocast::SubscriptionOptions & options)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastSubscription<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)),
      std::forward<Func>(callback), options);
  } else {
    return std::make_shared<ROS2Subscription<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)),
      std::forward<Func>(callback), options);
  }
}

}  // namespace autoware::agnocast_wrapper

#endif
