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

/// Translate Agnocast subscription options into the rclcpp ones, for the DDS path of an
/// agnocast-enabled build. agnocast::SubscriptionOptions has exactly these three fields, so
/// nothing is dropped.
inline rclcpp::SubscriptionOptions to_rclcpp_subscription_options(
  const agnocast::SubscriptionOptions & options)
{
  rclcpp::SubscriptionOptions result;
  result.callback_group = options.callback_group;
  result.ignore_local_publications = options.ignore_local_publications;
  result.qos_overriding_options = options.qos_overriding_options;
  return result;
}

/// Mixing callback delivery and take() on one subscription is not intended in either backend:
/// Agnocast splits them by type, and rclcpp's take() would steal the callback's message.
[[noreturn]] inline void throw_not_pollable(const std::string & topic_name)
{
  throw std::runtime_error(
    "subscription on '" + topic_name +
    "' was created with a callback, so it cannot be polled: the delivery mode is fixed at "
    "construction. Create it without a callback to use take().");
}

template <typename MessageT>
class Subscription
{
public:
  using SharedPtr = std::shared_ptr<Subscription<MessageT>>;

  virtual ~Subscription() = default;

  /// Effective QoS. On the Agnocast path this is the requested QoS with any
  /// qos_overriding_options applied, not the RMW-resolved profile
  /// rclcpp::SubscriptionBase::get_actual_qos() reports: Agnocast has no DDS entity to query.
  virtual rclcpp::QoS get_actual_qos() const = 0;

  /// Topic name after remapping.
  virtual const char * get_topic_name() const = 0;

  /// Polling retrieval, mirroring rclcpp::Subscription::take(). The Agnocast path copies out of
  /// shared memory to satisfy the caller-owned @p out.
  /// @return true if a new message was written to @p out.
  /// @throw std::runtime_error when the subscription was created with a callback.
  virtual bool take(MessageT & out, rclcpp::MessageInfo & info) = 0;
};

template <typename Func, typename MessageT>
inline constexpr bool is_message_ptr_subscription_callback_v =
  std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&> ||
  std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_CONST_SHARED_PTR(MessageT) &&>;

// For a callback whose parameter type is fixed outside the caller's control, so it cannot be
// spelled with AUTOWARE_MESSAGE_CONST_SHARED_PTR. A parameter that merely accepts the pointer, such
// as std::weak_ptr, is rejected because rclcpp rejects it too: it would compile only at
// ENABLE_AGNOCAST=1.
template <typename Func, typename MessageT>
inline constexpr bool is_std_shared_ptr_subscription_callback_v =
  !is_message_ptr_subscription_callback_v<Func, MessageT> &&
  std::is_invocable_v<std::decay_t<Func>, std::shared_ptr<const MessageT>> &&
  !std::is_invocable_v<std::decay_t<Func>, const MessageT &> &&
  !std::is_invocable_v<std::decay_t<Func>, std::weak_ptr<const MessageT>> &&
  !std::is_invocable_v<std::decay_t<Func>, std::shared_ptr<const void>>;

template <typename MessageT>
class AgnocastSubscription : public Subscription<MessageT>
{
  // Exactly one of these holds the subscription.
  typename agnocast::Subscription<MessageT>::SharedPtr callback_subscription_;
  typename agnocast::TakeSubscription<MessageT>::SharedPtr take_subscription_;
  // As passed to the constructor, for the error message below. Not remap-resolved.
  std::string topic_name_;

  agnocast::TakeSubscription<MessageT> & polling_handle()
  {
    if (!take_subscription_) {
      throw_not_pollable(topic_name_);
    }
    return *take_subscription_;
  }

  /// Both accessors below are on agnocast::SubscriptionBase, so neither cares which delivery
  /// mode is in use.
  const agnocast::SubscriptionBase & handle() const
  {
    if (callback_subscription_) {
      return *callback_subscription_;
    }
    return *take_subscription_;
  }

public:
  template <typename NodeT, typename Func>
  explicit AgnocastSubscription(
    NodeT * node, const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
    const agnocast::SubscriptionOptions & options)
  : topic_name_(topic_name)
  {
    // TODO(Koichi98): AUTOWARE_MESSAGE_UNIQUE_PTR should be disallowed for Agnocast subscriptions.
    // Agnocast uses shared memory, so mutable exclusive ownership is semantically incorrect and
    // risks corrupting data read by other subscribers. Currently kept for compatibility with
    // CudaPointcloudPreprocessorNode which uses UNIQUE_PTR callbacks.
    static_assert(
      is_message_ptr_subscription_callback_v<Func, MessageT> ||
        is_std_shared_ptr_subscription_callback_v<Func, MessageT> ||
        std::is_invocable_v<std::decay_t<Func>, const MessageT &>,
      "callback should be invocable with an rvalue reference to either "
      "AUTOWARE_MESSAGE_UNIQUE_PTR or AUTOWARE_MESSAGE_CONST_SHARED_PTR, with "
      "std::shared_ptr<const MessageT>, or with a const reference to the message type");

    constexpr auto ownership =
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&>
        ? OwnershipType::Unique
        : OwnershipType::Shared;

    callback_subscription_ = agnocast::create_subscription<MessageT>(
      node, topic_name, qos,
      [callback = std::forward<Func>(callback)](agnocast::ipc_shared_ptr<MessageT> && msg) {
        if constexpr (is_std_shared_ptr_subscription_callback_v<Func, MessageT>) {
          callback(
            detail::to_std_shared_ptr(agnocast::ipc_shared_ptr<const MessageT>(std::move(msg))));
        } else if constexpr (!is_message_ptr_subscription_callback_v<Func, MessageT>) {
          // msg keeps the shared-memory entry alive only while the callback runs: the
          // reference is valid for the duration of the callback and no copy is made, but
          // it must not be stored or used after the callback returns. Callbacks that need
          // to extend the message lifetime should take one of the owning forms.
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

  /// Callback-less construction, for polling via take().
  template <typename NodeT>
  explicit AgnocastSubscription(
    NodeT * node, const std::string & topic_name, const rclcpp::QoS & qos,
    const agnocast::SubscriptionOptions & options)
  : take_subscription_(
      std::make_shared<agnocast::TakeSubscription<MessageT>>(node, topic_name, qos, options)),
    topic_name_(topic_name)
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

  rclcpp::QoS get_actual_qos() const override { return handle().get_actual_qos(); }

  const char * get_topic_name() const override { return handle().get_topic_name(); }
};

/// DDS path of the Agnocast build, selected when use_agnocast() is false. The ENABLE_AGNOCAST=0
/// build never reaches this class: its Node forwards create_subscription() straight to rclcpp.
template <typename MessageT>
class ROS2Subscription : public Subscription<MessageT>
{
  typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;
  bool pollable_ = false;
  // As passed to the constructor, for the error message. Not remap-resolved.
  std::string topic_name_;

  rclcpp::Subscription<MessageT> & polling_handle()
  {
    if (!pollable_) {
      throw_not_pollable(topic_name_);
    }
    return *subscription_;
  }

public:
  template <typename Func>
  explicit ROS2Subscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
    const agnocast::SubscriptionOptions & options)
  : topic_name_(topic_name)
  {
    static_assert(
      is_message_ptr_subscription_callback_v<Func, MessageT> ||
        is_std_shared_ptr_subscription_callback_v<Func, MessageT> ||
        std::is_invocable_v<std::decay_t<Func>, const MessageT &>,
      "callback should be invocable with an rvalue reference to either "
      "AUTOWARE_MESSAGE_UNIQUE_PTR or AUTOWARE_MESSAGE_CONST_SHARED_PTR, with "
      "std::shared_ptr<const MessageT>, or with a const reference to the message type");

    constexpr auto ownership =
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&>
        ? OwnershipType::Unique
        : OwnershipType::Shared;

    const rclcpp::SubscriptionOptions ros2_options = to_rclcpp_subscription_options(options);
    if constexpr (ownership == OwnershipType::Unique) {
      subscription_ = node->create_subscription<MessageT>(
        topic_name, qos,
        [callback = std::forward<Func>(callback)](std::unique_ptr<MessageT> msg) {
          callback(message_ptr<MessageT, ownership>(std::move(msg)));
        },
        ros2_options);
    } else {
      // Shared-const rather than unique_ptr: nothing below needs ownership, and rclcpp copies the
      // message to satisfy a unique_ptr callback.
      subscription_ = node->create_subscription<MessageT>(
        topic_name, qos,
        [callback = std::forward<Func>(callback)](std::shared_ptr<const MessageT> msg) {
          if constexpr (is_std_shared_ptr_subscription_callback_v<Func, MessageT>) {
            callback(std::move(msg));
          } else if constexpr (!is_message_ptr_subscription_callback_v<Func, MessageT>) {
            callback(*msg);
          } else {
            callback(message_ptr<const MessageT, ownership>(std::move(msg)));
          }
        },
        ros2_options);
    }
  }

  /// Callback-less construction, for polling via take().
  explicit ROS2Subscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    const agnocast::SubscriptionOptions & options)
  : pollable_(true), topic_name_(topic_name)
  {
    rclcpp::SubscriptionOptions ros2_options = to_rclcpp_subscription_options(options);
    if (!ros2_options.callback_group) {
      ros2_options.callback_group =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    }
    subscription_ = node->create_subscription<MessageT>(
      topic_name, qos, [](std::unique_ptr<MessageT>) {}, ros2_options);
  }

  bool take(MessageT & out, rclcpp::MessageInfo & info) override
  {
    return polling_handle().take(out, info);
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
