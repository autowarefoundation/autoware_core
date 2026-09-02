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
#include <string>
#include <type_traits>
#include <utility>

namespace autoware::agnocast_wrapper
{

template <typename MessageT>
class Subscription
{
public:
  using SharedPtr = std::shared_ptr<Subscription<MessageT>>;

  virtual ~Subscription() = default;
};

namespace detail
{
// The dispatch lambdas below are not mutable, so a callback is invoked const-qualified. Classifying
// on an unqualified callable would accept one the call site cannot then call.
template <typename Func, typename... Args>
inline constexpr bool is_const_invocable_v =
  std::is_invocable_v<const std::decay_t<Func> &, Args...>;

// True when the callback takes the pointer itself rather than something the pointer converts to. A
// std::weak_ptr or std::shared_ptr<const void> parameter would accept it and then observe a pointer
// that dies with the dispatch expression, and rclcpp rejects both shapes.
template <typename Func, typename MessageT>
inline constexpr bool takes_std_shared_ptr_v =
  is_const_invocable_v<Func, std::shared_ptr<const MessageT>> &&
  !is_const_invocable_v<Func, std::weak_ptr<const MessageT>> &&
  !is_const_invocable_v<Func, std::shared_ptr<const void>>;
}  // namespace detail

template <typename Func, typename MessageT>
inline constexpr bool is_message_ptr_subscription_callback_v =
  detail::is_const_invocable_v<Func, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&> ||
  detail::is_const_invocable_v<Func, AUTOWARE_MESSAGE_CONST_SHARED_PTR(MessageT) &&>;

// For a callback whose parameter type is fixed outside the caller's control, so it cannot be
// spelled with AUTOWARE_MESSAGE_CONST_SHARED_PTR.
template <typename Func, typename MessageT>
inline constexpr bool is_std_shared_ptr_subscription_callback_v =
  detail::takes_std_shared_ptr_v<Func, MessageT> &&
  !is_message_ptr_subscription_callback_v<Func, MessageT> &&
  !detail::is_const_invocable_v<Func, const MessageT &>;

// How many of the accepted shapes the callback fits. rclcpp derives a subscription's callback
// signature from &FunctionT::operator(), which cannot be resolved for a callable that fits several
// -- a generic lambda, an overloaded functor -- so such a callback cannot form a subscription at
// ENABLE_AGNOCAST=0 at all. Requiring exactly one keeps the accepted set the same in both builds.
template <typename Func, typename MessageT>
inline constexpr int subscription_callback_shapes_v =
  static_cast<int>(is_message_ptr_subscription_callback_v<Func, MessageT>) +
  static_cast<int>(detail::takes_std_shared_ptr_v<Func, MessageT>) +
  static_cast<int>(detail::is_const_invocable_v<Func, const MessageT &>);

template <typename MessageT>
class AgnocastSubscription : public Subscription<MessageT>
{
  typename agnocast::Subscription<MessageT>::SharedPtr subscription_;

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
      subscription_callback_shapes_v<Func, MessageT> == 1,
      "callback should take exactly one of AUTOWARE_MESSAGE_UNIQUE_PTR, "
      "AUTOWARE_MESSAGE_CONST_SHARED_PTR, std::shared_ptr<const MessageT>, or a const reference to "
      "the message type");

    constexpr auto ownership =
      detail::is_const_invocable_v<Func, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&>
        ? OwnershipType::Unique
        : OwnershipType::Shared;

    subscription_ = agnocast::create_subscription<MessageT>(
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
          // as_const prevents the callback from mutating the shared-memory entry, which other
          // processes may be reading concurrently.
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
};

/// DDS path of the Agnocast build, selected when use_agnocast() is false. The ENABLE_AGNOCAST=0
/// build never reaches this class: its Node forwards create_subscription() straight to rclcpp.
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
      subscription_callback_shapes_v<Func, MessageT> == 1,
      "callback should take exactly one of AUTOWARE_MESSAGE_UNIQUE_PTR, "
      "AUTOWARE_MESSAGE_CONST_SHARED_PTR, std::shared_ptr<const MessageT>, or a const reference to "
      "the message type");

    constexpr auto ownership =
      detail::is_const_invocable_v<Func, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&>
        ? OwnershipType::Unique
        : OwnershipType::Shared;

    rclcpp::SubscriptionOptions ros2_options;
    ros2_options.callback_group = options.callback_group;
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
