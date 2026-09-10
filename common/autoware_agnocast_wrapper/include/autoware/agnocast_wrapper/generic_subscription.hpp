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

// Type-erased ("generic") subscription abstraction for the Agnocast build.

#include <rclcpp/serialized_message.hpp>

#include <functional>
#include <memory>

namespace autoware::agnocast_wrapper
{

/// Callback shape accepted by create_generic_subscription(): the intersection of what
/// rclcpp::GenericSubscription and agnocast::GenericSubscription both support, and what
/// autoware_topic_relay_controller (the motivating caller) already uses. Declared unconditionally
/// (unlike the classes below) so it is available in both the Agnocast and non-Agnocast builds — the
/// non-Agnocast Node::create_generic_subscription() (see node.hpp) needs it too.
///
/// Takes the message by `shared_ptr<const SerializedMessage>` rather than `shared_ptr<
/// SerializedMessage>`: rclcpp deprecated the non-const form (AnySubscriptionCallback's
/// SharedPtrSerializedMessageCallback) in favor of this one on both Humble and Jazzy, and
/// Agnocast's GenericSubscription accepts it as well, so this stays forward-compatible without a
/// per-distribution branch.
using GenericSubscriptionCallback =
  std::function<void(std::shared_ptr<const rclcpp::SerializedMessage>)>;

}  // namespace autoware::agnocast_wrapper

#ifdef USE_AGNOCAST_ENABLED

#include "autoware/agnocast_wrapper/runtime.hpp"
#include "autoware/agnocast_wrapper/subscription.hpp"

#include <agnocast/agnocast.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::agnocast_wrapper
{

/// Mirrors rclcpp::GenericSubscription / agnocast::GenericSubscription: the topic type is supplied
/// as a runtime string (e.g. "std_msgs/msg/String") rather than a compile-time template argument,
/// for nodes — such as autoware_topic_relay_controller — that relay arbitrary topics without
/// linking against their message packages.
///
/// Messages are always delivered as rclcpp::SerializedMessage: on the Agnocast path, the message
/// lives in shared memory as a concrete type unknown at compile time, so
/// agnocast::GenericSubscription serializes it before handing it to the callback. There is no
/// message_ptr overload here as there is for the typed Subscription — a type-erased message has no
/// compile-time type to hand out a zero-copy handle to.
///
/// @throws std::runtime_error if topic_type is unknown or its typesupport library cannot be
///         loaded. Both backends load the typesupport library for topic_type at construction
///         (rclcpp::GenericSubscription and agnocast::GenericSubscription document the same
///         behavior).
class GenericSubscription
{
public:
  using SharedPtr = std::shared_ptr<GenericSubscription>;

  virtual ~GenericSubscription() = default;

  virtual const char * get_topic_name() const = 0;

  /// Effective QoS. On the Agnocast path this is the requested QoS with any
  /// qos_overriding_options applied, not the RMW-resolved profile
  /// rclcpp::SubscriptionBase::get_actual_qos() reports: Agnocast has no DDS entity to query.
  virtual rclcpp::QoS get_actual_qos() const = 0;
};

namespace detail
{

/// rclcpp::create_generic_subscription() only honors event_callbacks, use_default_callbacks and
/// callback_group — it silently drops qos_overriding_options — while Agnocast's
/// GenericSubscription applies it. Rather than have the same option take effect on one backend
/// and not the other, reject it outright in both, so a caller can't end up depending on an
/// override that only works under ENABLE_AGNOCAST=1. Mirrors
/// generic_publisher.hpp's check_generic_publisher_options() for the same reason.
inline void check_generic_subscription_options(
  const agnocast::SubscriptionOptions & options, const std::string & topic_name)
{
  if (!options.qos_overriding_options.get_policy_kinds().empty()) {
    throw std::invalid_argument(
      "create_generic_subscription(" + topic_name +
      "): qos_overriding_options is not supported for a generic subscription (honored by "
      "Agnocast's GenericSubscription but silently ignored by rclcpp's, so it cannot behave "
      "consistently across backends)");
  }
}

}  // namespace detail

class AgnocastGenericSubscription : public GenericSubscription
{
  agnocast::GenericSubscription::SharedPtr subscription_;

public:
  template <typename NodeT>
  explicit AgnocastGenericSubscription(
    NodeT * node, const std::string & topic_name, const std::string & topic_type,
    const rclcpp::QoS & qos, GenericSubscriptionCallback callback,
    const agnocast::SubscriptionOptions & options)
  {
    detail::check_generic_subscription_options(options, topic_name);
    subscription_ = agnocast::create_generic_subscription(
      node, topic_name, topic_type, qos, std::move(callback), options);
  }

  const char * get_topic_name() const override { return subscription_->get_topic_name(); }
  rclcpp::QoS get_actual_qos() const override { return subscription_->get_actual_qos(); }
};

class ROS2GenericSubscription : public GenericSubscription
{
  rclcpp::GenericSubscription::SharedPtr subscription_;

public:
  explicit ROS2GenericSubscription(
    rclcpp::Node * node, const std::string & topic_name, const std::string & topic_type,
    const rclcpp::QoS & qos, GenericSubscriptionCallback callback,
    const agnocast::SubscriptionOptions & options)
  {
    detail::check_generic_subscription_options(options, topic_name);
    subscription_ = node->create_generic_subscription(
      topic_name, topic_type, qos, std::move(callback), to_rclcpp_subscription_options(options));
  }

  const char * get_topic_name() const override { return subscription_->get_topic_name(); }
  rclcpp::QoS get_actual_qos() const override { return subscription_->get_actual_qos(); }
};

/// Free-function form for incremental adoption on a node that stays an ordinary rclcpp::Node (see
/// the Node member of the same name for the wrapper-Node form, which also supports agnocast::Node).
/// This is the Method 1 (macro + free function) entry point; reach it through
/// AUTOWARE_CREATE_GENERIC_SUBSCRIPTION(_ON_NODE) rather than calling it directly, so the same
/// call site also compiles under ENABLE_AGNOCAST=0, where this free function does not exist and
/// the macro instead forwards to rclcpp::Node::create_generic_subscription().
inline GenericSubscription::SharedPtr create_generic_subscription(
  rclcpp::Node * node, const std::string & topic_name, const std::string & topic_type,
  const rclcpp::QoS & qos, GenericSubscriptionCallback callback,
  const agnocast::SubscriptionOptions & options = agnocast::SubscriptionOptions{})
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastGenericSubscription>(
      node, topic_name, topic_type, qos, std::move(callback), options);
  } else {
    return std::make_shared<ROS2GenericSubscription>(
      node, topic_name, topic_type, qos, std::move(callback), options);
  }
}

inline GenericSubscription::SharedPtr create_generic_subscription(
  rclcpp::Node * node, const std::string & topic_name, const std::string & topic_type,
  const size_t qos_history_depth, GenericSubscriptionCallback callback,
  const agnocast::SubscriptionOptions & options = agnocast::SubscriptionOptions{})
{
  return create_generic_subscription(
    node, topic_name, topic_type, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)),
    std::move(callback), options);
}

}  // namespace autoware::agnocast_wrapper

#endif
