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

#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#ifdef USE_AGNOCAST_ENABLED
#include <agnocast/agnocast.hpp>
#endif

namespace autoware::agnocast_wrapper::polling
{

// Reuse the polling policy tag types (Latest / Newest / All) from autoware_utils_rclcpp.
namespace polling_policy = autoware_utils_rclcpp::polling_policy;

// Latest re-delivers the cached message by default; Newest only delivers messages new since the
// last take.
template <typename MessageT, template <typename> class PollingPolicy>
inline constexpr bool polling_default_allow_same_message_v =
  !std::is_same_v<PollingPolicy<MessageT>, polling_policy::Newest<MessageT>>;

template <typename MessageT, template <typename> class PollingPolicy>
inline constexpr bool polling_policy_supported_v =
  !std::is_same_v<PollingPolicy<MessageT>, polling_policy::All<MessageT>>;

/// @brief Backend-agnostic polling subscriber. take_data() returns a plain
/// std::shared_ptr<const MessageT> regardless of ENABLE_AGNOCAST.
template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
class PollingSubscriber
{
public:
  static_assert(
    polling_policy_supported_v<MessageT, PollingPolicy>,
    "polling_policy::All is not supported by "
    "autoware::agnocast_wrapper::polling::create_polling_subscriber "
    "(take_data() returns a single message, not a vector). Use polling_policy::Latest or "
    "polling_policy::Newest.");

  using SharedPtr = std::shared_ptr<PollingSubscriber<MessageT, PollingPolicy>>;

  static constexpr bool default_allow_same_message =
    polling_default_allow_same_message_v<MessageT, PollingPolicy>;

  virtual ~PollingSubscriber() = default;

  virtual std::shared_ptr<const MessageT> take_data(
    bool allow_same_message = default_allow_same_message) = 0;
};

/// @brief rclcpp backend: delegates to autoware_utils_rclcpp::InterProcessPollingSubscriber.
template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
class ROS2PollingSubscriber : public PollingSubscriber<MessageT, PollingPolicy>
{
  typename autoware_utils_rclcpp::InterProcessPollingSubscriber<MessageT, PollingPolicy>::SharedPtr
    subscriber_;

public:
  explicit ROS2PollingSubscriber(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
  : subscriber_(
      autoware_utils_rclcpp::InterProcessPollingSubscriber<
        MessageT, PollingPolicy>::create_subscription(node, topic_name, qos))
  {
  }

  // The default argument for allow_same_message is declared on the base class; virtual overrides
  // inherit it via the static call type (PollingSubscriber<...>::SharedPtr).
  std::shared_ptr<const MessageT> take_data(bool allow_same_message) override
  {
    (void)allow_same_message;
    // InterProcessPollingSubscriber::take_data() already returns MessageT::ConstSharedPtr
    // (a std::shared_ptr<const MessageT>); no copy needed.
    return subscriber_->take_data();
  }
};

#ifdef USE_AGNOCAST_ENABLED

/// @brief agnocast backend: uses agnocast's native take-subscription and aliases the shared-memory
/// message into a plain std::shared_ptr (zero-copy).
template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
class AgnocastPollingSubscriber : public PollingSubscriber<MessageT, PollingPolicy>
{
  typename agnocast::TakeSubscription<MessageT>::SharedPtr subscriber_;

public:
  explicit AgnocastPollingSubscriber(
    agnocast::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
  : subscriber_(std::make_shared<agnocast::TakeSubscription<MessageT>>(node, topic_name, qos))
  {
  }

  std::shared_ptr<const MessageT> take_data(bool allow_same_message) override
  {
    agnocast::ipc_shared_ptr<const MessageT> data = subscriber_->take(allow_same_message);
    if (!data) {
      return nullptr;
    }
    // Zero-copy: alias the message that lives in agnocast shared memory instead of copying it out.
    // `holder` owns the ipc_shared_ptr; the returned std::shared_ptr shares that ownership
    // (aliasing constructor) while pointing at the shared-memory message. The kernel subscriber
    // reference is held for exactly the returned shared_ptr's lifetime and released when the last
    // copy is destroyed, so lifetime/refcount semantics match the rclcpp heap path (valid while
    // held). NOTE: while any copy is alive it keeps one agnocast shared-memory entry pinned.
    auto holder = std::make_shared<agnocast::ipc_shared_ptr<const MessageT>>(std::move(data));
    return std::shared_ptr<const MessageT>(holder, holder->get());
  }
};

/// @brief Create a polling subscriber attached to an agnocast_wrapper::Node.
/// Dispatches at runtime: agnocast backend when use_agnocast() is true, rclcpp backend otherwise.
/// @note The returned subscriber references the node's underlying backend by raw pointer, so it
/// must
///       not outlive @p node (same lifetime contract as the underlying rclcpp/agnocast
///       subscribers).
template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
typename PollingSubscriber<MessageT, PollingPolicy>::SharedPtr create_polling_subscriber(
  autoware::agnocast_wrapper::Node * node, const std::string & topic_name,
  const rclcpp::QoS & qos = rclcpp::QoS{1})
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastPollingSubscriber<MessageT, PollingPolicy>>(
      node->get_agnocast_node().get(), topic_name, qos);
  }
  return std::make_shared<ROS2PollingSubscriber<MessageT, PollingPolicy>>(
    node->get_rclcpp_node().get(), topic_name, qos);
}

#else  // USE_AGNOCAST_ENABLED

/// @brief Create a polling subscriber attached to an agnocast_wrapper::Node (rclcpp-only build).
/// @note The returned subscriber references the node's underlying rclcpp node by raw pointer, so it
///       must not outlive @p node.
template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
typename PollingSubscriber<MessageT, PollingPolicy>::SharedPtr create_polling_subscriber(
  autoware::agnocast_wrapper::Node * node, const std::string & topic_name,
  const rclcpp::QoS & qos = rclcpp::QoS{1})
{
  return std::make_shared<ROS2PollingSubscriber<MessageT, PollingPolicy>>(
    node->get_rclcpp_node().get(), topic_name, qos);
}

#endif  // USE_AGNOCAST_ENABLED

/// @brief History-depth overload.
template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
typename PollingSubscriber<MessageT, PollingPolicy>::SharedPtr create_polling_subscriber(
  autoware::agnocast_wrapper::Node * node, const std::string & topic_name, size_t qos_history_depth)
{
  return create_polling_subscriber<MessageT, PollingPolicy>(
    node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
}

}  // namespace autoware::agnocast_wrapper::polling
