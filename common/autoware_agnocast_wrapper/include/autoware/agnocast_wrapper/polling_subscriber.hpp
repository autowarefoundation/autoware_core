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
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#ifdef USE_AGNOCAST_ENABLED
#include "autoware/agnocast_wrapper/message_ptr.hpp"

#include <agnocast/agnocast.hpp>
#endif

namespace autoware::agnocast_wrapper::polling
{

namespace polling_policy = autoware_utils_rclcpp::polling_policy;

template <typename MessageT, template <typename> class PollingPolicy>
inline constexpr bool polling_policy_supported_v =
  !std::is_same_v<PollingPolicy<MessageT>, polling_policy::All<MessageT>>;

/// @brief Reject a QoS a polling subscriber cannot serve.
/// @throws std::invalid_argument if the history depth is not 1. A deeper queue makes take_data()
/// lag behind the newest message, which is why autoware_utils_rclcpp's policies reject it; depth 0
/// (KeepAll, or KeepLast(0)) is rejected here as well because the agnocast backend then never
/// delivers while the ROS 2 backend does.
inline void check_polling_qos(const rclcpp::QoS & qos, const std::string & topic_name)
{
  const auto depth = qos.get_rmw_qos_profile().depth;
  if (depth != 1) {
    throw std::invalid_argument(
      "polling::create_polling_subscriber(" + topic_name + "): history depth " +
      std::to_string(depth) + " is not supported, take_data() needs a single-depth queue");
  }
}

/// @brief Backend-agnostic polling subscriber. take_data() returns a plain
/// std::shared_ptr<const MessageT> regardless of ENABLE_AGNOCAST, and is the only policy method
/// exposed: the agnocast take path carries no source timestamp, so there is no
/// last_taken_data_timestamp().
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

  virtual ~PollingSubscriber() = default;

  /// @note Not synchronized, like autoware_utils_rclcpp's polling subscriber: call it from a
  /// single thread, or from callbacks in one mutually exclusive callback group.
  virtual std::shared_ptr<const MessageT> take_data() = 0;
};

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

  std::shared_ptr<const MessageT> take_data() override { return subscriber_->take_data(); }
};

#ifdef USE_AGNOCAST_ENABLED

/// @brief Agnocast-side counterpart of an autoware_utils_rclcpp polling policy.
/// Defined rather than left declared so that a policy without a counterpart is rejected here
/// instead of by an incomplete-type error on AgnocastPollingSubscriber::policy_.
template <typename MessageT, template <typename> class PollingPolicy>
class AgnocastPollingPolicy
{
  static_assert(
    polling_policy_supported_v<MessageT, PollingPolicy>,
    "This polling policy has no agnocast counterpart. Use polling_policy::Latest or "
    "polling_policy::Newest.");

public:
  std::shared_ptr<const MessageT> take_data(agnocast::TakeSubscription<MessageT> &) { return {}; }
};

/// @brief Counterpart of autoware_utils_rclcpp::polling_policy::Latest<MessageT>::take_data().
/// Where the ROS 2 policy holds a heap copy, this holds the shared-memory message itself, so one
/// agnocast entry stays pinned for as long as the subscriber lives.
template <typename MessageT>
class AgnocastPollingPolicy<MessageT, polling_policy::Latest>
{
  std::shared_ptr<const MessageT> data_;

public:
  std::shared_ptr<const MessageT> take_data(agnocast::TakeSubscription<MessageT> & subscriber)
  {
    if (auto new_data = detail::to_std_shared_ptr(subscriber.take(false))) {
      data_ = std::move(new_data);
    }
    return data_;
  }
};

/// @brief Counterpart of autoware_utils_rclcpp::polling_policy::Newest<MessageT>::take_data().
template <typename MessageT>
class AgnocastPollingPolicy<MessageT, polling_policy::Newest>
{
public:
  std::shared_ptr<const MessageT> take_data(agnocast::TakeSubscription<MessageT> & subscriber)
  {
    return detail::to_std_shared_ptr(subscriber.take(false));
  }
};

template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
class AgnocastPollingSubscriber : public PollingSubscriber<MessageT, PollingPolicy>
{
  typename agnocast::TakeSubscription<MessageT>::SharedPtr subscriber_;
  AgnocastPollingPolicy<MessageT, PollingPolicy> policy_;

public:
  explicit AgnocastPollingSubscriber(
    agnocast::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
  : subscriber_(std::make_shared<agnocast::TakeSubscription<MessageT>>(node, topic_name, qos))
  {
  }

  std::shared_ptr<const MessageT> take_data() override { return policy_.take_data(*subscriber_); }
};

/// @note The returned subscriber references the node's backend by raw pointer, so it must not
/// outlive @p node.
template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
typename PollingSubscriber<MessageT, PollingPolicy>::SharedPtr create_polling_subscriber(
  autoware::agnocast_wrapper::Node * node, const std::string & topic_name,
  const rclcpp::QoS & qos = rclcpp::QoS{1})
{
  check_polling_qos(qos, topic_name);

  if (use_agnocast()) {
    return std::make_shared<AgnocastPollingSubscriber<MessageT, PollingPolicy>>(
      node->get_agnocast_node().get(), topic_name, qos);
  }
  return std::make_shared<ROS2PollingSubscriber<MessageT, PollingPolicy>>(
    node->get_rclcpp_node().get(), topic_name, qos);
}

#else  // USE_AGNOCAST_ENABLED

/// @note The returned subscriber references the node's rclcpp node by raw pointer, so it must not
/// outlive @p node.
template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
typename PollingSubscriber<MessageT, PollingPolicy>::SharedPtr create_polling_subscriber(
  autoware::agnocast_wrapper::Node * node, const std::string & topic_name,
  const rclcpp::QoS & qos = rclcpp::QoS{1})
{
  check_polling_qos(qos, topic_name);

  return std::make_shared<ROS2PollingSubscriber<MessageT, PollingPolicy>>(
    node->get_rclcpp_node().get(), topic_name, qos);
}

#endif  // USE_AGNOCAST_ENABLED

template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
typename PollingSubscriber<MessageT, PollingPolicy>::SharedPtr create_polling_subscriber(
  autoware::agnocast_wrapper::Node * node, const std::string & topic_name, size_t qos_history_depth)
{
  return create_polling_subscriber<MessageT, PollingPolicy>(
    node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
}

}  // namespace autoware::agnocast_wrapper::polling
