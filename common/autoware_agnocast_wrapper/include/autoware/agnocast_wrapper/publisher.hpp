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

// Publisher abstraction for the Agnocast build.

#ifdef USE_AGNOCAST_ENABLED

#include "autoware/agnocast_wrapper/macros.hpp"
#include "autoware/agnocast_wrapper/message_ptr.hpp"
#include "autoware/agnocast_wrapper/runtime.hpp"

#include <agnocast/agnocast.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace autoware::agnocast_wrapper
{

template <typename MessageT>
class Publisher
{
public:
  using SharedPtr = std::shared_ptr<Publisher<MessageT>>;

  virtual ~Publisher() = default;

  virtual AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) allocate_output_message_unique() = 0;
  virtual AUTOWARE_MESSAGE_SHARED_PTR(MessageT) allocate_output_message_shared() = 0;

  virtual void publish(AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) && message) = 0;
  virtual void publish(AUTOWARE_MESSAGE_SHARED_PTR(MessageT) && message) = 0;

  /// Publish by const reference: copies @p data into a freshly allocated message (allocated in
  /// shared memory on the Agnocast path), then publishes it.
  ///
  /// Use this when the caller already holds a message it does not own or must retain — e.g.
  /// re-publishing a received message, or publishing a member kept as node state. When the outgoing
  /// message is being constructed anyway, prefer ALLOCATE_OUTPUT_MESSAGE_{UNIQUE,SHARED}(publisher)
  /// and the corresponding publish() overload instead, which builds in place and copies no payload.
  virtual void publish(const MessageT & data) = 0;

  virtual uint32_t get_subscription_count() const = 0;
  virtual uint32_t get_intra_process_subscription_count() const = 0;
  virtual const rmw_gid_t & get_gid() const = 0;
  virtual const char * get_topic_name() const = 0;

  /// Effective QoS. On the Agnocast path this is the requested QoS with any
  /// qos_overriding_options applied, not the RMW-resolved profile
  /// rclcpp::PublisherBase::get_actual_qos() reports: Agnocast has no DDS entity to query.
  virtual rclcpp::QoS get_actual_qos() const = 0;
};

template <typename MessageT>
class AgnocastPublisher : public Publisher<MessageT>
{
  typename agnocast::Publisher<MessageT>::SharedPtr publisher_;

public:
  template <typename NodeT>
  explicit AgnocastPublisher(
    NodeT * node, const std::string & topic_name, const rclcpp::QoS & qos,
    const agnocast::PublisherOptions & options)
  : publisher_(agnocast::create_publisher<MessageT>(node, topic_name, qos, options))
  {
  }

  AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) allocate_output_message_unique() override
  {
    return AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT){publisher_->borrow_loaned_message()};
  }

  AUTOWARE_MESSAGE_SHARED_PTR(MessageT) allocate_output_message_shared() override
  {
    return AUTOWARE_MESSAGE_SHARED_PTR(MessageT){publisher_->borrow_loaned_message()};
  }

  void publish(AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) && message) override
  {
    publisher_->publish(std::move(message).move_agnocast_ptr());
  }

  void publish(AUTOWARE_MESSAGE_SHARED_PTR(MessageT) && message) override
  {
    publisher_->publish(std::move(message).move_agnocast_ptr());
  }

  // See the comment on Publisher::publish(const MessageT &) for why this exists.
  void publish(const MessageT & data) override
  {
    auto msg = publisher_->borrow_loaned_message();
    *msg = data;
    publisher_->publish(std::move(msg));
  }

  uint32_t get_subscription_count() const override { return publisher_->get_subscription_count(); }
  uint32_t get_intra_process_subscription_count() const override
  {
    return publisher_->get_intra_subscription_count();
  }
  const rmw_gid_t & get_gid() const override { return publisher_->get_gid(); }
  const char * get_topic_name() const override { return publisher_->get_topic_name(); }
  rclcpp::QoS get_actual_qos() const override { return publisher_->get_actual_qos(); }
};

template <typename MessageT>
class ROS2Publisher : public Publisher<MessageT>
{
  typename rclcpp::Publisher<MessageT>::SharedPtr publisher_{nullptr};

public:
  explicit ROS2Publisher(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    const agnocast::PublisherOptions & options)
  {
    rclcpp::PublisherOptions ros2_options;
    ros2_options.qos_overriding_options = options.qos_overriding_options;
    publisher_ = node->create_publisher<MessageT>(topic_name, qos, ros2_options);
  }

  AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) allocate_output_message_unique() override
  {
    return AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT){std::make_unique<MessageT>()};
  }

  AUTOWARE_MESSAGE_SHARED_PTR(MessageT) allocate_output_message_shared() override
  {
    return AUTOWARE_MESSAGE_SHARED_PTR(MessageT){std::make_shared<MessageT>()};
  }

  void publish(AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) && message) override
  {
    publisher_->publish(std::move(message).move_ros2_ptr());
  }

  void publish(AUTOWARE_MESSAGE_SHARED_PTR(MessageT) && message) override
  {
    publisher_->publish(*message);
  }

  // See the comment on Publisher::publish(const MessageT &) for why this exists.
  void publish(const MessageT & data) override { publisher_->publish(data); }

  uint32_t get_subscription_count() const override { return publisher_->get_subscription_count(); }
  uint32_t get_intra_process_subscription_count() const override
  {
    return publisher_->get_intra_process_subscription_count();
  }
  const rmw_gid_t & get_gid() const override { return publisher_->get_gid(); }
  const char * get_topic_name() const override { return publisher_->get_topic_name(); }
  rclcpp::QoS get_actual_qos() const override { return publisher_->get_actual_qos(); }
};

template <typename MessageT>
typename Publisher<MessageT>::SharedPtr create_publisher(
  rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
{
  agnocast::PublisherOptions options;
  if (use_agnocast()) {
    return std::make_shared<AgnocastPublisher<MessageT>>(node, topic_name, qos, options);
  } else {
    return std::make_shared<ROS2Publisher<MessageT>>(node, topic_name, qos, options);
  }
}

template <typename MessageT>
typename Publisher<MessageT>::SharedPtr create_publisher(
  rclcpp::Node * node, const std::string & topic_name, const size_t qos_history_depth)
{
  agnocast::PublisherOptions options;
  if (use_agnocast()) {
    return std::make_shared<AgnocastPublisher<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)), options);
  } else {
    return std::make_shared<ROS2Publisher<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)), options);
  }
}

template <typename MessageT>
typename Publisher<MessageT>::SharedPtr create_publisher(
  rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
  const agnocast::PublisherOptions & options)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastPublisher<MessageT>>(node, topic_name, qos, options);
  } else {
    return std::make_shared<ROS2Publisher<MessageT>>(node, topic_name, qos, options);
  }
}

template <typename MessageT>
typename Publisher<MessageT>::SharedPtr create_publisher(
  rclcpp::Node * node, const std::string & topic_name, const size_t qos_history_depth,
  const agnocast::PublisherOptions & options)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastPublisher<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)), options);
  } else {
    return std::make_shared<ROS2Publisher<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)), options);
  }
}

}  // namespace autoware::agnocast_wrapper

#endif
