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

// Type-erased ("generic") publisher abstraction for the Agnocast build.

#ifdef USE_AGNOCAST_ENABLED

#include "autoware/agnocast_wrapper/runtime.hpp"

#include <agnocast/agnocast.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace autoware::agnocast_wrapper
{

/// Mirrors rclcpp::GenericPublisher / agnocast::GenericPublisher: the topic type is supplied as a
/// runtime string (e.g. "std_msgs/msg/String") rather than a compile-time template argument, for
/// nodes — such as autoware_topic_relay_controller — that relay arbitrary topics without linking
/// against their message packages.
///
/// Messages are always published from an rclcpp::SerializedMessage: Agnocast's GenericPublisher
/// deserializes it into shared memory inside publish(), the mirror image of what
/// GenericSubscription does on the way out. There is no message_ptr overload here as there is for
/// the typed Publisher — a type-erased message has no compile-time type to allocate in place.
class GenericPublisher
{
public:
  using SharedPtr = std::shared_ptr<GenericPublisher>;

  virtual ~GenericPublisher() = default;

  virtual void publish(const rclcpp::SerializedMessage & message) = 0;

  virtual uint32_t get_subscription_count() const = 0;
  virtual uint32_t get_intra_process_subscription_count() const = 0;
  virtual const char * get_topic_name() const = 0;

  /// Effective QoS. On the Agnocast path this is the requested QoS with any
  /// qos_overriding_options applied, not the RMW-resolved profile
  /// rclcpp::PublisherBase::get_actual_qos() reports: Agnocast has no DDS entity to query.
  virtual rclcpp::QoS get_actual_qos() const = 0;
};

class AgnocastGenericPublisher : public GenericPublisher
{
  agnocast::GenericPublisher::SharedPtr publisher_;

public:
  template <typename NodeT>
  explicit AgnocastGenericPublisher(
    NodeT * node, const std::string & topic_name, const std::string & topic_type,
    const rclcpp::QoS & qos, const agnocast::PublisherOptions & options)
  : publisher_(agnocast::create_generic_publisher(node, topic_name, topic_type, qos, options))
  {
  }

  void publish(const rclcpp::SerializedMessage & message) override { publisher_->publish(message); }

  uint32_t get_subscription_count() const override { return publisher_->get_subscription_count(); }
  uint32_t get_intra_process_subscription_count() const override
  {
    return publisher_->get_intra_subscription_count();
  }
  const char * get_topic_name() const override { return publisher_->get_topic_name(); }
  rclcpp::QoS get_actual_qos() const override { return publisher_->get_actual_qos(); }
};

class ROS2GenericPublisher : public GenericPublisher
{
  rclcpp::GenericPublisher::SharedPtr publisher_;

public:
  explicit ROS2GenericPublisher(
    rclcpp::Node * node, const std::string & topic_name, const std::string & topic_type,
    const rclcpp::QoS & qos, const agnocast::PublisherOptions & options)
  {
    rclcpp::PublisherOptions ros2_options;
    ros2_options.qos_overriding_options = options.qos_overriding_options;
    publisher_ = node->create_generic_publisher(topic_name, topic_type, qos, ros2_options);
  }

  void publish(const rclcpp::SerializedMessage & message) override { publisher_->publish(message); }

  uint32_t get_subscription_count() const override { return publisher_->get_subscription_count(); }
  uint32_t get_intra_process_subscription_count() const override
  {
    return publisher_->get_intra_process_subscription_count();
  }
  const char * get_topic_name() const override { return publisher_->get_topic_name(); }
  rclcpp::QoS get_actual_qos() const override { return publisher_->get_actual_qos(); }
};

/// Free-function form for incremental adoption on a node that stays an ordinary rclcpp::Node (see
/// the Node member of the same name for the wrapper-Node form, which also supports agnocast::Node).
/// This is the Method 1 (macro + free function) entry point; reach it through
/// AUTOWARE_CREATE_GENERIC_PUBLISHER3/4(_ON_NODE) rather than calling it directly, so the same
/// call site also compiles under ENABLE_AGNOCAST=0, where this free function does not exist and
/// the macro instead forwards to rclcpp::Node::create_generic_publisher().
inline GenericPublisher::SharedPtr create_generic_publisher(
  rclcpp::Node * node, const std::string & topic_name, const std::string & topic_type,
  const rclcpp::QoS & qos,
  const agnocast::PublisherOptions & options = agnocast::PublisherOptions{})
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastGenericPublisher>(node, topic_name, topic_type, qos, options);
  } else {
    return std::make_shared<ROS2GenericPublisher>(node, topic_name, topic_type, qos, options);
  }
}

inline GenericPublisher::SharedPtr create_generic_publisher(
  rclcpp::Node * node, const std::string & topic_name, const std::string & topic_type,
  const size_t qos_history_depth,
  const agnocast::PublisherOptions & options = agnocast::PublisherOptions{})
{
  return create_generic_publisher(
    node, topic_name, topic_type, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)), options);
}

}  // namespace autoware::agnocast_wrapper

#endif
