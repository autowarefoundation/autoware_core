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

#include <string>
#include <utility>

#include "autoware_utils_rclcpp/polling_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <type_traits>

#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/agnocast.hpp>

#include <chrono>
#include <cstdlib>
#include <future>

#define AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) \
  autoware::agnocast_wrapper::message_ptr<    \
    MessageT, autoware::agnocast_wrapper::OwnershipType::Unique>
// For publisher (mutable message)
#define AUTOWARE_MESSAGE_SHARED_PTR(MessageT) \
  autoware::agnocast_wrapper::message_ptr<    \
    MessageT, autoware::agnocast_wrapper::OwnershipType::Shared>
// For subscription (read-only message)
#define AUTOWARE_MESSAGE_CONST_SHARED_PTR(MessageT) \
  autoware::agnocast_wrapper::message_ptr<          \
    const MessageT, autoware::agnocast_wrapper::OwnershipType::Shared>
#define AUTOWARE_SERVICE_REQUEST_PTR(ServiceT) \
  autoware::agnocast_wrapper::message_ptr<     \
    typename ServiceT::Request, autoware::agnocast_wrapper::OwnershipType::Shared>
#define AUTOWARE_SERVICE_RESPONSE_PTR(ServiceT) \
  autoware::agnocast_wrapper::message_ptr<      \
    typename ServiceT::Response, autoware::agnocast_wrapper::OwnershipType::Shared>
#define AUTOWARE_SUBSCRIPTION_PTR(MessageT) \
  typename autoware::agnocast_wrapper::Subscription<MessageT>::SharedPtr
#define AUTOWARE_PUBLISHER_PTR(MessageT) \
  typename autoware::agnocast_wrapper::Publisher<MessageT>::SharedPtr
#define AUTOWARE_POLLING_SUBSCRIBER_PTR(MessageT) \
  typename autoware::agnocast_wrapper::PollingSubscriber<MessageT>::SharedPtr
#define AUTOWARE_CLIENT_PTR(ServiceT) \
  typename autoware::agnocast_wrapper::Client<ServiceT>::SharedPtr
#define AUTOWARE_SERVICE_PTR(ServiceT) \
  typename autoware::agnocast_wrapper::Service<ServiceT>::SharedPtr
#define AUTOWARE_CLIENT_FUTURE(ServiceT) \
  typename autoware::agnocast_wrapper::Client<ServiceT>::Future
#define AUTOWARE_CLIENT_SHARED_FUTURE(ServiceT) \
  typename autoware::agnocast_wrapper::Client<ServiceT>::SharedFuture
#define AUTOWARE_CLIENT_FUTURE_AND_REQUEST_ID(ServiceT) \
  typename autoware::agnocast_wrapper::Client<ServiceT>::FutureAndRequestId
#define AUTOWARE_CLIENT_SHARED_FUTURE_AND_REQUEST_ID(ServiceT) \
  typename autoware::agnocast_wrapper::Client<ServiceT>::SharedFutureAndRequestId

#define AUTOWARE_CREATE_SUBSCRIPTION(message_type, topic, qos, callback, options) \
  autoware::agnocast_wrapper::create_subscription<message_type>(this, topic, qos, callback, options)
#define AUTOWARE_CREATE_SUBSCRIPTION_ON_NODE(message_type, node, topic, qos, callback, options) \
  autoware::agnocast_wrapper::create_subscription<message_type>(node, topic, qos, callback, options)

#define AUTOWARE_CREATE_PUBLISHER2(message_type, arg1, arg2) \
  autoware::agnocast_wrapper::create_publisher<message_type>(this, arg1, arg2)
#define AUTOWARE_CREATE_PUBLISHER3(message_type, arg1, arg2, arg3) \
  autoware::agnocast_wrapper::create_publisher<message_type>(this, arg1, arg2, arg3)
#define AUTOWARE_CREATE_PUBLISHER2_ON_NODE(message_type, node, arg1, arg2) \
  autoware::agnocast_wrapper::create_publisher<message_type>(node, arg1, arg2)
#define AUTOWARE_CREATE_PUBLISHER3_ON_NODE(message_type, node, arg1, arg2, arg3) \
  autoware::agnocast_wrapper::create_publisher<message_type>(node, arg1, arg2, arg3)

#define AUTOWARE_CREATE_POLLING_SUBSCRIBER(message_type, topic, qos) \
  autoware::agnocast_wrapper::create_polling_subscriber<message_type>(this, topic, qos)
#define AUTOWARE_CREATE_POLLING_SUBSCRIBER_ON_NODE(message_type, node, topic, qos) \
  autoware::agnocast_wrapper::create_polling_subscriber<message_type>(node, topic, qos)

#define AUTOWARE_CREATE_CLIENT1(service_type, service_name) \
  autoware::agnocast_wrapper::create_client<service_type>(this, service_name)
#define AUTOWARE_CREATE_CLIENT2(service_type, service_name, qos) \
  autoware::agnocast_wrapper::create_client<service_type>(this, service_name, qos)
#define AUTOWARE_CREATE_CLIENT3(service_type, service_name, qos, group) \
  autoware::agnocast_wrapper::create_client<service_type>(this, service_name, qos, group)
#define AUTOWARE_CREATE_CLIENT1_ON_NODE(service_type, node, service_name) \
  autoware::agnocast_wrapper::create_client<service_type>(node, service_name)
#define AUTOWARE_CREATE_CLIENT2_ON_NODE(service_type, node, service_name, qos) \
  autoware::agnocast_wrapper::create_client<service_type>(node, service_name, qos)
#define AUTOWARE_CREATE_CLIENT3_ON_NODE(service_type, node, service_name, qos, group) \
  autoware::agnocast_wrapper::create_client<service_type>(node, service_name, qos, group)

#define AUTOWARE_CREATE_SERVICE2(service_type, service_name, callback) \
  autoware::agnocast_wrapper::create_service<service_type>(this, service_name, callback)
#define AUTOWARE_CREATE_SERVICE3(service_type, service_name, callback, qos) \
  autoware::agnocast_wrapper::create_service<service_type>(this, service_name, callback, qos)
#define AUTOWARE_CREATE_SERVICE4(service_type, service_name, callback, qos, group) \
  autoware::agnocast_wrapper::create_service<service_type>(this, service_name, callback, qos, group)
#define AUTOWARE_CREATE_SERVICE2_ON_NODE(service_type, node, service_name, callback) \
  autoware::agnocast_wrapper::create_service<service_type>(node, service_name, callback)
#define AUTOWARE_CREATE_SERVICE3_ON_NODE(service_type, node, service_name, callback, qos) \
  autoware::agnocast_wrapper::create_service<service_type>(node, service_name, callback, qos)
#define AUTOWARE_CREATE_SERVICE4_ON_NODE(service_type, node, service_name, callback, qos, group) \
  autoware::agnocast_wrapper::create_service<service_type>(node, service_name, callback, qos, group)

#define AUTOWARE_SUBSCRIPTION_OPTIONS agnocast::SubscriptionOptions
#define AUTOWARE_PUBLISHER_OPTIONS agnocast::PublisherOptions

#define ALLOCATE_OUTPUT_MESSAGE_UNIQUE(publisher) publisher->allocate_output_message_unique()
#define ALLOCATE_OUTPUT_MESSAGE_SHARED(publisher) publisher->allocate_output_message_shared()
#define ALLOCATE_OUTPUT_SERVICE_REQUEST(client) client->allocate_output_service_request()

namespace autoware::agnocast_wrapper
{

enum class OwnershipType { Unique, Shared };

template <typename MessageT, OwnershipType Ownership>
class message_interface;

template <typename MessageT>
class message_interface<MessageT, OwnershipType::Unique>
{
public:
  message_interface() = default;

  virtual ~message_interface() = default;

  message_interface(const message_interface & r) = delete;
  message_interface & operator=(const message_interface & r) = delete;

  message_interface(message_interface && r) = default;
  message_interface & operator=(message_interface && r) = default;

  virtual MessageT & as_ref() const noexcept = 0;
  virtual MessageT * as_ptr() const noexcept = 0;

  virtual agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept = 0;
  virtual std::unique_ptr<MessageT> move_ros2_ptr() && noexcept = 0;
};

template <typename MessageT>
class message_interface<MessageT, OwnershipType::Shared>
{
public:
  virtual ~message_interface() = default;

  virtual MessageT & as_ref() const noexcept = 0;
  virtual MessageT * as_ptr() const noexcept = 0;

  virtual agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept = 0;
  virtual std::shared_ptr<MessageT> move_ros2_ptr() && noexcept = 0;

  virtual std::unique_ptr<message_interface<MessageT, OwnershipType::Shared>> clone() const = 0;
};

template <typename MessageT, OwnershipType Ownership>
class agnocast_message;

template <typename MessageT>
class agnocast_message<MessageT, OwnershipType::Unique>
: public message_interface<MessageT, OwnershipType::Unique>
{
  agnocast::ipc_shared_ptr<MessageT> ptr_;

public:
  explicit agnocast_message(agnocast::ipc_shared_ptr<MessageT> && ptr) : ptr_(std::move(ptr)) {}

  MessageT & as_ref() const noexcept override { return *ptr_; }
  MessageT * as_ptr() const noexcept override { return ptr_.get(); }

  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept override
  {
    return std::move(ptr_);
  }

  // The following member function should never be called at runtime. They are implemented just for
  // inheriting `message_interface`.
  std::unique_ptr<MessageT> move_ros2_ptr() && noexcept override
  {
    return std::unique_ptr<MessageT>{};
  }
};

template <typename MessageT>
class agnocast_message<MessageT, OwnershipType::Shared>
: public message_interface<MessageT, OwnershipType::Shared>
{
  agnocast::ipc_shared_ptr<MessageT> ptr_;

public:
  explicit agnocast_message(agnocast::ipc_shared_ptr<MessageT> && ptr) : ptr_(std::move(ptr)) {}

  MessageT & as_ref() const noexcept override { return *ptr_; }
  MessageT * as_ptr() const noexcept override { return ptr_.get(); }

  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept override
  {
    return std::move(ptr_);
  }

  // The following member function should never be called at runtime. They are implemented just for
  // inheriting `message_interface`.
  std::shared_ptr<MessageT> move_ros2_ptr() && noexcept override
  {
    return std::shared_ptr<MessageT>{};
  }

  std::unique_ptr<message_interface<MessageT, OwnershipType::Shared>> clone() const override
  {
    return std::make_unique<agnocast_message<MessageT, OwnershipType::Shared>>(*this);
  }
};

template <typename MessageT, OwnershipType Ownership>
class ros2_message;

template <typename MessageT>
class ros2_message<MessageT, OwnershipType::Unique>
: public message_interface<MessageT, OwnershipType::Unique>
{
  std::unique_ptr<MessageT> ptr_;

public:
  explicit ros2_message(std::unique_ptr<MessageT> && ptr) : ptr_(std::move(ptr)) {}

  MessageT & as_ref() const noexcept override { return *ptr_; }
  MessageT * as_ptr() const noexcept override { return ptr_.get(); }

  std::unique_ptr<MessageT> move_ros2_ptr() && noexcept override { return std::move(ptr_); }

  // The following member function should never be called at runtime. They are implemented just for
  // inheriting `message_interface`.
  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept override
  {
    return agnocast::ipc_shared_ptr<MessageT>{};
  }
};

template <typename MessageT>
class ros2_message<MessageT, OwnershipType::Shared>
: public message_interface<MessageT, OwnershipType::Shared>
{
  std::shared_ptr<MessageT> ptr_;

public:
  explicit ros2_message(std::shared_ptr<MessageT> && ptr) : ptr_(std::move(ptr)) {}

  MessageT & as_ref() const noexcept override { return *ptr_; }
  MessageT * as_ptr() const noexcept override { return ptr_.get(); }

  std::shared_ptr<MessageT> move_ros2_ptr() && noexcept override { return std::move(ptr_); }

  // The following member function should never be called at runtime. They are implemented just for
  // inheriting `message_interface`.
  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept override
  {
    return agnocast::ipc_shared_ptr<MessageT>{};
  }

  std::unique_ptr<message_interface<MessageT, OwnershipType::Shared>> clone() const override
  {
    return std::make_unique<ros2_message<MessageT, OwnershipType::Shared>>(*this);
  }
};

template <typename MessageT, OwnershipType Ownership>
class message_ptr;

template <typename MessageT>
class message_ptr<MessageT, OwnershipType::Unique>
{
  using ros2_ptr_t = std::unique_ptr<MessageT>;

  std::unique_ptr<message_interface<MessageT, OwnershipType::Unique>> ptr_;

  template <typename U>
  friend class AgnocastPublisher;
  template <typename U>
  friend class ROS2Publisher;

private:
  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept
  {
    return std::move(*(std::move(ptr_))).move_agnocast_ptr();
  }

  auto move_ros2_ptr() && noexcept { return std::move(*(std::move(ptr_))).move_ros2_ptr(); }

public:
  message_ptr() : ptr_(nullptr) {}

  explicit message_ptr(agnocast::ipc_shared_ptr<MessageT> && ptr)
  : ptr_(std::make_unique<agnocast_message<MessageT, OwnershipType::Unique>>(std::move(ptr)))
  {
  }

  explicit message_ptr(ros2_ptr_t && ptr)
  : ptr_(std::make_unique<ros2_message<MessageT, OwnershipType::Unique>>(std::move(ptr)))
  {
  }

  message_ptr(const message_ptr & r) = delete;
  message_ptr & operator=(const message_ptr & r) = delete;

  message_ptr(message_ptr && r) noexcept = default;
  message_ptr & operator=(message_ptr && r) noexcept = default;

  MessageT & operator*() const noexcept { return ptr_->as_ref(); }

  MessageT * operator->() const noexcept { return ptr_->as_ptr(); }

  explicit operator bool() const noexcept { return ptr_ && static_cast<bool>(ptr_->as_ptr()); }

  MessageT * get() const noexcept { return ptr_ ? ptr_->as_ptr() : nullptr; }
};

template <typename MessageT>
class message_ptr<MessageT, OwnershipType::Shared>
{
  using ros2_ptr_t = std::shared_ptr<MessageT>;

  std::unique_ptr<message_interface<MessageT, OwnershipType::Shared>> ptr_;

  template <typename U>
  friend class AgnocastPublisher;
  template <typename U>
  friend class ROS2Publisher;
  template <typename U>
  friend class ROS2Client;
  template <typename U>
  friend class AgnocastClient;

private:
  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept
  {
    return std::move(*(std::move(ptr_))).move_agnocast_ptr();
  }

  auto move_ros2_ptr() && noexcept { return std::move(*(std::move(ptr_))).move_ros2_ptr(); }

public:
  message_ptr() : ptr_(nullptr) {}

  explicit message_ptr(agnocast::ipc_shared_ptr<MessageT> && ptr)
  : ptr_(std::make_unique<agnocast_message<MessageT, OwnershipType::Shared>>(std::move(ptr)))
  {
  }

  explicit message_ptr(ros2_ptr_t && ptr)
  : ptr_(std::make_unique<ros2_message<MessageT, OwnershipType::Shared>>(std::move(ptr)))
  {
  }

  message_ptr(const message_ptr & r)
  {
    if (r.ptr_ != nullptr) {
      ptr_ = r.ptr_->clone();
    }
  }
  message_ptr & operator=(const message_ptr & r)
  {
    if (this != &r) {
      if (r.ptr_ != nullptr) {
        ptr_ = r.ptr_->clone();
      } else {
        ptr_ = nullptr;
      }
    }
    return *this;
  }

  message_ptr(message_ptr && r) noexcept = default;
  message_ptr & operator=(message_ptr && r) noexcept = default;

  MessageT & operator*() const noexcept { return ptr_->as_ref(); }

  MessageT * operator->() const noexcept { return ptr_->as_ptr(); }

  explicit operator bool() const noexcept { return ptr_ && static_cast<bool>(ptr_->as_ptr()); }

  MessageT * get() const noexcept { return ptr_ ? ptr_->as_ptr() : nullptr; }
};

// Defaults to zero if the environment variable is missing or invalid.
inline int get_ENABLE_AGNOCAST()
{
  const char * env = std::getenv("ENABLE_AGNOCAST");
  if (env) {
    return std::atoi(env);
  }
  return 0;
}

inline bool use_agnocast()
{
  static const int sv = get_ENABLE_AGNOCAST();
  return sv == 1;
}

template <typename MessageT>
class Subscription
{
public:
  using SharedPtr = std::shared_ptr<Subscription<MessageT>>;

  virtual ~Subscription() = default;
};

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
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&> ||
        std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_CONST_SHARED_PTR(MessageT) &&>,
      "callback should be invocable with an rvalue reference to either "
      "AUTOWARE_MESSAGE_UNIQUE_PTR or AUTOWARE_MESSAGE_CONST_SHARED_PTR");

    constexpr auto ownership =
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&>
        ? OwnershipType::Unique
        : OwnershipType::Shared;

    subscription_ = agnocast::create_subscription<MessageT>(
      node, topic_name, qos,
      [callback = std::forward<Func>(callback)](agnocast::ipc_shared_ptr<MessageT> && msg) {
        if constexpr (ownership == OwnershipType::Unique) {
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
        std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_CONST_SHARED_PTR(MessageT) &&>,
      "callback should be invocable with an rvalue reference to either "
      "AUTOWARE_MESSAGE_UNIQUE_PTR or AUTOWARE_MESSAGE_CONST_SHARED_PTR");

    constexpr auto ownership =
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&>
        ? OwnershipType::Unique
        : OwnershipType::Shared;

    rclcpp::SubscriptionOptions ros2_options;
    ros2_options.callback_group = options.callback_group;
    subscription_ = node->create_subscription<MessageT>(
      topic_name, qos,
      [callback = std::forward<Func>(callback)](std::unique_ptr<MessageT> msg) {
        if constexpr (ownership == OwnershipType::Unique) {
          callback(message_ptr<MessageT, ownership>(std::move(msg)));
        } else {
          callback(
            message_ptr<const MessageT, ownership>(
              std::shared_ptr<const MessageT>(std::move(msg))));
        }
      },
      ros2_options);
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

template <typename MessageT>
class PollingSubscriber
{
public:
  using SharedPtr = std::shared_ptr<PollingSubscriber<MessageT>>;

  virtual ~PollingSubscriber() = default;

  virtual AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) takeData() = 0;
  virtual AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) take_data() = 0;
};

template <typename MessageT>
class AgnocastPollingSubscriber : public PollingSubscriber<MessageT>
{
  typename agnocast::PollingSubscriber<MessageT>::SharedPtr subscriber_;

public:
  template <typename NodeT>
  explicit AgnocastPollingSubscriber(
    NodeT * node, const std::string & topic_name, const rclcpp::QoS & qos)
  : subscriber_(agnocast::create_subscription<MessageT>(node, topic_name, qos))
  {
  }

  AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) takeData() override
  {
    auto data = subscriber_->take_data();
    return AUTOWARE_MESSAGE_SHARED_PTR(const MessageT)(std::move(data));
  }

  AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) take_data() override
  {
    auto data = subscriber_->take_data();
    return AUTOWARE_MESSAGE_SHARED_PTR(const MessageT)(std::move(data));
  }
};

template <typename MessageT>
class ROS2PollingSubscriber : public PollingSubscriber<MessageT>
{
  typename autoware_utils_rclcpp::InterProcessPollingSubscriber<MessageT>::SharedPtr subscriber_;

public:
  explicit ROS2PollingSubscriber(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
  : subscriber_(
      autoware_utils_rclcpp::InterProcessPollingSubscriber<MessageT>::create_subscription(
        node, topic_name, qos))
  {
  }

  AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) takeData() override
  {
    return AUTOWARE_MESSAGE_SHARED_PTR(const MessageT)(std::move(subscriber_->take_data()));
  }

  AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) take_data() override
  {
    return AUTOWARE_MESSAGE_SHARED_PTR(const MessageT)(std::move(subscriber_->take_data()));
  }
};

template <typename MessageT>
typename PollingSubscriber<MessageT>::SharedPtr create_polling_subscriber(
  rclcpp::Node * node, const std::string & topic_name, const size_t qos_history_depth)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastPollingSubscriber<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
  } else {
    return std::make_shared<ROS2PollingSubscriber<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
  }
}

template <typename MessageT>
typename PollingSubscriber<MessageT>::SharedPtr create_polling_subscriber(
  rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastPollingSubscriber<MessageT>>(node, topic_name, qos);
  } else {
    return std::make_shared<ROS2PollingSubscriber<MessageT>>(node, topic_name, qos);
  }
}

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

  /// Publish by const reference (internally copies into allocated message).
  /// This method is discouraged because it performs an implicit copy.
  /// Prefer ALLOCATE_OUTPUT_MESSAGE_{UNIQUE,SHARED}(publisher) + the corresponding publish()
  /// overload. May be marked [[deprecated]] in the future once autoware_cmake supports
  /// suppressing deprecation warnings for test targets.
  virtual void publish(const MessageT & data) = 0;

  virtual uint32_t get_subscription_count() const = 0;
  virtual uint32_t get_intra_process_subscription_count() const = 0;
  virtual const rmw_gid_t & get_gid() const = 0;
  virtual const char * get_topic_name() const = 0;
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

template <typename ServiceT>
class Client
{
protected:
  virtual bool wait_for_service_impl(std::chrono::nanoseconds timeout) const = 0;

public:
  using SharedPtr = std::shared_ptr<Client<ServiceT>>;

  using Future = std::future<AUTOWARE_SERVICE_RESPONSE_PTR(ServiceT)>;
  using SharedFuture = std::shared_future<AUTOWARE_SERVICE_RESPONSE_PTR(ServiceT)>;

  struct FutureAndRequestId : rclcpp::detail::FutureAndRequestId<Future>
  {
    using rclcpp::detail::FutureAndRequestId<Future>::FutureAndRequestId;
    SharedFuture share() noexcept { return this->future.share(); }
  };
  struct SharedFutureAndRequestId : rclcpp::detail::FutureAndRequestId<SharedFuture>
  {
    using rclcpp::detail::FutureAndRequestId<SharedFuture>::FutureAndRequestId;
  };

  virtual ~Client() = default;

  virtual AUTOWARE_SERVICE_REQUEST_PTR(ServiceT) allocate_output_service_request() = 0;

  virtual const char * get_service_name() const = 0;

  virtual bool service_is_ready() const = 0;

  template <typename RepT, typename RatioT>
  bool wait_for_service(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::nanoseconds(-1)) const
  {
    return wait_for_service_impl(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  virtual FutureAndRequestId async_send_request(
    AUTOWARE_SERVICE_REQUEST_PTR(ServiceT) && request) = 0;
  virtual SharedFutureAndRequestId async_send_request(
    AUTOWARE_SERVICE_REQUEST_PTR(ServiceT) && request,
    std::function<void(SharedFuture)> callback) = 0;
};

template <typename ServiceT>
class AgnocastClient : public Client<ServiceT>
{
  typename agnocast::Client<ServiceT>::SharedPtr client_;

protected:
  bool wait_for_service_impl(std::chrono::nanoseconds timeout) const override
  {
    return client_->wait_for_service(timeout);
  }

public:
  template <typename NodeT>
  explicit AgnocastClient(
    NodeT * node, const std::string & service_name, const rclcpp::QoS & qos,
    rclcpp::CallbackGroup::SharedPtr group)
  : client_(agnocast::create_client<ServiceT>(node, service_name, qos, group))
  {
  }

  AUTOWARE_SERVICE_REQUEST_PTR(ServiceT) allocate_output_service_request() override
  {
    return AUTOWARE_SERVICE_REQUEST_PTR(ServiceT){client_->borrow_loaned_request()};
  }

  const char * get_service_name() const override { return client_->get_service_name(); }

  bool service_is_ready() const override { return client_->service_is_ready(); }

  AUTOWARE_CLIENT_FUTURE_AND_REQUEST_ID(ServiceT)
  async_send_request(AUTOWARE_SERVICE_REQUEST_PTR(ServiceT) && request) override
  {
    // Wrap the promise in a shared_ptr so that the callback lambda can call set_value()
    // through the pointer without needing 'mutable'. A unique_ptr wouldn't work here because
    // the lambda is stored in a std::function, which requires its callable to be copyable.
    auto promise_ptr = std::make_shared<std::promise<AUTOWARE_SERVICE_RESPONSE_PTR(ServiceT)>>();
    AUTOWARE_CLIENT_FUTURE(ServiceT) future = promise_ptr->get_future();

    auto agnocast_request = std::move(request).move_agnocast_ptr();
    auto request_id =
      client_
        ->async_send_request(
          std::move(agnocast_request),
          [promise_ptr = std::move(promise_ptr)](
            typename agnocast::Client<ServiceT>::SharedFuture agnocast_shared_future) {
            typename agnocast::ipc_shared_ptr<typename ServiceT::Response> agnocast_response =
              agnocast_shared_future.get();
            promise_ptr->set_value(
              AUTOWARE_SERVICE_RESPONSE_PTR(ServiceT){std::move(agnocast_response)});
          })
        .request_id;

    return AUTOWARE_CLIENT_FUTURE_AND_REQUEST_ID(ServiceT)(std::move(future), request_id);
  }

  AUTOWARE_CLIENT_SHARED_FUTURE_AND_REQUEST_ID(ServiceT)
  async_send_request(
    AUTOWARE_SERVICE_REQUEST_PTR(ServiceT) && request,
    std::function<void(AUTOWARE_CLIENT_SHARED_FUTURE(ServiceT))> callback) override
  {
    auto promise_ptr = std::make_shared<std::promise<AUTOWARE_SERVICE_RESPONSE_PTR(ServiceT)>>();
    AUTOWARE_CLIENT_SHARED_FUTURE(ServiceT) shared_future = promise_ptr->get_future().share();

    auto agnocast_request = std::move(request).move_agnocast_ptr();
    auto request_id =
      client_
        ->async_send_request(
          std::move(agnocast_request),
          [callback = std::move(callback), promise_ptr = std::move(promise_ptr), shared_future](
            typename agnocast::Client<ServiceT>::SharedFuture agnocast_shared_future) {
            typename agnocast::ipc_shared_ptr<typename ServiceT::Response> agnocast_response =
              agnocast_shared_future.get();
            promise_ptr->set_value(
              AUTOWARE_SERVICE_RESPONSE_PTR(ServiceT){std::move(agnocast_response)});
            callback(std::move(shared_future));
          })
        .request_id;

    return AUTOWARE_CLIENT_SHARED_FUTURE_AND_REQUEST_ID(ServiceT)(
      std::move(shared_future), request_id);
  }
};

template <typename ServiceT>
class ROS2Client : public Client<ServiceT>
{
  typename rclcpp::Client<ServiceT>::SharedPtr client_;

protected:
  bool wait_for_service_impl(std::chrono::nanoseconds timeout) const override
  {
    return client_->wait_for_service(timeout);
  }

public:
  explicit ROS2Client(
    rclcpp::Node * node, const std::string & service_name, const rclcpp::QoS & qos,
    rclcpp::CallbackGroup::SharedPtr group)
#if RCLCPP_VERSION_MAJOR >= 28
  : client_(node->create_client<ServiceT>(service_name, qos, group))
#else
  : client_(node->create_client<ServiceT>(service_name, qos.get_rmw_qos_profile(), group))
#endif
  {
  }

  AUTOWARE_SERVICE_REQUEST_PTR(ServiceT) allocate_output_service_request() override
  {
    return AUTOWARE_SERVICE_REQUEST_PTR(ServiceT){std::make_shared<typename ServiceT::Request>()};
  }

  const char * get_service_name() const override { return client_->get_service_name(); }

  bool service_is_ready() const override { return client_->service_is_ready(); }

  AUTOWARE_CLIENT_FUTURE_AND_REQUEST_ID(ServiceT)
  async_send_request(AUTOWARE_SERVICE_REQUEST_PTR(ServiceT) && request) override
  {
    auto promise_ptr = std::make_shared<std::promise<AUTOWARE_SERVICE_RESPONSE_PTR(ServiceT)>>();
    AUTOWARE_CLIENT_FUTURE(ServiceT) future = promise_ptr->get_future();

    auto ros2_request = std::move(request).move_ros2_ptr();
    auto request_id = client_
                        ->async_send_request(
                          ros2_request,
                          [promise_ptr = std::move(promise_ptr)](
                            typename rclcpp::Client<ServiceT>::SharedFuture ros2_shared_future) {
                            typename ServiceT::Response::SharedPtr ros2_response =
                              ros2_shared_future.get();
                            promise_ptr->set_value(
                              AUTOWARE_SERVICE_RESPONSE_PTR(ServiceT){std::move(ros2_response)});
                          })
                        .request_id;

    return AUTOWARE_CLIENT_FUTURE_AND_REQUEST_ID(ServiceT)(std::move(future), request_id);
  }

  AUTOWARE_CLIENT_SHARED_FUTURE_AND_REQUEST_ID(ServiceT)
  async_send_request(
    AUTOWARE_SERVICE_REQUEST_PTR(ServiceT) && request,
    std::function<void(AUTOWARE_CLIENT_SHARED_FUTURE(ServiceT))> callback) override
  {
    auto promise_ptr = std::make_shared<std::promise<AUTOWARE_SERVICE_RESPONSE_PTR(ServiceT)>>();
    AUTOWARE_CLIENT_SHARED_FUTURE(ServiceT) shared_future = promise_ptr->get_future().share();

    auto ros2_request = std::move(request).move_ros2_ptr();
    auto request_id =
      client_
        ->async_send_request(
          ros2_request,
          [callback = std::move(callback), promise_ptr = std::move(promise_ptr),
           shared_future](typename rclcpp::Client<ServiceT>::SharedFuture ros2_shared_future) {
            typename ServiceT::Response::SharedPtr ros2_response = ros2_shared_future.get();
            promise_ptr->set_value(
              AUTOWARE_SERVICE_RESPONSE_PTR(ServiceT){std::move(ros2_response)});
            callback(std::move(shared_future));
          })
        .request_id;

    return AUTOWARE_CLIENT_SHARED_FUTURE_AND_REQUEST_ID(ServiceT)(
      std::move(shared_future), request_id);
  }
};

template <typename ServiceT>
AUTOWARE_CLIENT_PTR(ServiceT)
create_client(
  rclcpp::Node * node, const std::string & service_name,
  const rclcpp::QoS & qos = rclcpp::ServicesQoS(), rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastClient<ServiceT>>(node, service_name, qos, group);
  } else {
    return std::make_shared<ROS2Client<ServiceT>>(node, service_name, qos, group);
  }
}

template <typename ServiceT>
class Service
{
public:
  using SharedPtr = std::shared_ptr<Service<ServiceT>>;

  virtual ~Service() = default;
};

template <typename ServiceT>
class AgnocastService : public Service<ServiceT>
{
  typename agnocast::Service<ServiceT>::SharedPtr srv_;

public:
  template <typename NodeT, typename Func>
  explicit AgnocastService(
    NodeT * node, const std::string & service_name, Func && callback, const rclcpp::QoS & qos,
    rclcpp::CallbackGroup::SharedPtr group)
  {
    static_assert(
      std::is_invocable_v<
        std::decay_t<Func>, AUTOWARE_SERVICE_REQUEST_PTR(ServiceT) &&,
        AUTOWARE_SERVICE_RESPONSE_PTR(ServiceT) &&>,
      "Callback should be invocable with AUTOWARE_SERVICE_REQUEST_PTR and "
      "AUTOWARE_SERVICE_RESPONSE_PTR (const&, &&, or by-value)");

    srv_ = agnocast::create_service<ServiceT>(
      node, service_name,
      [callback = std::forward<Func>(callback)](
        agnocast::ipc_shared_ptr<typename ServiceT::Request> && agnocast_request,
        agnocast::ipc_shared_ptr<typename ServiceT::Response> && agnocast_response) {
        callback(
          AUTOWARE_SERVICE_REQUEST_PTR(ServiceT){std::move(agnocast_request)},
          AUTOWARE_SERVICE_RESPONSE_PTR(ServiceT){std::move(agnocast_response)});
      },
      qos, group);
  }
};

template <typename ServiceT>
class ROS2Service : public Service<ServiceT>
{
  typename rclcpp::Service<ServiceT>::SharedPtr srv_;

public:
  template <typename Func>
  explicit ROS2Service(
    rclcpp::Node * node, const std::string & service_name, Func && callback,
    const rclcpp::QoS & qos, rclcpp::CallbackGroup::SharedPtr group)
  {
    static_assert(
      std::is_invocable_v<
        std::decay_t<Func>, AUTOWARE_SERVICE_REQUEST_PTR(ServiceT) &&,
        AUTOWARE_SERVICE_RESPONSE_PTR(ServiceT) &&>,
      "Callback should be invocable with AUTOWARE_SERVICE_REQUEST_PTR and "
      "AUTOWARE_SERVICE_RESPONSE_PTR (const&, &&, or by-value)");

    srv_ = node->create_service<ServiceT>(
      service_name,
      [callback = std::forward<Func>(callback)](
        std::shared_ptr<typename ServiceT::Request> && ros2_request,
        std::shared_ptr<typename ServiceT::Response> && ros2_response) {
        callback(
          AUTOWARE_SERVICE_REQUEST_PTR(ServiceT){std::move(ros2_request)},
          AUTOWARE_SERVICE_RESPONSE_PTR(ServiceT){std::move(ros2_response)});
      },
#if RCLCPP_VERSION_MAJOR >= 28
      qos, group);
#else
      qos.get_rmw_qos_profile(), group);
#endif
  }
};

template <typename ServiceT, typename Func>
AUTOWARE_SERVICE_PTR(ServiceT)
create_service(
  rclcpp::Node * node, const std::string & service_name, Func && callback,
  const rclcpp::QoS & qos = rclcpp::ServicesQoS(), rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastService<ServiceT>>(
      node, service_name, std::forward<Func>(callback), qos, group);
  } else {
    return std::make_shared<ROS2Service<ServiceT>>(
      node, service_name, std::forward<Func>(callback), qos, group);
  }
}

}  // namespace autoware::agnocast_wrapper

#else

#define AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) std::unique_ptr<MessageT>
// For publisher (mutable message)
#define AUTOWARE_MESSAGE_SHARED_PTR(MessageT) std::shared_ptr<MessageT>
// For subscription (read-only message)
#define AUTOWARE_MESSAGE_CONST_SHARED_PTR(MessageT) std::shared_ptr<const MessageT>
#define AUTOWARE_SERVICE_REQUEST_PTR(ServiceT) std::shared_ptr<typename ServiceT::Request>
#define AUTOWARE_SERVICE_RESPONSE_PTR(ServiceT) std::shared_ptr<typename ServiceT::Response>
#define AUTOWARE_SUBSCRIPTION_PTR(MessageT) typename rclcpp::Subscription<MessageT>::SharedPtr
#define AUTOWARE_PUBLISHER_PTR(MessageT) typename rclcpp::Publisher<MessageT>::SharedPtr
#define AUTOWARE_POLLING_SUBSCRIBER_PTR(MessageT) \
  typename autoware_utils_rclcpp::InterProcessPollingSubscriber<MessageT>::SharedPtr
#define AUTOWARE_CLIENT_PTR(ServiceT) typename rclcpp::Client<ServiceT>::SharedPtr
#define AUTOWARE_SERVICE_PTR(ServiceT) typename rclcpp::Service<ServiceT>::SharedPtr
#define AUTOWARE_CLIENT_FUTURE(ServiceT) typename rclcpp::Client<ServiceT>::Future
#define AUTOWARE_CLIENT_SHARED_FUTURE(ServiceT) typename rclcpp::Client<ServiceT>::SharedFuture
#define AUTOWARE_CLIENT_FUTURE_AND_REQUEST_ID(ServiceT) \
  typename rclcpp::Client<ServiceT>::FutureAndRequestId
#define AUTOWARE_CLIENT_SHARED_FUTURE_AND_REQUEST_ID(ServiceT) \
  typename rclcpp::Client<ServiceT>::SharedFutureAndRequestId

#define AUTOWARE_CREATE_SUBSCRIPTION(message_type, topic, qos, callback, options) \
  this->create_subscription<message_type>(topic, qos, callback, options)
#define AUTOWARE_CREATE_SUBSCRIPTION_ON_NODE(message_type, node, topic, qos, callback, options) \
  node->create_subscription<message_type>(topic, qos, callback, options)

#define AUTOWARE_CREATE_PUBLISHER2(message_type, arg1, arg2) \
  this->create_publisher<message_type>(arg1, arg2)
#define AUTOWARE_CREATE_PUBLISHER3(message_type, arg1, arg2, arg3) \
  this->create_publisher<message_type>(arg1, arg2, arg3)
#define AUTOWARE_CREATE_PUBLISHER2_ON_NODE(message_type, node, arg1, arg2) \
  node->create_publisher<message_type>(arg1, arg2)
#define AUTOWARE_CREATE_PUBLISHER3_ON_NODE(message_type, node, arg1, arg2, arg3) \
  node->create_publisher<message_type>(arg1, arg2, arg3)

#define AUTOWARE_CREATE_POLLING_SUBSCRIBER(message_type, topic, qos)                       \
  autoware_utils_rclcpp::InterProcessPollingSubscriber<message_type>::create_subscription( \
    this, topic, qos)
#define AUTOWARE_CREATE_POLLING_SUBSCRIBER_ON_NODE(message_type, node, topic, qos)         \
  autoware_utils_rclcpp::InterProcessPollingSubscriber<message_type>::create_subscription( \
    node, topic, qos)

#define AUTOWARE_CREATE_CLIENT1(service_type, service_name) \
  this->create_client<service_type>(service_name)
#define AUTOWARE_CREATE_CLIENT2(service_type, service_name, qos) \
  this->create_client<service_type>(service_name, qos)
#define AUTOWARE_CREATE_CLIENT3(service_type, service_name, qos, group) \
  this->create_client<service_type>(service_name, qos, group)
#define AUTOWARE_CREATE_CLIENT1_ON_NODE(service_type, node, service_name) \
  node->create_client<service_type>(service_name)
#define AUTOWARE_CREATE_CLIENT2_ON_NODE(service_type, node, service_name, qos) \
  node->create_client<service_type>(service_name, qos)
#define AUTOWARE_CREATE_CLIENT3_ON_NODE(service_type, node, service_name, qos, group) \
  node->create_client<service_type>(service_name, qos, group)

#define AUTOWARE_CREATE_SERVICE2(service_type, service_name, callback) \
  this->create_service<service_type>(service_name, callback)
#define AUTOWARE_CREATE_SERVICE3(service_type, service_name, callback, qos) \
  this->create_service<service_type>(service_name, callback, qos)
#define AUTOWARE_CREATE_SERVICE4(service_type, service_name, callback, qos, group) \
  this->create_service<service_type>(service_name, callback, qos, group)
#define AUTOWARE_CREATE_SERVICE2_ON_NODE(service_type, node, service_name, callback) \
  node->create_service<service_type>(service_name, callback)
#define AUTOWARE_CREATE_SERVICE3_ON_NODE(service_type, node, service_name, callback, qos) \
  node->create_service<service_type>(service_name, callback, qos)
#define AUTOWARE_CREATE_SERVICE4_ON_NODE(service_type, node, service_name, callback, qos, group) \
  node->create_service<service_type>(service_name, callback, qos, group)

#define AUTOWARE_SUBSCRIPTION_OPTIONS rclcpp::SubscriptionOptions
#define AUTOWARE_PUBLISHER_OPTIONS rclcpp::PublisherOptions

#define ALLOCATE_OUTPUT_MESSAGE_UNIQUE(publisher) \
  std::make_unique<typename std::remove_reference<decltype(*publisher)>::type::ROSMessageType>()
#define ALLOCATE_OUTPUT_MESSAGE_SHARED(publisher) \
  std::make_shared<typename std::remove_reference<decltype(*publisher)>::type::ROSMessageType>()
#define ALLOCATE_OUTPUT_SERVICE_REQUEST(client) \
  std::make_shared<typename std::remove_reference<decltype(*client)>::type::Request>()

#endif
