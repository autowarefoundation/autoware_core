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

#ifdef USE_AGNOCAST_ENABLED

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
#define AUTOWARE_SERVER_REQUEST_PTR(ServiceT) \
  autoware::agnocast_wrapper::message_ptr<    \
    const typename ServiceT::Request, autoware::agnocast_wrapper::OwnershipType::Shared>
#define AUTOWARE_SERVER_RESPONSE_PTR(ServiceT) \
  autoware::agnocast_wrapper::message_ptr<     \
    typename ServiceT::Response, autoware::agnocast_wrapper::OwnershipType::Shared>
#define AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) \
  autoware::agnocast_wrapper::message_ptr<    \
    typename ServiceT::Request, autoware::agnocast_wrapper::OwnershipType::Shared>
#define AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT) \
  autoware::agnocast_wrapper::message_ptr<     \
    const typename ServiceT::Response, autoware::agnocast_wrapper::OwnershipType::Shared>
#define AUTOWARE_SUBSCRIPTION_PTR(MessageT) \
  typename autoware::agnocast_wrapper::Subscription<MessageT>::SharedPtr
#define AUTOWARE_PUBLISHER_PTR(MessageT) \
  typename autoware::agnocast_wrapper::Publisher<MessageT>::SharedPtr
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
#define AUTOWARE_TIMER_PTR autoware::agnocast_wrapper::Timer::SharedPtr

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

#else

#define AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) std::unique_ptr<MessageT>

// For publisher (mutable message)
#define AUTOWARE_MESSAGE_SHARED_PTR(MessageT) std::shared_ptr<MessageT>
// For subscription (read-only message)
#define AUTOWARE_MESSAGE_CONST_SHARED_PTR(MessageT) std::shared_ptr<const MessageT>
#define AUTOWARE_SERVER_REQUEST_PTR(ServiceT) std::shared_ptr<const typename ServiceT::Request>
#define AUTOWARE_SERVER_RESPONSE_PTR(ServiceT) std::shared_ptr<typename ServiceT::Response>
#define AUTOWARE_CLIENT_REQUEST_PTR(ServiceT) std::shared_ptr<typename ServiceT::Request>
#define AUTOWARE_CLIENT_RESPONSE_PTR(ServiceT) std::shared_ptr<const typename ServiceT::Response>
#define AUTOWARE_SUBSCRIPTION_PTR(MessageT) typename rclcpp::Subscription<MessageT>::SharedPtr
#define AUTOWARE_PUBLISHER_PTR(MessageT) typename rclcpp::Publisher<MessageT>::SharedPtr
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
#define AUTOWARE_TIMER_PTR rclcpp::TimerBase::SharedPtr

#define AUTOWARE_CREATE_SUBSCRIPTION(message_type, topic, qos, callback, options) \
  this->create_subscription<message_type>(topic, qos, callback, options)
#define AUTOWARE_CREATE_SUBSCRIPTION_ON_NODE(message_type, node, topic, qos, callback, options) \
  (node)->create_subscription<message_type>(topic, qos, callback, options)

#define AUTOWARE_CREATE_PUBLISHER2(message_type, arg1, arg2) \
  this->create_publisher<message_type>(arg1, arg2)
#define AUTOWARE_CREATE_PUBLISHER3(message_type, arg1, arg2, arg3) \
  this->create_publisher<message_type>(arg1, arg2, arg3)
#define AUTOWARE_CREATE_PUBLISHER2_ON_NODE(message_type, node, arg1, arg2) \
  (node)->create_publisher<message_type>(arg1, arg2)
#define AUTOWARE_CREATE_PUBLISHER3_ON_NODE(message_type, node, arg1, arg2, arg3) \
  (node)->create_publisher<message_type>(arg1, arg2, arg3)

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

#define AUTOWARE_SUBSCRIPTION_OPTIONS rclcpp::SubscriptionOptions
#define AUTOWARE_PUBLISHER_OPTIONS rclcpp::PublisherOptions

#define ALLOCATE_OUTPUT_MESSAGE_UNIQUE(publisher) \
  std::make_unique<typename std::remove_reference<decltype(*publisher)>::type::ROSMessageType>()
#define ALLOCATE_OUTPUT_MESSAGE_SHARED(publisher) \
  std::make_shared<typename std::remove_reference<decltype(*publisher)>::type::ROSMessageType>()
#define ALLOCATE_OUTPUT_SERVICE_REQUEST(client) client->allocate_output_service_request()

#endif
