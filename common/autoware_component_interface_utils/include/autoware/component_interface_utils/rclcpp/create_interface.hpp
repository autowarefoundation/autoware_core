// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__CREATE_INTERFACE_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__CREATE_INTERFACE_HPP_

#include <autoware/component_interface_utils/rclcpp/interface.hpp>
#include <autoware/component_interface_utils/rclcpp/registration.hpp>
#include <autoware/component_interface_utils/rclcpp/service_client.hpp>
#include <autoware/component_interface_utils/rclcpp/service_server.hpp>
#include <autoware/component_interface_utils/rclcpp/topic_publisher.hpp>
#include <autoware/component_interface_utils/rclcpp/topic_subscription.hpp>
#include <autoware/component_interface_utils/specs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

#include <memory>
#include <string>
#include <type_traits>
#include <utility>

namespace autoware::component_interface_utils
{

/// True when the node can create a subscription with no callback at all, which is what the
/// polling path below actually wants. rclcpp::Node cannot, so it keeps getting the classic
/// idiom (a no-op callback in a callback group that is never spun); node types that can do
/// better -- e.g. one that registers no notification at all -- are used directly. Spelled as a
/// detection idiom so this package stays independent of any particular node type.
template <class NodeT, class MessageT, class = void>
struct has_callbackless_create_subscription : std::false_type
{
};
template <class NodeT, class MessageT>
struct has_callbackless_create_subscription<
  NodeT, MessageT,
  std::void_t<decltype(std::declval<NodeT &>().template create_subscription<MessageT>(
    std::declval<const std::string &>(), std::declval<const rclcpp::QoS &>()))>> : std::true_type
{
};

/// Create a client wrapper for logging. This is a private implementation.
template <class SpecT, class NodeT>
typename Client<SpecT, NodeT>::SharedPtr create_client_impl(
  std::shared_ptr<NodeInterface<NodeT>> interface, rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  // This function is a wrapper for the following.
  // https://github.com/ros2/rclcpp/blob/48068130edbb43cdd61076dc1851672ff1a80408/rclcpp/include/rclcpp/node.hpp#L253-L265
  return Client<SpecT, NodeT>::make_shared(interface, group);
}

/// Create a service wrapper for logging. This is a private implementation.
template <class SpecT, class NodeT, class CallbackT>
typename Service<SpecT, NodeT>::SharedPtr create_service_impl(
  std::shared_ptr<NodeInterface<NodeT>> interface, CallbackT && callback,
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  // This function is a wrapper for the following.
  // https://github.com/ros2/rclcpp/blob/48068130edbb43cdd61076dc1851672ff1a80408/rclcpp/include/rclcpp/node.hpp#L267-L281
  return Service<SpecT, NodeT>::make_shared(interface, std::forward<CallbackT>(callback), group);
}

/// Create a publisher using traits like services. This is a private implementation.
template <class SpecT, class NodeT>
typename Publisher<SpecT, NodeT>::SharedPtr create_publisher_impl(
  std::shared_ptr<NodeInterface<NodeT>> interface)
{
  // This function is a wrapper for the following.
  // https://github.com/ros2/rclcpp/blob/48068130edbb43cdd61076dc1851672ff1a80408/rclcpp/include/rclcpp/node.hpp#L167-L205
  auto publisher = interface->node->template create_publisher<typename SpecT::Message>(
    SpecT::name, get_qos<SpecT>());
  interface->register_interface(
    make_record<SpecT>(
      InterfaceRecord::Kind::Topic, InterfaceRecord::Role::Provide, publisher->get_topic_name(),
      rosidl_generator_traits::name<typename SpecT::Message>(),
      publisher->get_actual_qos().get_rmw_qos_profile()));
  return Publisher<SpecT, NodeT>::make_shared(publisher);
}

/// Create a subscription using traits like services. This is a private implementation.
template <class SpecT, class NodeT, class CallbackT>
typename Subscription<SpecT, NodeT>::SharedPtr create_subscription_impl(
  std::shared_ptr<NodeInterface<NodeT>> interface, CallbackT && callback)
{
  // Hoisted out of the else-if below so the condition fits on one line.
  constexpr bool node_makes_callbackless_subscription =
    has_callbackless_create_subscription<NodeT, typename SpecT::Message>::value;

  typename Subscription<SpecT, NodeT>::WrapSharedPtr subscription;
  if constexpr (!std::is_null_pointer_v<CallbackT>) {
    // This function is a wrapper for the following.
    // https://github.com/ros2/rclcpp/blob/48068130edbb43cdd61076dc1851672ff1a80408/rclcpp/include/rclcpp/node.hpp#L207-L238
    subscription = interface->node->template create_subscription<typename SpecT::Message>(
      SpecT::name, get_qos<SpecT>(), std::forward<CallbackT>(callback));
  } else if constexpr (node_makes_callbackless_subscription) {
    // If the callback is nullptr, create a subscription for polling. The node builds whatever
    // a callback-less subscription means for it.
    subscription = interface->node->template create_subscription<typename SpecT::Message>(
      SpecT::name, get_qos<SpecT>());
  } else {
    // If the callback is nullptr, create a subscription for polling.
    // https://github.com/autowarefoundation/autoware.universe/tree/main/common/autoware_universe_utils/include/autoware/universe_utils/ros/polling_subscriber.hpp
    auto group =
      interface->node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto options = rclcpp::SubscriptionOptions();
    options.callback_group = group;

    subscription = interface->node->template create_subscription<typename SpecT::Message>(
      SpecT::name, get_qos<SpecT>(), [](const typename SpecT::Message) {}, options);
  }
  interface->register_interface(
    make_record<SpecT>(
      InterfaceRecord::Kind::Topic, InterfaceRecord::Role::Require, subscription->get_topic_name(),
      rosidl_generator_traits::name<typename SpecT::Message>(),
      subscription->get_actual_qos().get_rmw_qos_profile()));
  return Subscription<SpecT, NodeT>::make_shared(subscription);
}

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__CREATE_INTERFACE_HPP_
