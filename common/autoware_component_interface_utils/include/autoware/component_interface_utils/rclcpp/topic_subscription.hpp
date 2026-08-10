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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_SUBSCRIPTION_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_SUBSCRIPTION_HPP_

#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

namespace autoware::component_interface_utils
{

/// True when the subscription handle can hand back the latest message as a shared pointer.
/// Node types backed by shared memory offer this so take() need not copy the message out;
/// rclcpp::Subscription does not, and falls back to the take() loop below. Spelled as a
/// detection idiom so this package stays independent of any particular node type.
template <class T, class = void>
struct has_take_data : std::false_type
{
};
template <class T>
struct has_take_data<T, std::void_t<decltype(std::declval<T &>().take_data())>> : std::true_type
{
};

/// The wrapper class of a subscription. The handle type comes from the node's
/// create_subscription(). take() and take_and_update() additionally need the handle to provide
/// rclcpp's take(), and being ordinary members they are only instantiated when called.
template <class SpecT, class NodeT = rclcpp::Node>
class Subscription
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Subscription)
  using SpecType = SpecT;
  using NodeType = NodeT;
  using WrapSharedPtr =
    decltype(std::declval<NodeT &>().template create_subscription<typename SpecT::Message>(
      std::declval<const std::string &>(), std::declval<const rclcpp::QoS &>(),
      std::declval<std::function<void(const typename SpecT::Message &)>>()));
  using WrapType = typename WrapSharedPtr::element_type;

  /// Constructor.
  explicit Subscription(WrapSharedPtr subscription)
  {
    subscription_ = subscription;  // to keep the reference count
  }

  typename SpecType::Message::ConstSharedPtr take()
  {
    if constexpr (has_take_data<WrapType>::value) {
      // The handle already returns the latest message as a pointer, without copying it.
      return subscription_->take_data();
    } else {
      rclcpp::MessageInfo info;
      auto data = std::make_shared<typename SpecType::Message>();
      bool flag = false;
      for (size_t i = 0; i < subscription_->get_actual_qos().depth(); ++i) {
        if (!subscription_->take(*data, info)) {
          break;
        }
        flag = true;  // Whether there is at least one data.
      }
      return flag ? data : nullptr;
    }
  }

  bool take_and_update(typename SpecType::Message::ConstSharedPtr & ptr)
  {
    const auto msg = take();
    if (!msg) {
      return false;
    }
    ptr = msg;
    return true;
  }

  bool take_and_update(typename SpecType::Message & ref)
  {
    const auto msg = take();
    if (!msg) {
      return false;
    }
    ref = *msg;
    return true;
  }

private:
  RCLCPP_DISABLE_COPY(Subscription)
  WrapSharedPtr subscription_;
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_SUBSCRIPTION_HPP_
