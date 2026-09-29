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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_PUBLISHER_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_PUBLISHER_HPP_

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>

#include <string>
#include <utility>

namespace autoware::component_interface_utils
{

/// The wrapper class of a publisher. The handle type comes from the node's create_publisher().
template <class SpecT, class NodeT = rclcpp::Node>
class Publisher
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Publisher)
  using SpecType = SpecT;
  using NodeType = NodeT;
  using WrapSharedPtr =
    decltype(std::declval<NodeT &>().template create_publisher<typename SpecT::Message>(
      std::declval<const std::string &>(), std::declval<const rclcpp::QoS &>()));
  using WrapType = typename WrapSharedPtr::element_type;

  /// Constructor.
  explicit Publisher(WrapSharedPtr publisher)
  {
    publisher_ = publisher;  // to keep the reference count
  }

  /// Publish a message.
  void publish(const typename SpecT::Message & msg) { publisher_->publish(msg); }

private:
  RCLCPP_DISABLE_COPY(Publisher)
  WrapSharedPtr publisher_;
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_PUBLISHER_HPP_
