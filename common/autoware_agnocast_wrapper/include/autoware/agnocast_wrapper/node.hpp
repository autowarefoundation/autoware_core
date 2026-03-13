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

#include "autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/node/agnocast_node.hpp>

namespace autoware::agnocast_wrapper
{

/// @brief Node wrapper class that can switch between rclcpp::Node and agnocast::Node at runtime
/// based on the ENABLE_AGNOCAST environment variable.
class Node
{
public:
  using SharedPtr = std::shared_ptr<Node>;

  /// @brief Constructor with node name (Component Node compatible)
  /// @param node_name The name of the node
  /// @param options Node options
  explicit Node(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Constructor with node name and namespace
  /// @param node_name The name of the node
  /// @param namespace_ The namespace of the node
  /// @param options Node options
  explicit Node(
    const std::string & node_name, const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~Node() = default;

  // ===== Basic information =====
  std::string get_name() const;
  std::string get_namespace() const;
  std::string get_fully_qualified_name() const;
  rclcpp::Logger get_logger() const;

  // ===== Time =====
  rclcpp::Clock::SharedPtr get_clock();
  rclcpp::Time now() const;

  // ===== Node interfaces =====
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_node_topics_interface();
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr get_node_parameters_interface() const;

  // ===== Callback groups =====
  rclcpp::CallbackGroup::SharedPtr create_callback_group(
    rclcpp::CallbackGroupType group_type, bool automatically_add_to_executor_with_node = true);

  // ===== Parameters (non-template) =====
  const rclcpp::ParameterValue & declare_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & descriptor =
      rcl_interfaces::msg::ParameterDescriptor{},
    bool ignore_override = false);

  const rclcpp::ParameterValue & declare_parameter(
    const std::string & name, rclcpp::ParameterType type,
    const rcl_interfaces::msg::ParameterDescriptor & descriptor =
      rcl_interfaces::msg::ParameterDescriptor{},
    bool ignore_override = false);

  bool has_parameter(const std::string & name) const;
  void undeclare_parameter(const std::string & name);
  rclcpp::Parameter get_parameter(const std::string & name) const;
  bool get_parameter(const std::string & name, rclcpp::Parameter & parameter) const;
  std::vector<rclcpp::Parameter> get_parameters(const std::vector<std::string> & names) const;

  rcl_interfaces::msg::SetParametersResult set_parameter(const rclcpp::Parameter & parameter);
  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);
  rcl_interfaces::msg::SetParametersResult set_parameters_atomically(
    const std::vector<rclcpp::Parameter> & parameters);

  rcl_interfaces::msg::ParameterDescriptor describe_parameter(const std::string & name) const;
  std::vector<rcl_interfaces::msg::ParameterDescriptor> describe_parameters(
    const std::vector<std::string> & names) const;
  std::vector<uint8_t> get_parameter_types(const std::vector<std::string> & names) const;
  rcl_interfaces::msg::ListParametersResult list_parameters(
    const std::vector<std::string> & prefixes, uint64_t depth) const;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr add_on_set_parameters_callback(
    rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType callback);
  void remove_on_set_parameters_callback(
    const rclcpp::node_interfaces::OnSetParametersCallbackHandle * const handler);

  // ===== Parameters (template) =====
  template <typename ParameterT>
  ParameterT declare_parameter(
    const std::string & name, const ParameterT & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & descriptor =
      rcl_interfaces::msg::ParameterDescriptor{},
    bool ignore_override = false)
  {
    return declare_parameter(
             name, rclcpp::ParameterValue(default_value), descriptor, ignore_override)
      .get<ParameterT>();
  }

  template <typename ParameterT>
  ParameterT declare_parameter(
    const std::string & name,
    const rcl_interfaces::msg::ParameterDescriptor & descriptor =
      rcl_interfaces::msg::ParameterDescriptor{},
    bool ignore_override = false)
  {
    rclcpp::ParameterValue value{ParameterT{}};
    return declare_parameter(name, value.get_type(), descriptor, ignore_override)
      .template get<ParameterT>();
  }

  template <typename ParameterT>
  bool get_parameter(const std::string & name, ParameterT & parameter) const
  {
    rclcpp::Parameter param;
    if (get_parameter(name, param)) {
      parameter = param.get_value<ParameterT>();
      return true;
    }
    return false;
  }

  template <typename ParameterT>
  bool get_parameters(const std::string & prefix, std::map<std::string, ParameterT> & values) const
  {
    std::map<std::string, rclcpp::Parameter> params;
    auto params_interface = get_node_parameters_interface();
    bool result = params_interface->get_parameters_by_prefix(prefix, params);
    if (result) {
      for (const auto & param : params) {
        values[param.first] = param.second.get_value<ParameterT>();
      }
    }
    return result;
  }

  // ===== Publisher =====
  template <typename MessageT>
  typename Publisher<MessageT>::SharedPtr create_publisher(
    const std::string & topic_name, const rclcpp::QoS & qos,
    const agnocast::PublisherOptions & options = agnocast::PublisherOptions{})
  {
    return visit_node([&](auto & n) -> typename Publisher<MessageT>::SharedPtr {
      using NodeT = std::decay_t<decltype(*n)>;
      if constexpr (std::is_same_v<NodeT, agnocast::Node>) {
        return std::make_shared<AgnocastPublisher<MessageT>>(n.get(), topic_name, qos, options);
      } else {
        return std::make_shared<ROS2Publisher<MessageT>>(n.get(), topic_name, qos, options);
      }
    });
  }

  template <typename MessageT>
  typename Publisher<MessageT>::SharedPtr create_publisher(
    const std::string & topic_name, size_t qos_history_depth)
  {
    return create_publisher<MessageT>(topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
  }

  // ===== Subscription =====
  template <typename MessageT, typename Func>
  typename Subscription<MessageT>::SharedPtr create_subscription(
    const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
    const agnocast::SubscriptionOptions & options = agnocast::SubscriptionOptions{})
  {
    return visit_node([&](auto & n) -> typename Subscription<MessageT>::SharedPtr {
      return std::make_shared<Subscription<MessageT>>(
        n.get(), topic_name, qos, std::forward<Func>(callback), options);
    });
  }

  template <typename MessageT, typename Func>
  typename Subscription<MessageT>::SharedPtr create_subscription(
    const std::string & topic_name, size_t qos_history_depth, Func && callback,
    const agnocast::SubscriptionOptions & options = agnocast::SubscriptionOptions{})
  {
    return create_subscription<MessageT>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)), std::forward<Func>(callback),
      options);
  }

  // ===== Polling Subscriber =====
  template <typename MessageT>
  typename PollingSubscriber<MessageT>::SharedPtr create_polling_subscriber(
    const std::string & topic_name, const rclcpp::QoS & qos)
  {
    return visit_node([&](auto & n) -> typename PollingSubscriber<MessageT>::SharedPtr {
      using NodeT = std::decay_t<decltype(*n)>;
      if constexpr (std::is_same_v<NodeT, agnocast::Node>) {
        return std::make_shared<AgnocastPollingSubscriber<MessageT>>(n.get(), topic_name, qos);
      } else {
        return std::make_shared<ROS2PollingSubscriber<MessageT>>(n.get(), topic_name, qos);
      }
    });
  }

  template <typename MessageT>
  typename PollingSubscriber<MessageT>::SharedPtr create_polling_subscriber(
    const std::string & topic_name, size_t qos_history_depth)
  {
    return create_polling_subscriber<MessageT>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
  }

  // ===== Internal node access (for Executor) =====
  // Callers must check is_using_agnocast() before calling get_agnocast_node()/get_rclcpp_node().
  // Accessing the inactive variant will throw std::runtime_error.
  bool is_using_agnocast() const
  {
    return std::holds_alternative<std::shared_ptr<agnocast::Node>>(node_);
  }

  std::shared_ptr<agnocast::Node> get_agnocast_node() const
  {
    if (auto * p = std::get_if<std::shared_ptr<agnocast::Node>>(&node_)) {
      return *p;
    }
    throw std::runtime_error(
      "get_agnocast_node() called but Agnocast is not enabled. "
      "Check is_using_agnocast() before calling this method.");
  }

  std::shared_ptr<rclcpp::Node> get_rclcpp_node() const
  {
    if (auto * p = std::get_if<std::shared_ptr<rclcpp::Node>>(&node_)) {
      return *p;
    }
    throw std::runtime_error(
      "get_rclcpp_node() called but rclcpp mode is not enabled. "
      "Check is_using_agnocast() before calling this method.");
  }

private:
  using NodeVariant = std::variant<std::shared_ptr<rclcpp::Node>, std::shared_ptr<agnocast::Node>>;
  NodeVariant node_;

  template <typename Visitor>
  decltype(auto) visit_node(Visitor && vis)
  {
    return std::visit(std::forward<Visitor>(vis), node_);
  }

  template <typename Visitor>
  decltype(auto) visit_node(Visitor && vis) const
  {
    return std::visit(std::forward<Visitor>(vis), node_);
  }
};

/// @brief Get the underlying rclcpp::Node from a node that inherits agnocast_wrapper::Node.
template <typename T>
std::shared_ptr<rclcpp::Node> to_rclcpp_node(const std::shared_ptr<T> & node)
{
  return node->get_rclcpp_node();
}

}  // namespace autoware::agnocast_wrapper

#else

namespace autoware::agnocast_wrapper
{

using Node = rclcpp::Node;

/// @brief Get the underlying rclcpp::Node from a node that inherits rclcpp::Node.
template <typename T>
std::shared_ptr<rclcpp::Node> to_rclcpp_node(const std::shared_ptr<T> & node)
{
  return node;
}

}  // namespace autoware::agnocast_wrapper

#endif
