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

#include <rclcpp/version.h>

#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/node/agnocast_node.hpp>

namespace autoware::agnocast_wrapper
{

// rclcpp 28+ (Jazzy) renamed OnParametersSetCallbackType to OnSetParametersCallbackType
// and removed the old name from NodeParametersInterface. Humble uses rclcpp 16.x with the old name.
#if RCLCPP_VERSION_MAJOR >= 28
using OnSetParametersCallbackType =
  rclcpp::node_interfaces::NodeParametersInterface::OnSetParametersCallbackType;
#else
using OnSetParametersCallbackType =
  rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType;
#endif

/// @brief Node wrapper class that can switch between rclcpp::Node and agnocast::Node at runtime
/// based on the ENABLE_AGNOCAST environment variable.
///
/// @invariant The backend variant (rclcpp::Node or agnocast::Node) is chosen at construction
///            based on use_agnocast() and never mutates for the lifetime of the Node.
/// @invariant `use_agnocast() == true`  iff `get_agnocast_node()` returns a valid
///            shared_ptr without throwing.
/// @invariant `use_agnocast() == false` iff `get_rclcpp_node()`   returns a valid
///            shared_ptr without throwing.
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
  const char * get_name() const;
  const char * get_namespace() const;
  const char * get_fully_qualified_name() const;
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
    OnSetParametersCallbackType callback);
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
    return declare_parameter(name, value.get_type(), descriptor, ignore_override).get<ParameterT>();
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
      using NodeT = std::decay_t<decltype(*n)>;
      if constexpr (std::is_same_v<NodeT, agnocast::Node>) {
        return std::make_shared<AgnocastSubscription<MessageT>>(
          n.get(), topic_name, qos, std::forward<Func>(callback), options);
      } else {
        return std::make_shared<ROS2Subscription<MessageT>>(
          n.get(), topic_name, qos, std::forward<Func>(callback), options);
      }
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

  // ===== Client / Service =====
  template <typename ServiceT>
  AUTOWARE_CLIENT_PTR(ServiceT)
  create_client(
    const std::string & service_name, const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    return visit_node([&](auto & n) -> AUTOWARE_CLIENT_PTR(ServiceT) {
      using NodeT = std::decay_t<decltype(*n)>;
      if constexpr (std::is_same_v<NodeT, agnocast::Node>) {
        return std::make_shared<AgnocastClient<ServiceT>>(n.get(), service_name, qos, group);
      } else {
        return std::make_shared<ROS2Client<ServiceT>>(n.get(), service_name, qos, group);
      }
    });
  }

  // Service with a callback taking AUTOWARE_SERVER_REQUEST_PTR/RESPONSE_PTR (message_ptr).
  template <
    typename ServiceT, typename Func,
    std::enable_if_t<is_message_ptr_service_callback_v<Func, ServiceT>, int> = 0>
  AUTOWARE_SERVICE_PTR(ServiceT)
  create_service(
    const std::string & service_name, Func && callback,
    const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    return visit_node([&](auto & n) -> AUTOWARE_SERVICE_PTR(ServiceT) {
      using NodeT = std::decay_t<decltype(*n)>;
      if constexpr (std::is_same_v<NodeT, agnocast::Node>) {
        return std::make_shared<AgnocastService<ServiceT>>(
          n.get(), service_name, std::forward<Func>(callback), qos, group);
      } else {
        return std::make_shared<ROS2Service<ServiceT>>(
          n.get(), service_name, std::forward<Func>(callback), qos, group);
      }
    });
  }

  // Convenience overload for rclcpp-style std::shared_ptr callbacks. Gives the callback independent
  // owning copies of the request/response (safe to retain, like rclcpp) and copies the filled
  // response back, so prefer the message_ptr overload above to avoid the copies on hot paths.
  template <
    typename ServiceT, typename Func,
    std::enable_if_t<
      !is_message_ptr_service_callback_v<Func, ServiceT> &&
        is_shared_ptr_service_callback_v<Func, ServiceT>,
      int> = 0>
  AUTOWARE_SERVICE_PTR(ServiceT)
  create_service(
    const std::string & service_name, Func && callback,
    const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    return create_service<ServiceT>(
      service_name,
      [callback = std::forward<Func>(callback)](
        AUTOWARE_SERVER_REQUEST_PTR(ServiceT) && req,
        AUTOWARE_SERVER_RESPONSE_PTR(ServiceT) && res) {
        auto request = std::make_shared<const typename ServiceT::Request>(*req);
        auto response = std::make_shared<typename ServiceT::Response>();
        callback(request, response);
        *res = *response;
      },
      qos, group);
  }

  // Fallback overload: neither callback form matched. Exists only to turn the otherwise opaque
  // "no matching function" error into the static_assert message below.
  template <
    typename ServiceT, typename Func,
    std::enable_if_t<
      !is_message_ptr_service_callback_v<Func, ServiceT> &&
        !is_shared_ptr_service_callback_v<Func, ServiceT>,
      int> = 0>
  AUTOWARE_SERVICE_PTR(ServiceT)
  create_service(
    const std::string & service_name, Func && callback,
    const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    static_assert(
      is_message_ptr_service_callback_v<Func, ServiceT> ||
        is_shared_ptr_service_callback_v<Func, ServiceT>,
      "Service callback must be invocable with "
      "(AUTOWARE_SERVER_REQUEST_PTR(ServiceT), AUTOWARE_SERVER_RESPONSE_PTR(ServiceT)) or with "
      "(std::shared_ptr<ServiceT::Request>, std::shared_ptr<ServiceT::Response>).");
  }

  // ===== Timer =====
  template <typename DurationRepT = int64_t, typename DurationT = std::milli, typename CallbackT>
  Timer::SharedPtr create_wall_timer(
    std::chrono::duration<DurationRepT, DurationT> period, CallbackT && callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    return visit_node([&](auto & n) -> Timer::SharedPtr {
      using NodeT = std::decay_t<decltype(*n)>;
      if constexpr (std::is_same_v<NodeT, agnocast::Node>) {
        return std::make_shared<AgnocastTimer>(
          n->create_wall_timer(period, std::forward<CallbackT>(callback), group));
      } else {
        return std::make_shared<ROS2Timer>(
          n->create_wall_timer(period, std::forward<CallbackT>(callback), group));
      }
    });
  }

  // ===== Internal node access (for Executor) =====
  // Callers must check use_agnocast() before calling get_agnocast_node()/get_rclcpp_node().
  // Accessing the inactive variant will throw std::runtime_error.
  // The return value is fixed for the lifetime of the Node (see class-level @invariant).

  /// @pre `use_agnocast() == true`. Under this precondition, this method is guaranteed
  ///      to return a valid non-null shared_ptr without throwing.
  /// @throws std::runtime_error if Agnocast is not enabled (check use_agnocast() first)
  std::shared_ptr<agnocast::Node> get_agnocast_node() const
  {
    if (auto * p = std::get_if<std::shared_ptr<agnocast::Node>>(&node_)) {
      return *p;
    }
    throw std::runtime_error(
      "get_agnocast_node() called but Agnocast is not enabled. "
      "Check use_agnocast() before calling this method.");
  }

  /// @pre `use_agnocast() == false`. Under this precondition, this method is guaranteed
  ///      to return a valid non-null shared_ptr without throwing.
  /// @throws std::runtime_error if the node is in agnocast mode (check !use_agnocast() first)
  std::shared_ptr<rclcpp::Node> get_rclcpp_node() const
  {
    if (auto * p = std::get_if<std::shared_ptr<rclcpp::Node>>(&node_)) {
      return *p;
    }
    throw std::runtime_error(
      "get_rclcpp_node() called but the node is in agnocast mode. "
      "Check !use_agnocast() before calling this method.");
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
/// @throws std::runtime_error if the node is in agnocast mode (check !use_agnocast() first)
template <typename T>
std::shared_ptr<rclcpp::Node> to_rclcpp_node(const std::shared_ptr<T> & node)
{
  return node->get_rclcpp_node();
}

/// @brief Create a timer driven by an explicit clock.
///
/// Provided as a free function rather than a Node member because
/// rclcpp::Node::create_timer was added in Jazzy and does not exist on Humble;
/// the free rclcpp::create_timer() is available on both. Mirrors the
/// non-Agnocast-build overload so the same call site works in both builds.
template <typename CallbackT>
Timer::SharedPtr create_timer(
  Node * node, rclcpp::Clock::SharedPtr clock, rclcpp::Duration period, CallbackT && callback,
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastTimer>(agnocast::create_timer(
      node->get_agnocast_node().get(), clock, period, std::forward<CallbackT>(callback), group));
  }
  return std::make_shared<ROS2Timer>(rclcpp::create_timer(
    node->get_rclcpp_node().get(), clock, period, std::forward<CallbackT>(callback), group));
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

/// @brief Create a timer driven by an explicit clock (non-Agnocast build).
///
/// Provided as a free function rather than a Node member because
/// rclcpp::Node::create_timer was added in Jazzy and does not exist on Humble;
/// the free rclcpp::create_timer() is available on both.
template <typename CallbackT>
rclcpp::TimerBase::SharedPtr create_timer(
  Node * node, rclcpp::Clock::SharedPtr clock, rclcpp::Duration period, CallbackT && callback,
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return rclcpp::create_timer(node, clock, period, std::forward<CallbackT>(callback), group);
}

}  // namespace autoware::agnocast_wrapper

#endif
