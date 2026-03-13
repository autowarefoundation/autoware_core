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

#include "autoware/agnocast_wrapper/node.hpp"

#ifdef USE_AGNOCAST_ENABLED

namespace autoware::agnocast_wrapper
{

Node::Node(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, "", options)
{
}

Node::Node(
  const std::string & node_name, const std::string & namespace_,
  const rclcpp::NodeOptions & options)
{
  if (use_agnocast()) {
    node_ = std::make_shared<agnocast::Node>(node_name, namespace_, options);
  } else {
    node_ = std::make_shared<rclcpp::Node>(node_name, namespace_, options);
  }
}

std::string Node::get_name() const
{
  return visit_node([](const auto & n) { return std::string(n->get_name()); });
}

std::string Node::get_namespace() const
{
  return visit_node([](const auto & n) { return std::string(n->get_namespace()); });
}

std::string Node::get_fully_qualified_name() const
{
  return visit_node([](const auto & n) { return std::string(n->get_fully_qualified_name()); });
}

rclcpp::Logger Node::get_logger() const
{
  return visit_node([](const auto & n) { return n->get_logger(); });
}

rclcpp::Clock::SharedPtr Node::get_clock()
{
  return visit_node([](auto & n) { return n->get_clock(); });
}

rclcpp::Time Node::now() const
{
  return visit_node([](const auto & n) { return n->now(); });
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr Node::get_node_base_interface()
{
  return visit_node([](auto & n) { return n->get_node_base_interface(); });
}

rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr Node::get_node_topics_interface()
{
  return visit_node([](auto & n) { return n->get_node_topics_interface(); });
}

rclcpp::node_interfaces::NodeParametersInterface::SharedPtr Node::get_node_parameters_interface()
  const
{
  return visit_node([](const auto & n) { return n->get_node_parameters_interface(); });
}

rclcpp::CallbackGroup::SharedPtr Node::create_callback_group(
  rclcpp::CallbackGroupType group_type, bool automatically_add_to_executor_with_node)
{
  return visit_node([&](auto & n) {
    return n->create_callback_group(group_type, automatically_add_to_executor_with_node);
  });
}

const rclcpp::ParameterValue & Node::declare_parameter(
  const std::string & name, const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & descriptor, bool ignore_override)
{
  return visit_node([&](auto & n) -> const rclcpp::ParameterValue & {
    return n->declare_parameter(name, default_value, descriptor, ignore_override);
  });
}

const rclcpp::ParameterValue & Node::declare_parameter(
  const std::string & name, rclcpp::ParameterType type,
  const rcl_interfaces::msg::ParameterDescriptor & descriptor, bool ignore_override)
{
  return visit_node([&](auto & n) -> const rclcpp::ParameterValue & {
    return n->declare_parameter(name, type, descriptor, ignore_override);
  });
}

bool Node::has_parameter(const std::string & name) const
{
  return visit_node([&](const auto & n) { return n->has_parameter(name); });
}

void Node::undeclare_parameter(const std::string & name)
{
  visit_node([&](auto & n) { n->undeclare_parameter(name); });
}

rclcpp::Parameter Node::get_parameter(const std::string & name) const
{
  return visit_node([&](const auto & n) { return n->get_parameter(name); });
}

bool Node::get_parameter(const std::string & name, rclcpp::Parameter & parameter) const
{
  return visit_node([&](const auto & n) { return n->get_parameter(name, parameter); });
}

std::vector<rclcpp::Parameter> Node::get_parameters(const std::vector<std::string> & names) const
{
  return visit_node([&](const auto & n) { return n->get_parameters(names); });
}

rcl_interfaces::msg::SetParametersResult Node::set_parameter(const rclcpp::Parameter & parameter)
{
  return visit_node([&](auto & n) { return n->set_parameter(parameter); });
}

std::vector<rcl_interfaces::msg::SetParametersResult> Node::set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  return visit_node([&](auto & n) { return n->set_parameters(parameters); });
}

rcl_interfaces::msg::SetParametersResult Node::set_parameters_atomically(
  const std::vector<rclcpp::Parameter> & parameters)
{
  return visit_node([&](auto & n) { return n->set_parameters_atomically(parameters); });
}

rcl_interfaces::msg::ParameterDescriptor Node::describe_parameter(const std::string & name) const
{
  return visit_node([&](const auto & n) { return n->describe_parameter(name); });
}

std::vector<rcl_interfaces::msg::ParameterDescriptor> Node::describe_parameters(
  const std::vector<std::string> & names) const
{
  return visit_node([&](const auto & n) { return n->describe_parameters(names); });
}

std::vector<uint8_t> Node::get_parameter_types(const std::vector<std::string> & names) const
{
  return visit_node([&](const auto & n) { return n->get_parameter_types(names); });
}

rcl_interfaces::msg::ListParametersResult Node::list_parameters(
  const std::vector<std::string> & prefixes, uint64_t depth) const
{
  return visit_node([&](const auto & n) { return n->list_parameters(prefixes, depth); });
}

rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
Node::add_on_set_parameters_callback(
  rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType callback)
{
  return visit_node([&](auto & n) { return n->add_on_set_parameters_callback(callback); });
}

void Node::remove_on_set_parameters_callback(
  const rclcpp::node_interfaces::OnSetParametersCallbackHandle * const handler)
{
  visit_node([&](auto & n) { n->remove_on_set_parameters_callback(handler); });
}

}  // namespace autoware::agnocast_wrapper

#endif
